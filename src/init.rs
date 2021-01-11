use alloc::rc::Rc;
use core::cell::RefCell;
use core::f32::consts::PI;
use core::fmt::Write;

use components::{
    data_types::{AbsoluteDirection, Pattern, Pose, SearchKind},
    impls::{
        slalom_parameters_map, CommandConverter, EstimatorBuilder, Maze, NullLogger,
        ObstacleDetector, PoseConverter, RotationControllerBuilder, TrackerBuilder,
        TrajectoryGeneratorBuilder, TranslationControllerBuilder, WallConverter, WallManager,
    },
    utils::probability::Probability,
};
use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
use generic_array::arr;
use heapless::consts::*;
use jlink_rtt::Output;
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    i2c::I2c,
    prelude::*,
    pwm::tim1,
    qei::Qei,
    spi::Spi,
    stm32,
    timer::{Event, Timer},
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::radian_per_second_squared,
    angular_jerk::radian_per_second_cubed,
    angular_velocity::radian_per_second,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk,
        Length, Time, Velocity,
    },
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::meter,
    time::second,
    velocity::meter_per_second,
};

use crate::alias::{Commander, DistanceSensors, RunAgent, RunNode, RunOperator, Voltmeter};
use crate::logger::{ILogger, Log};
use crate::sensors::{IMotor, ICM20648, MA702GQ, VL6180X};
use crate::TIMER_TIM5;

fn costs(pattern: Pattern) -> u16 {
    use Pattern::*;

    match pattern {
        Straight(x) => 10 * x,
        StraightDiagonal(x) => 7 * x,
        Search90 => 8,
        FastRun45 => 12,
        FastRun90 => 15,
        FastRun135 => 20,
        FastRun180 => 25,
        FastRunDiagonal90 => 15,
        SpinBack => 15,
    }
}

type LogSize = U0;
#[allow(unused)]
type Logger = ILogger<LogSize>;

pub struct Bag {
    pub run_operator: RunOperator<NullLogger>,
    pub log: Rc<RefCell<Option<Log<LogSize>>>>,
}

unsafe impl Sync for Bag {}
unsafe impl Send for Bag {}

pub fn init_bag() -> Bag {
    let cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
    let device_peripherals = stm32::Peripherals::take().unwrap();

    let rcc = device_peripherals.RCC.constrain();

    let clocks = rcc
        .cfgr
        .hclk(100_000_000.hz())
        .sysclk(100_000_000.hz())
        .pclk1(50_000_000.hz())
        .pclk2(100_000_000.hz())
        .freeze();

    let mut delay = Delay::new(cortex_m_peripherals.SYST, clocks);

    let mut out = Output::new();
    writeln!(out, "waiting 5s...").unwrap();
    delay.delay_ms(5000u32);

    let gpioa = device_peripherals.GPIOA.split();
    let gpiob = device_peripherals.GPIOB.split();
    let gpioc = device_peripherals.GPIOC.split();
    let gpioh = device_peripherals.GPIOH.split();

    let wheel_radius = Length::new::<meter>(0.00675);

    let period = Time::new::<second>(0.0025);
    let mut timer = Timer::tim5(device_peripherals.TIM5, 400.hz(), clocks);

    let voltmeter = {
        let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
        let pa7 = gpioa.pa7.into_analog();
        Rc::new(RefCell::new(Voltmeter::new(
            adc,
            pa7,
            period,
            Frequency::new::<hertz>(1.0),
        )))
    };

    let i2c = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c_pins = (scl, sda);
        Rc::new(RefCell::new(I2c::i2c1(
            device_peripherals.I2C1,
            i2c_pins,
            400.khz(),
            clocks,
        )))
    };

    let agent = {
        let estimator = {
            let right_encoder = {
                let pins = (
                    gpioa.pa0.into_alternate_af1(),
                    gpioa.pa1.into_alternate_af1(),
                );
                let qei = Qei::tim2(device_peripherals.TIM2, pins);
                let encoder = MA702GQ::new(qei, wheel_radius);
                encoder
            };

            let left_encoder = {
                let pins = (
                    gpiob.pb6.into_alternate_af2(),
                    gpiob.pb7.into_alternate_af2(),
                );
                let qei = Qei::tim4(device_peripherals.TIM4, pins);
                let encoder = MA702GQ::new(qei, wheel_radius);
                encoder
            };

            let imu = {
                let spi_pins = (
                    gpiob.pb3.into_alternate_af5(),
                    gpiob.pb4.into_alternate_af5(),
                    gpiob.pb5.into_alternate_af5(),
                );

                let spi = Spi::spi1(
                    device_peripherals.SPI1,
                    spi_pins,
                    stm32f4xx_hal::hal::spi::MODE_3,
                    1_562_500.hz(),
                    clocks,
                );
                let mut cs = gpioc.pc15.into_push_pull_output();
                cs.set_high().unwrap();

                let mut imu = ICM20648::new(spi, cs, &mut delay, &mut timer);
                imu.spi = imu.spi.init(
                    stm32f4xx_hal::hal::spi::MODE_3,
                    6_250_000.hz(),
                    clocks.pclk2(),
                );
                imu
            };

            EstimatorBuilder::new()
                .left_encoder(left_encoder)
                .right_encoder(right_encoder)
                .imu(imu)
                .period(period)
                .cut_off_frequency(Frequency::new::<hertz>(50.0))
                .initial_posture(Angle::new::<degree>(90.0))
                .initial_x(Length::new::<meter>(0.045))
                .initial_y(Length::new::<meter>(0.045))
                .wheel_interval(Length::new::<meter>(0.0335))
                .build()
        };

        let tracker = {
            let (left_motor, right_motor) = {
                let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = {
                    let pins = (
                        gpioa.pa8.into_alternate_af1(),
                        gpioa.pa9.into_alternate_af1(),
                        gpioa.pa10.into_alternate_af1(),
                        gpioa.pa11.into_alternate_af1(),
                    );
                    tim1(device_peripherals.TIM1, pins, clocks, 100.khz())
                };
                pwm1.enable();
                pwm2.enable();
                pwm3.enable();
                pwm4.enable();
                (
                    IMotor::new(pwm2, pwm1, Rc::clone(&voltmeter)),
                    IMotor::new(pwm4, pwm3, voltmeter),
                )
            };

            let trans_controller = TranslationControllerBuilder::new()
                .kp(0.9)
                .ki(0.05)
                .kd(0.02)
                .period(period)
                .model_gain(1.0)
                .model_time_constant(Time::new::<second>(0.3694))
                .build();

            let rot_controller = RotationControllerBuilder::new()
                .kp(0.2)
                .ki(0.05)
                .kd(0.00001)
                .period(period)
                .model_gain(10.0)
                .model_time_constant(Time::new::<second>(0.1499))
                .build();

            TrackerBuilder::new()
                .right_motor(right_motor)
                .left_motor(left_motor)
                .period(period)
                .kx(40.0)
                .kdx(3.0)
                .ky(40.0)
                .kdy(3.0)
                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                .translation_controller(trans_controller)
                .rotation_controller(rot_controller)
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(Length::new::<meter>(0.05))
                .build()
        };

        let search_velocity = Velocity::new::<meter_per_second>(0.12);

        let trajectory_generator = TrajectoryGeneratorBuilder::new()
            .period(period)
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.5))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
            .search_velocity(search_velocity)
            .slalom_parameters_map(slalom_parameters_map)
            .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
            .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                36.0 * PI,
            ))
            .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(0.2))
            .build();

        let obstacle_detector = {
            let tof_left = {
                let enable_pin = gpioh.ph1.into_push_pull_output();
                VL6180X::<_, _>::new(
                    Rc::clone(&i2c),
                    enable_pin,
                    &mut delay,
                    0x31,
                    Pose {
                        x: Length::new::<meter>(-0.0115),
                        y: Length::new::<meter>(0.013),
                        theta: Angle::new::<degree>(90.0),
                    },
                )
            };
            let tof_right = {
                let enable_pin = gpioa.pa15.into_push_pull_output();
                VL6180X::<_, _>::new(
                    Rc::clone(&i2c),
                    enable_pin,
                    &mut delay,
                    0x30,
                    Pose {
                        x: Length::new::<meter>(0.0115),
                        y: Length::new::<meter>(0.013),
                        theta: Angle::new::<degree>(-90.0),
                    },
                )
            };
            let tof_front = {
                let enable_pin = gpioh.ph0.into_push_pull_output();
                VL6180X::<_, _>::new(
                    i2c,
                    enable_pin,
                    &mut delay,
                    0x29,
                    Pose {
                        x: Length::new::<meter>(0.0),
                        y: Length::new::<meter>(0.023),
                        theta: Angle::new::<degree>(0.0),
                    },
                )
            };

            ObstacleDetector::new(arr![
                DistanceSensors;
                DistanceSensors::Left(tof_left),
                DistanceSensors::Right(tof_right),
                DistanceSensors::Front(tof_front)
            ])
        };
        RunAgent::new(obstacle_detector, estimator, tracker, trajectory_generator)
    };
    let commander = {
        let existence_threshold = Probability::new(0.1).unwrap();

        let input_str = "+---+---+---+---+
|               |
+   +---+---+   +
|   |       |   |
+   +   +   +   +
|   |   |       |
+   +   +---+   +
|   |       |   |
+---+---+---+---+";

        use AbsoluteDirection::*;
        let wall_manager = WallManager::with_str(existence_threshold, input_str);
        let pose_converter = PoseConverter::default();
        let wall_converter = WallConverter::new(costs);
        let maze = Maze::new(wall_manager, pose_converter, wall_converter);
        let start = RunNode::new(0, 0, North, costs).unwrap();
        let goals = arr![
            RunNode;
            RunNode::new(2, 0, South, costs).unwrap(),
            RunNode::new(2, 0, West, costs).unwrap(),
        ];
        Commander::new(start, goals, SearchKind::Init, SearchKind::Final, maze)
    };

    timer.listen(Event::TimeOut);
    free(|cs| {
        TIMER_TIM5.borrow(cs).replace(Some(timer));
    });
    Bag {
        run_operator: RunOperator::new(agent, commander, CommandConverter::default()),
        log: Rc::new(RefCell::new(None)),
    }
}
