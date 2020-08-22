use alloc::rc::Rc;
use core::cell::RefCell;
use core::f32::consts::PI;
use core::fmt::Write;

use components::{
    data_types::{AbsoluteDirection, NodeId, Pattern, Pose, SearchNodeId},
    impls::{
        ControllerBuilder, EstimatorBuilder, MazeBuilder, ObstacleDetector, TrackerBuilder,
        TrajectoryGeneratorBuilder,
    },
};
use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
use generic_array::arr;
use heapless::consts::*;
use jlink_rtt::Output;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Frequency, Jerk,
    Speed, Time,
};
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

use crate::alias::{Agent, DistanceSensors, Maze, MazeWidth, SearchOperator, Solver, Voltmeter};
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

type LogSize = U2048;
type Logger = ILogger<LogSize>;

pub struct Storage {
    pub search_operator: SearchOperator<Logger>,
    pub log: Rc<RefCell<Log<LogSize>>>,
    pub maze: Rc<Maze>,
}

unsafe impl Sync for Storage {}
unsafe impl Send for Storage {}

pub fn init_storage() -> Storage {
    let log = Rc::new(RefCell::new(Log::new()));
    let logger = Logger::new(Rc::clone(&log));

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

    let wheel_radius = Distance::from_meters(0.00675);

    let period = Time::from_seconds(0.001);
    let mut timer = Timer::tim5(device_peripherals.TIM5, 1.khz(), clocks);
    // let mut timer = Timer::tim9(device_peripherals.TIM9, 1.khz(), clocks);

    let voltmeter = {
        let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
        let pa7 = gpioa.pa7.into_analog();
        Rc::new(RefCell::new(Voltmeter::new(
            adc,
            pa7,
            period,
            Frequency::from_hertz(1.0),
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
                .cut_off_frequency(Frequency::from_hertz(50.0))
                .initial_posture(Angle::from_degree(90.0))
                .initial_x(Distance::from_meters(0.045))
                .initial_y(Distance::from_meters(0.045))
                // .wheel_interval(Distance::from_meters(0.0335))
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

            let trans_controller = ControllerBuilder::new()
                .kp(0.9)
                .ki(0.05)
                .kd(0.01)
                .period(period)
                .model_gain(1.0)
                .model_time_constant(Time::from_seconds(0.3694))
                .build();

            let rot_controller = ControllerBuilder::new()
                .kp(0.2)
                .ki(0.2)
                .kd(0.0)
                .period(period)
                .model_gain(10.0)
                .model_time_constant(Time::from_seconds(0.1499))
                .build();

            TrackerBuilder::new()
                .right_motor(right_motor)
                .left_motor(left_motor)
                .period(period)
                .kx(40.0)
                .kdx(4.0)
                .ky(40.0)
                .kdy(4.0)
                .valid_control_lower_bound(Speed::from_meter_per_second(0.03))
                .translation_controller(trans_controller)
                .rotation_controller(rot_controller)
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(Distance::from_meters(0.05))
                .logger(logger)
                .build()
        };

        let search_speed = Speed::from_meter_per_second(0.12);

        let trajectory_generator = TrajectoryGeneratorBuilder::new()
            .period(period)
            .max_speed(Speed::from_meter_per_second(2.0))
            .max_acceleration(Acceleration::from_meter_per_second_squared(0.7))
            .max_jerk(Jerk::from_meter_per_second_cubed(1.0))
            .search_speed(search_speed)
            .slalom_speed_ref(Speed::from_meter_per_second(0.27178875))
            .angular_speed_ref(AngularSpeed::from_radian_per_second(3.0 * PI))
            .angular_acceleration_ref(AngularAcceleration::from_radian_per_second_squared(
                36.0 * PI,
            ))
            .angular_jerk_ref(AngularJerk::from_radian_per_second_cubed(1200.0 * PI))
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
                        x: Distance::from_meters(-0.0115),
                        y: Distance::from_meters(0.013),
                        theta: Angle::from_degree(90.0),
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
                        x: Distance::from_meters(0.0115),
                        y: Distance::from_meters(0.013),
                        theta: Angle::from_degree(-90.0),
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
                        x: Distance::from_meters(0.0),
                        y: Distance::from_meters(0.023),
                        theta: Angle::from_degree(0.0),
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
        Rc::new(Agent::new(
            obstacle_detector,
            estimator,
            tracker,
            trajectory_generator,
        ))
    };

    let maze = Rc::new(
        MazeBuilder::new()
            .costs(costs as fn(Pattern) -> u16)
            .wall_existence_probability_threshold(0.3)
            .build::<MazeWidth>(),
    );

    let solver = Rc::new(Solver::new(
        NodeId::new(0, 0, AbsoluteDirection::North).unwrap(),
        arr![
            NodeId<MazeWidth>;
            NodeId::new(2,0,AbsoluteDirection::South).unwrap(),
            NodeId::new(2,0,AbsoluteDirection::West).unwrap()
        ],
    ));
    timer.listen(Event::TimeOut);
    free(|cs| {
        TIMER_TIM5.borrow(cs).replace(Some(timer));
    });
    Storage {
        search_operator: SearchOperator::new(
            Pose::new(
                Distance::from_meters(0.045),
                Distance::from_meters(0.045),
                Angle::from_degree(90.0),
            ),
            SearchNodeId::new(0, 1, AbsoluteDirection::North).unwrap(),
            Rc::clone(&maze),
            agent,
            solver,
        ),
        log,
        maze,
    }
}
