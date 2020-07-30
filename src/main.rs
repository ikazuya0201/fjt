#![no_std]
#![no_main]

extern crate panic_rtt;
mod macros;
mod sensors;

use core::cell::RefCell;
use core::convert::Infallible;
use core::fmt::Write;

use components::{
    agent::{pose::Pose, StateEstimator, Tracker},
    controller::ControllerBuilder,
    estimator::EstimatorBuilder,
    maze::RelativeDirection,
    sensors::IMU,
    tracker::{Motor, TrackerBuilder},
    trajectory_generator::{SubTarget, Target, TrajectoryGeneratorBuilder},
    utils::vector::Vector2,
};
use cortex_m_rt::entry;
use embedded_hal::prelude::*;
use jlink_rtt::Output;
use nb::block;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Frequency, Jerk,
    Speed, Time, Voltage,
};
use sensors::{IMUError, IMotor, IVoltmeter, ICM20648, MA702GQ};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    i2c::{Error as I2cError, I2c},
    prelude::*,
    pwm::tim1,
    qei::Qei,
    spi::{Error as SpiError, Spi},
    stm32,
    timer::Timer,
};

impl From<SpiError> for IMUError {
    fn from(_error: SpiError) -> Self {
        Self
    }
}

impl From<Infallible> for IMUError {
    fn from(_error: Infallible) -> Self {
        Self
    }
}

#[entry]
fn main() -> ! {
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
    let gpioa = device_peripherals.GPIOA.split();
    let gpiob = device_peripherals.GPIOB.split();
    let gpioc = device_peripherals.GPIOC.split();
    let gpioh = device_peripherals.GPIOH.split();

    delay.delay_ms(3000u16);

    let wheel_radius = Distance::from_meters(0.0065);

    let right_encoder = {
        let pins = (
            gpioa.pa0.into_alternate_af1(),
            gpioa.pa1.into_alternate_af1(),
        );
        let qei = Qei::tim2(device_peripherals.TIM2, pins);
        let mut encoder = MA702GQ::new(qei, wheel_radius);
        encoder
    };

    let left_encoder = {
        let pins = (
            gpiob.pb6.into_alternate_af2(),
            gpiob.pb7.into_alternate_af2(),
        );
        let qei = Qei::tim4(device_peripherals.TIM4, pins);
        let mut encoder = MA702GQ::new(qei, wheel_radius);
        encoder
    };

    let period = Time::from_seconds(0.001);
    let mut timer = Timer::tim9(device_peripherals.TIM9, 1.khz(), clocks);

    let mut imu = {
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

    let mut estimator = EstimatorBuilder::new()
        .left_encoder(left_encoder)
        .right_encoder(right_encoder)
        .imu(imu)
        .period(period)
        .cut_off_frequency(Frequency::from_hertz(50.0))
        .initial_posture(Angle::from_degree(90.0))
        // .wheel_interval(Distance::from_meters(0.0335))
        .build();

    let voltmeter = {
        let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
        let pa7 = gpioa.pa7.into_analog();
        RefCell::new(IVoltmeter::new(
            adc,
            pa7,
            period,
            Frequency::from_hertz(1.0),
        ))
    };

    let mut tracker = {
        let (mut left_motor, mut right_motor) = {
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
                IMotor::new(pwm2, pwm1, &voltmeter),
                IMotor::new(pwm4, pwm3, &voltmeter),
            )
        };
        let trans_controller = ControllerBuilder::new()
            .kp(0.7)
            .ki(0.05)
            .kd(0.05)
            .period(period)
            .model_gain(1000.0)
            .model_time_constant(Time::from_seconds(0.001))
            .build();

        let rot_controller = ControllerBuilder::new()
            .kp(0.2)
            .ki(0.1)
            .kd(0.0)
            .period(period)
            .model_gain(1000.0)
            .model_time_constant(Time::from_seconds(0.001))
            .build();

        TrackerBuilder::new()
            .right_motor(right_motor)
            .left_motor(left_motor)
            .period(period)
            .kx(30.0)
            .kdx(3.0)
            .ky(30.0)
            .kdy(3.0)
            .valid_control_lower_bound(Speed::from_meter_per_second(0.005))
            .translation_controller(trans_controller)
            .rotation_controller(rot_controller)
            .kanayama_kx(1.0)
            .kanayama_ky(1.0)
            .kanayama_ktheta(5.0)
            .build()
    };

    let mut out = Output::new();

    let mut flag = false;

    use core::f32::consts::PI;

    let search_speed = Speed::from_meter_per_second(0.15);

    let trajectory_generator = TrajectoryGeneratorBuilder::new()
        .period(period)
        .max_speed(Speed::from_meter_per_second(2.0))
        .max_acceleration(Acceleration::from_meter_per_second_squared(1.0))
        .max_jerk(Jerk::from_meter_per_second_cubed(0.5))
        .search_speed(search_speed)
        .slalom_speed_ref(Speed::from_meter_per_second(0.24159))
        // .slalom_speed_ref(Speed::from_meter_per_second(0.5))
        .angular_speed_ref(AngularSpeed::from_radian_per_second(3.0 * PI))
        .angular_acceleration_ref(AngularAcceleration::from_radian_per_second_squared(
            36.0 * PI,
        ))
        .angular_jerk_ref(AngularJerk::from_radian_per_second_cubed(1200.0 * PI))
        .build();

    let stop_trajectory = core::iter::repeat(Target {
        x: SubTarget {
            x: Distance::from_meters(-0.14),
            ..Default::default()
        },
        y: SubTarget {
            x: Distance::from_meters(0.14),
            ..Default::default()
        },
        theta: SubTarget {
            x: Angle::from_radian(180.0),
            v: AngularSpeed::from_radian_per_second(0.0),
            a: AngularAcceleration::from_radian_per_second_squared(0.0),
            j: AngularJerk::from_radian_per_second_cubed(0.0),
        },
    });
    // let stop_trajectory = Trajectory::Move(Vector2 {
    //     x: Target {
    //         x: Distance::from_meters(0.0),
    //         v: Speed::from_meter_per_second(0.0),
    //         a: Acceleration::from_meter_per_second_squared(0.0),
    //         j: Jerk::from_meter_per_second_cubed(0.0),
    //     },
    //     y: Target {
    //         x: Distance::from_meters(0.1),
    //         v: Speed::from_meter_per_second(0.0),
    //         a: Acceleration::from_meter_per_second_squared(0.0),
    //         j: Jerk::from_meter_per_second_cubed(0.0),
    //     },
    // });

    // let mut trajectory = trajectory_generator
    //     .generate_straight(
    //         Distance::from_meters(0.0),
    //         Distance::from_meters(0.0),
    //         Distance::from_meters(0.0),
    //         Distance::from_meters(0.1),
    //         Speed::from_meter_per_second(0.0),
    //         search_speed,
    //     )
    //     // .chain(stop_trajectory);
    //     .chain(trajectory_generator.generate_search_trajectory(
    //         Pose {
    //             x: Distance::from_meters(0.0),
    //             y: Distance::from_meters(0.1),
    //             theta: Angle::from_degree(90.0),
    //         },
    //         RelativeDirection::Left,
    //     ))
    //     .chain(trajectory_generator.generate_straight(
    //         Distance::from_meters(-0.04),
    //         Distance::from_meters(0.14),
    //         Distance::from_meters(-0.14),
    //         Distance::from_meters(0.14),
    //         search_speed,
    //         Speed::from_meter_per_second(0.0),
    //     ))
    //     .chain(stop_trajectory);
    let mut trajectory = trajectory_generator
        .generate_straight(
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Distance::from_meters(0.1),
            Default::default(),
            search_speed,
        )
        .chain(trajectory_generator.generate_search_trajectory(
            Pose {
                x: Distance::from_meters(0.0),
                y: Distance::from_meters(0.1),
                theta: Angle::from_degree(90.0),
            },
            RelativeDirection::Back,
        ))
        .chain(trajectory_generator.generate_straight(
            Distance::from_meters(0.0),
            Distance::from_meters(0.1),
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            search_speed,
            Default::default(),
        ));

    let mut count = 0;
    loop {
        let state = estimator.estimate();
        // let target = Trajectory::Move(Vector2 {
        //     x: Target {
        //         x: Distance::from_meters(0.0),
        //         v: Speed::from_meter_per_second(0.0),
        //         a: Acceleration::from_meter_per_second_squared(0.0),
        //         j: Jerk::from_meter_per_second_cubed(0.0),
        //     },
        //     y: Target {
        //         x: Distance::from_meters(0.0),
        //         v: Speed::from_meter_per_second(0.0),
        //         a: Acceleration::from_meter_per_second_squared(0.0),
        //         j: Jerk::from_meter_per_second_cubed(0.0),
        //     },
        // });
        // let target = Trajectory::Spin(Target {
        //     x: Angle::from_radian(0.0),
        //     v: AngularSpeed::from_radian_per_second(0.0),
        //     a: AngularAcceleration::from_radian_per_second_squared(0.0),
        //     j: AngularJerk::from_radian_per_second_cubed(0.0),
        // });
        if let Some(target) = trajectory.next() {
            tracker.track(state, target);
        }
        // count += 1;
        // if count == 1000 {
        //     writeln!(out, "{:?}", state).unwrap();
        //     // writeln!(out, "{:?}", imu.get_acceleration_y());
        //     count = 0;
        // }
        block!(timer.wait()).ok();
    }
}
