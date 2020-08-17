use core::cell::RefCell;
use core::f32::consts::PI;

use components::{
    data_types::Pose,
    impls::{
        ControllerBuilder, EstimatorBuilder, ObstacleDetector, TrackerBuilder,
        TrajectoryGeneratorBuilder,
    },
};
use embedded_hal::prelude::*;
use generic_array::arr;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Frequency, Jerk,
    Speed, Time,
};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    gpio,
    i2c::I2c,
    prelude::*,
    pwm::tim1,
    qei::Qei,
    rcc::Clocks,
    spi::Spi,
    stm32::{self, TIM9},
    timer::Timer,
};

use crate::{
    alias::{Agent, DistanceSensors, SensorI2c, Voltmeter},
    sensors::{IMotor, ICM20648, MA702GQ, VL6180X},
};

pub fn init_agent<'a>() -> Agent<'a> {
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

    let wheel_radius = Distance::from_meters(0.00675);

    let period = Time::from_seconds(0.001);
    let mut timer = Timer::tim9(device_peripherals.TIM9, 1.khz(), clocks);

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

            let mut imu = ICM20648::new(spi, cs, &mut delay, timer);
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
            // .wheel_interval(Distance::from_meters(0.0335))
            .build()
    };

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
                IMotor::new(pwm2, pwm1, &voltmeter),
                IMotor::new(pwm4, pwm3, &voltmeter),
            )
        };

        let trans_controller = ControllerBuilder::new()
            .kp(0.9)
            .ki(0.05)
            .kd(0.01)
            .period(period)
            .model_gain(2.0)
            .model_time_constant(Time::from_seconds(0.3694))
            .build();

        let rot_controller = ControllerBuilder::new()
            .kp(0.2)
            .ki(0.2)
            .kd(0.0)
            .period(period)
            .model_gain(20.0)
            .model_time_constant(Time::from_seconds(0.1499))
            .build();

        TrackerBuilder::new()
            .right_motor(right_motor)
            .left_motor(left_motor)
            .period(period)
            .kx(30.0)
            .kdx(9.0)
            .ky(30.0)
            .kdy(9.0)
            .valid_control_lower_bound(Speed::from_meter_per_second(0.03))
            .translation_controller(trans_controller)
            .rotation_controller(rot_controller)
            .kanayama_kx(3.0)
            .kanayama_ky(3.0)
            .kanayama_ktheta(30.0)
            .build()
    };

    let search_speed = Speed::from_meter_per_second(0.15);

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

    let i2c = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c_pins = (scl, sda);
        RefCell::new(I2c::i2c1(
            device_peripherals.I2C1,
            i2c_pins,
            400.khz(),
            clocks,
        ))
    };

    let obstacle_detector = {
        let tof1 = {
            let enable_pin = gpioh.ph1.into_push_pull_output();
            VL6180X::<_, _>::new(
                &i2c,
                enable_pin,
                &mut delay,
                0x31,
                Pose {
                    x: Distance::from_meters(-0.015),
                    y: Distance::from_meters(0.015),
                    theta: Angle::from_degree(0.0),
                },
            )
        };
        let tof2 = {
            let enable_pin = gpioa.pa15.into_push_pull_output();
            VL6180X::<_, _>::new(
                &i2c,
                enable_pin,
                &mut delay,
                0x30,
                Pose {
                    x: Distance::from_meters(0.015),
                    y: Distance::from_meters(0.015),
                    theta: Angle::from_degree(180.0),
                },
            )
        };
        let tof3 = {
            let enable_pin = gpioh.ph0.into_push_pull_output();
            VL6180X::<_, _>::new(
                &i2c,
                enable_pin,
                &mut delay,
                0x29,
                Pose {
                    x: Distance::from_meters(0.0),
                    y: Distance::from_meters(0.025),
                    theta: Angle::from_degree(90.0),
                },
            )
        };

        ObstacleDetector::new(arr![
            DistanceSensors;
            DistanceSensors::Left(tof1),
            DistanceSensors::Right(tof2),
            DistanceSensors::Front(tof3)
        ])
    };

    Agent::new(obstacle_detector, estimator, tracker, trajectory_generator)
}
