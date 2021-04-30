use alloc::rc::Rc;
use core::cell::RefCell;
use core::fmt::Write;

use components::{
    defaults::{
        config::{ConfigBuilder, ConfigContainer},
        resource::{ResourceBuilder, ResourceContainer},
        state::{StateBuilder, StateContainer},
    },
    nodes::RunNode,
    prelude::*,
    types::data::{AbsoluteDirection, AngleState, LengthState, Pose, RobotState, SearchKind},
    utils::probability::Probability,
    wall_manager::WallManager,
};
use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
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
    angular_acceleration::degree_per_second_squared,
    angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second,
    electric_potential::volt,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, ElectricPotential,
        Frequency, Jerk, Length, Time, Velocity,
    },
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::meter,
    time::second,
    velocity::meter_per_second,
};

use crate::alias::{DistanceSensors, SearchOperator, Voltmeter, N};
use crate::sensors::{IMotor, Voltmeter as IVoltmeter, ICM20648, MA702GQ, VL6180X};
use crate::TIMER_TIM5;

pub struct Bag {
    pub operator: SearchOperator,
}

unsafe impl Sync for Bag {}
unsafe impl Send for Bag {}

pub fn init_bag() -> Bag {
    use AbsoluteDirection::*;

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

    let config = ConfigBuilder::new()
        .start(RunNode::<N>::new(0, 0, North).unwrap())
        .return_goal(RunNode::<N>::new(0, 0, South).unwrap())
        .goals(
            core::array::IntoIter::new([
                RunNode::<N>::new(2, 0, South).unwrap(),
                RunNode::<N>::new(2, 0, West).unwrap(),
            ])
            .collect(),
        )
        .search_initial_route(SearchKind::Init)
        .search_final_route(SearchKind::Final)
        .estimator_cut_off_frequency(Frequency::new::<hertz>(50.0))
        .period(Time::new::<second>(0.001))
        .translational_kp(5.1372)
        .translational_ki(30.81)
        .translational_kd(0.01)
        .translational_model_gain(0.8401)
        .translational_model_time_constant(Time::new::<second>(0.3619))
        .rotational_kp(0.048469)
        .rotational_ki(0.29326)
        .rotational_kd(0.0)
        .rotational_model_gain(75.33)
        .rotational_model_time_constant(Time::new::<second>(0.1999))
        .tracker_kx(40.0)
        .tracker_kdx(8.0)
        .tracker_ky(40.0)
        .tracker_kdy(8.0)
        .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.1))
        .low_zeta(1.0)
        .low_b(0.5)
        .front_offset(Length::new::<meter>(0.005))
        .ignore_radius_from_pillar(Length::new::<meter>(0.01))
        .fail_safe_distance(Length::new::<meter>(0.05))
        .search_velocity(Velocity::new::<meter_per_second>(0.12))
        .max_velocity(Velocity::new::<meter_per_second>(1.0))
        .max_acceleration(Acceleration::new::<meter_per_second_squared>(3.0))
        .max_jerk(Jerk::new::<meter_per_second_cubed>(50.0))
        .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
        .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
            1800.0,
        ))
        .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
        .run_slalom_velocity(Velocity::new::<meter_per_second>(0.3))
        .wheel_interval(Length::new::<meter>(0.0335))
        .estimator_correction_weight(0.0)
        .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
        .build()
        .expect("Should never panic");
    let wheel_radius = Length::new::<meter>(0.0069);

    let mut timer = Timer::tim5(device_peripherals.TIM5, 1000.hz(), clocks);

    let voltmeter = {
        let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
        let pa7 = gpioa.pa7.into_analog();
        Rc::new(RefCell::new(Voltmeter::new(
            adc,
            pa7,
            *config.period(),
            Frequency::new::<hertz>(1.0),
        )))
    };

    if voltmeter.borrow_mut().get_voltage() < ElectricPotential::new::<volt>(3.8) {
        panic!("Low voltage");
    }

    let i2c = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c_pins = (scl, sda);
        Rc::new(RefCell::new(I2c::new(
            device_peripherals.I2C1,
            i2c_pins,
            400.khz(),
            clocks,
        )))
    };

    let mut resource: ResourceContainer<_, _, _, _, _, _, N> = {
        let right_encoder = {
            let pins = (
                gpioa.pa0.into_alternate_af1(),
                gpioa.pa1.into_alternate_af1(),
            );
            let qei = Qei::new(device_peripherals.TIM2, pins);
            let encoder = MA702GQ::new(qei, wheel_radius);
            encoder
        };

        let left_encoder = {
            let pins = (
                gpiob.pb6.into_alternate_af2(),
                gpiob.pb7.into_alternate_af2(),
            );
            let qei = Qei::new(device_peripherals.TIM4, pins);
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

        let tof_left = {
            let enable_pin = gpioh.ph1.into_push_pull_output();
            VL6180X::<_, _>::new(
                Rc::clone(&i2c),
                enable_pin,
                &mut delay,
                0x31,
                Pose {
                    x: Length::new::<meter>(-0.012),
                    y: Length::new::<meter>(0.013),
                    theta: Angle::new::<degree>(90.0),
                },
                0.8333334,
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
                    x: Length::new::<meter>(0.012),
                    y: Length::new::<meter>(0.013),
                    theta: Angle::new::<degree>(-90.0),
                },
                0.9677419,
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
                    y: Length::new::<meter>(0.024),
                    theta: Angle::new::<degree>(0.0),
                },
                1.0,
            )
        };

        // loop {
        //     writeln!(
        //         out,
        //         "left:{:?}\nright:{:?}\nfront:{:?}",
        //         tof_left.get_distance(),
        //         tof_right.get_distance(),
        //         tof_front.get_distance()
        //     )
        //     .unwrap();
        //     delay.delay_ms(1000u16);
        // }

        let existence_threshold = Probability::new(0.1).unwrap();

        // let wall_manager = WallManager::with_str(
        //     existence_threshold,
        //     r"+---+---+---+---+
        // |               |
        // +   +---+---+   +
        // |   |       |   |
        // +   +   +   +   +
        // |   |   |       |
        // +   +   +---+   +
        // |   |       |   |
        // +---+---+---+---+",
        // );
        let wall_manager = WallManager::new(existence_threshold);

        ResourceBuilder::new()
            .left_encoder(left_encoder)
            .right_encoder(right_encoder)
            .left_motor(left_motor)
            .right_motor(right_motor)
            .wall_manager(Rc::new(wall_manager))
            .imu(imu)
            .distance_sensors(
                core::array::IntoIter::new([
                    DistanceSensors::Front(tof_front),
                    DistanceSensors::Right(tof_right),
                    DistanceSensors::Left(tof_left),
                ])
                .collect(),
            )
            .build()
            .expect("Should never panic")
            .into()
    };

    let state: StateContainer<N> = StateBuilder::new()
        .current_node(config.start().clone().into())
        .robot_state(RobotState {
            x: LengthState {
                x: Length::new::<meter>(0.045),
                ..Default::default()
            },
            y: LengthState {
                x: Length::new::<meter>(0.045),
                ..Default::default()
            },
            theta: AngleState {
                x: Angle::new::<degree>(90.0),
                ..Default::default()
            },
        })
        .build()
        .expect("Should never panic")
        .into();

    let config: ConfigContainer<N> = config.into();

    // let mut robot = crate::alias::Robot::construct(&config, &state, &mut resource);
    // let target = Target {
    //     x: LengthTarget {
    //         x: Length::new::<meter>(0.045),
    //         ..Default::default()
    //     },
    //     y: LengthTarget {
    //         x: Length::new::<meter>(0.045),
    //         ..Default::default()
    //     },
    //     theta: AngleTarget {
    //         x: Angle::new::<degree>(90.0),
    //         ..Default::default()
    //     },
    // };

    timer.listen(Event::TimeOut);
    free(|cs| {
        TIMER_TIM5.borrow(cs).replace(Some(timer));
    });

    Bag {
        operator: SearchOperator::construct(&config, &state, &mut resource),
    }
}
