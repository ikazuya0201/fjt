use alloc::rc::Rc;
use core::cell::RefCell;
use core::fmt::Write;

use components::{
    defaults::{
        config::{ConfigBuilder, ConfigContainer},
        operator::Operators,
        operator_store::{Mode, OperatorStore},
        resource::{ResourceBuilder, ResourceContainer},
        state::{StateBuilder, StateContainer},
    },
    prelude::*,
    types::data::{
        AbsoluteDirection, AngleState, ControlParameters, LengthState, Pose, Position, RobotState,
        RunNode, SearchKind,
    },
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

use crate::alias::{Administrator, DistanceSensors, SearchOperator, Voltmeter, N};
use crate::interrupt_manager::InterruptManager;
use crate::selector::Selector;
use crate::sensors::{IMotor, Voltmeter as IVoltmeter, ICM20648, MA702GQ, VL6180X};
use crate::TIMER_TIM5;

pub struct Bag {
    pub administrator: Administrator,
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
        .return_goal(Position::new(0, 0).unwrap())
        .goal(Position::new(2, 0).unwrap())
        .search_initial_route(SearchKind::Init)
        .search_final_route(SearchKind::Final)
        .estimator_cut_off_frequency(Frequency::new::<hertz>(50.0))
        .period(Time::new::<second>(0.001))
        .translational_parameters(ControlParameters {
            kp: 4.8497,
            ki: 29.5783,
            kd: 0.0,
            model_k: 1.865,
            model_t1: 0.4443,
        })
        .rotational_parameters(ControlParameters {
            kp: 0.21134,
            ki: 2.9317,
            kd: 0.0,
            model_k: 82.39,
            model_t1: 0.2855,
        })
        .tracker_gain(120.0)
        .tracker_dgain(8.0)
        .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.2))
        .low_zeta(1.0)
        .low_b(1.0)
        .front_offset(Length::new::<meter>(0.0))
        .ignore_radius_from_pillar(Length::new::<meter>(0.008))
        .fail_safe_voltage_threshold(ElectricPotential::new::<volt>(7.0))
        .search_velocity(Velocity::new::<meter_per_second>(0.3))
        .max_velocity(Velocity::new::<meter_per_second>(2.0))
        .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
        .max_jerk(Jerk::new::<meter_per_second_cubed>(50.0))
        .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(1440.0))
        .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
            14400.0,
        ))
        .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(57600.0))
        .run_slalom_velocity(Velocity::new::<meter_per_second>(0.6))
        .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
        .build()
        .expect("Should never panic");

    let wheel_radius = Length::new::<meter>(0.007);

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
                    x: Length::new::<meter>(0.013),
                    y: Length::new::<meter>(0.012),
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
                    x: Length::new::<meter>(0.013),
                    y: Length::new::<meter>(-0.012),
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
                    x: Length::new::<meter>(0.024),
                    y: Length::new::<meter>(0.0),
                    theta: Angle::new::<degree>(0.0),
                },
                1.0,
            )
        };

        let existence_threshold = Probability::new(0.1).unwrap();
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

    timer.listen(Event::TimeOut);
    free(|cs| {
        TIMER_TIM5.borrow(cs).replace(Some(timer));
    });

    let operator = Operators::Search(SearchOperator::construct(&config, &state, &mut resource));
    let administrator = Administrator::new(
        Selector,
        operator,
        OperatorStore::new(config),
        Mode::Search(1),
        InterruptManager,
    );

    Bag { administrator }
}
