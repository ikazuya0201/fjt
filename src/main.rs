#![no_std]
#![no_main]

extern crate panic_rtt;
mod alias;
mod macros;
mod sensors;

use core::cell::RefCell;
use core::f32::consts::PI;
use core::fmt::Write;
use core::ops::DerefMut;

use components::{
    data_types::{AbsoluteDirection, NodeId, Pattern, Pose, SearchNodeId},
    impls::{
        ControllerBuilder, EstimatorBuilder, MazeBuilder, ObstacleDetector, TrackerBuilder,
        TrajectoryGeneratorBuilder,
    },
    prelude::*,
};
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use embedded_hal::prelude::*;
use generic_array::arr;
use jlink_rtt::Output;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Frequency, Jerk,
    Speed, Time,
};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    i2c::I2c,
    interrupt,
    prelude::*,
    pwm::tim1,
    qei::Qei,
    spi::Spi,
    stm32,
    timer::{Event, Timer},
};

use alias::{
    Agent, DistanceSensors, Maze, MazeWidth, SearchOperator, SensorI2c, Solver, Voltmeter,
};
use sensors::{IMotor, ICM20648, MA702GQ, VL6180X};

static mut VOLTMETER: Option<RefCell<Voltmeter>> = None;
static mut I2C: Option<RefCell<SensorI2c>> = None;

static mut SOLVER: Option<Solver> = None;
static mut MAZE: Option<Maze> = None;
static mut AGENT: Option<Agent> = None;
static mut SEARCH_OPERATOR: Option<SearchOperator> = None;

static TIMER_TIM5: Mutex<RefCell<Option<Timer<stm32::TIM5>>>> = Mutex::new(RefCell::new(None));
static OUTPUT: Mutex<RefCell<Option<Output>>> = Mutex::new(RefCell::new(None));

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

#[interrupt]
fn TIM5() {
    // static mut COUNT: u32 = 0;

    free(|cs| {
        if let Some(ref mut tim5) = TIMER_TIM5.borrow(cs).borrow_mut().deref_mut() {
            tim5.clear_interrupt(Event::TimeOut);
        }
        unsafe {
            // *COUNT += 1;
            if let Some(ref search_operator) = SEARCH_OPERATOR.as_ref() {
                search_operator.tick();
            }
            // if *COUNT == 5000 {
            //     AGENT.as_ref().unwrap().stop();
            //     if let Some(ref mut out) = OUTPUT.borrow(cs).borrow_mut().deref_mut() {
            //         writeln!(out, "{:?}", MAZE.as_ref().unwrap());
            //     }
            // }
        }
    });
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
        RefCell::new(Voltmeter::new(adc, pa7, period, Frequency::from_hertz(1.0)))
    };

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

    let (voltmeter, i2c) = unsafe {
        VOLTMETER.replace(voltmeter);
        I2C.replace(i2c);
        (VOLTMETER.as_ref().unwrap(), I2C.as_ref().unwrap())
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
                    IMotor::new(pwm2, pwm1, voltmeter),
                    IMotor::new(pwm4, pwm3, voltmeter),
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
                .fail_safe_distance(Distance::from_meters(0.05))
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
                    &i2c,
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
                    &i2c,
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
                    &i2c,
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
        Agent::new(obstacle_detector, estimator, tracker, trajectory_generator)
    };

    let maze = MazeBuilder::new()
        .costs(costs as fn(Pattern) -> u16)
        .wall_existence_probability_threshold(0.3)
        .build::<MazeWidth>();

    let solver: Solver = Solver::new(
        NodeId::new(0, 0, AbsoluteDirection::North).unwrap(),
        arr![
            NodeId<MazeWidth>;
            NodeId::new(2,0,AbsoluteDirection::South).unwrap(),
            NodeId::new(2,0,AbsoluteDirection::West).unwrap()
        ],
    );

    let (agent, maze, solver) = unsafe {
        AGENT.replace(agent);
        MAZE.replace(maze);
        SOLVER.replace(solver);
        (
            AGENT.as_ref().unwrap(),
            MAZE.as_ref().unwrap(),
            SOLVER.as_ref().unwrap(),
        )
    };

    let search_operator: SearchOperator = SearchOperator::new(
        Pose::new(
            Distance::from_meters(0.045),
            Distance::from_meters(0.045),
            Angle::from_degree(90.0),
        ),
        SearchNodeId::new(0, 1, AbsoluteDirection::North).unwrap(),
        maze,
        agent,
        solver,
    );

    let search_operator = unsafe {
        SEARCH_OPERATOR.replace(search_operator);
        SEARCH_OPERATOR.as_ref().unwrap()
    };

    search_operator.init();
    search_operator.run().ok();

    timer.listen(Event::TimeOut);
    free(|cs| {
        TIMER_TIM5.borrow(cs).replace(Some(timer));
        OUTPUT.borrow(cs).replace(Some(out));
        cortex_m::peripheral::NVIC::unpend(interrupt::TIM5);
        unsafe {
            cortex_m::interrupt::enable();
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
        }
    });

    loop {
        search_operator.run().ok();
    }
    //stop interrupt
    // free(|cs| {
    //     cortex_m::peripheral::NVIC::mask(interrupt::TIM5);
    //     cortex_m::interrupt::disable();
    //     cortex_m::peripheral::NVIC::pend(interrupt::TIM5);
    // });
    // writeln!(out, "out").unwrap();
    // delay.delay_ms(5000u32);
    // writeln!(out, "{:?}", maze).unwrap();
    // panic!("finished!");

    // loop {
    //     free(|cs| {
    //         if let Some(ref mut tim5) = TIMER_TIM5.borrow(cs).borrow_mut().deref_mut() {
    //             nb::block!(tim5.wait());
    //         }
    //     })
    // }
}
