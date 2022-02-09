use core::{
    iter::Chain,
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
#[allow(unused_imports)]
use micromath::F32Ext;
use mousecore2::{
    control::{ControlParameters, Controller, Target, Tracker},
    solver::{AbsoluteDirection, Commander, Coordinate, RelativeDirection, SearchState, Searcher},
    state::{AngleState, Estimator, LengthState, SensorValue, State},
    trajectory::{
        slalom::{SlalomConfig, SlalomDirection, SlalomGenerator, SlalomKind, SlalomTrajectory},
        spin::{SpinGenerator, SpinTrajectory},
        straight::{StraightGenerator, StraightTrajectory},
        ShiftTrajectory, StopTrajectory,
    },
    wall::{Pose, WallDetector, Walls},
};
use sensors2::{encoder::MA702GQ, imu::ICM20648, motor::Motor, tof::VL6180X};
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    nb::block,
    prelude::*,
    pwm::tim1,
    qei::Qei,
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
    length::{meter, millimeter},
    time::second,
    velocity::meter_per_second,
};

use crate::types::{
    I2c, Imu, LeftEncoder, LeftMotor, PanicLed, RightEncoder, RightMotor, Spi, Tofs, Voltmeter,
};
use crate::TIMER_TIM5;

const W: u8 = 32;
const START_LENGTH: Length = Length {
    value: 0.02,
    units: PhantomData,
    dimension: PhantomData,
};
const SENSOR_STDDEV: Length = Length {
    value: 0.04,
    units: PhantomData,
    dimension: PhantomData,
};

pub static OPERATOR: Lazy<Mutex<Operator>> = Lazy::new(|| Mutex::new(Operator::new()));
pub static SOLVER: Lazy<Mutex<Solver>> = Lazy::new(|| Mutex::new(Solver::new()));
static WALLS: Lazy<Mutex<Walls<W>>> = Lazy::new(|| Mutex::new(Walls::new()));
static STATE: Lazy<Mutex<SearchState<W>>> = Lazy::new(|| {
    Mutex::new(
        SearchState::new(
            Coordinate::new(0, 0, true).unwrap(),
            AbsoluteDirection::North,
        )
        .unwrap(),
    )
});
static COMMANDER: Mutex<Option<Commander<W>>> = Mutex::new(None);
static IS_SEARCH_FINISH: AtomicBool = AtomicBool::new(false);

const FRONT_OFFSET: Length = Length {
    value: 0.01,
    dimension: PhantomData,
    units: PhantomData,
};
const SQUARE_WIDTH: Length = Length {
    value: 0.09,
    dimension: PhantomData,
    units: PhantomData,
};

type BackTrajectory = Chain<
    Chain<
        Chain<Chain<StraightTrajectory, StopTrajectory>, ShiftTrajectory<SpinTrajectory>>,
        StopTrajectory,
    >,
    ShiftTrajectory<StraightTrajectory>,
>;

#[derive(Clone)]
enum Trajectory {
    Straight(StraightTrajectory),
    Slalom(SlalomTrajectory),
    Back(BackTrajectory),
}

struct Linear {
    slope: f32,
    intercept: Length,
}

impl Default for Linear {
    fn default() -> Self {
        Self {
            slope: 1.0,
            intercept: Default::default(),
        }
    }
}

struct TofConfigs {
    front: (Pose, Linear),
    right: (Pose, Linear),
    left: (Pose, Linear),
}

impl Iterator for Trajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Trajectory::Straight(inner) => inner.next(),
            Trajectory::Slalom(inner) => inner.next(),
            Trajectory::Back(inner) => inner.next(),
        }
    }
}

pub struct Operator {
    tracker: Tracker,
    controller: Controller,
    estimator: Estimator,
    detector: WallDetector<W>,
    state: State,

    i2c: I2c,
    spi: Spi,
    imu: Imu,
    left_encoder: LeftEncoder,
    right_encoder: RightEncoder,
    left_motor: LeftMotor,
    right_motor: RightMotor,
    tofs: Tofs,
    tof_configs: TofConfigs,
    voltmeter: Voltmeter,

    trajectory: ShiftTrajectory<Trajectory>,
    next_trajectory: Option<ShiftTrajectory<Trajectory>>,
    front: Trajectory,
    right: Trajectory,
    left: Trajectory,
    back: Trajectory,
    fin: Option<Trajectory>,

    panic_led: PanicLed,
}

impl Operator {
    pub fn new() -> Self {
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

        let wheel_radius = Length::new::<meter>(0.007);
        let period = Time::new::<second>(0.001);

        let mut timer = Timer::tim5(device_peripherals.TIM5, 1000.hz(), clocks);

        let voltmeter = {
            let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
            let pa7 = gpioa.pa7.into_analog();
            Voltmeter::new(adc, pa7, period, Frequency::new::<hertz>(1.0))
        };

        if voltmeter.voltage() < ElectricPotential::new::<volt>(3.8) {
            panic!("Low voltage");
        }

        let mut i2c = {
            let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
            let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
            let i2c_pins = (scl, sda);
            I2c::new(device_peripherals.I2C1, i2c_pins, 400.khz(), clocks)
        };

        let tof_left = {
            let enable_pin = gpioh.ph1.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x31)
        };
        let mut tof_right = {
            let enable_pin = gpioa.pa15.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x30)
        };
        let tof_front = {
            let enable_pin = gpioh.ph0.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x29)
        };

        while block!(tof_right.distance(&mut i2c)).unwrap() >= START_LENGTH {}
        delay.delay_ms(1000u32);

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

        let spi_pins = (
            gpiob.pb3.into_alternate_af5(),
            gpiob.pb4.into_alternate_af5(),
            gpiob.pb5.into_alternate_af5(),
        );
        let mut spi = Spi::spi1(
            device_peripherals.SPI1,
            spi_pins,
            stm32f4xx_hal::hal::spi::MODE_3,
            1_562_500.hz(),
            clocks,
        );

        let imu = {
            let mut cs = gpioc.pc15.into_push_pull_output();
            cs.set_high().unwrap();

            ICM20648::new(&mut spi, cs, &mut delay, &mut timer)
        };
        spi = spi.init(
            stm32f4xx_hal::hal::spi::MODE_3,
            6_250_000.hz(),
            clocks.pclk2(),
        );

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
            (Motor::new(pwm2, pwm1), Motor::new(pwm4, pwm3))
        };

        // common settings
        let state = State {
            x: LengthState {
                x: Length::new::<millimeter>(45.0),
                ..Default::default()
            },
            y: LengthState {
                x: Length::new::<millimeter>(45.0),
                ..Default::default()
            },
            theta: AngleState {
                x: Angle::new::<degree>(90.0),
                ..Default::default()
            },
        };

        let state = state;
        let tracker = Tracker::builder()
            .period(period)
            .gain(120.0)
            .dgain(8.0)
            .zeta(1.0)
            .b(1.0)
            .xi_threshold(Velocity::new::<meter_per_second>(0.2))
            .build();
        let controller = Controller::builder()
            .trans_params(ControlParameters {
                kp: 4.8497,
                ki: 29.5783,
                kd: 0.0,
                model_k: 1.865,
                model_t1: 0.4443,
            })
            .rot_params(ControlParameters {
                kp: 0.21134,
                ki: 2.9317,
                kd: 0.0,
                model_k: 82.39,
                model_t1: 0.2855,
            })
            .period(period)
            .build();
        let estimator = Estimator::builder().period(period).build();
        let detector = WallDetector::<W>::default();

        let (init, fin, front, right, left, back) = {
            let search_velocity = Velocity::new::<meter_per_second>(0.3);
            let v_max = Velocity::new::<meter_per_second>(2.0);
            let a_max = Acceleration::new::<meter_per_second_squared>(10.0);
            let j_max = Jerk::new::<meter_per_second_cubed>(50.0);

            let square_width = SQUARE_WIDTH;
            let front_offset = FRONT_OFFSET;

            let slalom_config = SlalomConfig::new(square_width, front_offset);
            let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
            let spin = SpinGenerator::new(
                AngularVelocity::new::<degree_per_second>(1440.0),
                AngularAcceleration::new::<degree_per_second_squared>(14400.0),
                AngularJerk::new::<degree_per_second_cubed>(57600.0),
                period,
            );
            let straight = StraightGenerator::new(v_max, a_max, j_max, period);
            (
                Trajectory::Straight(straight.generate(
                    square_width / 2.0 + front_offset,
                    Default::default(),
                    search_velocity,
                )),
                Trajectory::Straight(straight.generate(
                    square_width / 2.0 - front_offset,
                    search_velocity,
                    Default::default(),
                )),
                Trajectory::Straight(StraightGenerator::generate_constant(
                    square_width,
                    search_velocity,
                    period,
                )),
                Trajectory::Slalom(slalom.generate_constant_slalom(
                    slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Right),
                    search_velocity,
                )),
                Trajectory::Slalom(slalom.generate_constant_slalom(
                    slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Left),
                    search_velocity,
                )),
                Trajectory::Back(
                    straight
                        .generate(
                            square_width / 2.0 - front_offset,
                            search_velocity,
                            Default::default(),
                        )
                        .chain(StopTrajectory::new(
                            Pose {
                                x: square_width / 2.0 - front_offset,
                                ..Default::default()
                            },
                            period,
                            Time::new::<second>(0.1),
                        ))
                        .chain(ShiftTrajectory::new(
                            Pose {
                                x: square_width / 2.0 - front_offset,
                                ..Default::default()
                            },
                            spin.generate(Angle::new::<degree>(180.0)),
                        ))
                        .chain(StopTrajectory::new(
                            Pose {
                                x: square_width / 2.0 - front_offset,
                                y: Default::default(),
                                theta: Angle::new::<degree>(180.0),
                            },
                            period,
                            Time::new::<second>(0.1),
                        ))
                        .chain(ShiftTrajectory::new(
                            Pose {
                                x: square_width / 2.0 - front_offset,
                                y: Default::default(),
                                theta: Angle::new::<degree>(180.0),
                            },
                            straight.generate(
                                square_width / 2.0 + front_offset,
                                Default::default(),
                                search_velocity,
                            ),
                        )),
                ),
            )
        };

        let trajectory: ShiftTrajectory<Trajectory> = ShiftTrajectory::new(
            Pose {
                x: state.x.x,
                y: state.y.x,
                theta: state.theta.x,
            },
            init,
        );

        timer.listen(Event::TimeOut);
        free(|cs| {
            TIMER_TIM5.borrow(cs).replace(Some(timer));
        });

        Operator {
            tracker,
            detector,
            estimator,
            controller,
            state,

            i2c,
            spi,
            imu,
            voltmeter,
            left_encoder,
            right_encoder,
            left_motor,
            right_motor,
            tofs: Tofs {
                front: tof_front,
                right: tof_right,
                left: tof_left,
            },
            tof_configs: TofConfigs {
                left: (
                    Pose {
                        x: Length::new::<meter>(0.013),
                        y: Length::new::<meter>(0.013),
                        theta: Angle::new::<degree>(90.0),
                    },
                    Linear {
                        slope: 0.983_606_6,
                        intercept: Length::new::<millimeter>(-4.426_23),
                    },
                ),
                right: (
                    Pose {
                        x: Length::new::<meter>(0.013),
                        y: Length::new::<meter>(-0.013),
                        theta: Angle::new::<degree>(-90.0),
                    },
                    Linear {
                        slope: 0.833_333_3,
                        intercept: Length::new::<millimeter>(1.388_889),
                    },
                ),
                front: (
                    Pose {
                        x: Length::new::<meter>(0.024),
                        y: Length::new::<meter>(0.0),
                        theta: Angle::new::<degree>(0.0),
                    },
                    Linear {
                        slope: 1.086_076,
                        intercept: Length::new::<millimeter>(-15.344_74),
                    },
                ),
            },

            trajectory,
            next_trajectory: None,
            front,
            right,
            left,
            back,
            fin: Some(fin),

            panic_led: gpiob.pb1.into_push_pull_output(),
        }
    }

    pub fn control(&mut self) {
        // estimate
        let sensor_value = {
            SensorValue {
                left_distance: block!(self.left_encoder.distance()).unwrap(),
                right_distance: block!(self.right_encoder.distance()).unwrap(),
                translational_acceleration: block!(self
                    .imu
                    .translational_acceleration(&mut self.spi))
                .unwrap(),
                angular_velocity: block!(self.imu.angular_velocity(&mut self.spi)).unwrap(),
            }
        };
        self.estimator.estimate(&mut self.state, &sensor_value);

        // track
        let target = if let Some(target) = self.trajectory.next() {
            target
        } else if let Some(next) = self.next_trajectory.take() {
            self.trajectory = next;
            self.trajectory.next().unwrap()
        } else if IS_SEARCH_FINISH.load(Ordering::SeqCst) {
            if let Some(fin) = self.fin.take() {
                let pose =
                    Pose::from_search_state::<W>(STATE.lock().clone(), SQUARE_WIDTH, FRONT_OFFSET);
                self.trajectory = ShiftTrajectory::new(pose, fin);
                self.trajectory.next().unwrap()
            } else {
                unreachable!()
            }
        } else {
            unreachable!()
        };
        let (control_target, control_state) = self.tracker.track(&self.state, &target);
        let vol = self.controller.control(&control_target, &control_state);

        const THRES: ElectricPotential = ElectricPotential {
            value: 7.0,
            dimension: PhantomData,
            units: PhantomData,
        };

        // fail safe
        if THRES < vol.left.abs() || THRES < vol.right.abs() {
            panic!("fail safe!");
        }

        self.left_motor.apply(vol.left, self.voltmeter.voltage());
        self.right_motor.apply(vol.right, self.voltmeter.voltage());

        // detect wall
        let cos_th = self.state.theta.x.value.cos();
        let sin_th = self.state.theta.x.value.sin();
        macro_rules! detect {
            ($pos: ident) => {
                if let Ok(distance) = self.tofs.$pos.distance(&mut self.i2c) {
                    let distance = self.tof_configs.$pos.1.slope * distance
                        + self.tof_configs.$pos.1.intercept;
                    let pose = Pose {
                        x: self.state.x.x + self.tof_configs.$pos.0.x * cos_th
                            - self.tof_configs.$pos.0.y * sin_th,
                        y: self.state.y.x
                            + self.tof_configs.$pos.0.x * sin_th
                            + self.tof_configs.$pos.0.y * cos_th,
                        theta: self.state.theta.x + self.tof_configs.$pos.0.theta,
                    };

                    let mut walls = WALLS.lock();
                    if let Some((coord, wall_state)) =
                        self.detector
                            .detect_and_update(&distance, &SENSOR_STDDEV, &pose)
                    {
                        walls.update(&coord, &wall_state);
                    }
                }
            };
        }
        detect!(front);
        detect!(right);
        detect!(left);

        if self.next_trajectory.is_some() {
            return;
        }

        match (COMMANDER.try_lock(), WALLS.try_lock(), STATE.try_lock()) {
            (Some(mut commander), Some(walls), Some(mut state)) => {
                match commander
                    .as_ref()
                    .map(|commander| commander.next_coordinate(|coord| walls.wall_state(coord)))
                {
                    Some(Ok(Some(next_coord))) => {
                        let pose =
                            Pose::from_search_state::<W>(state.clone(), SQUARE_WIDTH, FRONT_OFFSET);
                        let dir = state.update(&next_coord).unwrap();
                        macro_rules! gen {
                            ($traj: expr) => {
                                ShiftTrajectory::new(pose, $traj.clone())
                            };
                        }
                        self.next_trajectory = Some(match dir {
                            RelativeDirection::Front => gen!(self.front),
                            RelativeDirection::Right => gen!(self.right),
                            RelativeDirection::Left => gen!(self.left),
                            RelativeDirection::Back => gen!(self.back),
                        });
                        commander.take();
                    }
                    Some(Ok(None)) => (),
                    Some(Err(err)) => unreachable!("{:?}", err),
                    _ => (),
                }
            }
            _ => (),
        }
    }

    pub fn stop(&mut self) {
        self.left_motor
            .apply(Default::default(), self.voltmeter.voltage());
        self.right_motor
            .apply(Default::default(), self.voltmeter.voltage());
    }

    pub fn turn_on_panic_led(&mut self) {
        self.panic_led.set_high().unwrap();
    }
}

pub struct Solver {
    searcher: Searcher<W>,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            searcher: Searcher::new(
                Coordinate::new(0, 0, true).unwrap(),
                &[
                    Coordinate::new(1, 0, true).unwrap(),
                    Coordinate::new(1, 0, false).unwrap(),
                ],
            ),
        }
    }

    pub fn search(&mut self) {
        // search
        if IS_SEARCH_FINISH.load(Ordering::SeqCst) {
            return;
        }
        {
            let lock = COMMANDER.lock();
            if lock.is_some() {
                return;
            }
            core::mem::drop(lock);
        }

        let walls = {
            let lock = WALLS.lock();
            let walls = lock.clone();
            core::mem::drop(lock);
            walls
        };
        let coord = {
            let lock = STATE.lock();
            let coord = lock.coordinate().clone();
            core::mem::drop(lock);
            coord
        };
        match self
            .searcher
            .search(&coord, |coord| walls.wall_state(coord))
        {
            Ok(Some(next)) => {
                COMMANDER.lock().replace(next);
            }
            Ok(None) => {
                IS_SEARCH_FINISH.store(true, Ordering::SeqCst);
            }
            Err(err) => unreachable!("{:?}", err),
        }
    }
}
