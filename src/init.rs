use core::{
    iter::Chain,
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
use heapless::{Deque, Vec};
#[allow(unused_imports)]
use micromath::F32Ext;
use mousecore2::{
    control::{ControlParameters, Controller, Target, Tracker},
    estimate::{AngleState, Estimator, LengthState, SensorValue, State},
    solve::{
        run::{
            shortest_path, EdgeKind, Node, Posture as RunPosture,
            TrajectoryKind as RunTrajectoryKind,
        },
        search::{
            Commander, Coordinate, Posture as SearchPosture, SearchState, Searcher,
            TrajectoryKind as SearchTrajectoryKind, WallState,
        },
    },
    trajectory::{
        slalom::{SlalomConfig, SlalomDirection, SlalomGenerator, SlalomKind, SlalomTrajectory},
        spin::{SpinGenerator, SpinTrajectory},
        straight::{StraightGenerator, StraightTrajectory},
        ShiftTrajectory, StopTrajectory,
    },
    wall::{Pose, PoseConverter, WallDetector, Walls},
};
use sensors2::{encoder::MA702GQ, imu::ICM20648, motor::Motor, tof::VL6180X};
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    interrupt,
    nb::block,
    prelude::*,
    pwm::tim1,
    qei::Qei,
    stm32,
    timer::{Event, Timer},
};
use typed_builder::TypedBuilder;
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
    ratio::ratio,
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
const PATH_MAX: usize = 32 * 32;

pub static OPERATOR: Lazy<Mutex<Operator>> = Lazy::new(|| Mutex::new(Operator::new()));
pub static SOLVER: Lazy<Mutex<Solver>> = Lazy::new(|| Mutex::new(Solver::new()));
static WALLS: Lazy<Mutex<Walls<W>>> = Lazy::new(|| Mutex::new(Walls::new()));
static STATE: Lazy<Mutex<SearchState<W>>> = Lazy::new(|| {
    Mutex::new(SearchState::new(Coordinate::new(0, 1).unwrap(), SearchPosture::North).unwrap())
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

pub fn tick_on() {
    free(|_cs| {
        cortex_m::peripheral::NVIC::unpend(interrupt::TIM5);
        unsafe {
            cortex_m::interrupt::enable();
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
        }
    });
}

pub fn tick_off() {
    free(|_cs| {
        cortex_m::interrupt::disable();
        cortex_m::peripheral::NVIC::mask(interrupt::TIM5);
        cortex_m::peripheral::NVIC::pend(interrupt::TIM5);
    });
}

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
    Stop(StopTrajectory),
    Spin(SpinTrajectory),
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
        use Trajectory::*;
        match self {
            Straight(inner) => inner.next(),
            Slalom(inner) => inner.next(),
            Back(inner) => inner.next(),
            Spin(inner) => inner.next(),
            Stop(inner) => inner.next(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    Search,
    Return,
}

pub struct Operator {
    tracker: Tracker,
    controller: Controller,
    estimator: Estimator,
    detector: WallDetector<W>,
    state: State,

    stddev: Length,
    converter: PoseConverter<W>,

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

    manager: TrajectoryManager,
    mode: Mode,

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

        let search_manager = TrajectoryConfig::builder()
            .search_velocity(Velocity::new::<meter_per_second>(0.3))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(0.5))
            .v_max(Velocity::new::<meter_per_second>(2.0))
            .a_max(Acceleration::new::<meter_per_second_squared>(10.0))
            .j_max(Jerk::new::<meter_per_second_cubed>(50.0))
            .spin_v_max(AngularVelocity::new::<degree_per_second>(1440.0))
            .spin_a_max(AngularAcceleration::new::<degree_per_second_squared>(
                14400.0,
            ))
            .spin_j_max(AngularJerk::new::<degree_per_second_cubed>(57600.0))
            .period(period)
            .square_width(SQUARE_WIDTH)
            .front_offset(FRONT_OFFSET)
            .initial_pose(Pose {
                x: state.x.x,
                y: state.y.x,
                theta: state.theta.x,
            })
            .build()
            .into();

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

            stddev: Length::default(),
            converter: PoseConverter::default(),

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

            manager: search_manager,
            mode: Mode::Search,

            panic_led: gpiob.pb1.into_push_pull_output(),
        }
    }

    pub fn control(&mut self) {
        match self.mode {
            Mode::Search => self.control_search(),
            Mode::Return => self.control_return(),
        }
    }

    fn control_return(&mut self) {
        self.estimate();
        let target = self.manager.target().unwrap();
        self.track(&target);
    }

    fn estimate(&mut self) {
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
        self.stddev += Length::new::<millimeter>(0.005);
    }

    fn track(&mut self, target: &Target) {
        let (control_target, control_state) = self.tracker.track(&self.state, target);
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
    }

    fn control_search(&mut self) {
        self.estimate();

        if IS_SEARCH_FINISH.load(Ordering::SeqCst) && self.manager.next_is_none() {
            self.manager.set_final(Pose::from_search_state(
                STATE.lock().clone(),
                SQUARE_WIDTH,
                FRONT_OFFSET,
            ));
        }
        let target = if let Some(target) = self.manager.target() {
            target
        } else {
            self.stop();
            tick_off();
            let rev_start = Node::new(0, 0, RunPosture::South).unwrap();
            let state = STATE.lock();
            let x = state.x();
            let y = state.y();
            let (x, y, pos) = match state.posture() {
                SearchPosture::North => (x, y + 1, RunPosture::North),
                SearchPosture::East => (x + 1, y, RunPosture::East),
                SearchPosture::South => (x, y - 1, RunPosture::South),
                SearchPosture::West => (x - 1, y, RunPosture::West),
            };
            self.init_run(Node::new(x, y, pos).unwrap(), |node| &rev_start == node);
            self.mode = Mode::Return;
            tick_on();
            return;
        };
        self.track(&target);

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
                        let info = self.converter.convert(&pose).unwrap();
                        walls.update(&coord, &wall_state);
                        if matches!(
                            walls.wall_state(&info.coord),
                            WallState::Checked { exists: true }
                        ) {
                            let sensor_var = SENSOR_STDDEV * SENSOR_STDDEV;
                            let var = self.stddev * self.stddev;
                            let k = (var / (var + sensor_var)).get::<ratio>();

                            let distance_diff = k * (info.existing_distance - distance);
                            let cos_th = pose.theta.value.cos();
                            let sin_th = pose.theta.value.sin();
                            self.state.x.x += distance_diff * cos_th;
                            self.state.y.x += distance_diff * sin_th;
                            self.stddev = Length::new::<meter>((var * (1.0 - k)).value.sqrt());
                        }
                    }
                }
            };
        }
        detect!(front);
        detect!(right);
        detect!(left);

        if !self.manager.next_is_none() {
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
                        self.manager.set(pose, dir);
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

    fn init_run(&mut self, start: Node<W>, is_goal: impl Fn(&Node<W>) -> bool + Copy) {
        use RunPosture::*;

        let (_, path, npos) = IntoIterator::into_iter([North, East, South, West])
            .filter_map(|pos| {
                shortest_path(
                    Node::new(start.x(), start.y(), pos).unwrap(),
                    is_goal,
                    |coord| {
                        !matches!(
                            WALLS.lock().wall_state(coord),
                            WallState::Checked { exists: false }
                        )
                    },
                    |kind| match kind {
                        EdgeKind::Straight(x) => *x as u16 * 10,
                        EdgeKind::StraightDiagonal(x) => *x as u16 * 7,
                        EdgeKind::Slalom45 => 12,
                        EdgeKind::Slalom90 => 15,
                        EdgeKind::Slalom135 => 20,
                        EdgeKind::Slalom180 => 25,
                        EdgeKind::SlalomDiagonal90 => 15,
                    },
                )
                .map(|(path, cost)| (cost, path, pos))
            })
            .min_by_key(|v| v.0)
            .unwrap();

        let pos_to_u8 = |pos| {
            use RunPosture::*;
            match pos {
                North => 0u8,
                East => 1,
                South => 2,
                West => 3,
                _ => unreachable!(),
            }
        };

        let pos = pos_to_u8(start.posture());
        let npos = pos_to_u8(npos);

        let init_angle = Angle::new::<degree>(match (4 + npos - pos) % 4 {
            0 => 0.0,
            1 => -90.0,
            2 => 180.0,
            3 => 90.0,
            _ => unreachable!(),
        });
        self.manager.init_run(path, init_angle);
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
                Coordinate::new(0, 1).unwrap(),
                &[
                    Coordinate::new(2, 1).unwrap(),
                    Coordinate::new(3, 0).unwrap(),
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

#[derive(TypedBuilder)]
struct TrajectoryConfig {
    square_width: Length,
    front_offset: Length,
    v_max: Velocity,
    a_max: Acceleration,
    j_max: Jerk,
    spin_v_max: AngularVelocity,
    spin_a_max: AngularAcceleration,
    spin_j_max: AngularJerk,
    search_velocity: Velocity,
    period: Time,
    initial_pose: Pose,
    run_slalom_velocity: Velocity,
}

struct TrajectoryManager {
    trajectories: Deque<ShiftTrajectory<Trajectory>, 3>,
    fin: Option<Trajectory>,
    front: Trajectory,
    right: Trajectory,
    left: Trajectory,
    back: Trajectory,

    square_width: Length,
    period: Time,
    run_slalom_velocity: Velocity,

    straight: StraightGenerator,
    spin: SpinGenerator,
    slalom: SlalomGenerator,
    slalom_config: SlalomConfig,

    run_iter: usize,
    cur_velocity: Velocity,
    is_run: bool,
    path: Vec<Node<W>, PATH_MAX>,
}

impl TrajectoryManager {
    fn target(&mut self) -> Option<Target> {
        if self.is_run && !self.trajectories.is_full() && self.run_iter < self.path.len() - 1 {
            use RunTrajectoryKind::*;
            let i = self.run_iter;
            let velocity = self.cur_velocity;

            let pose = Pose::from_node(self.path[i], self.square_width);
            let kind = self.path[i]
                .trajectory_kind(&self.path[i + 1])
                .unwrap_or_else(|| unreachable!("{:?}", (self.path[i], self.path[i + 1])));
            let (start, middle, end) = if i <= 1 {
                (velocity, self.run_slalom_velocity, self.run_slalom_velocity)
            } else if i + 2 == self.path.len() {
                (velocity, velocity * 0.666_666_7, Default::default())
            } else if i + 3 == self.path.len() {
                (velocity, velocity * 0.5, velocity * 0.5)
            } else {
                (
                    self.run_slalom_velocity,
                    self.run_slalom_velocity,
                    self.run_slalom_velocity,
                )
            };
            let (trajectory, terminal_velocity) = match kind {
                Straight(x) => {
                    let (trajectory, terminal_velocity) = self
                        .straight
                        .generate_with_terminal_velocity(x as f32 * self.square_width, start, end);
                    (
                        ShiftTrajectory::new(pose, Trajectory::Straight(trajectory)),
                        terminal_velocity,
                    )
                }
                StraightDiagonal(x) => {
                    let (trajectory, terminal_velocity) =
                        self.straight.generate_with_terminal_velocity(
                            x as f32 * self.square_width / 2.0f32.sqrt(),
                            start,
                            end,
                        );
                    (
                        ShiftTrajectory::new(pose, Trajectory::Straight(trajectory)),
                        terminal_velocity,
                    )
                }
                Slalom(kind, dir) => {
                    let (trajectory, terminal_velocity) =
                        self.slalom.generate_slalom_with_terminal_velocity(
                            self.slalom_config.parameters(kind, dir),
                            start,
                            middle,
                            end,
                        );
                    (
                        ShiftTrajectory::new(pose, Trajectory::Slalom(trajectory)),
                        terminal_velocity,
                    )
                }
            };
            self.cur_velocity = terminal_velocity;
            self.trajectories.push_back(trajectory).ok();
            self.run_iter += 1;
        }

        let front = self.trajectories.front_mut()?;
        if let Some(target) = front.next() {
            Some(target)
        } else {
            core::mem::drop(front);
            self.trajectories.pop_front();
            self.trajectories.front_mut()?.next()
        }
    }

    fn set_final(&mut self, pose: Pose) {
        if let Some(fin) = self.fin.take() {
            self.trajectories
                .push_back(ShiftTrajectory::new(pose, fin))
                .ok();
        }
    }

    fn next_is_none(&self) -> bool {
        self.trajectories.len() < 2
    }

    fn set(&mut self, pose: Pose, kind: SearchTrajectoryKind) {
        use SearchTrajectoryKind::*;

        self.trajectories
            .push_back(ShiftTrajectory::new(
                pose,
                match kind {
                    Front => self.front.clone(),
                    Right => self.right.clone(),
                    Left => self.left.clone(),
                    Back => self.back.clone(),
                },
            ))
            .ok();
    }

    fn init_run(&mut self, path: Vec<Node<W>, PATH_MAX>, init_angle: Angle) {
        let init_pose = {
            let mut pose = Pose::from_node(path[0], self.square_width);
            pose.theta -= init_angle;
            pose
        };
        self.trajectories
            .push_back(ShiftTrajectory::new(
                init_pose,
                if init_angle == Angle::default() {
                    Trajectory::Stop(StopTrajectory::new(
                        Default::default(),
                        self.period,
                        Time::new::<second>(0.5),
                    ))
                } else {
                    Trajectory::Spin(self.spin.generate(init_angle))
                },
            ))
            .ok();
        self.run_iter = 0;
        self.path = path;
        self.is_run = true;
        self.cur_velocity = Default::default();
    }
}

impl From<TrajectoryConfig> for TrajectoryManager {
    fn from(
        TrajectoryConfig {
            square_width,
            front_offset,
            v_max,
            a_max,
            j_max,
            spin_v_max,
            spin_a_max,
            spin_j_max,
            search_velocity,
            period,
            initial_pose,
            run_slalom_velocity,
        }: TrajectoryConfig,
    ) -> Self {
        let slalom_config = SlalomConfig::new(square_width, front_offset);
        let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
        let spin = SpinGenerator::new(spin_v_max, spin_a_max, spin_j_max, period);
        let straight = StraightGenerator::new(v_max, a_max, j_max, period);

        let init = Trajectory::Straight(straight.generate(
            square_width / 2.0 + front_offset,
            Default::default(),
            search_velocity,
        ));
        let fin = Trajectory::Straight(straight.generate(
            square_width / 2.0 - front_offset,
            search_velocity,
            Default::default(),
        ));
        let front = Trajectory::Straight(StraightGenerator::generate_constant(
            square_width,
            search_velocity,
            period,
        ));
        let right = Trajectory::Slalom(slalom.generate_constant_slalom(
            slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Right),
            search_velocity,
        ));
        let left = Trajectory::Slalom(slalom.generate_constant_slalom(
            slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Left),
            search_velocity,
        ));
        let back = Trajectory::Back(
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
        );
        let mut trajectories = Deque::new();
        trajectories
            .push_back(ShiftTrajectory::new(initial_pose, init))
            .ok();
        Self {
            trajectories,
            fin: Some(fin),
            front,
            left,
            right,
            back,

            square_width,
            period,
            run_slalom_velocity,

            straight,
            spin,
            slalom,
            slalom_config,

            run_iter: 0,
            path: Vec::new(),
            cur_velocity: Default::default(),
            is_run: false,
        }
    }
}
