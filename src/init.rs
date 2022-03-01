mod trajectory;
mod types;

use core::{
    fmt,
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m::interrupt::free;
use heapless::Vec;
#[allow(unused_imports)]
use micromath::F32Ext;
use mousecore2::{
    control::{
        ControlParameters, Controller, NavigationController, SupervisoryController, Target, Tracker,
    },
    estimate::{AngleState, Estimator, LengthState, SensorValue, State},
    solve::{
        run::{shortest_path, EdgeKind, Node, Posture as RunPosture},
        search::{
            Commander, Coordinate, Posture as SearchPosture, SearchState, Searcher, WallState,
        },
    },
    wall::{Pose, PoseConverter, WallDetector, Walls},
};
use sensors2::{encoder::MA702GQ, imu::ICM20648, motor::Motor, tof::VL6180X};
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    interrupt,
    nb::block,
    pac,
    prelude::*,
    qei::Qei,
    timer::{Event, SysDelay},
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::degree_per_second_squared,
    angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second,
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

use trajectory::{TrajectoryConfig, TrajectoryManager};
use types::{
    ControlTimer, I2c, Imu, LeftEncoder, LeftMotor, PanicLed, RightEncoder, RightMotor,
    SensorTimer, Spi, Tofs, Voltmeter,
};

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
const WHEEL_RADIUS: Length = Length {
    value: 0.007,
    units: PhantomData,
    dimension: PhantomData,
};
const PATH_MAX: usize = 32 * 32;
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

pub static BAG: Lazy<Bag> = Lazy::new(|| Bag::new());

pub fn tick_on() {
    use cortex_m::peripheral::NVIC;
    use interrupt::{TIM5, TIM7};

    free(|_cs| {
        NVIC::unpend(TIM5);
        NVIC::unpend(TIM7);
        unsafe {
            cortex_m::interrupt::enable();
            NVIC::unmask(TIM5);
            NVIC::unmask(TIM7);
        }
    });
}

pub fn tick_off() {
    use cortex_m::peripheral::NVIC;
    use interrupt::{TIM5, TIM7};

    free(|_cs| {
        cortex_m::interrupt::disable();
        NVIC::mask(TIM5);
        NVIC::pend(TIM5);
        NVIC::mask(TIM7);
        NVIC::pend(TIM7);
    });
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

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum TofPosition {
    Front,
    Right,
    Left,
}

struct TofConfigs {
    front: (Pose, Linear),
    right: (Pose, Linear),
    left: (Pose, Linear),
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    Idle,
    Search,
    Return,
    Run,
}

pub struct Bag {
    pub operator: Mutex<Operator>,
    pub pollar: Mutex<Pollar>,
    pub solver: Mutex<Solver>,
    pub walls: Mutex<Walls<W>>,
    state: Mutex<SearchState<W>>,
    pub commander: Mutex<Option<Commander<W>>>,
    is_search_finish: AtomicBool,
    tof_queue: Mutex<Vec<(Length, TofPosition), 3>>,
}

impl Bag {
    pub fn new() -> Self {
        let cortex_m::Peripherals { SYST, mut NVIC, .. } = cortex_m::Peripherals::take().unwrap();
        let pac::Peripherals {
            RCC,
            GPIOA,
            GPIOB,
            GPIOC,
            GPIOH,
            ADC1,
            I2C1,
            SPI1,
            TIM1,
            TIM2,
            TIM4,
            TIM5,
            TIM7,
            ..
        } = pac::Peripherals::take().unwrap();
        let rcc = RCC.constrain();

        let clocks = rcc
            .cfgr
            .hclk(100_000_000.Hz())
            .sysclk(100_000_000.Hz())
            .pclk1(50_000_000.Hz())
            .pclk2(100_000_000.Hz())
            .freeze();
        let mut delay = SYST.delay(&clocks);
        let gpioa = GPIOA.split();
        let gpiob = GPIOB.split();
        let gpioc = GPIOC.split();
        let gpioh = GPIOH.split();

        let period = Time::new::<second>(0.001);

        let mut i2c = {
            let scl = gpiob.pb8.into_alternate().set_open_drain();
            let sda = gpiob.pb9.into_alternate().set_open_drain();
            let i2c_pins = (scl, sda);
            I2c::new(I2C1, i2c_pins, 400.kHz(), &clocks)
        };

        let tof_left = {
            let enable_pin = gpioh.ph1.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x31)
        };
        let tof_right = {
            let enable_pin = gpioa.pa15.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x30)
        };
        let tof_front = {
            let enable_pin = gpioh.ph0.into_push_pull_output();
            VL6180X::new(&mut i2c, enable_pin, &mut delay, 0x29)
        };

        let mut control_timer = TIM5.counter::<1_000_000>(&clocks);
        control_timer.start(1.millis()).unwrap();

        let voltmeter = {
            let adc = Adc::adc1(ADC1, true, AdcConfig::default());
            let pa7 = gpioa.pa7.into_analog();
            Voltmeter::new(adc, pa7, period, Frequency::new::<hertz>(1.0), 1.86)
        };

        let right_encoder = {
            let pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
            let qei = Qei::new(TIM2, pins);
            let encoder = MA702GQ::new(qei);
            encoder
        };

        let left_encoder = {
            let pins = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate());
            let qei = Qei::new(TIM4, pins);
            let encoder = MA702GQ::new(qei);
            encoder
        };

        let spi_pins = (
            gpiob.pb3.into_alternate(),
            gpiob.pb4.into_alternate(),
            gpiob.pb5.into_alternate(),
        );
        let mut spi = SPI1.spi(
            spi_pins,
            stm32f4xx_hal::hal::spi::MODE_3,
            1_562_500.Hz(),
            &clocks,
        );

        let imu = {
            let mut cs = gpioc.pc15.into_push_pull_output();
            cs.set_high();

            ICM20648::new(&mut spi, cs, &mut delay, &mut control_timer)
        };
        let (spi, spi_pins) = spi.release();
        let spi = spi.spi(
            spi_pins,
            stm32f4xx_hal::hal::spi::MODE_3,
            6_250_000.Hz(),
            &clocks,
        );

        let (left_motor, right_motor) = {
            let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = {
                let pins = (
                    gpioa.pa8.into_alternate(),
                    gpioa.pa9.into_alternate(),
                    gpioa.pa10.into_alternate(),
                    gpioa.pa11.into_alternate(),
                );

                TIM1.pwm_hz(pins, 100.kHz(), &clocks).split()
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
            .zeta(1.0)
            .b(1.0)
            .xi_threshold(Velocity::new::<meter_per_second>(0.2))
            .build();
        let navigator = NavigationController::builder()
            .gain(120.0)
            .dgain(30.0)
            .build();
        let supervisor = SupervisoryController::builder()
            .avoidance_distance(Length::new::<millimeter>(25.0))
            .margin(60.0)
            .square_width(SQUARE_WIDTH)
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
        let estimator = Estimator::builder()
            .period(period)
            .slip_angle_const(Acceleration::new::<meter_per_second_squared>(35.0))
            .build();
        let detector = WallDetector::<W>::default();

        let search_manager = TrajectoryConfig::builder()
            .search_velocity(Velocity::new::<meter_per_second>(0.15))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(0.15))
            .v_max(Velocity::new::<meter_per_second>(1.0))
            .a_max(Acceleration::new::<meter_per_second_squared>(1.0))
            .j_max(Jerk::new::<meter_per_second_cubed>(20.0))
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

        let mut sensor_timer = TIM7.counter::<1_000_000>(&clocks);
        sensor_timer.start(5.millis()).unwrap();

        control_timer.listen(Event::Update);
        sensor_timer.listen(Event::Update);

        unsafe {
            NVIC.set_priority(interrupt::TIM5, 0);
            NVIC.set_priority(interrupt::TIM7, 1);
        }

        Bag {
            operator: Mutex::new(Operator {
                tracker,
                navigator,
                supervisor,
                detector,
                estimator,
                controller,
                state,

                stddev: Length::default(),
                converter: PoseConverter::default(),

                spi,
                imu,
                voltmeter,
                left_encoder,
                right_encoder,
                left_motor,
                right_motor,
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
                mode: Mode::Idle,

                panic_led: gpiob.pb1.into_push_pull_output(),
                control_timer,
                delay,
            }),
            pollar: Mutex::new(Pollar {
                i2c,
                tofs: Tofs {
                    front: tof_front,
                    right: tof_right,
                    left: tof_left,
                },
                timer: sensor_timer,
            }),
            solver: Mutex::new(Solver::new()),
            walls: Mutex::new(Walls::new()),
            state: Mutex::new(
                SearchState::new(Coordinate::new(0, 1).unwrap(), SearchPosture::North).unwrap(),
            ),
            commander: Mutex::new(None),
            is_search_finish: AtomicBool::new(false),
            tof_queue: Mutex::new(Vec::new()),
        }
    }
}

pub struct Operator {
    tracker: Tracker,
    navigator: NavigationController,
    #[allow(unused)]
    supervisor: SupervisoryController,
    controller: Controller,
    estimator: Estimator,
    detector: WallDetector<W>,
    state: State,

    stddev: Length,
    #[allow(unused)]
    converter: PoseConverter<W>,

    spi: Spi,
    imu: Imu,
    left_encoder: LeftEncoder,
    right_encoder: RightEncoder,
    left_motor: LeftMotor,
    right_motor: RightMotor,
    tof_configs: TofConfigs,
    voltmeter: Voltmeter,

    manager: TrajectoryManager,
    mode: Mode,

    panic_led: PanicLed,
    control_timer: ControlTimer,
    delay: SysDelay,
}

impl fmt::Debug for Operator {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Operator")
            .field("manager", &self.manager)
            .finish()
    }
}

impl Operator {
    pub fn battery_voltage(&mut self) -> ElectricPotential {
        self.voltmeter.voltage()
    }

    pub fn control(&mut self) {
        match self.mode {
            Mode::Idle => self.control_idle(),
            Mode::Search => self.control_search(),
            Mode::Return => self.control_return(),
            Mode::Run => self.control_run(),
        }
    }

    fn control_idle(&mut self) {
        if let Some(mut que) = BAG.tof_queue.try_lock() {
            while let Some((distance, pos)) = que.pop() {
                if pos == TofPosition::Right && distance < START_LENGTH {
                    tick_off();
                    self.mode = Mode::Search;
                    self.imu
                        .init(&mut self.spi, &mut self.delay, &mut self.control_timer);
                    tick_on();
                }
            }
        }
    }

    fn control_run(&mut self) {
        self.estimate();
        self.detect_and_correct();
        let target = self.manager.target().unwrap();
        self.track(&target);
    }

    fn control_return(&mut self) {
        self.estimate();
        self.detect_and_correct();
        if let Some(target) = self.manager.target() {
            self.track(&target);
        } else {
            use RunPosture::*;

            self.stop();
            tick_off();
            let goals =
                [(2, 0, South), (2, 0, West)].map(|(x, y, pos)| Node::new(x, y, pos).unwrap());
            self.init_run(Node::new(0, 0, South).unwrap(), |node| {
                goals.iter().any(|goal| node == goal)
            });
            self.mode = Mode::Run;
            tick_on();
        }
    }

    fn estimate(&mut self) {
        // estimate
        let sensor_value = {
            SensorValue {
                left_distance: -1.02 * block!(self.left_encoder.angle()).unwrap() * WHEEL_RADIUS,
                right_distance: -1.02 * block!(self.right_encoder.angle()).unwrap() * WHEEL_RADIUS,
                translational_acceleration: block!(self
                    .imu
                    .translational_acceleration(&mut self.spi))
                .unwrap(),
                angular_velocity: 1.003 * block!(self.imu.angular_velocity(&mut self.spi)).unwrap(),
            }
        };
        self.estimator.estimate(&mut self.state, &sensor_value);
        self.stddev += Length::new::<millimeter>(0.005);
    }

    fn track(&mut self, target: &Target) {
        let input = self.navigator.navigate(&self.state, target);
        // let input = self.supervisor.supervise(&input, &self.state);
        let (control_target, control_state) = self.tracker.track(&self.state, target, &input);
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

        // let voltage = ElectricPotential::new::<volt>(3.9);
        let voltage = self.voltmeter.voltage();
        self.left_motor.apply(vol.left, voltage);
        self.right_motor.apply(vol.right, voltage);
    }

    fn detect_and_correct(&mut self) {
        let cos_th = self.state.theta.x.value.cos();
        let sin_th = self.state.theta.x.value.sin();
        if let Some(mut que) = BAG.tof_queue.try_lock() {
            while let Some((distance, pos)) = que.pop() {
                let config = match pos {
                    TofPosition::Front => &self.tof_configs.front,
                    TofPosition::Right => &self.tof_configs.right,
                    TofPosition::Left => &self.tof_configs.left,
                };
                let distance = config.1.slope * distance + config.1.intercept;
                let pose = Pose {
                    x: self.state.x.x + config.0.x * cos_th - config.0.y * sin_th,
                    y: self.state.y.x + config.0.x * sin_th + config.0.y * cos_th,
                    theta: self.state.theta.x + config.0.theta,
                };

                let mut walls = BAG.walls.lock();
                if let Some((coord, wall_state)) =
                    self.detector
                        .detect_and_update(&distance, &SENSOR_STDDEV, &pose)
                {
                    walls.update(&coord, &wall_state);
                    let info = self.converter.convert(&pose).unwrap();
                    if matches!(
                        walls.wall_state(&info.coord),
                        WallState::Checked { exists: true }
                    ) {
                        let sensor_var = SENSOR_STDDEV * SENSOR_STDDEV;
                        let var = self.stddev * self.stddev;
                        let k = (var / (var + sensor_var)).get::<uom::si::ratio::ratio>();

                        let distance_diff = k * (info.existing_distance - distance);
                        let cos_th = pose.theta.value.cos();
                        let sin_th = pose.theta.value.sin();
                        self.state.x.x += distance_diff * cos_th;
                        self.state.y.x += distance_diff * sin_th;
                        self.stddev = Length::new::<meter>((var * (1.0 - k)).value.sqrt());
                    }
                }
            }
        }
    }

    fn control_search(&mut self) {
        self.estimate();

        if let Some(target) = self.manager.target() {
            self.track(&target);
        } else {
            if !BAG.is_search_finish.load(Ordering::SeqCst) {
                self.manager.set_emergency(Pose::from_search_state(
                    BAG.state.lock().clone(),
                    SQUARE_WIDTH,
                    FRONT_OFFSET,
                ));
                let target = self.manager.target().unwrap();
                self.track(&target);
            } else if self.manager.set_final(Pose::from_search_state(
                BAG.state.lock().clone(),
                SQUARE_WIDTH,
                FRONT_OFFSET,
            )) {
                let target = self.manager.target().unwrap();
                self.track(&target);
            } else {
                self.stop();
                tick_off();
                let rev_start = Node::new(0, 0, RunPosture::South).unwrap();
                let state = BAG.state.lock();
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
            }
        };

        self.detect_and_correct();

        if self.manager.is_full() {
            return;
        }

        match (
            BAG.commander.try_lock(),
            BAG.walls.try_lock(),
            BAG.state.try_lock(),
        ) {
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
                            BAG.walls.lock().wall_state(coord),
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
        self.panic_led.set_high();
    }

    pub fn clear_interrupt(&mut self) {
        self.control_timer.clear_interrupt(Event::Update);
    }
}

pub struct Pollar {
    i2c: I2c,
    tofs: Tofs,
    timer: SensorTimer,
}

impl Pollar {
    pub fn poll(&mut self) {
        macro_rules! poll {
            ($field: ident, $pos: expr) => {
                if let Ok(distance) = self.tofs.$field.distance(&mut self.i2c) {
                    BAG.tof_queue.lock().push((distance, $pos)).ok();
                }
            };
        }
        poll!(front, TofPosition::Front);
        poll!(right, TofPosition::Right);
        poll!(left, TofPosition::Left);
    }

    pub fn clear_interrupt(&mut self) {
        self.timer.clear_interrupt(Event::Update);
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
        if BAG.is_search_finish.load(Ordering::SeqCst) {
            return;
        }
        {
            let lock = BAG.commander.lock();
            if lock.is_some() {
                return;
            }
            core::mem::drop(lock);
        }

        let walls = {
            let lock = BAG.walls.lock();
            let walls = lock.clone();
            core::mem::drop(lock);
            walls
        };
        let coord = {
            let lock = BAG.state.lock();
            let coord = lock.coordinate().clone();
            core::mem::drop(lock);
            coord
        };
        match self
            .searcher
            .search(&coord, |coord| walls.wall_state(coord))
        {
            Ok(Some(next)) => {
                BAG.commander.lock().replace(next);
            }
            Ok(None) => {
                BAG.is_search_finish.store(true, Ordering::SeqCst);
            }
            Err(err) => unreachable!("{:?}", err),
        }
    }
}
