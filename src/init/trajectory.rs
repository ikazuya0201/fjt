use core::iter::Chain;

use super::{PATH_MAX, W};
use heapless::{Deque, Vec};
#[allow(unused)]
use micromath::F32Ext;
use mousecore2::{
    control::Target,
    solve::{
        run::{Node, TrajectoryKind as RunTrajectoryKind},
        search::TrajectoryKind as SearchTrajectoryKind,
    },
    trajectory::{
        slalom::{SlalomConfig, SlalomDirection, SlalomGenerator, SlalomKind, SlalomTrajectory},
        spin::{SpinGenerator, SpinTrajectory},
        straight::{StraightGenerator, StraightTrajectory},
        ShiftTrajectory, StopTrajectory,
    },
    wall::Pose,
};
use typed_builder::TypedBuilder;
use uom::si::{
    angle::degree,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    time::second,
};

type BackBeforeTrajectory = Chain<StraightTrajectory, StopTrajectory>;

type BackAfterTrajectory = Chain<
    Chain<ShiftTrajectory<SpinTrajectory>, StopTrajectory>,
    ShiftTrajectory<StraightTrajectory>,
>;

type SetupTrajectory = Chain<Chain<StopTrajectory, SpinTrajectory>, StopTrajectory>;

type EmergencyRecoveryTrajectory =
    ShiftTrajectory<Chain<SpinTrajectory, ShiftTrajectory<StraightTrajectory>>>;

#[derive(Clone)]
enum Trajectory {
    Straight(ShiftTrajectory<StraightTrajectory>),
    Slalom(ShiftTrajectory<SlalomTrajectory>),
    Back {
        state: BackState,
        before: ShiftTrajectory<BackBeforeTrajectory>,
        after: ShiftTrajectory<BackAfterTrajectory>,
    },
    Stop(StopTrajectory),
    ShiftStop(ShiftTrajectory<StopTrajectory>),
    Setup(ShiftTrajectory<SetupTrajectory>),
    EmergencyRecovery(ShiftTrajectory<EmergencyRecoveryTrajectory>),
}

#[derive(Clone, Copy)]
enum BackState {
    Before,
    After,
}

pub enum Command {
    Track(Target),
    Operation,
}

impl Iterator for Trajectory {
    type Item = Command;

    fn next(&mut self) -> Option<Self::Item> {
        use Trajectory::*;
        match self {
            Straight(inner) => inner.next().map(Command::Track),
            Slalom(inner) => inner.next().map(Command::Track),
            Back {
                state,
                before,
                after,
            } => match *state {
                BackState::Before => Some(if let Some(target) = before.next() {
                    Command::Track(target)
                } else {
                    *state = BackState::After;
                    Command::Operation
                }),
                BackState::After => after.next().map(Command::Track),
            },
            Stop(inner) => inner.next().map(Command::Track),
            ShiftStop(inner) => inner.next().map(Command::Track),
            Setup(inner) => inner.next().map(Command::Track),
            EmergencyRecovery(inner) => inner.next().map(Command::Track),
        }
    }
}

#[derive(TypedBuilder, Clone)]
pub struct TrajectoryConfig {
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

pub struct TrajectoryManager {
    config: TrajectoryConfig,
    state: ManagerState,
}

impl TrajectoryManager {
    pub fn command(&mut self) -> Option<Command> {
        match &mut self.state {
            ManagerState::Search(inner) => inner.command(),
            ManagerState::Run(inner) => inner.command(),
        }
    }

    pub fn set_final(&mut self, pose: Pose) -> bool {
        match &mut self.state {
            ManagerState::Search(inner) => inner.set_final(pose),
            _ => unreachable!(),
        }
    }

    pub fn is_full(&self) -> bool {
        match &self.state {
            ManagerState::Search(inner) => inner.trajectories.is_full(),
            ManagerState::Run(inner) => inner.trajectories.is_full(),
        }
    }

    pub fn set(&mut self, pose: Pose, kind: SearchTrajectoryKind) {
        match &mut self.state {
            ManagerState::Search(inner) => inner.set(pose, kind),
            _ => unreachable!(),
        }
    }

    pub fn set_emergency(&mut self, pose: Pose) {
        match &mut self.state {
            ManagerState::Search(inner) => inner.set_emergency(pose),
            _ => unreachable!(),
        }
    }

    pub fn into_run(&mut self, path: Vec<Node<W>, PATH_MAX>, init_angle: Angle) {
        self.state = ManagerState::Run(RunTrajectoryManager::new(
            self.config.clone(),
            path,
            init_angle,
        ));
    }

    pub fn into_search(&mut self) {
        self.state = ManagerState::Search(SearchTrajectoryManager::new(self.config.clone()));
    }

    pub fn last_node(&mut self) -> Option<Node<W>> {
        match &mut self.state {
            ManagerState::Run(inner) => inner.path.last().cloned(),
            _ => None,
        }
    }
}

impl From<TrajectoryConfig> for TrajectoryManager {
    fn from(config: TrajectoryConfig) -> Self {
        Self {
            config: config.clone(),
            state: ManagerState::Search(SearchTrajectoryManager::new(config)),
        }
    }
}

enum ManagerState {
    Search(SearchTrajectoryManager),
    Run(RunTrajectoryManager),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SearchManagerState {
    Normal,
    Emergency,
}

struct SearchTrajectoryManager {
    trajectories: Deque<Trajectory, 3>,

    fin: Option<StraightTrajectory>,
    front: StraightTrajectory,
    right: SlalomTrajectory,
    left: SlalomTrajectory,
    back_before: BackBeforeTrajectory,
    back_after: BackAfterTrajectory,

    front_emergency: ShiftTrajectory<StraightTrajectory>,
    right_emergency: EmergencyRecoveryTrajectory,
    left_emergency: EmergencyRecoveryTrajectory,
    back_emergency: EmergencyRecoveryTrajectory,
    emergency_stop: StopTrajectory,

    state: SearchManagerState,
}

impl SearchTrajectoryManager {
    fn new(
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
            ..
        }: TrajectoryConfig,
    ) -> Self {
        let slalom_config = SlalomConfig::new(square_width, front_offset);
        let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
        let spin = SpinGenerator::new(spin_v_max, spin_a_max, spin_j_max, period);
        let straight = StraightGenerator::new(v_max, a_max, j_max, period);
        let square_width_half = square_width / 2.0;

        let init_raw = straight.generate(
            square_width_half + front_offset,
            Default::default(),
            search_velocity,
        );
        let init = Trajectory::Straight(ShiftTrajectory::new(initial_pose, init_raw.clone()));
        let fin = straight.generate(
            square_width_half - front_offset,
            search_velocity,
            Default::default(),
        );
        let front = StraightGenerator::generate_constant(square_width, search_velocity, period);
        let right = slalom.generate_constant_slalom(
            slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Right),
            search_velocity,
        );
        let left = slalom.generate_constant_slalom(
            slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Left),
            search_velocity,
        );

        let right_angle = Angle::new::<degree>(-90.0);
        let left_angle = Angle::new::<degree>(90.0);
        let back_angle = Angle::new::<degree>(180.0);
        let back_before = straight
            .generate(
                square_width_half - front_offset,
                search_velocity,
                Default::default(),
            )
            .chain(StopTrajectory::new(
                Pose {
                    x: square_width_half - front_offset,
                    ..Default::default()
                },
                period,
                Time::new::<second>(0.1),
            ));
        let back_after = ShiftTrajectory::new(
            Pose {
                x: square_width_half - front_offset,
                ..Default::default()
            },
            spin.generate(back_angle),
        )
        .chain(StopTrajectory::new(
            Pose {
                x: square_width_half - front_offset,
                y: Default::default(),
                theta: back_angle,
            },
            period,
            Time::new::<second>(0.1),
        ))
        .chain(ShiftTrajectory::new(
            Pose {
                x: square_width_half - front_offset,
                y: Default::default(),
                theta: back_angle,
            },
            straight.generate(
                square_width_half + front_offset,
                Default::default(),
                search_velocity,
            ),
        ));
        let emergency_shift = Pose {
            x: square_width_half - front_offset,
            ..Default::default()
        };
        let front_emergency = ShiftTrajectory::new(emergency_shift, init_raw.clone());
        let right_emergency = ShiftTrajectory::new(
            emergency_shift,
            spin.generate(right_angle).chain(ShiftTrajectory::new(
                Pose {
                    theta: right_angle,
                    ..Default::default()
                },
                init_raw.clone(),
            )),
        );
        let left_emergency = ShiftTrajectory::new(
            emergency_shift,
            spin.generate(left_angle).chain(ShiftTrajectory::new(
                Pose {
                    theta: left_angle,
                    ..Default::default()
                },
                init_raw.clone(),
            )),
        );
        let back_emergency = ShiftTrajectory::new(
            emergency_shift,
            spin.generate(back_angle).chain(ShiftTrajectory::new(
                Pose {
                    theta: back_angle,
                    ..Default::default()
                },
                init_raw,
            )),
        );
        let emergency_stop = StopTrajectory::new(
            Pose {
                x: square_width_half - front_offset,
                ..Default::default()
            },
            period,
            Time::new::<second>(0.2),
        );
        let mut trajectories = Deque::new();
        trajectories.push_back(init).ok();
        Self {
            trajectories,
            fin: Some(fin),
            front,
            left,
            right,
            back_before,
            back_after,
            front_emergency,
            right_emergency,
            left_emergency,
            back_emergency,
            emergency_stop,
            state: SearchManagerState::Normal,
        }
    }

    fn command(&mut self) -> Option<Command> {
        let front = self.trajectories.front_mut()?;
        if let Some(target) = front.next() {
            Some(target)
        } else {
            core::mem::drop(front);
            self.trajectories.pop_front();
            self.trajectories.front_mut()?.next()
        }
    }

    fn set_final(&mut self, pose: Pose) -> bool {
        if let Some(fin) = self.fin.take() {
            self.trajectories
                .push_back(Trajectory::Straight(ShiftTrajectory::new(pose, fin)))
                .ok();
            true
        } else {
            false
        }
    }

    fn set(&mut self, pose: Pose, kind: SearchTrajectoryKind) {
        use SearchTrajectoryKind::*;

        let trajectory = match self.state {
            SearchManagerState::Normal => match kind {
                Front => Trajectory::Straight(ShiftTrajectory::new(pose, self.front.clone())),
                Right => Trajectory::Slalom(ShiftTrajectory::new(pose, self.right.clone())),
                Left => Trajectory::Slalom(ShiftTrajectory::new(pose, self.left.clone())),
                Back => Trajectory::Back {
                    state: BackState::Before,
                    before: ShiftTrajectory::new(pose, self.back_before.clone()),
                    after: ShiftTrajectory::new(pose, self.back_after.clone()),
                },
            },
            SearchManagerState::Emergency => match kind {
                Front => Trajectory::Straight(self.front_emergency.clone()),
                Right => Trajectory::EmergencyRecovery(ShiftTrajectory::new(
                    pose,
                    self.right_emergency.clone(),
                )),
                Left => Trajectory::EmergencyRecovery(ShiftTrajectory::new(
                    pose,
                    self.left_emergency.clone(),
                )),
                Back => Trajectory::EmergencyRecovery(ShiftTrajectory::new(
                    pose,
                    self.back_emergency.clone(),
                )),
            },
        };
        self.trajectories.push_back(trajectory).ok();
        self.state = SearchManagerState::Normal;
    }

    fn set_emergency(&mut self, pose: Pose) {
        if self.state == SearchManagerState::Emergency {
            self.trajectories
                .push_back(Trajectory::ShiftStop(ShiftTrajectory::new(
                    pose,
                    self.emergency_stop.clone(),
                )))
                .ok();
        } else {
            self.state = SearchManagerState::Emergency;
            self.trajectories
                .push_back(Trajectory::Straight(ShiftTrajectory::new(
                    pose,
                    self.fin.clone().unwrap(),
                )))
                .ok();
        }
    }
}

struct RunTrajectoryManager {
    trajectories: Deque<Trajectory, 3>,
    run_iter: usize,
    cur_velocity: Velocity,
    path: Vec<Node<W>, PATH_MAX>,
    slalom: SlalomGenerator,
    slalom_config: SlalomConfig,
    straight: StraightGenerator,
    run_slalom_velocity: Velocity,
    square_width: Length,
}

impl RunTrajectoryManager {
    fn new(
        TrajectoryConfig {
            square_width,
            front_offset,
            v_max,
            a_max,
            j_max,
            spin_v_max,
            spin_a_max,
            spin_j_max,
            period,
            run_slalom_velocity,
            ..
        }: TrajectoryConfig,
        path: Vec<Node<W>, PATH_MAX>,
        init_angle: Angle,
    ) -> Self {
        let slalom_config = SlalomConfig::new(square_width, front_offset);
        let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
        let spin = SpinGenerator::new(spin_v_max, spin_a_max, spin_j_max, period);
        let straight = StraightGenerator::new(v_max, a_max, j_max, period);
        let init_pose = {
            let mut pose = Pose::from_node(path[0], square_width);
            pose.theta -= init_angle;
            pose
        };
        let mut trajectories = Deque::new();
        trajectories
            .push_back(if init_angle == Angle::default() {
                Trajectory::Stop(StopTrajectory::new(
                    init_pose,
                    period,
                    Time::new::<second>(0.5),
                ))
            } else {
                Trajectory::Setup(ShiftTrajectory::new(
                    init_pose,
                    StopTrajectory::new(Default::default(), period, Time::new::<second>(0.5))
                        .chain(spin.generate(init_angle))
                        .chain(StopTrajectory::new(
                            Pose {
                                theta: init_angle,
                                ..Default::default()
                            },
                            period,
                            Time::new::<second>(0.5),
                        )),
                ))
            })
            .ok();
        Self {
            trajectories,
            cur_velocity: Default::default(),
            run_iter: 0,
            path,
            slalom,
            slalom_config,
            run_slalom_velocity,
            straight,
            square_width,
        }
    }

    fn command(&mut self) -> Option<Command> {
        if !self.trajectories.is_full() && self.run_iter < self.path.len() - 1 {
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
                (velocity, velocity * 0.5, Default::default())
            } else if i + 3 == self.path.len() {
                (velocity, velocity * 0.3, velocity * 0.3)
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
                        Trajectory::Straight(ShiftTrajectory::new(pose, trajectory)),
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
                        Trajectory::Straight(ShiftTrajectory::new(pose, trajectory)),
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
                        Trajectory::Slalom(ShiftTrajectory::new(pose, trajectory)),
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
}
