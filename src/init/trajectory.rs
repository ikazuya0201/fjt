use core::{fmt, iter::Chain};

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

type BackTrajectory = Chain<
    Chain<
        Chain<Chain<StraightTrajectory, StopTrajectory>, ShiftTrajectory<SpinTrajectory>>,
        StopTrajectory,
    >,
    ShiftTrajectory<StraightTrajectory>,
>;

type SetupTrajectory = Chain<Chain<StopTrajectory, SpinTrajectory>, StopTrajectory>;

type EmergencyTrajectory = Chain<BackTrajectory, ShiftTrajectory<BackTrajectory>>;

#[derive(Clone)]
enum Trajectory {
    Straight(StraightTrajectory),
    Slalom(SlalomTrajectory),
    Back(BackTrajectory),
    Stop(StopTrajectory),
    Setup(SetupTrajectory),
    Emergency(EmergencyTrajectory),
}

impl Iterator for Trajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        use Trajectory::*;
        match self {
            Straight(inner) => inner.next(),
            Slalom(inner) => inner.next(),
            Back(inner) => inner.next(),
            Stop(inner) => inner.next(),
            Setup(inner) => inner.next(),
            Emergency(inner) => inner.next(),
        }
    }
}

#[derive(TypedBuilder)]
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
    trajectories: Deque<ShiftTrajectory<Trajectory>, 3>,
    fin: Option<Trajectory>,
    init: ShiftTrajectory<Trajectory>,
    front: Trajectory,
    right: Trajectory,
    left: Trajectory,
    back: Trajectory,
    emergency: Trajectory,

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

impl fmt::Debug for TrajectoryManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TrajectoryManager")
            .field("fin_is_none", &self.fin.is_none())
            .finish()
    }
}

impl TrajectoryManager {
    pub fn target(&mut self) -> Option<Target> {
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

    pub fn set_init(&mut self) {
        self.trajectories.push_back(self.init.clone()).ok();
    }

    pub fn set_final(&mut self, pose: Pose) -> bool {
        if let Some(fin) = self.fin.take() {
            self.trajectories
                .push_back(ShiftTrajectory::new(pose, fin))
                .ok();
            true
        } else {
            false
        }
    }

    pub fn is_full(&self) -> bool {
        self.trajectories.is_full()
    }

    pub fn set(&mut self, pose: Pose, kind: SearchTrajectoryKind) {
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

    pub fn set_emergency(&mut self, pose: Pose) {
        self.trajectories
            .push_back(ShiftTrajectory::new(pose, self.emergency.clone()))
            .ok();
    }

    pub fn init_run(&mut self, path: Vec<Node<W>, PATH_MAX>, init_angle: Angle) {
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
                    Trajectory::Setup(
                        StopTrajectory::new(
                            Default::default(),
                            self.period,
                            Time::new::<second>(0.5),
                        )
                        .chain(self.spin.generate(init_angle))
                        .chain(StopTrajectory::new(
                            Pose {
                                theta: init_angle,
                                ..Default::default()
                            },
                            self.period,
                            Time::new::<second>(0.5),
                        )),
                    )
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

        let init = ShiftTrajectory::new(
            initial_pose,
            Trajectory::Straight(straight.generate(
                square_width / 2.0 + front_offset,
                Default::default(),
                search_velocity,
            )),
        );
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
        let back = straight
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
            ));
        let emergency = back.clone().chain(ShiftTrajectory::new(
            Pose {
                x: -2.0 * front_offset,
                y: Default::default(),
                theta: Angle::new::<degree>(180.0),
            },
            back.clone(),
        ));
        Self {
            trajectories: Deque::new(),
            fin: Some(fin),
            init,
            front,
            left,
            right,
            back: Trajectory::Back(back),
            emergency: Trajectory::Emergency(emergency),

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
