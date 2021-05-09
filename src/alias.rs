use components::{sensors::DistanceSensor, utils::sample::Sample};
use stm32f4xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA15, PA7},
        gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9},
        gpioc::PC15,
        gpioh::{PH0, PH1},
        Alternate, AlternateOD, Analog, Output, PushPull, AF1, AF2, AF4, AF5,
    },
    i2c::{Error as I2cError, I2c},
    pwm::*,
    qei::Qei,
    spi::Spi,
    stm32::{ADC1, I2C1, SPI1, TIM1, TIM2, TIM4},
};
use uom::si::f32::Length;

use crate::sensors::{IMotor, IVoltmeter, VL6180XError, ICM20648, MA702GQ, VL6180X};

pub type Voltmeter = IVoltmeter<Adc<ADC1>, ADC1, PA7<Analog>>;

pub type LeftMotor = IMotor<PwmChannels<TIM1, C2>, PwmChannels<TIM1, C1>, Voltmeter>;

pub type RightMotor = IMotor<PwmChannels<TIM1, C4>, PwmChannels<TIM1, C3>, Voltmeter>;

pub type LeftEncoder = MA702GQ<Qei<TIM4, (PB6<Alternate<AF2>>, PB7<Alternate<AF2>>)>>;

pub type RightEncoder = MA702GQ<Qei<TIM2, (PA0<Alternate<AF1>>, PA1<Alternate<AF1>>)>>;

pub type Imu = ICM20648<
    Spi<
        SPI1,
        (
            PB3<Alternate<AF5>>,
            PB4<Alternate<AF5>>,
            PB5<Alternate<AF5>>,
        ),
    >,
    PC15<Output<PushPull>>,
>;

impl From<I2cError> for VL6180XError {
    fn from(_error: I2cError) -> VL6180XError {
        VL6180XError
    }
}

pub type SensorI2c = I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>;

pub type DistanceSensorFront = VL6180X<SensorI2c, PH0<Output<PushPull>>>;

pub type DistanceSensorRight = VL6180X<SensorI2c, PA15<Output<PushPull>>>;

pub type DistanceSensorLeft = VL6180X<SensorI2c, PH1<Output<PushPull>>>;

pub enum DistanceSensors {
    Front(DistanceSensorFront),
    Right(DistanceSensorRight),
    Left(DistanceSensorLeft),
}

impl DistanceSensor for DistanceSensors {
    type Error = VL6180XError;

    fn pose(&self) -> components::types::data::Pose {
        use DistanceSensors::*;
        match self {
            Front(front) => front.pose(),
            Right(right) => right.pose(),
            Left(left) => left.pose(),
        }
    }

    fn get_distance(&mut self) -> nb::Result<Sample<Length>, VL6180XError> {
        use DistanceSensors::*;
        match self {
            Front(front) => front.get_distance(),
            Right(right) => right.get_distance(),
            Left(left) => left.get_distance(),
        }
    }
}

pub const N: usize = 4;

pub type SearchOperator = components::defaults::alias::SearchOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensors,
    N,
>;

#[allow(unused)]
pub type RunOperator = components::defaults::alias::RunOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensors,
    N,
>;

#[allow(unused)]
pub type Robot = components::robot::Robot<
    components::estimator::Estimator<LeftEncoder, RightEncoder, Imu>,
    components::tracker::Tracker<
        LeftMotor,
        RightMotor,
        components::controllers::TranslationalController,
        components::controllers::RotationalController,
    >,
    components::wall_detector::WallDetector<
        components::wall_manager::WallManager<N>,
        components::obstacle_detector::ObstacleDetector<DistanceSensors>,
        N,
    >,
    components::types::data::RobotState,
>;
