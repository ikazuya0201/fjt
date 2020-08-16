use components::{
    data_types::*, defaults::DefaultSearchOperator, impls::*, prelude::*, sensors::DistanceSensor,
    utils::sample::Sample,
};
use heapless::consts::*;
use quantities::Distance;
use stm32f4xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA15, PA7},
        gpiob::{PB3, PB4, PB5, PB6, PB7, PB8, PB9},
        gpioc::PC15,
        gpioh::{PH0, PH1},
        Alternate, Analog, Output, PushPull, AF1, AF2, AF4, AF5,
    },
    i2c::{Error as I2cError, I2c},
    pwm::*,
    qei::Qei,
    spi::Spi,
    stm32::{ADC1, I2C1, SPI1, TIM1, TIM2, TIM4},
};

use crate::sensors::{IMotor, IVoltmeter, VL6180XError, ICM20648, MA702GQ, VL6180X};

pub type LeftMotor = IMotor<
    'static,
    PwmChannels<TIM1, C2>,
    PwmChannels<TIM1, C1>,
    IVoltmeter<Adc<ADC1>, ADC1, PA7<Analog>>,
>;

pub type RightMotor = IMotor<
    'static,
    PwmChannels<TIM1, C4>,
    PwmChannels<TIM1, C3>,
    IVoltmeter<Adc<ADC1>, ADC1, PA7<Analog>>,
>;

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

pub type DistanceSensorFront =
    VL6180X<'static, I2c<I2C1, (PB8<Alternate<AF4>>, PB9<Alternate<AF4>>)>, PH0<Output<PushPull>>>;

pub type DistanceSensorRight =
    VL6180X<'static, I2c<I2C1, (PB8<Alternate<AF4>>, PB9<Alternate<AF4>>)>, PA15<Output<PushPull>>>;

pub type DistanceSensorLeft =
    VL6180X<'static, I2c<I2C1, (PB8<Alternate<AF4>>, PB9<Alternate<AF4>>)>, PH1<Output<PushPull>>>;

pub enum DistanceSensors {
    Front(DistanceSensorFront),
    Right(DistanceSensorRight),
    Left(DistanceSensorLeft),
}

impl DistanceSensor for DistanceSensors {
    type Error = VL6180XError;

    fn pose(&self) -> Pose {
        use DistanceSensors::*;
        match self {
            Front(front) => front.pose(),
            Right(right) => right.pose(),
            Left(left) => left.pose(),
        }
    }

    fn get_distance(&mut self) -> nb::Result<Sample<Distance>, VL6180XError> {
        use DistanceSensors::*;
        match self {
            Front(front) => front.get_distance(),
            Right(right) => right.get_distance(),
            Left(left) => left.get_distance(),
        }
    }
}

pub type SearchOperator = DefaultSearchOperator<
    LeftMotor,
    RightMotor,
    LeftEncoder,
    RightEncoder,
    Imu,
    DistanceSensors,
    U3,
    U4,
    U256,
    U1,
>;
