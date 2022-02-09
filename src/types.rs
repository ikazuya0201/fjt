use sensors2::{encoder::MA702GQ, imu::ICM20648, motor::Motor, tof::VL6180X};
use stm32f4xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA15, PA7},
        gpiob::{PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9},
        gpioc::PC15,
        gpioh::{PH0, PH1},
        Alternate, AlternateOD, Analog, Output, PushPull, AF1, AF2, AF4, AF5,
    },
    pwm::*,
    qei::Qei,
    spi::Spi,
    stm32::{ADC1, I2C1, SPI1, TIM1, TIM2, TIM4},
};

pub type PanicLed = PB1<Output<PushPull>>;

pub type Voltmeter = sensors2::voltmeter::Voltmeter<Adc<ADC1>, ADC1, PA7<Analog>>;

pub type LeftMotor = Motor<PwmChannels<TIM1, C2>, PwmChannels<TIM1, C1>>;

pub type RightMotor = Motor<PwmChannels<TIM1, C4>, PwmChannels<TIM1, C3>>;

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

pub type I2c = stm32f4xx_hal::i2c::I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>;

pub struct Tofs {
    pub front: VL6180X<PH0<Output<PushPull>>>,
    pub right: VL6180X<PA15<Output<PushPull>>>,
    pub left: VL6180X<PH1<Output<PushPull>>>,
}
