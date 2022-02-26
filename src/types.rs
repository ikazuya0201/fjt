use sensors2::{encoder::MA702GQ, imu::ICM20648, motor::Motor, tof::VL6180X};
use stm32f4xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA15, PA7},
        gpiob::{PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9},
        gpioc::PC15,
        gpioh::{PH0, PH1},
        Alternate, Analog, OpenDrain, Output, PushPull,
    },
    pac::{ADC1, I2C1, SPI1, TIM1, TIM2, TIM4, TIM5, TIM7},
    qei::Qei,
    spi::TransferModeNormal,
    timer::{counter::Counter, pwm::PwmChannel},
};

pub type PanicLed = PB1<Output<PushPull>>;

pub type Voltmeter = sensors2::voltmeter::Voltmeter<Adc<ADC1>, ADC1, PA7<Analog>>;

pub type LeftMotor = Motor<PwmChannel<TIM1, 1>, PwmChannel<TIM1, 0>>;

pub type RightMotor = Motor<PwmChannel<TIM1, 3>, PwmChannel<TIM1, 2>>;

pub type LeftEncoder =
    MA702GQ<Qei<TIM4, (PB6<Alternate<PushPull, 2>>, PB7<Alternate<PushPull, 2>>)>>;

pub type RightEncoder =
    MA702GQ<Qei<TIM2, (PA0<Alternate<PushPull, 1>>, PA1<Alternate<PushPull, 1>>)>>;

pub type Imu = ICM20648<PC15<Output<PushPull>>>;

pub type Spi = stm32f4xx_hal::spi::Spi<
    SPI1,
    (
        PB3<Alternate<PushPull, 5>>,
        PB4<Alternate<PushPull, 5>>,
        PB5<Alternate<PushPull, 5>>,
    ),
    TransferModeNormal,
>;

pub type I2c =
    stm32f4xx_hal::i2c::I2c<I2C1, (PB8<Alternate<OpenDrain, 4>>, PB9<Alternate<OpenDrain, 4>>)>;

pub struct Tofs {
    pub front: VL6180X<PH0<Output<PushPull>>>,
    pub right: VL6180X<PA15<Output<PushPull>>>,
    pub left: VL6180X<PH1<Output<PushPull>>>,
}

pub type ControlTimer = Counter<TIM5, 1_000_000>;

pub type SensorTimer = Counter<TIM7, 1_000_000>;
