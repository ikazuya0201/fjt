#![no_std]
#![no_main]

extern crate panic_rtt;
mod macros;
mod sensors;

use core::convert::Infallible;
use core::fmt::Write;

use components::{agent::StateEstimator, estimator::EstimatorBuilder};
use cortex_m_rt::entry;
use jlink_rtt::Output;
use nb::block;
use quantities::{Angle, Distance, Frequency, Time};
use sensors::{IMUError, ICM20648, MA702GQ};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    gpio,
    i2c::{Error as I2cError, I2c},
    prelude::*,
    qei::Qei,
    spi::{Error as SpiError, Spi},
    stm32,
    timer::Timer,
};

impl From<SpiError> for IMUError {
    fn from(_error: SpiError) -> Self {
        Self
    }
}

impl From<Infallible> for IMUError {
    fn from(_error: Infallible) -> Self {
        Self
    }
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
    let gpioa = device_peripherals.GPIOA.split();
    let gpiob = device_peripherals.GPIOB.split();
    let gpioc = device_peripherals.GPIOC.split();
    let gpioh = device_peripherals.GPIOH.split();

    let wheel_radius = Distance::from_meters(0.0069);

    let right_encoder = {
        let pins = (
            gpioa.pa0.into_alternate_af1(),
            gpioa.pa1.into_alternate_af1(),
        );
        let qei = Qei::tim2(device_peripherals.TIM2, pins);
        let mut encoder = MA702GQ::new(qei, wheel_radius);
        encoder
    };

    let left_encoder = {
        let pins = (
            gpiob.pb6.into_alternate_af2(),
            gpiob.pb7.into_alternate_af2(),
        );
        let qei = Qei::tim4(device_peripherals.TIM4, pins);
        let mut encoder = MA702GQ::new(qei, wheel_radius);
        encoder
    };

    let period = Time::from_seconds(0.001);
    let mut timer = Timer::tim9(device_peripherals.TIM9, 1.khz(), clocks);

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
            6_250_000.hz(),
            clocks,
        );
        let mut cs = gpioc.pc15.into_push_pull_output();
        cs.set_high().unwrap();

        let mut imu = ICM20648::new(spi, cs, &mut delay, &mut timer);
        imu
    };

    let mut estimator = EstimatorBuilder::new()
        .left_encoder(left_encoder)
        .right_encoder(right_encoder)
        .imu(imu)
        .period(period)
        .cut_off_frequency(Frequency::from_hertz(333.0))
        .initial_posture(Angle::from_degree(90.0))
        .build();

    let mut out = Output::new();

    let mut count = 0;
    loop {
        let state = estimator.estimate();
        count += 1;
        if count == 1000 {
            writeln!(out, "{:?}", state).unwrap();
            count = 0;
        }
        block!(timer.wait()).ok();
    }
}
