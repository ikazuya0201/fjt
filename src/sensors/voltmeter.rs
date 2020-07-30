use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};
use nb::block;
use quantities::{Frequency, Time, Voltage};

use super::Voltmeter;

pub struct IVoltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
{
    adc: T,
    adc_pin: PIN,
    voltage: Voltage,
    alpha: f32,
    _adc_marker: PhantomData<ADC>,
}

impl<T, ADC, PIN> IVoltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
    <T as OneShot<ADC, u16, PIN>>::Error: core::fmt::Debug,
{
    const RATIO: f32 = 1.85;
    const AVDD_VOLTAGE: Voltage = Voltage::from_volts(3.3);
    const MAX_ADC_VALUE: f32 = 4096.0;

    pub fn new(adc: T, adc_pin: PIN, period: Time, cut_off_frequency: Frequency) -> Self {
        let alpha = 1.0 / (2.0 * core::f32::consts::PI * period * cut_off_frequency + 1.0);
        let mut voltmeter = Self {
            adc,
            adc_pin,
            voltage: Voltage::from_volts(0.0),
            alpha,
            _adc_marker: PhantomData,
        };

        voltmeter.voltage = voltmeter.get_current_voltage();
        voltmeter
    }

    pub fn update_voltage(&mut self) {
        self.voltage = self.alpha * self.voltage + (1.0 - self.alpha) * self.get_current_voltage();
    }

    pub fn get_current_voltage(&mut self) -> Voltage {
        let value = block!(self.adc.read(&mut self.adc_pin)).unwrap() as f32;
        value * Self::AVDD_VOLTAGE * Self::RATIO / Self::MAX_ADC_VALUE
    }
}

impl<T, ADC, PIN> Voltmeter for IVoltmeter<T, ADC, PIN>
where
    T: OneShot<ADC, u16, PIN>,
    PIN: Channel<ADC>,
    <T as OneShot<ADC, u16, PIN>>::Error: core::fmt::Debug,
{
    fn get_voltage(&mut self) -> Voltage {
        // self.update_voltage();
        self.voltage
    }
}
