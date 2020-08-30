use core::convert::Infallible;

use components::sensors::IMU;
use embedded_hal::{
    blocking::delay::DelayMs, blocking::spi::Transfer, digital::v2::OutputPin, timer::CountDown,
};
use nb::block;
use quantities::{Acceleration, AngularSpeed};
use stm32f4xx_hal::spi::Error as SpiError;

use crate::wait_ok;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IMUError;

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

pub struct ICM20648<T, U> {
    pub spi: T,
    cs: U,
    accel_offset: Acceleration,
    gyro_offset: AngularSpeed,
}

impl<T, U> ICM20648<T, U>
where
    T: Transfer<u8>,
    U: OutputPin,
    IMUError: From<<T as Transfer<u8>>::Error> + From<<U as OutputPin>::Error>,
{
    //RA: register address
    //all bank
    const RA_REG_BANK_SEL: u8 = 0x7F;
    //user bank 0
    const RA_WHO_AM_I: u8 = 0x00;
    const RA_LP_CONFIG: u8 = 0x05;
    const RA_PWR_MGMT_1: u8 = 0x06;
    //gyrometer
    const RA_GYRO_Z_OUT_H: u8 = 0x37;
    //accelerometer
    const RA_ACCEL_Y_OUT_H: u8 = 0x2F;
    //user bank 2
    const RA_GYRO_CONFIG_1: u8 = 0x01;
    const RA_ACCEL_CONFIG: u8 = 0x14;

    const ICM20648_DEVICE_ID: u8 = 0xE0;

    const ACCEL_RATIO: f32 = 2.0;
    const GYRO_RATIO: f32 = 1000.0;

    const CALIBRATION_NUM: u16 = 1000;

    pub fn new<'a, V, W>(spi: T, cs: U, delay: &'a mut V, timer: &'a mut W) -> Self
    where
        V: DelayMs<u32>,
        W: CountDown,
    {
        let mut icm = Self {
            spi,
            cs,
            accel_offset: Default::default(),
            gyro_offset: Default::default(),
        };

        icm.init(delay, timer);

        icm
    }

    pub fn init<'a, V, W>(&mut self, delay: &'a mut V, timer: &'a mut W)
    where
        V: DelayMs<u32>,
        W: CountDown,
    {
        wait_ok!(self.write_to_register(Self::RA_PWR_MGMT_1, 0x80)); //reset icm20648

        delay.delay_ms(10); //wait while reset

        wait_ok!(block!(self.check_who_am_i())); //wait for who am i checking

        wait_ok!(self.write_to_register(Self::RA_PWR_MGMT_1, 0x01));

        wait_ok!(self.write_to_register(Self::RA_LP_CONFIG, 0x00)); //disable duty cycle mode for gyro

        wait_ok!(self.write_to_register(Self::RA_REG_BANK_SEL, 0x20)); //switch to user bank 2

        //configure gryo to +-1000dps in full scale
        wait_ok!(self.write_to_register(Self::RA_GYRO_CONFIG_1, 0x04));

        //disable digital low path filter
        //configure accelerometer to +-2g
        wait_ok!(self.write_to_register(Self::RA_ACCEL_CONFIG, 0x00));

        wait_ok!(self.write_to_register(Self::RA_REG_BANK_SEL, 0x00)); //switch to user bank 0

        wait_ok!(self.calibrate(timer));
        wait_ok!(self.calibrate(timer));
    }

    pub fn calibrate<'a, W>(&mut self, timer: &'a mut W) -> Result<(), IMUError>
    where
        W: CountDown,
    {
        let mut accel_offset_sum = Acceleration::default();
        let mut gyro_offset_sum = AngularSpeed::default();
        for _ in 0..Self::CALIBRATION_NUM {
            let accel = block!(self.get_translational_acceleration())?;
            let gyro = block!(self.get_angular_speed())?;
            accel_offset_sum += accel;
            gyro_offset_sum += gyro;
            block!(timer.wait()).ok();
        }
        self.accel_offset += accel_offset_sum / Self::CALIBRATION_NUM as f32;
        self.gyro_offset += gyro_offset_sum / Self::CALIBRATION_NUM as f32;
        Ok(())
    }

    fn check_who_am_i(&mut self) -> nb::Result<(), IMUError> {
        let mut buffer = [0; 2];
        let buffer = self.read_from_registers(Self::RA_WHO_AM_I, &mut buffer)?;
        if buffer[0] == Self::ICM20648_DEVICE_ID {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn assert(&mut self) -> Result<(), IMUError> {
        self.cs.set_low()?;
        Ok(())
    }

    fn deassert(&mut self) -> Result<(), IMUError> {
        self.cs.set_high()?;
        Ok(())
    }

    fn write_to_register(&mut self, address: u8, data: u8) -> Result<(), IMUError> {
        self.assert()?;
        let res = self._write_to_register(address, data);
        self.deassert()?;
        res
    }

    fn _write_to_register(&mut self, address: u8, data: u8) -> Result<(), IMUError> {
        self.spi.transfer(&mut [address, data])?;
        Ok(())
    }

    //size of buffer should be equal to {data length}+1
    fn read_from_registers<'w>(
        &mut self,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], IMUError> {
        self.assert()?;
        let res = self._read_from_registers(address, buffer);
        self.deassert()?;
        res
    }

    fn _read_from_registers<'w>(
        &mut self,
        address: u8,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], IMUError> {
        buffer[0] = address | 0x80;
        let buffer = self.spi.transfer(buffer)?;
        Ok(&buffer[1..])
    }

    #[inline]
    fn connect_raw_data(&self, higher: u8, lower: u8) -> i16 {
        ((higher as u16) << 8 | lower as u16) as i16
    }

    fn convert_raw_data_to_angular_speed(&mut self, gyro_value: i16) -> AngularSpeed {
        let raw_angular_speed =
            AngularSpeed::from_degree_per_second((gyro_value as f32) / (core::i16::MAX as f32));
        Self::GYRO_RATIO * raw_angular_speed
    }

    fn convert_raw_data_to_acceleration(&mut self, accel_value: i16) -> Acceleration {
        let raw_acceleration =
            Acceleration::from_gravity((accel_value as f32) / (core::i16::MAX as f32));
        Self::ACCEL_RATIO * raw_acceleration
    }
}

impl<T, U> IMU for ICM20648<T, U>
where
    T: Transfer<u8>,
    U: OutputPin,
    IMUError: From<<T as Transfer<u8>>::Error> + From<<U as OutputPin>::Error>,
{
    type Error = IMUError;

    fn get_angular_speed(&mut self) -> nb::Result<AngularSpeed, Self::Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(Self::RA_GYRO_Z_OUT_H, &mut buffer)?;
        Ok(
            self.convert_raw_data_to_angular_speed(self.connect_raw_data(buffer[0], buffer[1]))
                - self.gyro_offset,
        )
    }

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(Self::RA_ACCEL_Y_OUT_H, &mut buffer)?;
        Ok(
            -self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[0], buffer[1]))
                - self.accel_offset,
        )
    }
}
