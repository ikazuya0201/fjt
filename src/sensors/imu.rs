use components::{
    sensors::{IMUError, IMU},
    utils::vector::Vector3,
};
use embedded_hal::{
    blocking::delay::DelayMs, blocking::spi::Transfer, digital::OutputPin, timer::CountDown,
};
use nb::block;
use quantities::{Acceleration, AngularSpeed};

use crate::wait_ok;

pub struct ICM20648<T, U> {
    spi: T,
    cs: U,
    accel_offsets: [Acceleration; 3],
    gyro_offsets: [AngularSpeed; 3],
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
    const RA_GYRO_X_OUT_H: u8 = 0x33;
    const RA_GYRO_Y_OUT_H: u8 = 0x35;
    const RA_GYRO_Z_OUT_H: u8 = 0x37;
    //accelerometer
    const RA_ACCEL_X_OUT_H: u8 = 0x2D;
    const RA_ACCEL_Y_OUT_H: u8 = 0x2F;
    const RA_ACCEL_Z_OUT_H: u8 = 0x31;
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
            accel_offsets: [Acceleration::from_meter_per_second_squared(0.0); 3],
            gyro_offsets: [AngularSpeed::from_radian_per_second(0.0); 3],
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

        delay.try_delay_ms(10).ok(); //wait while reset

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
        let mut accel_offset_sums = [Acceleration::from_meter_per_second_squared(0.0); 3];
        let mut gyro_offset_sums = [AngularSpeed::from_radian_per_second(0.0); 3];
        for _ in 0..Self::CALIBRATION_NUM {
            let accels = block!(self.get_accelerations())?;
            let gyros = block!(self.get_angular_speeds())?;
            accel_offset_sums[0] += accels.x;
            accel_offset_sums[1] += accels.y;
            accel_offset_sums[2] += accels.z;
            gyro_offset_sums[0] += gyros.x;
            gyro_offset_sums[1] += gyros.y;
            gyro_offset_sums[2] += gyros.z;
            block!(timer.try_wait()).ok();
        }
        for i in 0..3 {
            self.accel_offsets[i] += accel_offset_sums[i] / Self::CALIBRATION_NUM as f32;
            self.gyro_offsets[i] += gyro_offset_sums[i] / Self::CALIBRATION_NUM as f32;
        }
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
        self.cs.try_set_low()?;
        Ok(())
    }

    fn deassert(&mut self) -> Result<(), IMUError> {
        self.cs.try_set_high()?;
        Ok(())
    }

    fn write_to_register(&mut self, address: u8, data: u8) -> Result<(), IMUError> {
        self.assert()?;
        let res = self._write_to_register(address, data);
        self.deassert()?;
        res
    }

    fn _write_to_register(&mut self, address: u8, data: u8) -> Result<(), IMUError> {
        self.spi.try_transfer(&mut [address, data])?;
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
        let buffer = self.spi.try_transfer(buffer)?;
        Ok(&buffer[1..])
    }

    #[inline]
    fn connect_raw_data(&self, higher: u8, lower: u8) -> i16 {
        ((higher as u16) << 8 | lower as u16) as i16
    }

    fn axis_to_index(&self, axis: Axis) -> usize {
        match axis {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
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

    #[inline]
    fn get_angular_speed(&mut self, axis: Axis) -> nb::Result<AngularSpeed, IMUError> {
        let address = match axis {
            Axis::X => Self::RA_GYRO_X_OUT_H,
            Axis::Y => Self::RA_GYRO_Y_OUT_H,
            Axis::Z => Self::RA_GYRO_Z_OUT_H,
        };
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(address, &mut buffer)?;
        Ok(
            self.convert_raw_data_to_angular_speed(self.connect_raw_data(buffer[0], buffer[1]))
                - self.gyro_offsets[self.axis_to_index(axis)],
        )
    }

    #[inline]
    fn get_acceleration(&mut self, axis: Axis) -> nb::Result<Acceleration, IMUError> {
        let address = match axis {
            Axis::X => Self::RA_ACCEL_X_OUT_H,
            Axis::Y => Self::RA_ACCEL_Y_OUT_H,
            Axis::Z => Self::RA_ACCEL_Z_OUT_H,
        };
        let mut buffer = [0; 3];
        let buffer = self.read_from_registers(address, &mut buffer)?;
        Ok(
            self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[0], buffer[1]))
                - self.accel_offsets[self.axis_to_index(axis)],
        )
    }
}

enum Axis {
    X,
    Y,
    Z,
}

impl<T, U> IMU for ICM20648<T, U>
where
    T: Transfer<u8>,
    U: OutputPin,
    IMUError: From<<T as Transfer<u8>>::Error> + From<<U as OutputPin>::Error>,
{
    fn get_angular_speed_x(&mut self) -> nb::Result<AngularSpeed, IMUError> {
        self.get_angular_speed(Axis::X)
    }

    fn get_angular_speed_y(&mut self) -> nb::Result<AngularSpeed, IMUError> {
        self.get_angular_speed(Axis::Y)
    }

    fn get_angular_speed_z(&mut self) -> nb::Result<AngularSpeed, IMUError> {
        self.get_angular_speed(Axis::Z)
    }

    fn get_acceleration_x(&mut self) -> nb::Result<Acceleration, IMUError> {
        self.get_acceleration(Axis::X)
    }

    fn get_acceleration_y(&mut self) -> nb::Result<Acceleration, IMUError> {
        self.get_acceleration(Axis::Y)
    }

    fn get_acceleration_z(&mut self) -> nb::Result<Acceleration, IMUError> {
        self.get_acceleration(Axis::Z)
    }

    fn get_angular_speeds(&mut self) -> nb::Result<Vector3<AngularSpeed>, IMUError> {
        let mut buffer = [0; 7];
        let buffer = self.read_from_registers(Self::RA_GYRO_X_OUT_H, &mut buffer)?;
        Ok(Vector3 {
            x: self.convert_raw_data_to_angular_speed(self.connect_raw_data(buffer[0], buffer[1]))
                - self.gyro_offsets[self.axis_to_index(Axis::X)],
            y: self.convert_raw_data_to_angular_speed(self.connect_raw_data(buffer[2], buffer[3]))
                - self.gyro_offsets[self.axis_to_index(Axis::Y)],
            z: self.convert_raw_data_to_angular_speed(self.connect_raw_data(buffer[4], buffer[5]))
                - self.gyro_offsets[self.axis_to_index(Axis::Z)],
        })
    }

    fn get_accelerations(&mut self) -> nb::Result<Vector3<Acceleration>, IMUError> {
        let mut buffer = [0; 7];
        let buffer = self.read_from_registers(Self::RA_ACCEL_X_OUT_H, &mut buffer)?;
        Ok(Vector3 {
            x: self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[0], buffer[1]))
                - self.accel_offsets[self.axis_to_index(Axis::X)],
            y: self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[2], buffer[3]))
                - self.accel_offsets[self.axis_to_index(Axis::Y)],
            z: self.convert_raw_data_to_acceleration(self.connect_raw_data(buffer[4], buffer[5]))
                - self.accel_offsets[self.axis_to_index(Axis::Z)],
        })
    }
}
