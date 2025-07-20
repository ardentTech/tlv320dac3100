use embedded_hal::i2c::{ErrorType, I2c};

pub const ADDRESS: u8 = 0x18;
pub const PAGE_CONTROL_REGISTER: u8 = 0x00;

pub trait Page {
    fn get_id() -> u8;

    fn read_register<I2C: I2c>(&self, i2c: &mut I2C, reg: u8) -> Result<u8, <I2C as ErrorType>::Error> {
        let mut buf = [0u8];
        i2c.write_read(ADDRESS, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn select_page<I2C: I2c>(&self, i2c: &mut I2C) -> Result<(), <I2C as ErrorType>::Error> {
        Ok(i2c.write(ADDRESS, &[PAGE_CONTROL_REGISTER, Self::get_id()])?)
    }

    fn write_register<I2C: I2c>(&self, i2c: &mut I2C, reg: u8, payload: u8) -> Result<(), <I2C as ErrorType>::Error> {
        Ok(i2c.write(ADDRESS, &[reg, payload])?)
    }
}