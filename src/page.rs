use embedded_hal::i2c::{ErrorType, I2c};

pub const ADDRESS: u8 = 0x18;
pub const PAGE_CONTROL_REGISTER: u8 = 0x00;

#[derive(Debug, PartialEq)]
pub enum TLV320DAC3100Error<E> {
    I2C(E),
    InvalidArgument
}

pub trait Page {
    fn get_id() -> u8;

    fn read_register<I2C: I2c>(&self, i2c: &mut I2C, reg: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        // TODO should select_page and write_read be in a transaction?
        self.select_page(i2c)?;
        let mut buf = [0u8];
        i2c.write_read(ADDRESS, &[reg], &mut buf).map_err(TLV320DAC3100Error::I2C)?;
        Ok(buf[0])
    }

    fn select_page<I2C: I2c>(&self, i2c: &mut I2C) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        Ok(i2c.write(ADDRESS, &[PAGE_CONTROL_REGISTER, Self::get_id()]).map_err(TLV320DAC3100Error::I2C)?)
    }

    fn write_register<I2C: I2c>(&self, i2c: &mut I2C, reg: u8, payload: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        // TODO should select_page and write be in a transaction?
        self.select_page(i2c)?;
        Ok(i2c.write(ADDRESS, &[reg, payload]).map_err(TLV320DAC3100Error::I2C)?)
    }
}