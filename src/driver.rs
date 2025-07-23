use embedded_hal as hal;
use hal::i2c::I2c;
use crate::error::TLV320DAC3100Error;
use crate::page0::Page0;

pub struct TLV320DAC3100<I2C, D> {
    delay: D,
    i2c: I2C,
    page0: Page0
}

impl<I2C: I2c, D: hal::delay::DelayNs> TLV320DAC3100<I2C, D> {
    pub fn new(delay: D, i2c: I2C) -> Self {
        TLV320DAC3100 { delay, i2c, page0: Page0 {} }
    }

    pub fn over_temperature(&mut self) -> Result<bool, TLV320DAC3100Error<I2C::Error>> {
        let reg = self.page0.ot_flag(&mut self.i2c)?;
        Ok(!(reg == 0x1))
    }

    pub fn reset(&mut self) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.page0.set_software_reset(&mut self.i2c, &mut self.delay)
    }

    pub fn set_dac_volume(&mut self) {}
}