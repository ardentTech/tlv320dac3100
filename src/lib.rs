#![no_std]
extern crate alloc;

mod page0;
mod page;

use embedded_hal as hal;
use hal::i2c::{ErrorType, I2c};
use crate::page0::Page0;

// TODO custom driver errors

pub struct TLV320DAC3100<I2C, D> {
    delay: D,
    i2c: I2C,
    page0: Page0
}

impl<I2C: I2c, D: hal::delay::DelayNs> TLV320DAC3100<I2C, D> {
    pub fn new(delay: D, i2c: I2C) -> Self {
        TLV320DAC3100 { delay, i2c, page0: Page0 {} }
    }

    pub fn over_temperature(&mut self) -> Result<bool, <I2C as ErrorType>::Error> {
        let reg = self.page0.ot_flag(&mut self.i2c)?;
        Ok(!(reg == 0x1))
    }

    pub fn reset(&mut self) -> Result<(), <I2C as ErrorType>::Error> {
        self.page0.software_reset(&mut self.i2c, &mut self.delay)
    }

    pub fn set_dac_volume(&mut self) {

    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn over_temperature_err() {}

    #[test]
    fn over_temperature_ok_false() {}

    #[test]
    fn over_temperature_ok_true() {}

    #[test]
    fn reset_err() {}

    #[test]
    fn reset_ok() {}
}