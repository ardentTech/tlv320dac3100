use embedded_hal as hal;
use hal::i2c::{ErrorType, I2c};
use crate::page::Page;

const PAGE_ID: u8 = 0x0;
// registers
const SOFTWARE_RESET: u8 = 0x01;
const OT_FLAG: u8 = 0x03;

pub(crate) struct Page0 {}

impl Page for Page0 {
    fn get_id() -> u8 { PAGE_ID }
}

impl Page0 {

    // register 3
    pub(crate) fn ot_flag<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, <I2C as ErrorType>::Error> {
        self.select_page(i2c)?;
        Ok(self.read_register(i2c, OT_FLAG)? >> 1)
    }

    // register 1
    pub(crate) fn software_reset<I2C: I2c, D: hal::delay::DelayNs>(&mut self, i2c: &mut I2C, mut delay: D) -> Result<(), <I2C as ErrorType>::Error> {
        self.select_page(i2c)?;
        self.write_register(i2c, SOFTWARE_RESET, 0x1)?;
        delay.delay_ns(10);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use crate::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use crate::page0::{OT_FLAG, SOFTWARE_RESET};
    use crate::page::{ADDRESS, PAGE_CONTROL_REGISTER};

    #[test]
    fn ot_flag_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write_read(ADDRESS, [OT_FLAG].to_vec(), [0x10].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.ot_flag(&mut i2c).unwrap_err();
        assert_eq!(err, ErrorKind::Other);
        i2c.done();
    }

    #[test]
    fn ot_flag_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write_read(ADDRESS, [OT_FLAG].to_vec(), [0x10].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let res = page.ot_flag(&mut i2c).unwrap();
        assert_eq!(res, 0x8);
        i2c.done();
    }

    #[test]
    fn software_reset_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [SOFTWARE_RESET, 0x01].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.software_reset(&mut i2c, NoopDelay).unwrap_err();
        assert_eq!(err, ErrorKind::Other);
        i2c.done();
    }

    #[test]
    fn software_reset_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [SOFTWARE_RESET, 0x01].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.software_reset(&mut i2c, NoopDelay).unwrap();
        i2c.done();
    }
}