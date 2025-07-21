use core::cmp::PartialEq;
use embedded_hal as hal;
use hal::i2c::{ErrorType, I2c};
use crate::page::Page;

const PAGE_ID: u8 = 0x0;
// registers
const SOFTWARE_RESET: u8 = 0x01;
const OT_FLAG: u8 = 0x03;
const CLOCK_GEN_MUXING: u8 = 0x04;
const DAC_LEFT_VOLUME_CONTROL: u8 = 0x41;
const DAC_RIGHT_VOLUME_CONTROL: u8 = 0x42;

#[derive(PartialEq)]
pub(crate) enum Channel {
    Left,
    Right
}

pub(crate) enum PllClockIn {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    Din = 0x3,
}

pub(crate) enum CodecClkIn {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    PllClk = 0x3,
}

pub(crate) struct Page0 {}

impl Page for Page0 {
    fn get_id() -> u8 { PAGE_ID }
}

impl Page0 {

    pub(crate) fn clock_gen_muxing<I2C: I2c>(
        &mut self,
        i2c: &mut I2C,
        pll_clock_in: PllClockIn,
        codec_clk_in: CodecClkIn
    ) -> Result<(), <I2C as ErrorType>::Error> {
        let reg_value = ((pll_clock_in as u8) << 2) | codec_clk_in as u8;
        self.write_register(i2c, CLOCK_GEN_MUXING, reg_value)
    }

    pub(crate) fn dac_volume_control<I2C: I2c>(&mut self, i2c: &mut I2C, channel: Channel, db: f32) -> Result<(), <I2C as ErrorType>::Error> {
        let db_constrained: f32 = if db > 24.0 {
            24.0
        } else if db < -63.5 {
            -63.5
        } else { db };

        let reg_val = (db_constrained * 2.0) as i8;

        // TODO should this support Channel::Both?
        let register = if channel == Channel::Left {
            DAC_LEFT_VOLUME_CONTROL
        } else {
            DAC_RIGHT_VOLUME_CONTROL
        };
        self.write_register(i2c, register, reg_val as u8)
    }

    pub(crate) fn ot_flag<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, <I2C as ErrorType>::Error> {
        Ok(self.read_register(i2c, OT_FLAG)? >> 1)
    }

    pub(crate) fn software_reset<I2C: I2c, D: hal::delay::DelayNs>(&mut self, i2c: &mut I2C, mut delay: D) -> Result<(), <I2C as ErrorType>::Error> {
        self.write_register(i2c, SOFTWARE_RESET, 0x1)?;
        delay.delay_ns(10);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use crate::page0::*;
    use crate::page::{ADDRESS, PAGE_CONTROL_REGISTER};

    #[test]
    fn clock_gen_muxing_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [CLOCK_GEN_MUXING, 0xa].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.clock_gen_muxing(&mut i2c, PllClockIn::Gpio1, CodecClkIn::Gpio1).unwrap_err();
        assert_eq!(err, ErrorKind::Other);
        i2c.done();
    }

    #[test]
    fn clock_gen_muxing_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [CLOCK_GEN_MUXING, 0xa].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.clock_gen_muxing(&mut i2c, PllClockIn::Gpio1, CodecClkIn::Gpio1).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_volume_control_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_LEFT_VOLUME_CONTROL, 0b1000_0001].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.dac_volume_control(&mut i2c, Channel::Left, -63.5).unwrap_err();
        assert_eq!(err, ErrorKind::Other);
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_floor_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_LEFT_VOLUME_CONTROL, 0b1000_0001].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.dac_volume_control(&mut i2c, Channel::Left, -63.5).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_floor_constrained_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_RIGHT_VOLUME_CONTROL, 0b1000_0001].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.dac_volume_control(&mut i2c, Channel::Right, -64.0).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_ceiling_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_RIGHT_VOLUME_CONTROL, 0b0011_0000].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.dac_volume_control(&mut i2c, Channel::Right, 24.0).unwrap();
        i2c.done();
    }
    #[test]
    fn dac_volume_control_db_ceiling_constrained_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_LEFT_VOLUME_CONTROL, 0b0011_0000].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.dac_volume_control(&mut i2c, Channel::Left, 24.5).unwrap();
        i2c.done();
    }

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