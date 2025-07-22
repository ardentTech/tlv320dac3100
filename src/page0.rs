use core::cmp::PartialEq;
use embedded_hal as hal;
use embedded_hal::i2c::Error;
use hal::i2c::{ErrorType, I2c};
use crate::page::{Page, TLV320DAC3100Error};

const PAGE_ID: u8 = 0x0;
const MAX_PLL_D_MSB: u8 = 0x3f;
const MAX_PLL_P: u8 = 0x8;
const MIN_PLL_P: u8 = 0x1;
const MAX_PLL_R: u8 = 0x10;
const MIN_PLL_R: u8 = 0x1;
const MAX_PLL_J: u8 = 0x3f;
const MIN_PLL_J: u8 = 0x1;
const MAX_DAC_NDAC_DIVIDER: u8 = 0x80;
const MIN_DAC_NDAC_DIVIDER: u8 = 0x1;
const MAX_DAC_VOLUME_CONTROL_DB: f32 = 24.0;
const MIN_DAC_VOLUME_CONTROL_DB: f32 = -63.5;

// registers
const SOFTWARE_RESET: u8 = 0x01;
const OT_FLAG: u8 = 0x03;
const CLOCK_GEN_MUXING: u8 = 0x04;
const PLL_P_AND_R_VALUES: u8 = 0x05;
const PLL_J_VALUE: u8 = 0x06;
const PLL_D_VALUE_MSB: u8 = 0x07;
const PLL_D_VALUE_LSB: u8 = 0x08;
const DAC_NDAC_VAL: u8 = 0x0b;
const DAC_LEFT_VOLUME_CONTROL: u8 = 0x41;
const DAC_RIGHT_VOLUME_CONTROL: u8 = 0x42;

// TODO should this support Channel::Both?
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

    // TODO unit test
    pub(crate) fn clock_gen_muxing<I2C: I2c>(&mut self, i2c: &mut I2C,) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, CLOCK_GEN_MUXING)
    }

    pub(crate) fn set_clock_gen_muxing<I2C: I2c>(
        &mut self,
        i2c: &mut I2C,
        pll_clock_in: PllClockIn,
        codec_clk_in: CodecClkIn
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_value = ((pll_clock_in as u8) << 2) | codec_clk_in as u8;
        self.write_register(i2c, CLOCK_GEN_MUXING, reg_value)
    }

    // TODO unit test
    pub(crate) fn dac_ndac_val<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, DAC_NDAC_VAL)
    }

    pub(crate) fn set_dac_ndac_val<I2C: I2c>(&mut self, i2c: &mut I2C, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider < MIN_DAC_NDAC_DIVIDER || divider > MAX_DAC_NDAC_DIVIDER {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        let mut reg_val: u8 = (if powered { 0x1 } else { 0x0 }) << 7;
        if divider < 128 {
            reg_val |= divider;
        }
        self.write_register(i2c, DAC_NDAC_VAL, reg_val)
    }

    // TODO unit test
    pub(crate) fn dac_left_volume_control<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, DAC_LEFT_VOLUME_CONTROL)
    }

    // TODO unit test
    pub(crate) fn dac_right_volume_control<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, DAC_RIGHT_VOLUME_CONTROL)
    }

    // TODO split this up and eliminate Channel enum?
    pub(crate) fn set_dac_volume_control<I2C: I2c>(&mut self, i2c: &mut I2C, channel: Channel, db: f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if db < MIN_DAC_VOLUME_CONTROL_DB || db > MAX_DAC_VOLUME_CONTROL_DB {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }

        let reg_val = (db * 2.0) as i8;

        let register = if channel == Channel::Left {
            DAC_LEFT_VOLUME_CONTROL
        } else {
            DAC_RIGHT_VOLUME_CONTROL
        };
        self.write_register(i2c, register, reg_val as u8)
    }

    pub(crate) fn ot_flag<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        Ok(self.read_register(i2c, OT_FLAG)? >> 1)
    }

    // TODO unit test
    pub(crate) fn pll_d_value_msb<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, PLL_D_VALUE_MSB)
    }

    pub(crate) fn set_pll_d_value_msb<I2C: I2c>(&mut self, i2c: &mut I2C, d: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if d > MAX_PLL_D_MSB {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        self.write_register(i2c, PLL_D_VALUE_MSB, d)
    }

    // TODO unit test
    pub(crate) fn pll_d_value_lsb<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, PLL_D_VALUE_LSB)
    }

    pub(crate) fn set_pll_d_value_lsb<I2C: I2c>(&mut self, i2c: &mut I2C, d: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_register(i2c, PLL_D_VALUE_LSB, d)
    }

    // TODO unit test
    pub(crate) fn pll_j_value<I2C: I2c>(&mut self, i2c: &mut I2C, j: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, PLL_J_VALUE)
    }

    pub(crate) fn set_pll_j_value<I2C: I2c>(&mut self, i2c: &mut I2C, j: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if j < MIN_PLL_J || j > MAX_PLL_J {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        self.write_register(i2c, PLL_J_VALUE, j)
    }

    // TODO unit test
    pub(crate) fn pll_p_and_r_values<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_register(i2c, PLL_P_AND_R_VALUES)
    }

    pub(crate) fn set_pll_p_and_r_values<I2C: I2c>(&mut self, i2c: &mut I2C, powered: bool, p: u8, r: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if p < MIN_PLL_P || p > MAX_PLL_P {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        if r < MIN_PLL_R || r > MAX_PLL_R {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }

        let mut reg_val = (if powered { 0x1 } else { 0x0 }) << 3;
        reg_val |= p;
        reg_val <<= 4;
        reg_val |= r;

        self.write_register(i2c, PLL_P_AND_R_VALUES, reg_val)
    }

    pub(crate) fn set_software_reset<I2C: I2c, D: hal::delay::DelayNs>(&mut self, i2c: &mut I2C, mut delay: D) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
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
        let err = page.set_clock_gen_muxing(&mut i2c, PllClockIn::Gpio1, CodecClkIn::Gpio1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
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
        page.set_clock_gen_muxing(&mut i2c, PllClockIn::Gpio1, CodecClkIn::Gpio1).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_ndac_val_i2c_err() {
        let mut i2c = I2cMock::new(&[
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_NDAC_VAL, 0b1000_0001].to_vec()).with_error(ErrorKind::Other),
        ]);
        let mut page = Page0 {};
        let err = page.set_dac_ndac_val(&mut i2c, true, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn dac_ndac_val_invalid_arg_min_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_dac_ndac_val(&mut i2c, false, 0).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn dac_ndac_val_invalid_arg_max_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_dac_ndac_val(&mut i2c, false, 129).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn dac_ndac_val_min_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_NDAC_VAL, 0b1000_0001].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_dac_ndac_val(&mut i2c, true, 1).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_ndac_val_max_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_NDAC_VAL, 0x0].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_dac_ndac_val(&mut i2c, false, 128).unwrap();
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
        let err = page.set_dac_volume_control(&mut i2c, Channel::Left, -63.5).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_min_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_LEFT_VOLUME_CONTROL, 0b1000_0001].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_dac_volume_control(&mut i2c, Channel::Left, -63.5).unwrap();
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_min_invalid_arg_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_dac_volume_control(&mut i2c, Channel::Right, -64.0).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn dac_volume_control_db_max_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [DAC_RIGHT_VOLUME_CONTROL, 0b0011_0000].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_dac_volume_control(&mut i2c, Channel::Right, 24.0).unwrap();
        i2c.done();
    }
    #[test]
    fn dac_volume_control_db_max_invalid_arg_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_dac_volume_control(&mut i2c, Channel::Right, 24.5).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
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
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
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
    fn pll_d_value_lsb_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_D_VALUE_LSB, 0x1].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.set_pll_d_value_lsb(&mut i2c, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn pll_d_value_lsb_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_D_VALUE_LSB, 0x5].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_d_value_lsb(&mut i2c, 5).unwrap();
        i2c.done();
    }

    #[test]
    fn pll_d_value_msb_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_D_VALUE_MSB, 0x1].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.set_pll_d_value_msb(&mut i2c, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn pll_d_value_msb_invalid_arg_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_pll_d_value_msb(&mut i2c, 64).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn pll_d_value_msb_max_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_D_VALUE_MSB, 0x3f].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_d_value_msb(&mut i2c, 0x3f).unwrap();
        i2c.done();
    }

    #[test]
    fn pll_d_value_msb_min_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_D_VALUE_MSB, 0x0].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_d_value_msb(&mut i2c, 0x0).unwrap();
        i2c.done();
    }

    #[test]
    fn pll_j_value_i2c_err() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_J_VALUE, 0x1].to_vec()).with_error(ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        let err = page.set_pll_j_value(&mut i2c, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn pll_j_value_invalid_j_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_pll_j_value(&mut i2c, 0).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn pll_j_value_min_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_J_VALUE, 0x1].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_j_value(&mut i2c, 1).unwrap();
        i2c.done();
    }

    #[test]
    fn pll_j_value_max_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_J_VALUE, 0x3f].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_j_value(&mut i2c, 63).unwrap();
        i2c.done();
    }

    #[test]
    fn pll_p_and_r_values_invalid_pll_div_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_pll_p_and_r_values(&mut i2c, false, 9, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn pll_p_and_r_values_invalid_pll_i2c_err() {
        let mut i2c = I2cMock::new(&[
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()).with_error(ErrorKind::Other),
        ]);
        let mut page = Page0 {};
        let err = page.set_pll_p_and_r_values(&mut i2c, false, 2, 1).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
        i2c.done();
    }

    #[test]
    fn pll_p_and_r_values_invalid_pll_mul_err() {
        let mut i2c = I2cMock::new(&[]);
        let mut page = Page0 {};
        let err = page.set_pll_p_and_r_values(&mut i2c, false, 2, 0).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::InvalidArgument);
        i2c.done();
    }

    #[test]
    fn pll_p_and_r_values_ok() {
        let expectations = [
            I2cTransaction::write(ADDRESS, [PAGE_CONTROL_REGISTER, 0x0].to_vec()),
            I2cTransaction::write(ADDRESS, [PLL_P_AND_R_VALUES, 0x24].to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut page = Page0 {};
        page.set_pll_p_and_r_values(&mut i2c, false, 2, 4).unwrap();
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
        let err = page.set_software_reset(&mut i2c, NoopDelay).unwrap_err();
        assert_eq!(err, TLV320DAC3100Error::I2C(ErrorKind::Other));
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
        page.set_software_reset(&mut i2c, NoopDelay).unwrap();
        i2c.done();
    }
}