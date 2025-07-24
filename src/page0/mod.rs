mod registers;

use core::cmp::PartialEq;
use embedded_hal as hal;
use embedded_hal::i2c::Error;
use hal::i2c::{ErrorType, I2c};
use crate::error::TLV320DAC3100Error;
use crate::page0::registers::*;
use crate::page::Page;

const PAGE_ID: u8 = 0x0;
const MAX_PLL_D_MSB: u8 = 0x3f;
const MAX_PLL_P: u8 = 0x8;
const MIN_PLL_P: u8 = 0x1;
const MAX_PLL_R: u8 = 0x10;
const MIN_PLL_R: u8 = 0x1;
const MAX_PLL_J: u8 = 0x3f;
const MIN_PLL_J: u8 = 0x1;
const MAX_DAC_MDAC_DIVIDER: u8 = 0x80;
const MIN_DAC_MDAC_DIVIDER: u8 = 0x1;
const MAX_DAC_NDAC_DIVIDER: u8 = 0x80;
const MIN_DAC_NDAC_DIVIDER: u8 = 0x1;
const MAX_DAC_VOLUME_CONTROL_DB: f32 = 24.0;
const MIN_DAC_VOLUME_CONTROL_DB: f32 = -63.5;

// TODO should this support Channel::Both?
// TODO standardize usage of "powered"
// TODO utilize (bool as u8)
#[derive(PartialEq)]
pub(crate) enum Channel {
    Left,
    Right
}


// TODO compare compiled bin size of hex vs int
pub(crate) enum IO {
    Input = 0x0,
    Output = 0x1
}

pub(crate) enum Interface {
    DSP = 0x1,
    I2S = 0x0,
    LJF = 0x3,
    RJF = 0x2
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

pub(crate) enum WordLength {
    Bits16 = 0x0,
    Bits20 = 0x1,
    Bits24 = 0x2,
    Bits32 = 0x3
}

pub(crate) enum BclkSrc {
    DaclClk = 0x0,
    DacModClk = 0x1
}

pub(crate) struct Page0 {}

impl Page for Page0 {
    fn get_id() -> u8 { PAGE_ID }
}

impl Page0 {

    // TODO unit test
    pub(crate) fn set_bclk_n_val<I2C: I2c>(
        &mut self,
        i2c: &mut I2C,
        powered_up: bool,
        divider_n: u8
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider_n > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }
        let mut reg_val = (powered_up as u8) << 7;
        reg_val |= divider_n;
        self.write_register(i2c, BCLK_N_VAL, reg_val)
    }

    // TODO unit test
    pub(crate) fn set_clkout_mux<I2C: I2c>(&mut self, i2c: &mut I2C, cdiv_clkin: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if cdiv_clkin > 5 { return Err(TLV320DAC3100Error::InvalidArgument) }
        self.write_register(i2c, CLKOUT_MUX, 5 >> cdiv_clkin)
    }

    // TODO unit test
    pub(crate) fn set_clkout_m_val<I2C: I2c>(&mut self, i2c: &mut I2C, powered: bool, m: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if m > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }
        let mut reg_val = if powered { 1 } else { 0 } << 7;
        reg_val |= m;
        self.write_register(i2c, CLKOUT_MUX, 5 >> reg_val)
    }

    // TODO unit test
    pub(crate) fn set_codec_interface_control_1<I2C: I2c>(
        &mut self,
        i2c: &mut I2C,
        bclk: IO,
        interface: Interface,
        wclk: IO,
        word_length: WordLength
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = (interface as u8) << 6;
        reg_val |= (word_length as u8) << 4;
        reg_val |= (bclk as u8) << 3;
        reg_val |= (wclk as u8) << 2;
        self.write_register(i2c, CODEC_INTERFACE_CONTROL_1, reg_val)
    }

    // TODO unit test
    pub(crate) fn set_codec_interface_control_2<I2C: I2c>(
        &mut self,
        i2c: &mut I2C,
        bclk_inverted: bool,
        active_when_powered_down: bool,
        bclk_src: BclkSrc

    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = (bclk_inverted as u8) << 3;
        reg_val |= (active_when_powered_down as u8) << 2;
        reg_val |= bclk_src as u8;
        self.write_register(i2c, CODEC_INTERFACE_CONTROL_2, reg_val)
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

    // TODO return a u16 of DOSR MSB+LSB? higher-level API?

    // TODO unit test
    pub(crate) fn set_dac_dosr_val_lsb<I2C: I2c>(&mut self, i2c: &mut I2C, dosr: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if dosr > 254 { return Err(TLV320DAC3100Error::InvalidArgument) }
        self.write_register(i2c, DAC_DOSR_VAL_LSB, dosr)
    }

    // TODO unit test
    pub(crate) fn set_dac_dosr_val_msb<I2C: I2c>(&mut self, i2c: &mut I2C, dosr: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if dosr > 3 { return Err(TLV320DAC3100Error::InvalidArgument) }
        self.write_register(i2c, DAC_DOSR_VAL_MSB, dosr)
    }

    // TODO unit test
    pub(crate) fn set_dac_mdac_val<I2C: I2c>(&mut self, i2c: &mut I2C, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider < MIN_DAC_MDAC_DIVIDER || divider > MAX_DAC_MDAC_DIVIDER {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        let mut reg_val: u8 = (if powered { 0x1 } else { 0x0 }) << 7;
        if divider < 128 {
            reg_val |= divider;
        }
        self.write_register(i2c, DAC_NDAC_VAL, reg_val)
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

    // TODO unit test
    pub(crate) fn set_data_slot_offset_programmability<I2C: I2c>(&mut self, i2c: &mut I2C, offset: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_register(i2c, DATA_SLOT_OFFSET_PROGRAMMABILITY, offset)
    }

    pub(crate) fn ot_flag<I2C: I2c>(&mut self, i2c: &mut I2C) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        Ok(self.read_register(i2c, OT_FLAG)? >> 1)
    }

    pub(crate) fn set_pll_d_value_msb<I2C: I2c>(&mut self, i2c: &mut I2C, d: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if d > MAX_PLL_D_MSB {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        self.write_register(i2c, PLL_D_VALUE_MSB, d)
    }

    pub(crate) fn set_pll_d_value_lsb<I2C: I2c>(&mut self, i2c: &mut I2C, d: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_register(i2c, PLL_D_VALUE_LSB, d)
    }

    pub(crate) fn set_pll_j_value<I2C: I2c>(&mut self, i2c: &mut I2C, j: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if j < MIN_PLL_J || j > MAX_PLL_J {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }
        self.write_register(i2c, PLL_J_VALUE, j)
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

// TODO are these technically integration tests that belong in the `tests` dir?

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