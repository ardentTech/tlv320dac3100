use embedded_hal as hal;
use hal::i2c::I2c;
use crate::error::TLV320DAC3100Error;

const I2C_DEVICE_ADDRESS: u8 = 0x18;
const PAGE_CONTROL_REGISTER: u8 = 0x00;

// page 0 registers
const SOFTWARE_RESET: u8 = 0x01;
const OT_FLAG: u8 = 0x03;
const CLOCK_GEN_MUXING: u8 = 0x04;
const PLL_P_AND_R_VALUES: u8 = 0x05;
const PLL_J_VALUE: u8 = 0x06;
const PLL_D_VALUE_MSB: u8 = 0x07;
const PLL_D_VALUE_LSB: u8 = 0x08;
const DAC_NDAC_VAL: u8 = 0x0b;
const DAC_DOSR_VAL_MSB: u8 = 0x0d;
const DAC_DOSR_VAL_LSB: u8 = 0x0e;
//const CODEC_SECONDARY_INTERFACE_CONTROL_1: u8 = 0x1f;
const DAC_DATA_PATH_SETUP: u8 = 0x3f;
const DAC_VOLUME_CONTROL: u8 = 0x40;
const DAC_LEFT_VOLUME_CONTROL: u8 = 0x41;
const DAC_RIGHT_VOLUME_CONTROL: u8 = 0x42;
const VOL_MICDET_PIN_GAIN: u8 = 0x75;

// page 1 registers
const HEADPHONE_DRIVERS: u8 = 0x1f;
const DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING: u8 = 0x23;
const HPL_DRIVER: u8 = 0x28;

#[derive(Debug, PartialEq)]
enum PllClkin {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    Din = 0x3
}
impl TryFrom<u8> for PllClkin {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(PllClkin::Mclk),
            0x1 => Ok(PllClkin::Bclk),
            0x2 => Ok(PllClkin::Gpio1),
            0x3 => Ok(PllClkin::Din),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
enum CodecClkin {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    PllClk = 0x3,
}
impl TryFrom<u8> for CodecClkin {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(CodecClkin::Mclk),
            0x1 => Ok(CodecClkin::Bclk),
            0x2 => Ok(CodecClkin::Gpio1),
            0x3 => Ok(CodecClkin::PllClk),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
enum HeadphoneCommon {
    C135V = 0x00,
    C15V = 0x01,
    C165V = 0x02,
    C18V = 0x03,
}

#[derive(Debug, PartialEq)]
enum LeftDataPath {
    Off = 0x0,
    Left = 0x1,
    Right = 0x2,
    Both = 0x3
}

#[derive(Debug, PartialEq)]
enum RightDataPath {
    Off = 0x0,
    Right = 0x1,
    Left = 0x2,
    Both = 0x3
}

#[derive(Debug, PartialEq)]
enum VolumeControl {
    IndependentChannels = 0x0,
    LeftToRight = 0x1,
    RightToLeft = 0x2,
}

#[derive(Debug, PartialEq)]
enum VolumeControlSoftStepping {
    OneStepPerPeriod = 0x0,
    OneStepPerTwoPeriods = 0x1,
    Disabled = 0x2
}

pub struct TLV320DAC3100<I2C, D> {
    delay: D,
    i2c: I2C,
}

// TODO compiler warning about bounds
type TLV320Result<I2C: I2c> = Result<(), TLV320DAC3100Error<I2C::Error>>;

impl<I2C: I2c, D: hal::delay::DelayNs> TLV320DAC3100<I2C, D> {
    pub fn new(delay: D, i2c: I2C) -> Self {
        TLV320DAC3100 { delay, i2c }
    }

    pub fn get_clock_gen_muxing(&mut self, pll_clkin: &mut PllClkin, codec_clkin: &mut CodecClkin) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLOCK_GEN_MUXING)?;
        *pll_clkin = ((reg_val & 0xc) >> 2).try_into().unwrap();
        *codec_clkin = (reg_val & 0x3).try_into().unwrap();
        Ok(())
    }

    pub fn get_ot_flag(&mut self, ot: &mut bool) -> TLV320Result<I2C> {
        *ot = self.read_reg(0, OT_FLAG)? >> 1 == 0;
        Ok(())
    }

    pub fn get_pll_d_value(&mut self, d: &mut u16) -> TLV320Result<I2C> {
        let msb = self.read_reg(0, PLL_D_VALUE_MSB)?;
        let lsb = self.read_reg(0, PLL_D_VALUE_LSB)?;
        *d = (msb as u16) << 8 | lsb as u16;
        Ok(())
    }

    pub fn get_pll_j_value(&mut self, j: &mut u8) -> TLV320Result<I2C> {
        *j = self.read_reg(0, PLL_J_VALUE)?;
        Ok(())
    }

    pub fn get_pll_p_and_r_values(&mut self, powered: &mut bool, p: &mut u8, r: &mut u8) -> TLV320Result<I2C> {
        let raw = self.read_reg(0, PLL_P_AND_R_VALUES)?;
        *powered = raw >> 7 == 0x1;
        *p = (0x70 & raw) >> 4;
        *r = 0xf & raw;
        Ok(())
    }

    pub fn get_vol_micdet_pin_gain(&mut self, gain: &mut u8) -> TLV320Result<I2C> {
        *gain = self.read_reg(0, VOL_MICDET_PIN_GAIN)?;
        Ok(())
    }

    fn read_reg(&mut self, page: u8, reg: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.select_page(page)?;
        let mut buf = [0u8];
        self.i2c.write_read(I2C_DEVICE_ADDRESS, &[reg], &mut buf).map_err(TLV320DAC3100Error::I2C)?;
        Ok(buf[0])
    }

    fn select_page(&mut self, page: u8) -> TLV320Result<I2C> {
        Ok(self.i2c.write(I2C_DEVICE_ADDRESS, &[PAGE_CONTROL_REGISTER, page]).map_err(TLV320DAC3100Error::I2C)?)
    }

    pub fn set_clock_gen_muxing(&mut self, pll_clkin: PllClkin, codec_clkin: CodecClkin) -> TLV320Result<I2C> {
        let mut reg_val = (pll_clkin as u8) << 2;
        reg_val |= codec_clkin as u8;
        self.write_reg(0, CLOCK_GEN_MUXING, reg_val)
    }

    // pub fn set_codec_secondary_interface_control_1(&mut self, bclk_from_gpio1: bool, wclk_from_gpio1: bool, din_from_gpio1: bool) -> TLV320Result<I2C> {
    //     let mut reg_val: u8 = (bclk_from_gpio1 as u8) << 5;
    //     reg_val |= (wclk_from_gpio1 as u8) << 2;
    //     reg_val |= din_from_gpio1 as u8;
    //     self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_1, reg_val)
    // }

    pub fn set_dac_data_path_setup(
        &mut self,
        left_powered: bool,
        right_powered: bool,
        left_data_path: LeftDataPath,
        right_data_path: RightDataPath,
        soft_stepping: VolumeControlSoftStepping
    ) -> TLV320Result<I2C> {
        let mut reg_val: u8 = (left_powered as u8) << 7;
        reg_val |= (right_powered as u8) << 6;
        reg_val |= (left_data_path as u8) << 4;
        reg_val |= (right_data_path as u8) << 2;
        reg_val |= soft_stepping as u8;
        self.write_reg(0, DAC_DATA_PATH_SETUP, reg_val)
    }

    pub fn set_dac_dosr_val(&mut self, osr: u16) -> TLV320Result<I2C> {
        // TODO validate
        let msb = (osr >> 14) as u8;
        let lsb: u8 = (0x00ff & osr) as u8;
        self.write_reg(0, DAC_DOSR_VAL_MSB, msb)?;
        self.write_reg(0, DAC_DOSR_VAL_LSB, lsb)
    }

    pub fn set_dac_ndac_val(&mut self, divider: u8) -> TLV320Result<I2C> {
        if divider == 0 || divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }
        self.write_reg(0, DAC_NDAC_VAL, divider)
    }

    // TODO unit test
    pub fn set_dac_volume_control(
        &mut self,
        left_muted: bool,
        right_muted: bool,
        control: VolumeControl
    ) -> TLV320Result<I2C> {
        let mut reg_val = (left_muted as u8) << 3;
        reg_val |= (right_muted as u8) << 2;
        reg_val |= control as u8;
        self.write_reg(0, DAC_VOLUME_CONTROL, reg_val)
    }

    // TODO unit test
    pub fn set_headphone_drivers(
        &mut self,
        left_powered: bool,
        right_powered: bool,
        common: HeadphoneCommon,
        power_down_on_scd: bool
    ) -> TLV320Result<I2C> {
        let mut reg_val = (left_powered as u8) << 7;
        reg_val |= (right_powered as u8) << 6;
        reg_val |= (common as u8) << 3;
        reg_val |= (power_down_on_scd as u8) << 1;
        self.write_reg(1, HEADPHONE_DRIVERS, reg_val)
    }

    pub fn set_pll_d_value(&mut self, d: u16) -> TLV320Result<I2C> {
        let msb = (d >> 8) as u8;
        if msb > 63 { return Err(TLV320DAC3100Error::InvalidArgument) };
        self.write_reg(0, PLL_D_VALUE_MSB, msb)?;
        self.write_reg(0, PLL_D_VALUE_LSB, (0xff & d) as u8)
    }

    pub fn set_pll_j_value(&mut self, j: u8) -> TLV320Result<I2C> {
        if j == 0 || j > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }
        self.write_reg(0, PLL_J_VALUE, j)
    }
    pub fn set_pll_p_and_r_values(&mut self, powered: bool, p: u8, r: u8) -> TLV320Result<I2C> {
        if p == 0 || p > 8 { return Err(TLV320DAC3100Error::InvalidArgument) }
        if r == 0 || r > 16 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= (if p == 0 { 0x0 } else { p }) << 4;
        reg_val |= if r == 16 { 0x0 } else { r };
        self.write_reg(0, PLL_P_AND_R_VALUES, reg_val)
    }

    pub fn set_software_reset(&mut self, reset: bool) -> TLV320Result<I2C> {
        self.write_reg(0, SOFTWARE_RESET, reset as u8)
    }

    fn write_reg(&mut self, page: u8, reg_addr: u8, reg_val: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.select_page(page)?;
        Ok(self.i2c.write(I2C_DEVICE_ADDRESS, &[reg_addr, reg_val]).map_err(TLV320DAC3100Error::I2C)?)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use crate::driver::*;

    #[test]
    fn get_clock_gen_muxing_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(CLOCK_GEN_MUXING, 0b0000_1010),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut pll_clkin = PllClkin::Din;
        let mut codec_clkin = CodecClkin::Mclk;
        driver.get_clock_gen_muxing(&mut pll_clkin, &mut codec_clkin).unwrap();
        assert_eq!(pll_clkin, PllClkin::Gpio1);
        assert_eq!(codec_clkin, CodecClkin::Gpio1);
        i2c.done();
    }

    #[test]
    fn get_ot_flag_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(OT_FLAG, 0x1),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut ot_flag: bool = false;
        driver.get_ot_flag(&mut ot_flag).unwrap();
        assert_eq!(ot_flag, true);
        i2c.done();
    }

    #[test]
    fn get_pll_d_value_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(PLL_D_VALUE_MSB, 0x3a),
            i2c_page_set(0),
            i2c_reg_read(PLL_D_VALUE_LSB, 0x5b),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut d: u16 = 0;
        driver.get_pll_d_value(&mut d).unwrap();
        assert_eq!(d, 0x3a5b);
        i2c.done();
    }

    #[test]
    fn get_pll_j_value_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(PLL_J_VALUE, 0x39),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut j: u8 = 0;
        driver.get_pll_j_value(&mut j).unwrap();
        assert_eq!(j, 0x39);
        i2c.done();
    }

    #[test]
    fn get_pll_p_and_r_values_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(PLL_P_AND_R_VALUES, 0xb2),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut powered = false;
        let mut p: u8 = 0x7;
        let mut r: u8 = 0x9;
        driver.get_pll_p_and_r_values(&mut powered, &mut p, &mut r).unwrap();
        assert_eq!(powered, true);
        assert_eq!(p, 0x3);
        assert_eq!(r, 0x2);
        i2c.done();
    }

    #[test]
    fn get_vol_micdet_pin_gain_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_read(VOL_MICDET_PIN_GAIN, 0x3f),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        let mut gain: u8 = 0;
        driver.get_vol_micdet_pin_gain(&mut gain).unwrap();
        assert_eq!(gain, 0x3f);
        i2c.done();
    }

    #[test]
    fn set_pll_d_value_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_write(PLL_D_VALUE_MSB, 0x3f),
            i2c_page_set(0),
            i2c_reg_write(PLL_D_VALUE_LSB, 0x8a), // 138
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        driver.set_pll_d_value(0x3f8a).unwrap(); // 60042
        i2c.done();
    }

    #[test]
    fn set_pll_p_and_r_values_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_write(PLL_P_AND_R_VALUES, 0xb2),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        driver.set_pll_p_and_r_values(true, 3, 2).unwrap(); // 60042
        i2c.done();
    }

    #[test]
    fn set_pll_j_value_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_write(PLL_J_VALUE, 0x31),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        driver.set_pll_j_value(0x31).unwrap();
        i2c.done();
    }

    #[test]
    fn set_clock_gen_muxing_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_write(CLOCK_GEN_MUXING, 0x4),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        driver.set_clock_gen_muxing(PllClkin::Bclk, CodecClkin::Mclk).unwrap();
        i2c.done();
    }

    #[test]
    fn set_software_reset_ok() {
        let expectations = [
            i2c_page_set(0),
            i2c_reg_write(SOFTWARE_RESET, 0x1),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
        driver.set_software_reset(true).unwrap();
        i2c.done();
    }

    fn i2c_page_set(n: u8) -> I2cTransaction {
        i2c_reg_write(PAGE_CONTROL_REGISTER, n)
    }

    fn i2c_reg_read(reg: u8, payload: u8) -> I2cTransaction {
        I2cTransaction::write_read(I2C_DEVICE_ADDRESS, [reg].to_vec(), [payload].to_vec())
    }

    fn i2c_reg_write(reg: u8, payload: u8) -> I2cTransaction {
        I2cTransaction::write(I2C_DEVICE_ADDRESS, [reg, payload].to_vec())
    }

}