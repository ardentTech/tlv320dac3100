use embedded_hal as hal;
use hal::i2c::I2c;
use crate::error::TLV320DAC3100Error;

const I2C_DEVICE_ADDRESS: u8 = 0x18;
const PAGE_CONTROL_REGISTER: u8 = 0x00;
const SOFTWARE_RESET: u8 = 0x01;
const OT_FLAG: u8 = 0x03;
const CLOCK_GEN_MUXING: u8 = 0x04;
const VOL_MICDET_PIN_GAIN: u8 = 0x75;

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

pub struct TLV320DAC3100<I2C, D> {
    delay: D,
    i2c: I2C,
}

type TLV320Result<I2C: I2c> = Result<(), TLV320DAC3100Error<I2C::Error>>;

impl<I2C: I2c, D: hal::delay::DelayNs> TLV320DAC3100<I2C, D> {
    pub fn new(delay: D, i2c: I2C) -> Self {
        TLV320DAC3100 { delay, i2c }
    }

    pub fn get_ot_flag(&mut self, ot: &mut bool) -> TLV320Result<I2C> {
        *ot = self.read_reg(0, OT_FLAG)? >> 1 == 0;
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

    pub fn get_clock_gen_muxing(&mut self, pll_clkin: &mut PllClkin, codec_clkin: &mut CodecClkin) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLOCK_GEN_MUXING)?;
        *pll_clkin = ((reg_val & 0xc) >> 2).try_into().unwrap();
        *codec_clkin = (reg_val & 0x3).try_into().unwrap();
        Ok(())
    }

    pub fn set_clock_gen_muxing(&mut self, pll_clkin: PllClkin, codec_clkin: CodecClkin) -> TLV320Result<I2C> {
        let mut reg_val = (pll_clkin as u8) << 2;
        reg_val |= codec_clkin as u8;
        self.write_reg(0, CLOCK_GEN_MUXING, reg_val)
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
    use crate::driver::{CodecClkin, PllClkin, CLOCK_GEN_MUXING, I2C_DEVICE_ADDRESS, OT_FLAG, PAGE_CONTROL_REGISTER, SOFTWARE_RESET, TLV320DAC3100, VOL_MICDET_PIN_GAIN};

    #[test]
    fn get_clock_gen_muxing_ok() {
        let expectations = [
            switch_to_page(0),
            i2c_reg_read(CLOCK_GEN_MUXING, 0b0000_1010),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new( NoopDelay, &mut i2c);
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
            switch_to_page(0),
            i2c_reg_read(OT_FLAG, 0x1),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new( NoopDelay, &mut i2c);
        let mut ot_flag: bool = false;
        driver.get_ot_flag(&mut ot_flag).unwrap();
        assert_eq!(ot_flag, true);
        i2c.done();
    }
    #[test]
    fn get_vol_micdet_pin_gain_ok() {
        let expectations = [
            switch_to_page(0),
            i2c_reg_read(VOL_MICDET_PIN_GAIN, 0x3f),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new( NoopDelay, &mut i2c);
        let mut gain: u8 = 0;
        driver.get_vol_micdet_pin_gain(&mut gain).unwrap();
        assert_eq!(gain, 0x3f);
        i2c.done();
    }

    #[test]
    fn set_clock_gen_muxing_ok() {
        let expectations = [
            switch_to_page(0),
            i2c_reg_write(CLOCK_GEN_MUXING, 0x4),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new( NoopDelay, &mut i2c);
        driver.set_clock_gen_muxing(PllClkin::Bclk, CodecClkin::Mclk).unwrap();
        i2c.done();
    }

    #[test]
    fn set_software_reset_ok() {
        let expectations = [
            switch_to_page(0),
            i2c_reg_write(SOFTWARE_RESET, 0x1),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut driver = TLV320DAC3100::new( NoopDelay, &mut i2c);
        driver.set_software_reset(true).unwrap();
        i2c.done();
    }

    fn switch_to_page(n: u8) -> I2cTransaction {
        i2c_reg_write(PAGE_CONTROL_REGISTER, 0x0)
    }

    fn i2c_reg_read(reg: u8, payload: u8) -> I2cTransaction {
        I2cTransaction::write_read(I2C_DEVICE_ADDRESS, [reg].to_vec(), [payload].to_vec())
    }

    fn i2c_reg_write(reg: u8, payload: u8) -> I2cTransaction {
        I2cTransaction::write(I2C_DEVICE_ADDRESS, [reg, payload].to_vec())
    }

}