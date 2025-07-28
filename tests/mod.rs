use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tlv320dac3100::driver::*;
use tlv320dac3100::registers::{CLOCK_GEN_MUXING, DAC_DATA_PATH_SETUP, OT_FLAG, PAGE_CONTROL, SOFTWARE_RESET};
use tlv320dac3100::typedefs::{CodecClkin, LeftDataPath, PllClkin, RightDataPath, SoftStepping};

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
fn get_ot_flag_false_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(OT_FLAG, 0b10),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut ot_flag: bool = true;
    driver.get_ot_flag(&mut ot_flag).unwrap();
    assert_eq!(ot_flag, false);
    i2c.done();
}

#[test]
fn get_ot_flag_true_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(OT_FLAG, 0b00),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut ot_flag: bool = false;
    driver.get_ot_flag(&mut ot_flag).unwrap();
    assert_eq!(ot_flag, true);
    i2c.done();
}

#[test]
fn set_class_d_speaker_amplifier_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_DATA_PATH_SETUP, 0xd4),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_data_path_setup(true, true, LeftDataPath::Left, RightDataPath::Right, SoftStepping::OneStepPerPeriod).unwrap();
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
    i2c_reg_write(PAGE_CONTROL, n)
}

fn i2c_reg_read(reg: u8, payload: u8) -> I2cTransaction {
    I2cTransaction::write_read(I2C_DEVICE_ADDRESS, [reg].to_vec(), [payload].to_vec())
}

fn i2c_reg_write(reg: u8, payload: u8) -> I2cTransaction {
    I2cTransaction::write(I2C_DEVICE_ADDRESS, [reg, payload].to_vec())
}