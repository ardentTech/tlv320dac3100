use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tlv320dac3100::driver::*;
use tlv320dac3100::registers::{CLKOUT_MUX, CLKOUT_M_VAL, CLOCK_GEN_MUXING, CODEC_INTERFACE_CONTROL_1, DAC_DATA_PATH_SETUP, DAC_DOSR_VAL_LSB, DAC_DOSR_VAL_MSB, DAC_MDAC_VAL, DAC_NDAC_VAL, HEADPHONE_DRIVERS, OT_FLAG, PAGE_CONTROL, PLL_D_VALUE_LSB, PLL_D_VALUE_MSB, PLL_J_VALUE, PLL_P_AND_R_VALUES, SOFTWARE_RESET};
use tlv320dac3100::typedefs::{CdivClkin, CodecClkin, CodecInterface, CodecInterfaceWordLength, HeadphoneOutputVoltage, LeftDataPath, PllClkin, RightDataPath, SoftStepping};

#[test]
fn get_clkout_m_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CLKOUT_M_VAL, 0b1111_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut powered: bool = false;
    let mut divider: u8 = 0;
    driver.get_clkout_m_val(&mut powered, &mut divider).unwrap();
    assert!(powered);
    assert_eq!(divider, 126);
    i2c.done();
}

#[test]
fn get_clkout_mux_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CLKOUT_MUX, 0b0000_0101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut cdiv_clkin = CdivClkin::Din;
    driver.get_clkout_mux(&mut cdiv_clkin).unwrap();
    assert_eq!(cdiv_clkin, CdivClkin::DacModClk);
    i2c.done();
}

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
fn get_codec_interface_control_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_INTERFACE_CONTROL_1, 0b1000_1100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut interface = CodecInterface::I2S;
    let mut word_length = CodecInterfaceWordLength::Word32Bits;
    let mut bclk_output = false;
    let mut wclk_output = false;
    driver.get_codec_interface_control_1(&mut interface, &mut word_length, &mut bclk_output, &mut wclk_output).unwrap();
    assert_eq!(interface, CodecInterface::RJF);
    assert_eq!(word_length, CodecInterfaceWordLength::Word16Bits);
    assert!(bclk_output);
    assert!(wclk_output);
    i2c.done();
}

#[test]
fn get_dac_mdac_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_MDAC_VAL, 0b01010000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut divider: u8 = 0;
    let mut powered: bool = true;
    driver.get_dac_mdac_val(&mut powered, &mut divider).unwrap();
    assert_eq!(powered, false);
    assert_eq!(divider, 80);
    i2c.done();
}

#[test]
fn get_dac_ndac_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_NDAC_VAL, 0b11010011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut divider: u8 = 0;
    let mut powered: bool = false;
    driver.get_dac_ndac_val(&mut powered, &mut divider).unwrap();
    assert_eq!(powered, true);
    assert_eq!(divider, 83);
    i2c.done();
}

#[test]
fn get_headphone_drivers_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_DRIVERS, 0b1101_0111),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut left_power = false;
    let mut right_power = false;
    let mut output_v = HeadphoneOutputVoltage::Common1_35V;
    let mut power_down_on_scd = false;
    let mut scd_detected = false;
    driver.get_headphone_drivers(&mut left_power, &mut right_power, &mut output_v, &mut power_down_on_scd, &mut scd_detected).unwrap();
    assert!(left_power);
    assert!(right_power);
    assert_eq!(output_v, HeadphoneOutputVoltage::Common1_65V);
    assert!(power_down_on_scd);
    assert!(scd_detected);
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
fn get_pll_d_value_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(PLL_D_VALUE_MSB, 0x97),
        i2c_page_set(0),
        i2c_reg_read(PLL_D_VALUE_LSB, 0x5b),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut d: u16 = 0;
    driver.get_pll_d_value(&mut d).unwrap();
    assert_eq!(d, 0x175b);
    i2c.done();
}

#[test]
fn get_pll_j_value_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(PLL_J_VALUE, 0xeb),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    let mut j: u8 = 0;
    driver.get_pll_j_value(&mut j).unwrap();
    assert_eq!(j, 0x2b);
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
fn set_clkout_m_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_clkout_m_val(false, 0x0).unwrap_err();
    i2c.done();
}

#[test]
fn set_clkout_m_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(CLKOUT_M_VAL, 0b1000_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_clkout_m_val(true, 0x2).unwrap();
    i2c.done();
}

#[test]
fn set_clkout_mux_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CLKOUT_MUX, 0x1),
        i2c_page_set(0),
        i2c_reg_write(CLKOUT_MUX, 0x5),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_clkout_mux(CdivClkin::DacModClk).unwrap();
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
fn set_codec_interface_control_1() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_INTERFACE_CONTROL_1, 0b1111_1100),
        i2c_page_set(0),
        i2c_reg_write(CODEC_INTERFACE_CONTROL_1, 0b0000_0000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_codec_interface_control_1(CodecInterface::I2S, CodecInterfaceWordLength::Word16Bits, false, false).unwrap();
    i2c.done();
}

#[test]
fn set_dac_dosr_val_invalid_osr_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_dosr_val(1521u16).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_dosr_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_DOSR_VAL_MSB, 0x1),
        i2c_page_set(0),
        i2c_reg_write(DAC_DOSR_VAL_LSB, 0xf1),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_dosr_val(497u16).unwrap();
    i2c.done();
}

#[test]
fn set_dac_mdac_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_mdac_val(true, 0x0).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_mdac_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_MDAC_VAL, 0xd0),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_mdac_val(true, 0x50).unwrap();
    i2c.done();
}

#[test]
fn set_dac_ndac_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_ndac_val(true, 0xff).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_ndac_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_NDAC_VAL, 0x82),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_dac_ndac_val(true, 0x2).unwrap();
    i2c.done();
}

#[test]
fn set_pll_d_value_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(PLL_D_VALUE_MSB, 0x3f),
        i2c_page_set(0),
        i2c_reg_write(PLL_D_VALUE_LSB, 0x8a),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_pll_d_value(0x3f8a).unwrap();
    i2c.done();
}

#[test]
fn set_headphone_drivers_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_DRIVERS, 0b1101_0111),
        i2c_page_set(1),
        i2c_reg_write(HEADPHONE_DRIVERS, 0b0000_0101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_headphone_drivers(false, false, HeadphoneOutputVoltage::Common1_35V, false).unwrap();
    i2c.done();
}
//     let expectations = [
//         i2c_page_set(1),
//         i2c_reg_read(HEADPHONE_DRIVERS, 0b1110_0100),
//         i2c_page_set(1),
//         i2c_reg_write(HEADPHONE_DRIVERS, 0b0100_1110), // 0x78, 78
//     ];
//     let mut i2c = I2cMock::new(&expectations);
//     let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
//     driver.set_headphone_drivers(false, true, HeadphoneOutputVoltage::Common1_5V, true).unwrap();
//     i2c.done();
// }

#[test]
fn set_pll_j_value_invalid_j_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_pll_j_value(0xa2).unwrap_err();
    i2c.done();
}

#[test]
fn set_pll_j_value_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(PLL_J_VALUE, 0xaf),
        i2c_page_set(0),
        i2c_reg_write(PLL_J_VALUE, 0xa9),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_pll_j_value(0x29).unwrap();
    i2c.done();
}

#[test]
fn set_pll_p_and_r_values_invalid_p_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_pll_p_and_r_values(true, 9, 2).unwrap_err();
    i2c.done();
}

#[test]
fn set_pll_p_and_r_values_invalid_r_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(NoopDelay, &mut i2c);
    driver.set_pll_p_and_r_values(true, 3, 17).unwrap_err();
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
    driver.set_pll_p_and_r_values(true, 3, 2).unwrap();
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