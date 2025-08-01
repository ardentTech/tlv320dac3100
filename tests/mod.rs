use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tlv320dac3100::driver::*;
use tlv320dac3100::registers::*;
use tlv320dac3100::typedefs::*;

#[test]
fn get_class_d_spk_driver_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CLASS_D_SPK_DRIVER, 0b0000_1001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut gain = OutputStage::Gain6dB;
    let mut muted = false;
    let mut gains_applied = false;
    driver.get_class_d_spk_driver(&mut gain, &mut muted, &mut gains_applied).unwrap();
    assert_eq!(gain, OutputStage::Gain12dB);
    assert!(muted);
    assert!(gains_applied);
    i2c.done();
}

#[test]
fn get_clkout_m_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CLKOUT_M_VAL, 0b1111_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
fn get_dac_interrupt_flags() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_INTERRUPT_FLAGS_STICKY_BITS, 0b1111_1100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_scd = false;
    let mut right_scd = false;
    let mut headset_button_pressed = false;
    let mut headset_insert_removal_detected = false;
    let mut left_dac_signal_above_drc = false;
    let mut right_dac_signal_above_drc = false;
    driver.get_dac_interrupt_flags(&mut left_scd, &mut right_scd, &mut headset_button_pressed, &mut headset_insert_removal_detected, &mut left_dac_signal_above_drc, &mut right_dac_signal_above_drc).unwrap();
    assert!(left_scd);
    assert!(right_scd);
    assert!(headset_button_pressed);
    assert!(headset_insert_removal_detected);
    assert!(left_dac_signal_above_drc);
    assert!(left_dac_signal_above_drc);
    i2c.done();
}

#[test]
fn get_dac_mdac_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_MDAC_VAL, 0b01010000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut divider: u8 = 0;
    let mut powered: bool = false;
    driver.get_dac_ndac_val(&mut powered, &mut divider).unwrap();
    assert_eq!(powered, true);
    assert_eq!(divider, 83);
    i2c.done();
}

#[test]
fn get_dac_processing_block_selection_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_PROCESSING_BLOCK_SECTION, 0b1010_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut prb: u8 = 0;
    driver.get_dac_processing_block_selection(&mut prb).unwrap();
    assert_eq!(prb, 11);
    i2c.done();
}

#[test]
fn get_headphone_drivers_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_DRIVERS, 0b1101_0111),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_power = false;
    let mut right_power = false;
    let mut output_v = HpOutputVoltage::Common1_35V;
    let mut power_down_on_scd = false;
    let mut scd_detected = false;
    driver.get_headphone_drivers(&mut left_power, &mut right_power, &mut output_v, &mut power_down_on_scd, &mut scd_detected).unwrap();
    assert!(left_power);
    assert!(right_power);
    assert_eq!(output_v, HpOutputVoltage::Common1_65V);
    assert!(power_down_on_scd);
    assert!(scd_detected);
    i2c.done();
}

#[test]
fn get_headset_detection_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(HEADSET_DETECTION, 0b1011_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut enabled: bool = false;
    let mut detected = HeadsetDetected::None;
    let mut debounce = HeadsetDetectionDebounce::Debounce16ms;
    let mut button_debounce = HeadsetButtonPressDebounce::Debounce0ms;
    driver.get_headset_detection(&mut enabled, &mut detected, &mut debounce, &mut button_debounce).unwrap();
    assert_eq!(enabled, true);
    assert_eq!(detected, HeadsetDetected::WithoutMic);
    assert_eq!(debounce, HeadsetDetectionDebounce::Debounce512ms);
    assert_eq!(button_debounce, HeadsetButtonPressDebounce::Debounce16ms);
    i2c.done();
}

#[test]
fn get_ot_flag_false_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(OT_FLAG, 0b10),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
fn get_vol_micdet_pin_sar_adc_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(VOL_MICDET_PIN_SAR_ADC, 0b1110_0101)
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut pin_control = false;
    let mut use_mclk = false;
    let mut hysteresis = VolumeControlHysteresis::HysteresisNone;
    let mut throughput = VolumeControlThroughput::Rate2kHz;
    driver.get_vol_micdet_pin_sar_adc(&mut pin_control, &mut use_mclk, &mut hysteresis, &mut throughput).unwrap();
    assert!(pin_control);
    assert!(use_mclk);
    assert_eq!(hysteresis, VolumeControlHysteresis::Hysteresis2Bits);
    assert_eq!(throughput, VolumeControlThroughput::Rate500Hz);
    i2c.done();
}

#[test]
fn set_class_d_speaker_amplifier_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(CLASS_D_SPEAKER_AMPLIFIER, 0b0000_0110),
        i2c_page_set(1),
        i2c_reg_write(CLASS_D_SPEAKER_AMPLIFIER, 0b1000_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_class_d_spk_amp(true).unwrap();
    i2c.done();
}

#[test]
fn set_class_d_spk_driver_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(CLASS_D_SPK_DRIVER, 0b0000_0000),
        i2c_page_set(1),
        i2c_reg_write(CLASS_D_SPK_DRIVER, 0b0001_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_class_d_spk_driver(OutputStage::Gain18dB, false).unwrap();
    i2c.done();
}

#[test]
fn set_clkout_m_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_codec_interface_control_1(CodecInterface::I2S, CodecInterfaceWordLength::Word16Bits, false, false).unwrap();
    i2c.done();
}

#[test]
fn set_dac_data_path_setup_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_DATA_PATH_SETUP, 0b1000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_data_path_setup(true, false, LeftDataPath::Off, RightDataPath::Right, SoftStepping::OneStepPerPeriod).unwrap();
    i2c.done();
}

#[test]
fn set_dac_dosr_val_invalid_osr_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_dosr_val(497u16).unwrap();
    i2c.done();
}

#[test]
fn set_dac_l_and_dac_r_output_mixer_routing_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING, 0b1101_0111),
        i2c_page_set(1),
        i2c_reg_write(DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING, 0b0110_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_l_and_dac_r_output_mixer_routing(DacLeftOutputMixerRouting::LeftChannelMixerAmplifier, true, false, DacRightOutputMixerRouting::None, false, true).unwrap();
    i2c.done();
}

#[test]
fn set_dac_left_volume_control_invalid_db_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_left_volume_control(-64.0).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_left_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_LEFT_VOLUME_CONTROL, 46u8),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_left_volume_control(23.25).unwrap();
    i2c.done();
}

#[test]
fn set_dac_mdac_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_mdac_val(true, 0x50).unwrap();
    i2c.done();
}

#[test]
fn set_dac_ndac_val_invalid_divider_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_ndac_val(true, 0x2).unwrap();
    i2c.done();
}

#[test]
fn set_dac_processing_block_selection_invalid_prb_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_processing_block_selection(29).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_processing_block_selection_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_PROCESSING_BLOCK_SECTION, 0b1010_0000),
        i2c_page_set(0),
        i2c_reg_write(DAC_PROCESSING_BLOCK_SECTION, 0b1010_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_processing_block_selection(11).unwrap();
    i2c.done();
}

#[test]
fn set_dac_right_volume_control_invalid_db_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_right_volume_control(25.0).unwrap_err();
    i2c.done();
}

#[test]
fn set_dac_right_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DAC_RIGHT_VOLUME_CONTROL, 22u8),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_right_volume_control(11.0).unwrap();
    i2c.done();
}

#[test]
fn set_dac_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_VOLUME_CONTROL, 0b1110_1100),
        i2c_page_set(0),
        i2c_reg_write(DAC_VOLUME_CONTROL, 0b1110_0000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_dac_volume_control(false, false, VolumeControl::IndependentChannels).unwrap();
    i2c.done();
}

#[test]
fn set_micbias_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(MICBIAS, 0b0000_0000),
        i2c_page_set(1),
        i2c_reg_write(MICBIAS, 0b0000_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_micbias(false, true, MicBiasOutput::PoweredAVDD).unwrap();
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_pll_d_value(0x3f8a).unwrap();
    i2c.done();
}

#[test]
fn set_gpio1_io_pin_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(GPIO1_IN_OUT_PIN_CONTROL, 0b1100_0000),
        i2c_page_set(0),
        i2c_reg_write(GPIO1_IN_OUT_PIN_CONTROL, 0b1101_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_gpio1_io_pin_control(Gpio1Mode::Int1).unwrap();
    i2c.done();
}

#[test]
fn set_headphone_drivers_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_DRIVERS, 0b1101_0111),
        i2c_page_set(1),
        i2c_reg_write(HEADPHONE_DRIVERS, 0b1000_0101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_headphone_drivers(true, false, HpOutputVoltage::Common1_35V, false).unwrap();
    i2c.done();
}

#[test]
fn set_headset_detection_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(HEADSET_DETECTION, 0b0000_0000),
        i2c_page_set(0),
        i2c_reg_write(HEADSET_DETECTION, 0b1000_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_headset_detection(true, HeadsetDetectionDebounce::Debounce128ms, HeadsetButtonPressDebounce::Debounce16ms).unwrap();
    i2c.done();
}

#[test]
fn set_hp_output_drivers_pop_removal_settings_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS, 0b1001_0101),
        i2c_page_set(1),
        i2c_reg_write(HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS,0b0000_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hp_output_drivers_pop_removal_settings(false, HpPowerOn::Time0us, HpRampUp::Time0ms).unwrap();
    i2c.done();
}

#[test]
fn set_hpl_driver_invalid_hpl_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hpl_driver(0b1010, false).unwrap_err();
    i2c.done();
}

#[test]
fn set_hpl_driver_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HPL_DRIVER, 0b0100_1010),
        i2c_page_set(1),
        i2c_reg_write(HPL_DRIVER, 0b0001_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hpl_driver(2u8, true).unwrap();
    i2c.done();
}

#[test]
fn set_hpr_driver_invalid_hpl_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hpr_driver(0b1010, false).unwrap_err();
    i2c.done();
}

#[test]
fn set_hpr_driver_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HPR_DRIVER, 0b0001_0110),
        i2c_page_set(1),
        i2c_reg_write(HPR_DRIVER, 0b0100_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hpr_driver(8u8, true).unwrap();
    i2c.done();
}

#[test]
fn set_input_cm_settings_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(INPUT_CM_SETTINGS, 0b0011_1111),
        i2c_page_set(1),
        i2c_reg_write(INPUT_CM_SETTINGS, 0b1111_1111),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_input_cm_settings(true, true).unwrap();
    i2c.done();
}

#[test]
fn set_int1_control_register_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(INT1_CONTROL_REGISTER, 0b0000_0000),
        i2c_page_set(0),
        i2c_reg_write(INT1_CONTROL_REGISTER, 0b1110_1101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_int1_control_register(true, true, true, true, true, true).unwrap();
    i2c.done();
}

#[test]
fn set_left_analog_volume_to_hpl_invalid_gain_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_left_analog_volume_to_hpl(true, 128).unwrap_err();
    i2c.done();
}

#[test]
fn set_left_analog_volume_to_hpl_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(LEFT_ANALOG_VOLUME_TO_HPL, 0b1101_1001),
        i2c_page_set(1),
        i2c_reg_write(LEFT_ANALOG_VOLUME_TO_HPL, 0b11010001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_left_analog_volume_to_hpl(true, 81u8).unwrap();
    i2c.done();
}

#[test]
fn set_left_analog_volume_to_spk_invalid_gain_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_left_analog_volume_to_spk(true, 128).unwrap_err();
    i2c.done();
}

#[test]
fn set_left_analog_volume_to_spk_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(LEFT_ANALOG_VOLUME_TO_SPK, 0b1101_1001),
        i2c_page_set(1),
        i2c_reg_write(LEFT_ANALOG_VOLUME_TO_SPK, 0b1100_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_left_analog_volume_to_spk(true, 67u8).unwrap();
    i2c.done();
}

#[test]
fn set_pll_j_value_invalid_j_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_pll_j_value(0x29).unwrap();
    i2c.done();
}

#[test]
fn set_pll_p_and_r_values_invalid_p_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_pll_p_and_r_values(true, 9, 2).unwrap_err();
    i2c.done();
}

#[test]
fn set_pll_p_and_r_values_invalid_r_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
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
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_pll_p_and_r_values(true, 3, 2).unwrap();
    i2c.done();
}

#[test]
fn set_right_analog_volume_to_hpr_invalid_gain_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_right_analog_volume_to_hpr(true, 128).unwrap_err();
    i2c.done();
}

#[test]
fn set_right_analog_volume_to_hpr_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(RIGHT_ANALOG_VOLUME_TO_HPR, 0b1010_0110),
        i2c_page_set(1),
        i2c_reg_write(RIGHT_ANALOG_VOLUME_TO_HPR, 0b1011_1001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_right_analog_volume_to_hpr(true, 57u8).unwrap();
    i2c.done();
}

#[test]
fn set_software_reset_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(SOFTWARE_RESET, 0x1),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_software_reset(true).unwrap();
    i2c.done();
}

#[test]
fn set_timer_clock_mclk_divider_ok() {
    let expectations = [
        i2c_page_set(3),
        i2c_reg_read(TIMER_CLOCK_MCLK_DIVIDER, 0b0000_0000),
        i2c_page_set(3),
        i2c_reg_write(TIMER_CLOCK_MCLK_DIVIDER, 0b1111_1111),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_timer_clock_mclk_divider(true, 127).unwrap();
    i2c.done();
}

#[test]
fn get_vol_micdet_pin_gain_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(VOL_MICDET_PIN_GAIN, 0x3f),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut gain: u8 = 0;
    driver.get_vol_micdet_pin_gain(&mut gain).unwrap();
    assert_eq!(gain, 0x3f);
    i2c.done();
}

#[test]
fn set_vol_micdet_pin_sar_adc_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(VOL_MICDET_PIN_SAR_ADC, 0b1111_1100),
        i2c_page_set(0),
        i2c_reg_write(VOL_MICDET_PIN_SAR_ADC, 0b0000_1010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_vol_micdet_pin_sar_adc(false, false, VolumeControlHysteresis::HysteresisNone, VolumeControlThroughput::Rate62_5Hz).unwrap();
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