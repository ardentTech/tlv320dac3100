use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tlv320dac3100::driver::*;
use tlv320dac3100::registers::*;
use tlv320dac3100::typedefs::*;

#[test]
fn get_bclk_n_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(BCLK_N_VAL, 0b1000_1101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut powered = false;
    let mut divider = 12u8;
    driver.get_bclk_n_val(&mut powered, &mut divider).unwrap();
    assert!(powered);
    assert_eq!(divider, 13);
    i2c.done();
}

#[test]
fn get_beep_cos_x_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(BEEP_COS_X_MSB, 0b1001_0101),
        i2c_page_set(0),
        i2c_reg_read(BEEP_COS_X_LSB, 0b0010_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut val = 0u16;
    driver.get_beep_cos_x(&mut val).unwrap();
    assert_eq!(val, 0b1001_0101_0010_0011);
    i2c.done();
}

#[test]
fn get_beep_length_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(BEEP_LENGTH_MSB, 0b0101_1100),
        i2c_page_set(0),
        i2c_reg_read(BEEP_LENGTH_MIDDLE_BITS, 0b0001_1000),
        i2c_page_set(0),
        i2c_reg_read(BEEP_LENGTH_LSB, 0b1000_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut samples = 0u32;
    driver.get_beep_length(&mut samples).unwrap();
    assert_eq!(samples, 0b0101_1100_0001_1000_1000_0001);
    i2c.done();
}

#[test]
fn get_beep_sin_x_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(BEEP_SIN_X_MSB, 0b0010_0011),
        i2c_page_set(0),
        i2c_reg_read(BEEP_SIN_X_LSB, 0b1001_0101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut val = 0u16;
    driver.get_beep_sin_x(&mut val).unwrap();
    assert_eq!(val, 0b0010_0011_1001_0101);
    i2c.done();
}

#[test]
fn get_class_d_spk_amplifier_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(CLASS_D_SPEAKER_AMPLIFIER, 0b1000_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut powered = false;
    let mut scd = false;
    driver.get_class_d_spk_amplifier(&mut powered, &mut scd).unwrap();
    assert!(powered);
    assert!(scd);
    i2c.done();
}

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
fn get_codec_interface_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_INTERFACE_CONTROL_2, 0b0000_1101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut bclk_inverted = false;
    let mut bclk_wclk_always_active = false;
    let mut bdiv_clkin = BdivClkin::DacClk;
    driver.get_codec_interface_control_2(&mut bclk_inverted, &mut bclk_wclk_always_active, &mut bdiv_clkin).unwrap();
    assert!(bclk_inverted);
    assert!(bclk_wclk_always_active);
    assert_eq!(bdiv_clkin, BdivClkin::DacModClk);
    i2c.done();
}

#[test]
fn get_codec_secondary_interface_control_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_SECONDARY_INTERFACE_CONTROL_1, 0b0000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut bclk_from_gpio1 = false;
    let mut wclk_from_gpio1 = true;
    let mut din_from_gpio1 = false;
    driver.get_codec_secondary_interface_control_1(&mut bclk_from_gpio1, &mut wclk_from_gpio1, &mut din_from_gpio1).unwrap();
    assert!(bclk_from_gpio1);
    assert!(!wclk_from_gpio1);
    assert!(din_from_gpio1);
    i2c.done();
}

#[test]
fn get_codec_secondary_interface_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_SECONDARY_INTERFACE_CONTROL_2, 0b0000_1001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut secondary_bclk = false;
    let mut secondary_wclk = true;
    let mut secondary_din = false;
    driver.get_codec_secondary_interface_control_2(&mut secondary_bclk, &mut secondary_wclk, &mut secondary_din).unwrap();
    assert!(secondary_bclk);
    assert!(!secondary_wclk);
    assert!(secondary_din);
    i2c.done()
}

#[test]
fn get_codec_secondary_interface_control_3_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_SECONDARY_INTERFACE_CONTROL_3, 0b1010_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut primary_bclk = PrimaryBclkOutput::InternalBclk;
    let mut secondary_bclk = SecondaryBclkOutput::InternalBclk;
    let mut primary_wclk = PrimaryWclkOutput::InternalDacFs;
    let mut secondary_wclk = SecondaryWclkOutput::InternalDacFs;
    driver.get_codec_secondary_interface_control_3(&mut primary_bclk, &mut secondary_bclk, &mut primary_wclk, &mut secondary_wclk).unwrap();
    assert_eq!(primary_bclk, PrimaryBclkOutput::SecondaryBclk);
    assert_eq!(secondary_bclk, SecondaryBclkOutput::PrimaryBclk);
    assert_eq!(primary_wclk, PrimaryWclkOutput::SecondaryWclk);
    assert_eq!(secondary_wclk, SecondaryWclkOutput::PrimaryWclk);
    i2c.done()
}

#[test]
fn get_dac_data_path_setup_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_DATA_PATH_SETUP, 0b1001_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_powered = false;
    let mut right_powered = true;
    let mut left_data_path = LeftDataPath::Off;
    let mut right_data_path = RightDataPath::Right;
    let mut soft_stepping = SoftStepping::Disabled;
    driver.get_dac_data_path_setup(&mut left_powered, &mut right_powered, &mut left_data_path, &mut right_data_path, &mut soft_stepping).unwrap();
    assert!(left_powered);
    assert!(!right_powered);
    assert_eq!(left_data_path, LeftDataPath::Left);
    assert_eq!(right_data_path, RightDataPath::Off);
    assert_eq!(soft_stepping, SoftStepping::OneStepPerTwoPeriods);
    i2c.done();
}

#[test]
fn get_dac_flag_register_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_FLAG_REGISTER_1, 0b1011_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut dac_left_powered = false;
    let mut hp_left_powered = false;
    let mut amp_left_powered = false;
    let mut dac_right_powered = false;
    let mut hp_right_powered = false;
    let mut amp_right_powered = false;
    driver.get_dac_flag_register_1(&mut dac_left_powered, &mut hp_left_powered, &mut amp_left_powered, &mut dac_right_powered, &mut hp_right_powered, &mut amp_right_powered).unwrap();
    assert!(dac_left_powered);
    assert!(hp_left_powered);
    assert!(amp_left_powered);
    assert!(dac_right_powered);
    assert!(hp_right_powered);
    assert!(amp_right_powered);
    i2c.done();
}

#[test]
fn get_dac_flag_register_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_FLAG_REGISTER_2, 0b0001_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_gain_equiv = false;
    let mut right_gain_equiv = false;
    driver.get_dac_flag_register_2(&mut left_gain_equiv, &mut right_gain_equiv).unwrap();
    assert!(left_gain_equiv);
    assert!(right_gain_equiv);
    i2c.done();
}

#[test]
fn get_dac_interrupt_flags_ok() {
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
fn get_dac_l_and_dac_r_output_mixer_routing_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING, 0b1010_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_routing = DacLeftOutputMixerRouting::None;
    let mut ain1_input_routed_left = false;
    let mut ain2_input_routed_left = true;
    let mut right_routing = DacRightOutputMixerRouting::HprDriver;
    let mut ain2_input_routed_right = false;
    let mut hpl_output_routed_to_hpr = true;
    driver.get_dac_l_and_dac_r_output_mixer_routing(
        &mut left_routing,
        &mut ain1_input_routed_left,
        &mut ain2_input_routed_left,
        &mut right_routing,
        &mut ain2_input_routed_right,
        &mut hpl_output_routed_to_hpr
    ).unwrap();
    assert_eq!(left_routing, DacLeftOutputMixerRouting::HplDriver);
    assert!(ain1_input_routed_left);
    assert!(!ain2_input_routed_left);
    assert_eq!(right_routing, DacRightOutputMixerRouting::RightChannelMixerAmplifier);
    assert!(ain2_input_routed_right);
    assert!(!hpl_output_routed_to_hpr);
    i2c.done();
}

#[test]
fn get_dac_left_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_LEFT_VOLUME_CONTROL, 0b1000_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut db = 0.0f32;
    driver.get_dac_left_volume_control(&mut db).unwrap();
    assert_eq!(db, -63.0f32);
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
fn get_dac_right_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_RIGHT_VOLUME_CONTROL, 0b0010_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut db = 0.0f32;
    driver.get_dac_right_volume_control(&mut db).unwrap();
    assert_eq!(db, 23.0f32);
    i2c.done();
}

#[test]
fn get_dac_volume_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DAC_VOLUME_CONTROL, 0b0000_1000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left_muted = false;
    let mut right_muted = true;
    let mut control = VolumeControl::LeftToRight;
    driver.get_dac_volume_control(&mut left_muted, &mut right_muted, &mut control).unwrap();
    assert!(left_muted);
    assert!(!right_muted);
    assert_eq!(control, VolumeControl::IndependentChannels);
    i2c.done();
}

#[test]
fn get_din_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DIN_CONTROL, 0b0000_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut control = DinControl::Gpi;
    let mut input_buffer = 3u8;
    driver.get_din_control(&mut control, &mut input_buffer).unwrap();
    assert_eq!(control, DinControl::Enabled);
    assert_eq!(input_buffer, 0b1);
    i2c.done();
}
#[test]
fn get_drc_control_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_1, 0b0100_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left = false;
    let mut right = true;
    let mut threshold = 5u8;
    let mut hysteresis = 2u8;
    driver.get_drc_control_1(&mut left, &mut right, &mut threshold, &mut hysteresis).unwrap();
    assert!(left);
    assert!(!right);
    assert_eq!(threshold, 2);
    assert_eq!(hysteresis, 3);
    i2c.done();
}

#[test]
fn get_drc_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_2, 0b0110_1000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut hold_time = 0u8;
    driver.get_drc_control_2(&mut hold_time).unwrap();
    assert_eq!(hold_time, 13);
    i2c.done();
}

#[test]
fn get_drc_control_3_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_3, 0b0010_1001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut attack_rate = 0u8;
    let mut decay_rate = 2u8;
    driver.get_drc_control_3(&mut attack_rate, &mut decay_rate).unwrap();
    assert_eq!(attack_rate, 2);
    assert_eq!(decay_rate, 9);
    i2c.done();
}

#[test]
fn get_gpio1_io_pin_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(GPIO1_IN_OUT_PIN_CONTROL, 0b0001_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut mode = Gpio1Mode::Disabled;
    let mut input_buffer = 0u8;
    let mut output_value = 1u8;
    driver.get_gpio1_io_pin_control(&mut mode, &mut input_buffer, &mut output_value).unwrap();
    assert_eq!(mode, Gpio1Mode::Int1);
    assert_eq!(input_buffer, 1);
    assert_eq!(output_value, 0);
    i2c.done();
}

#[test]
fn get_headphone_and_speaker_amplifier_error_control_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL, 0b0000_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut preserve_spk_ctrl_bit = false;
    let mut preserve_hp_ctrl_bit = true;
    driver.get_headphone_and_speaker_amplifier_error_control(&mut preserve_spk_ctrl_bit, &mut preserve_hp_ctrl_bit).unwrap();
    assert!(preserve_spk_ctrl_bit);
    assert!(!preserve_hp_ctrl_bit);
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
fn get_hp_driver_control_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HP_DRIVER_CONTROL, 0b0110_1100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut scd_debounce = HpScdDebounce::Time0us;
    let mut mode = HpMode::Default;
    let mut hpl_lineout = false;
    let mut hpr_lineout = true;
    driver.get_hp_driver_control(&mut scd_debounce, &mut mode, &mut hpl_lineout, &mut hpr_lineout).unwrap();
    assert_eq!(scd_debounce, HpScdDebounce::Time32us);
    assert_eq!(mode, HpMode::CurrentBoost);
    assert!(hpl_lineout);
    assert!(!hpr_lineout);
    i2c.done();
}

#[test]
fn get_hpl_driver_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HPL_DRIVER, 0b0100_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut pga = 0u8;
    let mut hpl_muted = false;
    let mut gains_applied = true;
    driver.get_hpl_driver(&mut pga, &mut hpl_muted, &mut gains_applied).unwrap();
    assert_eq!(pga, 9u8);
    assert!(hpl_muted);
    assert!(!gains_applied);
    i2c.done();
}

#[test]
fn get_hpr_driver_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HPR_DRIVER, 0b0100_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut pga = 0u8;
    let mut hpl_muted = true;
    let mut gains_applied = false;
    driver.get_hpr_driver(&mut pga, &mut hpl_muted, &mut gains_applied).unwrap();
    assert_eq!(pga, 8u8);
    assert!(!hpl_muted);
    assert!(gains_applied);
    i2c.done();
}

#[test]
fn get_i2c_bus_condition_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(I2C_BUS_CONDITION, 0b0010_0000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut accepted = false;
    driver.get_i2c_bus_condition(&mut accepted).unwrap();
    assert!(accepted);
    i2c.done();
}

#[test]
fn get_input_cm_settings_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(INPUT_CM_SETTINGS, 0b1000_0000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut ain1_to_cm = false;
    let mut ain2_to_cm = true;
    driver.get_input_cm_settings(&mut ain1_to_cm, &mut ain2_to_cm).unwrap();
    assert!(ain1_to_cm);
    assert!(!ain2_to_cm);
    i2c.done();
}

#[test]
fn get_int1_control_register_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(INT1_CONTROL_REGISTER, 0b1011_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut headset_detect = false;
    let mut button_press = true;
    let mut use_drc_signal_power = false;
    let mut short_circuit = true;
    let mut data_overflow = false;
    let mut multiple_pulses = true;
    driver.get_int1_control_register(
        &mut headset_detect,
        &mut button_press,
        &mut use_drc_signal_power,
        &mut short_circuit,
        &mut data_overflow,
        &mut multiple_pulses
    ).unwrap();
    assert!(headset_detect);
    assert!(!button_press);
    assert!(use_drc_signal_power);
    assert!(!short_circuit);
    assert!(data_overflow);
    assert!(!multiple_pulses);
    i2c.done();
}

#[test]
fn get_int2_control_register_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(INT2_CONTROL_REGISTER, 0b0101_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut headset_detect = true;
    let mut button_press = false;
    let mut use_drc_signal_power = true;
    let mut short_circuit = false;
    let mut data_overflow = true;
    let mut multiple_pulses = false;
    driver.get_int2_control_register(
        &mut headset_detect,
        &mut button_press,
        &mut use_drc_signal_power,
        &mut short_circuit,
        &mut data_overflow,
        &mut multiple_pulses
    ).unwrap();
    assert!(!headset_detect);
    assert!(button_press);
    assert!(!use_drc_signal_power);
    assert!(short_circuit);
    assert!(!data_overflow);
    assert!(multiple_pulses);
    i2c.done();
}

#[test]
fn get_interrupt_flags_dac_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(INTERRUPT_FLAGS_DAC, 0b1111_1100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut hpl_scd_detected = false;
    let mut hpr_scd_detected = false;
    let mut headset_btn_pressed = false;
    let mut headset_insertion_detected = false;
    let mut left_signal_gt_drc = false;
    let mut right_signal_gt_drc = false;
    driver.get_interrupt_flags_dac(&mut hpl_scd_detected, &mut hpr_scd_detected, &mut headset_btn_pressed, &mut headset_insertion_detected, &mut left_signal_gt_drc, &mut right_signal_gt_drc).unwrap();
    assert!(hpl_scd_detected);
    assert!(hpr_scd_detected);
    assert!(headset_btn_pressed);
    assert!(headset_insertion_detected);
    assert!(left_signal_gt_drc);
    assert!(right_signal_gt_drc);
    i2c.done();
}

#[test]
fn get_left_analog_volume_to_hpl_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(LEFT_ANALOG_VOLUME_TO_HPL, 0b1101_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut route_to_hpl = false;
    let mut gain = 0u8;
    driver.get_left_analog_volume_to_hpl(&mut route_to_hpl, &mut gain).unwrap();
    assert!(route_to_hpl);
    assert_eq!(gain, 83);
    i2c.done();
}

#[test]
fn get_left_analog_volume_to_spk_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(LEFT_ANALOG_VOLUME_TO_SPK, 0b1010_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut route_to_class_d = false;
    let mut gain = 0u8;
    driver.get_left_analog_volume_to_spk(&mut route_to_class_d, &mut gain).unwrap();
    assert!(route_to_class_d);
    assert_eq!(gain, 33);
    i2c.done();
}

#[test]
fn get_left_beep_generator_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(LEFT_BEEP_GENERATOR, 0b1000_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut enabled = false;
    let mut volume = 0u8;
    driver.get_left_beep_generator(&mut enabled, &mut volume).unwrap();
    assert!(enabled);
    assert_eq!(volume, 3);
    i2c.done();
}

#[test]
fn get_micbias_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(MICBIAS, 0b1000_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut power_down = false;
    let mut always_on = true;
    let mut output = MicBiasOutput::PoweredDown;
    driver.get_micbias(&mut power_down, &mut always_on, &mut output).unwrap();
    assert!(power_down);
    assert!(!always_on);
    assert_eq!(output, MicBiasOutput::PoweredAVDD);
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
fn get_output_driver_pga_ramp_down_period_control_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL, 0b0001_1101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut ramp_down = PgaRampDown::Time0ms;
    driver.get_output_driver_pga_ramp_down_period_control(&mut ramp_down).unwrap();
    assert_eq!(ramp_down, PgaRampDown::Time3_04ms);
    i2c.done();
}

#[test]
fn get_overflow_flags_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(OVERFLOW_FLAGS, 0b1110_0000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut left = false;
    let mut right = false;
    let mut barrel_shifter = false;
    driver.get_overflow_flags(&mut left, &mut right, &mut barrel_shifter).unwrap();
    assert!(left);
    assert!(right);
    assert!(barrel_shifter);
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
fn get_right_analog_volume_to_hpr_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(RIGHT_ANALOG_VOLUME_TO_HPR, 0b0001_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut route_to_hpr = true;
    let mut gain = 0u8;
    driver.get_right_analog_volume_to_hpr(&mut route_to_hpr, &mut gain).unwrap();
    assert!(!route_to_hpr);
    assert_eq!(gain, 18);
    i2c.done();
}

#[test]
fn get_right_beep_generator_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(RIGHT_BEEP_GENERATOR, 0b0010_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut mode = RightBeepMode::LeftToRight;
    let mut volume = 0u8;
    driver.get_right_beep_generator(&mut mode, &mut volume).unwrap();
    assert_eq!(mode, RightBeepMode::IndependentControl);
    assert_eq!(volume, 35);
    i2c.done();
}

#[test]
fn get_timer_clock_mclk_divider_ok() {
    let expectations = [
        i2c_page_set(3),
        i2c_reg_read(TIMER_CLOCK_MCLK_DIVIDER, 0b1111_1110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let mut external_mclk = false;
    let mut mclk_divider = 0u8;
    driver.get_timer_clock_mclk_divider(&mut external_mclk, &mut mclk_divider).unwrap();
    assert!(external_mclk);
    assert_eq!(mclk_divider, 126);
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
fn read_reg_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(PLL_J_VALUE, 0b1010_1011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    let j = driver.read_reg(0, PLL_J_VALUE).unwrap();
    assert_eq!(j, 0xab);
    i2c.done();
}

#[test]
fn set_bclk_n_val_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(BCLK_N_VAL, 0b1000_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_bclk_n_val(true, 2).unwrap();
    i2c.done();
}

#[test]
fn set_beep_length_invalid_samples_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_beep_length(16777216u32).unwrap_err();
    i2c.done();
}

#[test]
fn set_beep_length_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(BEEP_LENGTH_MSB, 0b1111_1111),
        i2c_page_set(0),
        i2c_reg_write(BEEP_LENGTH_MIDDLE_BITS, 0b1111_1111),
        i2c_page_set(0),
        i2c_reg_write(BEEP_LENGTH_LSB, 0b1111_1100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_beep_length(16777212u32).unwrap();
    i2c.done();
}


#[test]
fn set_beep_cos_x_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(BEEP_COS_X_MSB, 0b100),
        i2c_page_set(0),
        i2c_reg_write(BEEP_COS_X_LSB, 0b1101_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_beep_cos_x(1234).unwrap();
    i2c.done();
}


#[test]
fn set_beep_sin_x_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(BEEP_SIN_X_MSB, 0b1101_0100),
        i2c_page_set(0),
        i2c_reg_write(BEEP_SIN_X_LSB, 0b0011_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_beep_sin_x(54321).unwrap();
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
fn set_codec_interface_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_INTERFACE_CONTROL_2, 0b0),
        i2c_page_set(0),
        i2c_reg_write(CODEC_INTERFACE_CONTROL_2, 0b0000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_codec_interface_control_2(false, true, BdivClkin::DacClk).unwrap();
    i2c.done();
}

#[test]
fn set_codec_secondary_interface_control_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(CODEC_SECONDARY_INTERFACE_CONTROL_1, 0b0000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_codec_secondary_interface_control_1(true, false, true).unwrap();
    i2c.done();
}

#[test]
fn set_codec_secondary_interface_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_SECONDARY_INTERFACE_CONTROL_2, 0b0),
        i2c_page_set(0),
        i2c_reg_write(CODEC_SECONDARY_INTERFACE_CONTROL_2, 0b0000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_codec_secondary_interface_control_2(false, true, false).unwrap();
    i2c.done();
}

#[test]
fn set_codec_secondary_interface_control_3_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(CODEC_SECONDARY_INTERFACE_CONTROL_3, 0b1111_1111),
        i2c_page_set(0),
        i2c_reg_write(CODEC_SECONDARY_INTERFACE_CONTROL_3, 0b0000_0011),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_codec_secondary_interface_control_3(
        PrimaryBclkOutput::InternalBclk,
        SecondaryBclkOutput::PrimaryBclk,
        PrimaryWclkOutput::InternalDacFs,
        SecondaryWclkOutput::PrimaryWclk
    ).unwrap();
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
fn set_data_slot_offset_programmability_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_write(DATA_SLOT_OFFSET_PROGRAMMABILITY, 0b0001_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_data_slot_offset_programmability(17u8).unwrap();
    i2c.done();
}

#[test]
fn set_din_control_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DIN_CONTROL, 0b0),
        i2c_page_set(0),
        i2c_reg_write(DIN_CONTROL, 0b0000_0100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_din_control(DinControl::Gpi).unwrap();
    i2c.done();
}

#[test]
fn set_drc_control_1_invalid_hysteresis_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_1(true, false, 6u8, 7u8).unwrap_err();
    i2c.done();
}

#[test]
fn set_drc_control_1_invalid_threshold_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_1(true, false, 9u8, 2u8).unwrap_err();
    i2c.done();
}

#[test]
fn set_drc_control_1_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_1, 0b0),
        i2c_page_set(0),
        i2c_reg_write(DRC_CONTROL_1, 0b0101_0110),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_1(true, false, 5u8, 2u8).unwrap();
    i2c.done();
}

#[test]
fn set_drc_control_2_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_2, 0b0),
        i2c_page_set(0),
        i2c_reg_write(DRC_CONTROL_2, 0b0110_1000),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_2(13u8).unwrap();
    i2c.done();
}

#[test]
fn set_drc_control_3_invalid_attack_rate_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_3(18u8, 2u8).unwrap_err();
    i2c.done();
}

#[test]
fn set_drc_control_3_invalid_decay_rate_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_3(6u8, 17u8).unwrap_err();
    i2c.done();
}

#[test]
fn set_drc_control_3_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(DRC_CONTROL_3, 0b0),
        i2c_page_set(0),
        i2c_reg_write(DRC_CONTROL_3, 0b0111_1111),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_drc_control_3(7u8, 15u8).unwrap();
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
fn set_headphone_and_speaker_amplifier_error_control_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL, 0b0),
        i2c_page_set(1),
        i2c_reg_write(HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL,0b0000_0001),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_headphone_and_speaker_amplifier_error_control(false, true).unwrap();
    i2c.done();
}

#[test]
fn set_hp_driver_control_ok() {
    let expectations = [
        i2c_page_set(1),
        i2c_reg_read(HP_DRIVER_CONTROL, 0b0),
        i2c_page_set(1),
        i2c_reg_write(HP_DRIVER_CONTROL,0b0011_1010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_hp_driver_control(
        HpScdDebounce::Time8us,
        HpMode::CurrentBoostX2,
        false,
        true
    ).unwrap();
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
fn set_i2c_bus_condition_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(I2C_BUS_CONDITION, 0b1101_0010),
        i2c_page_set(0),
        i2c_reg_write(I2C_BUS_CONDITION, 0b1111_0010),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_i2c_bus_condition(true).unwrap();
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
fn set_int2_control_register_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(INT2_CONTROL_REGISTER, 0b0),
        i2c_page_set(0),
        i2c_reg_write(INT2_CONTROL_REGISTER, 0b10001100),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_int2_control_register(true, false, false, true, true, false).unwrap();
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
fn set_left_beep_generator_invalid_volume_err() {
    let mut i2c = I2cMock::new(&[]);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_right_beep_generator(RightBeepMode::IndependentControl, 64u8).unwrap_err();
    i2c.done();
}

#[test]
fn set_left_beep_generator_ok() {
    let expectations = [
        i2c_page_set(0),
        i2c_reg_read(RIGHT_BEEP_GENERATOR, 0b0),
        i2c_page_set(0),
        i2c_reg_write(RIGHT_BEEP_GENERATOR, 0b0000_0101),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut driver = TLV320DAC3100::new(&mut i2c);
    driver.set_right_beep_generator(RightBeepMode::IndependentControl, 5u8).unwrap();
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