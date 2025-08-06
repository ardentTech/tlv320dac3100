use embedded_hal as hal;
use hal::i2c::I2c;
use crate::bits::{get_bits, set_bits};
use crate::error::TLV320DAC3100Error;
use crate::registers::*;
use crate::typedefs::*;

pub const I2C_DEVICE_ADDRESS: u8 = 0x18;

pub struct TLV320DAC3100<I2C> {
    i2c: I2C
}

impl<I2C: I2c> TLV320DAC3100<I2C> {
    pub fn new(i2c: I2C) -> Self {
        TLV320DAC3100 { i2c }
    }

    pub fn get_bclk_n_val(&mut self, powered: &mut bool, divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, BCLK_N_VAL)?;
        *powered = get_bits(reg_val, 1, 7) == 1;
        *divider = get_bits(reg_val, 7, 0);
        Ok(())
    }

    pub fn get_beep_cos_x(&mut self, val: &mut u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb = (self.read_reg(0, BEEP_COS_X_MSB)? as u16) << 8;
        let lsb = self.read_reg(0, BEEP_COS_X_LSB)? as u16;
        *val = msb | lsb;
        Ok(())
    }

    pub fn get_beep_cos_x_msb(&mut self, msb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *msb = self.read_reg(0, BEEP_COS_X_MSB)?;
        Ok(())
    }

    pub fn get_beep_cos_x_lsb(&mut self, lsb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *lsb = self.read_reg(0, BEEP_COS_X_LSB)?;
        Ok(())
    }

    pub fn get_beep_length(&mut self, samples: &mut u32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut msb = (self.read_reg(0, BEEP_LENGTH_MSB)? as u32) << 16;
        let mid = (self.read_reg(0, BEEP_LENGTH_MIDDLE_BITS)? as u32) << 8;
        let lsb = self.read_reg(0, BEEP_LENGTH_LSB)? as u32;
        msb |= mid;
        msb |= lsb;
        *samples = msb;
        Ok(())
    }

    pub fn get_beep_length_msb(&mut self, msb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *msb = self.read_reg(0, BEEP_LENGTH_MSB)?;
        Ok(())
    }

    pub fn get_beep_length_middle_bits(&mut self, middle_bits: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *middle_bits = self.read_reg(0, BEEP_LENGTH_MIDDLE_BITS)?;
        Ok(())
    }

    pub fn get_beep_length_lsb(&mut self, lsb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *lsb = self.read_reg(0, BEEP_LENGTH_LSB)?;
        Ok(())
    }

    pub fn get_beep_sin_x(&mut self, val: &mut u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb = (self.read_reg(0, BEEP_SIN_X_MSB)? as u16) << 8;
        let lsb = self.read_reg(0, BEEP_SIN_X_LSB)? as u16;
        *val = msb | lsb;
        Ok(())
    }

    pub fn get_beep_sin_x_msb(&mut self, msb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *msb = self.read_reg(0, BEEP_SIN_X_MSB)?;
        Ok(())
    }

    pub fn get_beep_sin_x_lsb(&mut self, lsb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *lsb = self.read_reg(0, BEEP_SIN_X_LSB)?;
        Ok(())
    }

    pub fn get_class_d_spk_amplifier(&mut self, powered_up: &mut bool, scd: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, CLASS_D_SPEAKER_AMPLIFIER)?;
        *powered_up = get_bits(reg_val, 1, 7) == 1;
        *scd = get_bits(reg_val, 1, 0) == 1;
        Ok(())
    }

    pub fn get_class_d_spk_driver(&mut self, gain: &mut OutputStage, muted: &mut bool, gains_applied: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CLASS_D_SPK_DRIVER)?;
        *gain = get_bits(reg_val, 2, 3).try_into().unwrap();
        *muted = get_bits(reg_val, 1, 2) == 0;
        *gains_applied = get_bits(reg_val, 1, 0) == 1;
        Ok(())
    }

    pub fn get_clkout_m_val(&mut self, powered: &mut bool, divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CLKOUT_M_VAL)?;
        *powered = get_bits(reg_val, 1, 7) == 1;
        *divider = reg_val & 0b0111_1111;
        Ok(())
    }

    pub fn get_clkout_mux(&mut self, cdiv_clkin: &mut CdivClkin) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *cdiv_clkin = get_bits(self.read_reg(0, CLKOUT_MUX)?, 3, 0).try_into().unwrap();
        Ok(())
    }

    pub fn get_clock_gen_muxing(&mut self, pll_clkin: &mut PllClkin, codec_clkin: &mut CodecClkin) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CLOCK_GEN_MUXING)?;
        *pll_clkin = get_bits(reg_val, 2, 2).try_into().unwrap();
        *codec_clkin = get_bits(reg_val, 2, 0).try_into().unwrap();
        Ok(())
    }

    pub fn get_codec_interface_control_1(
        &mut self,
        codec_interface: &mut CodecInterface,
        word_length: &mut CodecInterfaceWordLength,
        bclk_output: &mut bool,
        wclk_output: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_1)?;
        *codec_interface = get_bits(reg_val, 2, 6).try_into().unwrap();
        *word_length = get_bits(reg_val, 2, 4).try_into().unwrap();
        *bclk_output = get_bits(reg_val, 1, 3) == 1;
        *wclk_output = get_bits(reg_val, 1, 2) == 1;
        Ok(())
    }

    pub fn get_codec_interface_control_2(
        &mut self,
        bclk_inverted: &mut bool,
        bclk_wclk_always_active: &mut bool,
        bdiv_clkin: &mut BdivClkin
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_2)?;
        *bclk_inverted = get_bits(reg_val, 1, 3) == 1;
        *bclk_wclk_always_active = get_bits(reg_val, 1, 2) == 1;
        *bdiv_clkin = get_bits(reg_val, 2, 0).try_into().unwrap();
        Ok(())
    }

    pub fn get_codec_secondary_interface_control_1(
        &mut self,
        bclk_from_gpio1: &mut bool,
        wclk_from_gpio1: &mut bool,
        din_from_gpio1: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_1)?;
        *bclk_from_gpio1 = get_bits(reg_val, 3, 5) == 0;
        *wclk_from_gpio1 = get_bits(reg_val, 3, 2) == 0;
        *din_from_gpio1 = get_bits(reg_val, 2, 0) == 0;
        Ok(())
    }

    pub fn get_codec_secondary_interface_control_2(
        &mut self,
        secondary_bclk: &mut bool,
        secondary_wclk: &mut bool,
        secondary_din: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_2)?;
        *secondary_bclk = get_bits(reg_val, 1, 3) == 1;
        *secondary_wclk = get_bits(reg_val, 1, 2) == 1;
        *secondary_din = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_codec_secondary_interface_control_3(
        &mut self,
        primary_bclk: &mut PrimaryBclkOutput,
        secondary_bclk: &mut SecondaryBclkOutput,
        primary_wclk: &mut PrimaryWclkOutput,
        secondary_wclk: &mut SecondaryWclkOutput
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_3)?;
        *primary_bclk = get_bits(reg_val, 1, 7).try_into().unwrap();
        *secondary_bclk = get_bits(reg_val, 1, 6).try_into().unwrap();
        *primary_wclk = get_bits(reg_val, 2, 4).try_into().unwrap();
        *secondary_wclk = get_bits(reg_val, 2, 2).try_into().unwrap();
        Ok(())
    }

    pub fn get_dac_data_path_setup(
        &mut self,
        left_powered: &mut bool,
        right_powered: &mut bool,
        left_data_path: &mut LeftDataPath,
        right_data_path: &mut RightDataPath,
        soft_stepping: &mut SoftStepping
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DAC_DATA_PATH_SETUP)?;
        *left_powered = get_bits(reg_val, 1, 7) == 1;
        *right_powered = get_bits(reg_val, 1, 6) == 1;
        *left_data_path = get_bits(reg_val, 2, 4).try_into().unwrap();
        *right_data_path = get_bits(reg_val, 2, 2).try_into().unwrap();
        *soft_stepping = (reg_val & 0b11).try_into().unwrap();
        Ok(())
    }

    pub fn get_dac_dosr_val(&mut self, dosr: &mut u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb = self.read_reg(0, DAC_DOSR_VAL_MSB)?;
        let lsb = self.read_reg(0, DAC_DOSR_VAL_LSB)?;
        *dosr = (msb as u16) << 8 | lsb as u16;
        Ok(())
    }

    pub fn get_dac_dosr_val_msb(&mut self, msb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *msb = self.read_reg(0, DAC_DOSR_VAL_MSB)?;
        Ok(())
    }

    pub fn get_dac_dosr_val_lsb(&mut self, lsb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *lsb = self.read_reg(0, DAC_DOSR_VAL_LSB)?;
        Ok(())
    }

    pub fn get_dac_flag_register_1(
        &mut self,
        dac_left_powered: &mut bool,
        hp_left_powered: &mut bool,
        amp_left_powered: &mut bool,
        dac_right_powered: &mut bool,
        hp_right_powered: &mut bool,
        amp_right_powered: &mut bool,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DAC_FLAG_REGISTER_1)?;
        *dac_left_powered = get_bits(reg_val, 1, 7) == 1;
        *hp_left_powered = get_bits(reg_val, 1, 5) == 1;
        *amp_left_powered = get_bits(reg_val, 1, 4) == 1;
        *dac_right_powered = get_bits(reg_val, 1, 3) == 1;
        *hp_right_powered = get_bits(reg_val, 1, 1) == 1;
        *amp_right_powered = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_dac_flag_register_2(&mut self, left_gain_equiv: &mut bool, right_gain_equiv: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DAC_FLAG_REGISTER_2)?;
        *left_gain_equiv = get_bits(reg_val, 1, 4) == 1;
        *right_gain_equiv = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_dac_interrupt_flags(
        &mut self,
        left_scd: &mut bool,
        right_scd: &mut bool,
        headset_button_pressed: &mut bool,
        headset_insert_removal_detected: &mut bool,
        left_dac_signal_above_drc: &mut bool,
        right_dac_signal_above_drc: &mut bool,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DAC_INTERRUPT_FLAGS_STICKY_BITS)?;
        *left_scd = get_bits(reg_val, 1, 7) == 1;
        *right_scd = get_bits(reg_val, 1, 6) == 1;
        *headset_button_pressed = get_bits(reg_val, 1, 5) == 1;
        *headset_insert_removal_detected = get_bits(reg_val, 1, 4) == 1;
        *left_dac_signal_above_drc = get_bits(reg_val, 1, 3) == 1;
        *right_dac_signal_above_drc = get_bits(reg_val, 1, 2) == 1;
        Ok(())
    }

    pub fn get_dac_l_and_dac_r_output_mixer_routing(
        &mut self,
        left_routing: &mut DacLeftOutputMixerRouting,
        ain1_input_routed_left: &mut bool,
        ain2_input_routed_left: &mut bool,
        right_routing: &mut DacRightOutputMixerRouting,
        ain2_input_routed_right: &mut bool,
        hpl_output_routed_to_hpr: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING)?;
        *left_routing = get_bits(reg_val, 2, 6).try_into().unwrap();
        *ain1_input_routed_left = get_bits(reg_val, 1, 5) == 1;
        *ain2_input_routed_left = get_bits(reg_val, 1, 4) == 1;
        *right_routing = get_bits(reg_val, 2, 2).try_into().unwrap();
        *ain2_input_routed_right = get_bits(reg_val, 1, 1) == 1;
        *hpl_output_routed_to_hpr = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_dac_left_volume_control(&mut self, db: &mut f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *db = ((self.read_reg(0, DAC_LEFT_VOLUME_CONTROL)? as i8) as f32) / 2.0;
        Ok(())
    }

    fn get_dac_xdac_val(&mut self, mdac: bool, powered: &mut bool, divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, if mdac { DAC_MDAC_VAL } else { DAC_NDAC_VAL })?;
        *powered = get_bits(reg_val, 1, 7) == 1;
        *divider = reg_val & 0b111_1111;
        Ok(())
    }

    pub fn get_dac_mdac_val(&mut self, powered: &mut bool, divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_dac_xdac_val(true, powered, divider)
    }

    pub fn get_dac_ndac_val(&mut self, powered: &mut bool, divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_dac_xdac_val(false, powered, divider)
    }

    pub fn get_dac_processing_block_selection(&mut self, prb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *prb = self.read_reg(0, DAC_PROCESSING_BLOCK_SECTION)? & 0b1_1111;
        Ok(())
    }

    pub fn get_dac_right_volume_control(&mut self, db: &mut f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *db = ((self.read_reg(0, DAC_RIGHT_VOLUME_CONTROL)? as i8) as f32) / 2.0;
        Ok(())
    }

    pub fn get_dac_volume_control(
        &mut self,
        left_muted: &mut bool,
        right_muted: &mut bool,
        control: &mut VolumeControl
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DAC_VOLUME_CONTROL)?;
        *left_muted = get_bits(reg_val, 1, 3) == 1;
        *right_muted = get_bits(reg_val, 1, 2) == 1;
        *control = (reg_val & 0b11).try_into().unwrap();
        Ok(())
    }

    pub fn get_data_slot_offset_programmability(&mut self, offset: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *offset = self.read_reg(0, DATA_SLOT_OFFSET_PROGRAMMABILITY)?;
        Ok(())
    }

    pub fn get_din_control(&mut self, din_control: &mut DinControl, input_buffer: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DIN_CONTROL)?;
        *din_control = get_bits(reg_val, 2, 1).try_into().unwrap();
        *input_buffer = reg_val & 0b1;
        Ok(())
    }

    pub fn get_drc_control_1(
        &mut self,
        left: &mut bool,
        right: &mut bool,
        threshold: &mut u8,
        hysteresis: &mut u8
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DRC_CONTROL_1)?;
        *left = get_bits(reg_val, 1, 6) == 1;
        *right = get_bits(reg_val, 1, 5) == 1;
        *threshold = get_bits(reg_val, 3, 2);
        *hysteresis = reg_val & 0b11;
        Ok(())
    }

    pub fn get_drc_control_2(&mut self, hold_time: &mut HoldTime) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *hold_time = get_bits(self.read_reg(0, DRC_CONTROL_2)?, 4, 3).try_into().unwrap();
        Ok(())
    }


    pub fn get_drc_control_3(
        &mut self,
        attack_rate: &mut u8,
        decay_rate: &mut u8
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, DRC_CONTROL_3)?;
        *attack_rate = get_bits(reg_val, 4, 4);
        *decay_rate = reg_val & 0b1111;
        Ok(())
    }

    pub fn get_gpio1_io_pin_control(
        &mut self,
        mode: &mut Gpio1Mode,
        input_buffer_value: &mut u8,
        output_value: &mut u8
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, GPIO1_IN_OUT_PIN_CONTROL)?;
        *mode = get_bits(reg_val, 4, 2).try_into().unwrap();
        *input_buffer_value = get_bits(reg_val, 1, 1);
        *output_value = reg_val & 0b1;
        Ok(())
    }

    pub fn get_headphone_and_speaker_amplifier_error_control(
        &mut self,
        preserve_spk_ctrl_bits: &mut bool,
        preserve_hp_ctrl_bits: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL)?;
        *preserve_spk_ctrl_bits = get_bits(reg_val, 1, 1) == 1;
        *preserve_hp_ctrl_bits = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_headset_detection(
        &mut self,
        enabled: &mut bool,
        detected: &mut HeadsetDetected,
        debounce: &mut HeadsetDetectionDebounce,
        button_debounce: &mut HeadsetButtonPressDebounce
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, HEADSET_DETECTION)?;
        *enabled = get_bits(reg_val, 1, 7) == 1;
        *detected = get_bits(reg_val, 2, 5).try_into().unwrap();
        *debounce = get_bits(reg_val, 3, 2).try_into().unwrap();
        *button_debounce = (reg_val & 0b11).try_into().unwrap();
        Ok(())
    }

    pub fn get_headphone_drivers(
        &mut self,
        left_powered: &mut bool,
        right_powered: &mut bool,
        output_voltage: &mut HpOutputVoltage,
        power_down_on_scd: &mut bool,
        scd_detected: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, HEADPHONE_DRIVERS)?;
        *left_powered = get_bits(reg_val, 1, 7) == 1;
        *right_powered = get_bits(reg_val, 1, 6) == 1;
        *output_voltage = get_bits(reg_val, 2, 3).try_into().unwrap();
        *power_down_on_scd = get_bits(reg_val, 1, 1) == 1;
        *scd_detected = reg_val & 0b1 == 1;
        Ok(())
    }

    pub fn get_hp_driver_control(
        &mut self,
        scd_debounce: &mut HpScdDebounce,
        mode: &mut HpMode,
        hpl_lineout: &mut bool,
        hpr_lineout: &mut bool,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, HP_DRIVER_CONTROL)?;
        *scd_debounce = ((reg_val & 0b1110_0000) >> 5).try_into().unwrap();
        *mode = ((reg_val & 0b1_1000) >> 3).try_into().unwrap();
        *hpl_lineout = ((reg_val & 0b100) >> 2) == 1;
        *hpr_lineout = ((reg_val & 0b10) >> 1) == 1;
        Ok(())
    }

    pub fn get_hp_output_drivers_pop_removal_settings(
        &mut self,
        optimize_power_down_pop: &mut bool,
        power_on: &mut HpPowerOn,
        ramp_up: &mut HpRampUp,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS)?;
        *optimize_power_down_pop = (reg_val & 0b1000_0000) == 1;
        *power_on = (reg_val & 0b111_1000).try_into().unwrap();
        *ramp_up = (reg_val & 0b110).try_into().unwrap();
        Ok(())
    }

    pub fn get_hpl_driver(&mut self, pga: &mut u8, hpl_muted: &mut bool, gains_applied: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_hpx_driver(true, pga, hpl_muted, gains_applied)
    }

    pub fn get_hpr_driver(&mut self, pga: &mut u8, hpr_muted: &mut bool, gains_applied: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_hpx_driver(false, pga, hpr_muted, gains_applied)
    }

    fn get_hpx_driver(&mut self, hpl: bool, pga: &mut u8, hpx_muted: &mut bool, gains_applied: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, if hpl { HPL_DRIVER } else { HPR_DRIVER })?;
        *pga = get_bits(reg_val, 4, 3);
        *hpx_muted = get_bits(reg_val, 1, 2) == 1;
        *gains_applied = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_i2c_bus_condition(&mut self, accept_address: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *accept_address = get_bits(self.read_reg(0, I2C_BUS_CONDITION)?, 1, 5) == 1;
        Ok(())
    }

    pub fn get_input_cm_settings(&mut self, ain1_to_cm: &mut bool, ain2_to_cm: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, INPUT_CM_SETTINGS)?;
        *ain1_to_cm = get_bits(reg_val, 1, 7) == 1;
        *ain2_to_cm = get_bits(reg_val, 1, 6) == 1;
        Ok(())
    }

    pub fn get_interrupt_flags_dac(
        &mut self,
        hpl_scd_detected: &mut bool,
        hpr_scd_detected: &mut bool,
        headset_btn_pressed: &mut bool,
        headset_insertion_detected: &mut bool,
        left_signal_gt_drc: &mut bool,
        right_signal_gt_drc: &mut bool,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, INTERRUPT_FLAGS_DAC)?;
        *hpl_scd_detected = get_bits(reg_val, 1, 7) == 1;
        *hpr_scd_detected = get_bits(reg_val, 1, 6) == 1;
        *headset_btn_pressed = get_bits(reg_val, 1, 5) == 1;
        *headset_insertion_detected = get_bits(reg_val, 1, 4) == 1;
        *left_signal_gt_drc = get_bits(reg_val, 1, 3) == 1;
        *right_signal_gt_drc = get_bits(reg_val, 1, 2) == 1;
        Ok(())
    }

    fn get_int_control_register(
        &mut self,
        int1: bool,
        headset_detect: &mut bool,
        button_press: &mut bool,
        use_drc_signal_power: &mut bool,
        short_circuit: &mut bool,
        data_overflow: &mut bool,
        multiple_pulses: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, if int1 { INT1_CONTROL_REGISTER } else { INT2_CONTROL_REGISTER })?;
        *headset_detect = get_bits(reg_val, 1, 7) == 1;
        *button_press = get_bits(reg_val, 1, 6) == 1;
        *use_drc_signal_power = get_bits(reg_val, 1, 5) == 1;
        *short_circuit = get_bits(reg_val, 1, 3) == 1;
        *data_overflow = get_bits(reg_val, 1, 2) == 1;
        *multiple_pulses = (reg_val & 0b1) == 1;
        Ok(())
    }

    pub fn get_int1_control_register(
        &mut self,
        headset_detect: &mut bool,
        button_press: &mut bool,
        use_drc_signal_power: &mut bool,
        short_circuit: &mut bool,
        data_overflow: &mut bool,
        multiple_pulses: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_int_control_register(
            true,
            headset_detect,
            button_press,
            use_drc_signal_power,
            short_circuit,
            data_overflow,
            multiple_pulses
        )
    }

    pub fn get_int2_control_register(
        &mut self,
        headset_detect: &mut bool,
        button_press: &mut bool,
        use_drc_signal_power: &mut bool,
        short_circuit: &mut bool,
        data_overflow: &mut bool,
        multiple_pulses: &mut bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.get_int_control_register(
            false,
            headset_detect,
            button_press,
            use_drc_signal_power,
            short_circuit,
            data_overflow,
            multiple_pulses
        )
    }

    pub fn get_left_analog_volume_to_hpl(&mut self, route_to_hpl: &mut bool, gain: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_HPL)?;
        *route_to_hpl = get_bits(reg_val, 1, 7) == 1;
        *gain = (reg_val & 0b111_1111).try_into().unwrap();
        Ok(())
    }

    pub fn get_left_analog_volume_to_spk(&mut self, route_to_class_d: &mut bool, gain: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_SPK)?;
        *route_to_class_d = get_bits(reg_val, 1, 7) == 1;
        *gain = reg_val & 0b111_1111;
        Ok(())
    }

    pub fn get_left_beep_generator(&mut self, enabled: &mut bool, volume: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, LEFT_BEEP_GENERATOR)?;
        *enabled = get_bits(reg_val, 1, 7) == 1;
        *volume = reg_val & 0b11_1111;
        Ok(())
    }

    pub fn get_micbias(
        &mut self,
        power_down: &mut bool,
        always_on: &mut bool,
        output: &mut MicBiasOutput
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, MICBIAS)?;
        *power_down = get_bits(reg_val, 1, 7) == 1;
        *always_on = get_bits(reg_val, 1, 3) == 1;
        *output = (reg_val & 0b11).try_into().unwrap();
        Ok(())
    }

    pub fn get_ot_flag(&mut self, ot_flag: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *ot_flag = self.read_reg(0, OT_FLAG)? >> 1 == 0; // active low
        Ok(())
    }

    pub fn get_output_driver_pga_ramp_down_period_control(&mut self, ramp_down: &mut PgaRampDown) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *ramp_down = get_bits(self.read_reg(1, OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL)?, 3, 4).try_into().unwrap();
        Ok(())
    }

    pub fn get_overflow_flags(&mut self, left: &mut bool, right: &mut bool, barrel_shifter: &mut bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, OVERFLOW_FLAGS)?;
        *left = get_bits(reg_val, 1, 7) == 1;
        *right = get_bits(reg_val, 1, 6) == 1;
        *barrel_shifter = get_bits(reg_val, 1, 5) == 1;
        Ok(())
    }

    pub fn get_pll_d_value(&mut self, d: &mut u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb = self.read_reg(0, PLL_D_VALUE_MSB)?;
        let lsb = self.read_reg(0, PLL_D_VALUE_LSB)?;
        *d = (msb as u16) << 8 | lsb as u16;
        Ok(())
    }

    pub fn get_pll_d_value_msb(&mut self, msb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *msb = self.read_reg(0, PLL_D_VALUE_MSB)?;
        Ok(())
    }

    pub fn get_pll_d_value_lsb(&mut self, lsb: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *lsb = self.read_reg(0, PLL_D_VALUE_LSB)?;
        Ok(())
    }

    pub fn get_pll_j_value(&mut self, j: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *j = self.read_reg(0, PLL_J_VALUE)? & 0b11_1111;
        Ok(())
    }

    pub fn get_pll_p_and_r_values(&mut self, powered: &mut bool, p: &mut u8, r: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, PLL_P_AND_R_VALUES)?;
        *powered = get_bits(reg_val, 1, 7) == 1;
        *p = get_bits(reg_val, 3, 4);
        *r = 0b1111 & reg_val;
        Ok(())
    }

    pub fn get_right_analog_volume_to_hpr(&mut self, route_to_hpr: &mut bool, gain: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(1, RIGHT_ANALOG_VOLUME_TO_HPR)?;
        *route_to_hpr = get_bits(reg_val, 1, 7) == 1;
        *gain = (reg_val & 0b111_1111).try_into().unwrap();
        Ok(())
    }

    pub fn get_right_beep_generator(&mut self, mode: &mut RightBeepMode, volume: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, RIGHT_BEEP_GENERATOR)?;
        *mode = get_bits(reg_val, 2, 6).try_into().unwrap();
        *volume = reg_val & 0b11_1111;
        Ok(())
    }

    pub fn get_timer_clock_mclk_divider(&mut self, external_mclk: &mut bool, mclk_divider: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(3, TIMER_CLOCK_MCLK_DIVIDER)?;
        *external_mclk = get_bits(reg_val, 1, 7) == 1;
        *mclk_divider = reg_val & 0b111_1111;
        Ok(())
    }

    pub fn get_vol_micdet_pin_gain(&mut self, gain: &mut u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        *gain = self.read_reg(0, VOL_MICDET_PIN_GAIN)?;
        Ok(())
    }

    pub fn get_vol_micdet_pin_sar_adc(&mut self, pin_control: &mut bool, use_mclk: &mut bool, hysteresis: &mut VolumeControlHysteresis, throughput: &mut VolumeControlThroughput) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_val = self.read_reg(0, VOL_MICDET_PIN_SAR_ADC)?;
        *pin_control = get_bits(reg_val, 1, 7) == 1;
        *use_mclk = get_bits(reg_val, 1, 6) == 1;
        *hysteresis = get_bits(reg_val,2, 4).try_into().unwrap();
        *throughput = (reg_val & 0b111).try_into().unwrap();
        Ok(())
    }

    pub fn read_reg(&mut self, page: u8, reg: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.select_page(page)?;
        let mut buf = [0u8];
        self.i2c.write_read(I2C_DEVICE_ADDRESS, &[reg], &mut buf).map_err(TLV320DAC3100Error::I2C)?;
        Ok(buf[0])
    }

    fn select_page(&mut self, page: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        Ok(self.i2c.write(I2C_DEVICE_ADDRESS, &[PAGE_CONTROL, page]).map_err(TLV320DAC3100Error::I2C)?)
    }

    pub fn set_bclk_n_val(&mut self, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = (powered as u8) << 7;
        if divider < 128 {
            set_bits(&mut reg_val, divider, 0, 0b0111_1111);
        }
        self.write_reg(0, BCLK_N_VAL, reg_val)
    }

    pub fn set_beep_length(&mut self, samples: u32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if samples > 16777215 { return Err(TLV320DAC3100Error::InvalidArgument) }

        self.write_reg(0, BEEP_LENGTH_MSB, ((samples >> 16) & 0xff) as u8)?;
        self.write_reg(0, BEEP_LENGTH_MIDDLE_BITS, ((samples >> 8) & 0xff) as u8)?;
        self.write_reg(0, BEEP_LENGTH_LSB, (samples & 0xff) as u8)
    }

    pub fn set_beep_cos_x(&mut self, val: u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_reg(0, BEEP_COS_X_MSB, (val >> 8) as u8)?;
        self.write_reg(0, BEEP_COS_X_LSB, (val & 0xff) as u8)
    }

    pub fn set_beep_sin_x(&mut self, val: u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_reg(0, BEEP_SIN_X_MSB, (val >> 8) as u8)?;
        self.write_reg(0, BEEP_SIN_X_LSB, (val & 0xff) as u8)
    }

    pub fn set_class_d_spk_amp(&mut self, powered_up: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, CLASS_D_SPEAKER_AMPLIFIER)?;
        reg_val |= (powered_up as u8) << 7;
        self.write_reg(1, CLASS_D_SPEAKER_AMPLIFIER, reg_val)
    }

    pub fn set_class_d_spk_driver(&mut self, output_stage_gain: OutputStage, driver_muted: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, CLASS_D_SPK_DRIVER)?;
        set_bits(&mut reg_val, output_stage_gain as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, !driver_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, CLASS_D_SPK_DRIVER, reg_val)
    }

    pub fn set_clock_gen_muxing(&mut self, pll_clkin: PllClkin, codec_clkin: CodecClkin) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = (pll_clkin as u8) << 2;
        reg_val |= codec_clkin as u8;
        self.write_reg(0, CLOCK_GEN_MUXING, reg_val)
    }

    pub fn set_clkout_m_val(&mut self, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider == 0 || divider == 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, CLKOUT_M_VAL, reg_val)
    }

    pub fn set_clkout_mux(&mut self, cdiv_clkin: CdivClkin) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, CLKOUT_MUX)?;
        set_bits(&mut reg_val, cdiv_clkin as u8, 0, 0b0000_0111);
        self.write_reg(0, CLKOUT_MUX, reg_val)
    }

    pub fn set_codec_interface_control_1(
        &mut self,
        codec_interface: CodecInterface,
        word_length: CodecInterfaceWordLength,
        bclk_output: bool,
        wclk_output: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_1)?;
        set_bits(&mut reg_val, codec_interface as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, word_length as u8, 4, 0b0011_0000);
        set_bits(&mut reg_val, bclk_output as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, wclk_output as u8, 2, 0b0000_0100);
        self.write_reg(0, CODEC_INTERFACE_CONTROL_1, reg_val)
    }

    pub fn set_codec_interface_control_2(
        &mut self,
        bclk_inverted: bool,
        bclk_wclk_always_active: bool,
        bdiv_clkin: BdivClkin
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_2)?;
        set_bits(&mut reg_val, bclk_inverted as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, bclk_wclk_always_active as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, bdiv_clkin as u8, 0, 0b0000_0011);
        self.write_reg(0, CODEC_INTERFACE_CONTROL_2, reg_val)
    }

    pub fn set_codec_secondary_interface_control_1(
        &mut self,
        bclk_from_gpio1: bool,
        wclk_from_gpio1: bool,
        din_from_gpio1: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = (!bclk_from_gpio1 as u8) << 5;
        set_bits(&mut reg_val, !wclk_from_gpio1 as u8, 2, 0b1100);
        set_bits(&mut reg_val, !din_from_gpio1 as u8, 0, 0b11);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_1, reg_val)
    }

    pub fn set_codec_secondary_interface_control_2(
        &mut self,
        secondary_bclk: bool,
        secondary_wclk: bool,
        secondary_din: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_2)?;
        set_bits(&mut reg_val, secondary_bclk as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, secondary_wclk as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, secondary_din as u8, 0, 0b0000_0001);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_2, reg_val)
    }

    pub fn set_codec_secondary_interface_control_3(
        &mut self,
        primary_bclk: PrimaryBclkOutput,
        secondary_bclk: SecondaryBclkOutput,
        primary_wclk: PrimaryWclkOutput,
        secondary_wclk: SecondaryWclkOutput
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_3)?;
        set_bits(&mut reg_val, primary_bclk as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, secondary_bclk as u8, 6, 0b100_0000);
        set_bits(&mut reg_val, primary_wclk as u8, 4, 0b11_0000);
        set_bits(&mut reg_val, secondary_wclk as u8, 2, 0b1100);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_3, reg_val)
    }

    pub fn set_dac_data_path_setup(
        &mut self,
        left_powered: bool,
        right_powered: bool,
        left_data_path: LeftDataPath,
        right_data_path: RightDataPath,
        soft_stepping: SoftStepping
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val: u8 = (left_powered as u8) << 7;
        reg_val |= (right_powered as u8) << 6;
        reg_val |= (left_data_path as u8) << 4;
        reg_val |= (right_data_path as u8) << 2;
        reg_val |= soft_stepping as u8;
        self.write_reg(0, DAC_DATA_PATH_SETUP, reg_val)
    }

    pub fn set_dac_dosr_val(&mut self, osr: u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb = ((0xff00 & osr) >> 8) as u8;
        if msb > 3 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let lsb: u8 = (0x00ff & osr) as u8;
        if lsb == 1 || lsb == 255 { return Err(TLV320DAC3100Error::InvalidArgument) }

        self.write_reg(0, DAC_DOSR_VAL_MSB, msb)?;
        self.write_reg(0, DAC_DOSR_VAL_LSB, lsb)
    }

    pub fn set_dac_l_and_dac_r_output_mixer_routing(
        &mut self,
        left_routing: DacLeftOutputMixerRouting,
        ain1_input_routed_left: bool,
        ain2_input_routed_left: bool,
        right_routing: DacRightOutputMixerRouting,
        ain2_input_routed_right: bool,
        hpl_output_routed_to_hpr: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING)?;
        set_bits(&mut reg_val, left_routing as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, ain1_input_routed_left as u8, 5, 0b0010_0000);
        set_bits(&mut reg_val, ain2_input_routed_left as u8, 4, 0b0001_0000);
        set_bits(&mut reg_val, right_routing as u8, 2, 0b0000_1100);
        set_bits(&mut reg_val, ain2_input_routed_right as u8, 1, 0b0000_0010);
        set_bits(&mut reg_val, hpl_output_routed_to_hpr as u8, 0, 0b0000_0001);
        self.write_reg(1, DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING, reg_val)
    }

    pub fn set_dac_left_volume_control(&mut self, db: f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.set_dac_channel_volume_control(true, db)
    }

    pub fn set_dac_processing_block_selection(&mut self, prb: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if prb == 0 || prb > 25 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, DAC_PROCESSING_BLOCK_SECTION)?;
        set_bits(&mut reg_val, prb, 0, 0b0001_1111);
        self.write_reg(0, DAC_PROCESSING_BLOCK_SECTION, reg_val)
    }

    pub fn set_dac_right_volume_control(&mut self, db: f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.set_dac_channel_volume_control(false, db)
    }

    pub fn set_dac_mdac_val(&mut self, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider == 0 || divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, DAC_MDAC_VAL, reg_val)
    }

    pub fn set_dac_ndac_val(&mut self, powered: bool, divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if divider == 0 || divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, DAC_NDAC_VAL, reg_val)
    }

    pub fn set_dac_volume_control(
        &mut self,
        left_muted: bool,
        right_muted: bool,
        control: VolumeControl
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, DAC_VOLUME_CONTROL)?;
        set_bits(&mut reg_val, left_muted as u8, 3, 0b1000);
        set_bits(&mut reg_val, right_muted as u8, 2, 0b100);
        set_bits(&mut reg_val, control as u8, 0, 0b11);
        self.write_reg(0, DAC_VOLUME_CONTROL, reg_val)
    }

    fn set_dac_channel_volume_control(&mut self, left_channel: bool, db: f32) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if db < -63.5f32 || db > 24.0f32 {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }

        let reg_val = (db * 2.0) as i8;
        self.write_reg(0, if left_channel { DAC_LEFT_VOLUME_CONTROL } else { DAC_RIGHT_VOLUME_CONTROL }, reg_val as u8)
    }

    pub fn set_data_slot_offset_programmability(&mut self, offset: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_reg(0, DATA_SLOT_OFFSET_PROGRAMMABILITY, offset)
    }

    pub fn set_din_control(&mut self, din_control: DinControl) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, DIN_CONTROL)?;
        set_bits(&mut reg_val, din_control as u8, 1, 0b0000_0110);
        self.write_reg(0, DIN_CONTROL, reg_val)
    }

    pub fn set_drc_control_1(
        &mut self,
        left: bool,
        right: bool,
        threshold: u8,
        hysteresis: u8
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if threshold > 7 || hysteresis > 3 { return Err(TLV320DAC3100Error::InvalidArgument) }
        let mut reg_val = self.read_reg(0, DRC_CONTROL_1)?;
        set_bits(&mut reg_val, left as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, right as u8, 5, 0b0010_0000);
        set_bits(&mut reg_val, threshold, 2, 0b0001_1100);
        set_bits(&mut reg_val, hysteresis, 0, 0b0000_0011);
        self.write_reg(0, DRC_CONTROL_1, reg_val)
    }

    pub fn set_drc_control_2(&mut self, hold_time: HoldTime) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, DRC_CONTROL_2)?;
        set_bits(&mut reg_val, hold_time as u8, 3, 0b0111_1000);
        self.write_reg(0, DRC_CONTROL_2, reg_val)
    }

    pub fn set_drc_control_3(&mut self, attack_rate: u8, decay_rate: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if attack_rate > 15 || decay_rate > 15 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, DRC_CONTROL_3)?;
        set_bits(&mut reg_val, attack_rate, 4, 0b1111_0000);
        set_bits(&mut reg_val, decay_rate, 0, 0b0000_1111);
        self.write_reg(0, DRC_CONTROL_3, reg_val)
    }

    pub fn set_gpio1_io_pin_control(&mut self, mode: Gpio1Mode) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, GPIO1_IN_OUT_PIN_CONTROL)?;
        set_bits(&mut reg_val, mode as u8, 2, 0b0011_1100);
        self.write_reg(0, GPIO1_IN_OUT_PIN_CONTROL, reg_val)
    }

    pub fn set_headphone_and_speaker_amplifier_error_control(
        &mut self,
        preserve_spk_ctrl_bits: bool,
        preserve_hp_ctrl_bits: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL)?;
        set_bits(&mut reg_val, preserve_spk_ctrl_bits as u8, 1, 0b10);
        set_bits(&mut reg_val, preserve_hp_ctrl_bits as u8, 0, 0b1);
        self.write_reg(1, HEADPHONE_AND_SPEAKER_AMPLIFIER_ERROR_CONTROL, reg_val)
    }

    pub fn set_headphone_drivers(
        &mut self,
        left_powered: bool,
        right_powered: bool,
        common: HpOutputVoltage,
        power_down_on_scd: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, HEADPHONE_DRIVERS)?;
        set_bits(&mut reg_val, left_powered as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, right_powered as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, common as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, power_down_on_scd as u8, 1, 0b0000_0010);
        self.write_reg(1, HEADPHONE_DRIVERS, reg_val)
    }

    pub fn set_headset_detection(&mut self, enabled: bool, debounce: HeadsetDetectionDebounce, button_debounce: HeadsetButtonPressDebounce) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, HEADSET_DETECTION)?;
        set_bits(&mut reg_val, enabled as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, debounce as u8, 2, 0b0001_1100);
        set_bits(&mut reg_val, button_debounce as u8, 0, 0b0001_0011);
        self.write_reg(0, HEADSET_DETECTION, reg_val)
    }

    pub fn set_hp_driver_control(
        &mut self,
        scd_debounce: HpScdDebounce,
        mode: HpMode,
        hpl_lineout: bool,
        hpr_lineout: bool,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, HP_DRIVER_CONTROL)?;
        set_bits(&mut reg_val, scd_debounce as u8, 5, 0b1110_0000);
        set_bits(&mut reg_val, mode as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, hpl_lineout as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, hpr_lineout as u8, 1, 0b0000_0010);
        self.write_reg(1, HP_DRIVER_CONTROL, reg_val)
    }

    pub fn set_hp_output_drivers_pop_removal_settings(
        &mut self,
        optimize_power_down_pop: bool,
        power_on: HpPowerOn,
        ramp_up: HpRampUp,
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS)?;
        set_bits(&mut reg_val, optimize_power_down_pop as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, power_on as u8, 3, 0b0111_1000);
        set_bits(&mut reg_val, ramp_up as u8, 1, 0b0000_0110);
        self.write_reg(1, HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS, reg_val)
    }

    pub fn set_hpl_driver(&mut self, pga: u8, hpl_muted: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if pga > 9 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, HPL_DRIVER)?;
        set_bits(&mut reg_val, pga, 3, 0b0111_1000);
        set_bits(&mut reg_val, hpl_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, HPL_DRIVER, reg_val)
    }

    pub fn set_hpr_driver(&mut self, pga: u8, hpr_muted: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if pga > 9 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, HPR_DRIVER)?;
        set_bits(&mut reg_val, pga, 3, 0b0111_1000);
        set_bits(&mut reg_val, !hpr_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, HPR_DRIVER, reg_val)
    }

    pub fn set_i2c_bus_condition(&mut self, accept_address: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, I2C_BUS_CONDITION)?;
        set_bits(&mut reg_val, accept_address as u8, 5, 0b0010_0000);
        self.write_reg(0, I2C_BUS_CONDITION, reg_val)
    }

    pub fn set_input_cm_settings(&mut self, ain1_to_cm: bool, ain2_to_cm: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, INPUT_CM_SETTINGS)?;
        set_bits(&mut reg_val, ain1_to_cm as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, ain2_to_cm as u8, 6, 0b0100_0000);
        self.write_reg(1, INPUT_CM_SETTINGS, reg_val)
    }

    pub fn set_int_control_register(
        &mut self,
        int1: bool,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let reg_addr = if int1 { INT1_CONTROL_REGISTER } else { INT2_CONTROL_REGISTER };
        let mut reg_val = self.read_reg(0, reg_addr)?;
        set_bits(&mut reg_val, headset_detect as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, button_press as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, use_drc_signal_power as u8, 5, 0b010_0000);
        set_bits(&mut reg_val, short_circuit as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, data_overflow as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, multiple_pulses as u8, 0, 0b0000_0001);
        self.write_reg(0, reg_addr, reg_val)
    }

    pub fn set_int1_control_register(
        &mut self,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.set_int_control_register(true, headset_detect, button_press, use_drc_signal_power, short_circuit, data_overflow, multiple_pulses)
    }

    pub fn set_int2_control_register(
        &mut self,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.set_int_control_register(false, headset_detect, button_press, use_drc_signal_power, short_circuit, data_overflow, multiple_pulses)
    }

    pub fn set_left_analog_volume_to_hpl(&mut self, route_to_hpl: bool, gain: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_HPL)?;
        set_bits(&mut reg_val, route_to_hpl as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, LEFT_ANALOG_VOLUME_TO_HPL, reg_val)
    }

    pub fn set_left_beep_generator(&mut self, enabled: bool, volume: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if volume > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, LEFT_BEEP_GENERATOR)?;
        set_bits(&mut reg_val, enabled as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, volume, 0, 0b0011_1111);
        self.write_reg(0, LEFT_BEEP_GENERATOR, reg_val)
    }

    pub fn set_left_analog_volume_to_spk(&mut self, route_to_class_d: bool, gain: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_SPK)?;
        set_bits(&mut reg_val, route_to_class_d as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, LEFT_ANALOG_VOLUME_TO_SPK, reg_val)
    }

    pub fn set_micbias(
        &mut self,
        power_down: bool,
        always_on: bool,
        output: MicBiasOutput
    ) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, MICBIAS)?;
        set_bits(&mut reg_val, power_down as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, always_on as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, output as u8, 0, 0b0000_0011);
        self.write_reg(1, MICBIAS, reg_val)
    }

    pub fn set_output_driver_pga_ramp_down_period_control(&mut self, ramp_down: PgaRampDown) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(1, OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL)?;
        set_bits(&mut reg_val, ramp_down as u8, 4, 0b0111_0000);
        self.write_reg(1, OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL, reg_val)
    }

    pub fn set_pll_d_value(&mut self, d: u16) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let msb: u8 = ((d >> 8) & 0b0011_1111) as u8;
        self.write_reg(0, PLL_D_VALUE_MSB, msb)?;
        self.write_reg(0, PLL_D_VALUE_LSB, (0xff & d) as u8)
    }

    pub fn set_pll_j_value(&mut self, j: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if j == 0 || j > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, PLL_J_VALUE)?;
        set_bits(&mut reg_val, j, 0, 0b0011_1111);
        self.write_reg(0, PLL_J_VALUE, reg_val)
    }

    pub fn set_pll_p_and_r_values(&mut self, powered: bool, p: u8, r: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if p == 0 || p > 8 { return Err(TLV320DAC3100Error::InvalidArgument) }
        if r == 0 || r > 16 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= (if p == 0 { 0x0 } else { p }) << 4;
        reg_val |= if r == 16 { 0x0 } else { r };
        self.write_reg(0, PLL_P_AND_R_VALUES, reg_val)
    }

    pub fn set_right_analog_volume_to_hpr(&mut self, route_to_hpr: bool, gain: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, RIGHT_ANALOG_VOLUME_TO_HPR)?;
        set_bits(&mut reg_val, route_to_hpr as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, RIGHT_ANALOG_VOLUME_TO_HPR, reg_val)
    }

    pub fn set_right_beep_generator(&mut self, mode: RightBeepMode, volume: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if volume > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, RIGHT_BEEP_GENERATOR)?;
        set_bits(&mut reg_val, mode as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, volume, 0, 0b0011_1111);
        self.write_reg(0, RIGHT_BEEP_GENERATOR, reg_val)
    }

    pub fn set_software_reset(&mut self, reset: bool) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.write_reg(0, SOFTWARE_RESET, reset as u8)
    }

    pub fn set_timer_clock_mclk_divider(&mut self, external_mclk: bool, mclk_divider: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        if mclk_divider == 0 || mclk_divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(3, TIMER_CLOCK_MCLK_DIVIDER)?;
        set_bits(&mut reg_val, external_mclk as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, mclk_divider, 0, 0b0111_1111);
        self.write_reg(3, TIMER_CLOCK_MCLK_DIVIDER, reg_val)
    }

    pub fn set_vol_micdet_pin_sar_adc(&mut self, pin_control: bool, use_mclk: bool, hysteresis: VolumeControlHysteresis, throughput: VolumeControlThroughput) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        let mut reg_val = self.read_reg(0, VOL_MICDET_PIN_SAR_ADC)?;
        set_bits(&mut reg_val, pin_control as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, use_mclk as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, hysteresis as u8, 4, 0b0011_0000);
        set_bits(&mut reg_val, throughput as u8, 0, 0b0000_0111);
        self.write_reg(0, VOL_MICDET_PIN_SAR_ADC, reg_val)
    }

    fn write_reg(&mut self, page: u8, reg_addr: u8, reg_val: u8) -> Result<(), TLV320DAC3100Error<I2C::Error>> {
        self.select_page(page)?;
        Ok(self.i2c.write(I2C_DEVICE_ADDRESS, &[reg_addr, reg_val]).map_err(TLV320DAC3100Error::I2C)?)
    }
}