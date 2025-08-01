use embedded_hal as hal;
use hal::i2c::I2c;
use crate::bits::set_bits;
use crate::error::TLV320DAC3100Error;
use crate::registers::*;
use crate::typedefs::*;

pub const I2C_DEVICE_ADDRESS: u8 = 0x18;
const MAX_DAC_VOLUME_CONTROL_DB: f32 = 24.0;
const MIN_DAC_VOLUME_CONTROL_DB: f32 = -63.5;

pub struct TLV320DAC3100<I2C> {
    i2c: I2C
}

// TODO compiler warning about bounds
type TLV320Result<I2C: I2c> = Result<(), TLV320DAC3100Error<I2C::Error>>;

impl<I2C: I2c> TLV320DAC3100<I2C> {
    pub fn new(i2c: I2C) -> Self {
        TLV320DAC3100 { i2c }
    }

    pub fn get_class_d_spk_driver(&mut self, gain: &mut OutputStage, muted: &mut bool, gains_applied: &mut bool) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLASS_D_SPK_DRIVER)?;
        *gain = ((reg_val & 0x18) >> 3).try_into().unwrap();
        *muted = (reg_val & 0x4) >> 2 == 0;
        *gains_applied = (reg_val & 0x1) == 1;
        Ok(())
    }

    pub fn get_clkout_m_val(&mut self, powered: &mut bool, divider: &mut u8) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLKOUT_M_VAL)?;
        *powered = (reg_val & 0b1000_0000) >> 7 == 1;
        *divider = reg_val & 0b0111_1111;
        Ok(())
    }

    pub fn get_clkout_mux(&mut self, cdiv_clkin: &mut CdivClkin) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLKOUT_MUX)?;
        *cdiv_clkin = (reg_val & 0b0000_0111).try_into().unwrap();
        Ok(())
    }

    pub fn get_clock_gen_muxing(&mut self, pll_clkin: &mut PllClkin, codec_clkin: &mut CodecClkin) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CLOCK_GEN_MUXING)?;
        *pll_clkin = ((reg_val & 0b1100) >> 2).try_into().unwrap();
        *codec_clkin = (reg_val & 0b11).try_into().unwrap();
        Ok(())
    }

    pub fn get_codec_interface_control_1(
        &mut self,
        codec_interface: &mut CodecInterface,
        word_length: &mut CodecInterfaceWordLength,
        bclk_output: &mut bool,
        wclk_output: &mut bool
    ) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_1)?;
        *codec_interface = ((reg_val >> 6) & 0b11).try_into().unwrap();
        *word_length = ((reg_val >> 4) & 0b11).try_into().unwrap();
        *bclk_output = ((reg_val >> 3) & 0b1) == 1;
        *wclk_output = ((reg_val >> 2) & 0b1) == 1;
        Ok(())
    }

    // TODO test
    pub fn get_dac_flag_register_1(
        &mut self,
        dac_left_powered: &mut bool,
        hp_left_powered: &mut bool,
        amp_left_powered: &mut bool,
        dac_right_powered: &mut bool,
        hp_right_powered: &mut bool,
        amp_right_powered: &mut bool,
    ) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_FLAG_REGISTER_1)?;
        *dac_left_powered = ((reg_val >> 7) & 0b1) == 1;
        *hp_left_powered = ((reg_val >> 5) & 0b1) == 1;
        *amp_left_powered = ((reg_val >> 4) & 0b1) == 1;
        *dac_right_powered = ((reg_val >> 3) & 0b1) == 1;
        *hp_right_powered = ((reg_val >> 1) & 0b1) == 1;
        *amp_right_powered = (reg_val & 0b1) == 1;
        Ok(())
    }

    // TODO test
    pub fn get_dac_flag_register_2(&mut self, left_gain_equiv: &mut bool, right_gain_equiv: &mut bool) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_FLAG_REGISTER_2)?;
        *left_gain_equiv = ((reg_val >> 4) & 0b1) == 1;
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
    ) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_INTERRUPT_FLAGS_STICKY_BITS)?;
        *left_scd = (reg_val >> 7) & 0b1 == 1;
        *right_scd = (reg_val >> 6) & 0b1 == 1;
        *headset_button_pressed = (reg_val >> 5) & 0b1 == 1;
        *headset_insert_removal_detected = (reg_val >> 4) & 0b1 == 1;
        *left_dac_signal_above_drc = (reg_val >> 3) & 0b1 == 1;
        *right_dac_signal_above_drc = (reg_val >> 2) & 0b1 == 1;
        Ok(())
    }

    pub fn get_dac_mdac_val(&mut self, powered: &mut bool, divider: &mut u8) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_MDAC_VAL)?;
        *powered = (reg_val & 0b1000_0000) >> 7 == 1;
        *divider = reg_val & 0b0111_1111;
        Ok(())
    }

    pub fn get_dac_ndac_val(&mut self, powered: &mut bool, divider: &mut u8) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_NDAC_VAL)?;
        *powered = (reg_val & 0b1000_0000) >> 7 == 1;
        *divider = reg_val & 0b0111_1111;
        Ok(())
    }

    pub fn get_dac_processing_block_selection(&mut self, prb: &mut u8) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, DAC_PROCESSING_BLOCK_SECTION)?;
        *prb = reg_val & 0b0001_1111;
        Ok(())
    }

    // TODO test
    pub fn get_interrupt_flags(
        &mut self,
        hpl_scd_detected: &mut bool,
        hpr_scd_detected: &mut bool,
        headset_btn_pressed: &mut bool,
        headset_insertion_detected: &mut bool,
        left_signal_gt_drc: &mut bool,
        right_signal_gt_drc: &mut bool,
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, INTERRUPT_FLAGS_DAC)?;
        *hpl_scd_detected = ((reg_val >> 7) & 0b1) == 1;
        *hpr_scd_detected = ((reg_val >> 6) & 0b1) == 1;
        *headset_btn_pressed = (reg_val >> 5) & 0b1 == 1;
        *headset_insertion_detected = (reg_val >> 4) & 0b1 == 1;
        *left_signal_gt_drc = (reg_val >> 3) & 0b1 == 1;
        *right_signal_gt_drc = (reg_val >> 2) & 0b1 == 1;
        Ok(())
    }

    pub fn get_headset_detection(
        &mut self,
        enabled: &mut bool,
        detected: &mut HeadsetDetected,
        debounce: &mut HeadsetDetectionDebounce,
        button_debounce: &mut HeadsetButtonPressDebounce
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, HEADSET_DETECTION)?;
        *enabled = (reg_val >> 7) == 1;
        *detected = ((reg_val >> 5) & 0b11).try_into().unwrap();
        *debounce = ((reg_val >> 2) &0b111).try_into().unwrap();
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
    ) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(1, HEADPHONE_DRIVERS)?;
        *left_powered = (reg_val >> 7) & 0x1 == 1;
        *right_powered = (reg_val >> 6) & 0x1 == 1;
        *output_voltage = ((reg_val & 0b0001_1000) >> 3).try_into().unwrap();
        *power_down_on_scd = (reg_val >> 1) & 0x1 == 1;
        *scd_detected = reg_val & 0x1 == 1;
        Ok(())
    }

    pub fn get_ot_flag(&mut self, ot_flag: &mut bool) -> TLV320Result<I2C> {
        *ot_flag = self.read_reg(0, OT_FLAG)? >> 1 == 0; // active low
        Ok(())

    }

    // TODO test
    pub fn get_overflow_flags(&mut self, left: &mut bool, right: &mut bool, barrel_shifter: &mut bool) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(1, OVERFLOW_FLAGS)?;
        *left = ((reg_val >> 7) & 0b1) == 1;
        *right = ((reg_val >> 6) & 0b1) == 1;
        *barrel_shifter = ((reg_val >> 5) & 0b1) == 1;
        Ok(())
    }

    pub fn get_pll_d_value(&mut self, d: &mut u16) -> TLV320Result<I2C> {
        let msb = self.read_reg(0, PLL_D_VALUE_MSB)? & 0b11_1111;
        let lsb = self.read_reg(0, PLL_D_VALUE_LSB)?;
        *d = (msb as u16) << 8 | lsb as u16;
        Ok(())
    }

    pub fn get_pll_j_value(&mut self, j: &mut u8) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, PLL_J_VALUE)?;
        *j = reg_val & 0b111111;
        Ok(())
    }

    pub fn get_pll_p_and_r_values(&mut self, powered: &mut bool, p: &mut u8, r: &mut u8) -> TLV320Result<I2C> {
        let raw = self.read_reg(0, PLL_P_AND_R_VALUES)?;
        *powered = raw >> 7 == 1;
        *p = (0b111_0000 & raw) >> 4;
        *r = 0b1111 & raw;
        Ok(())
    }

    pub fn get_vol_micdet_pin_gain(&mut self, gain: &mut u8) -> TLV320Result<I2C> {
        *gain = self.read_reg(0, VOL_MICDET_PIN_GAIN)?;
        Ok(())
    }

    pub fn get_vol_micdet_pin_sar_adc(&mut self, pin_control: &mut bool, use_mclk: &mut bool, hysteresis: &mut VolumeControlHysteresis, throughput: &mut VolumeControlThroughput) -> TLV320Result<I2C> {
        let reg_val = self.read_reg(0, VOL_MICDET_PIN_SAR_ADC)?;
        *pin_control = (reg_val >> 7) & 0b1 == 1;
        *use_mclk = (reg_val >> 6) & 0b1 == 1;
        *hysteresis = ((reg_val >> 4) & 0b11).try_into().unwrap();
        *throughput = (reg_val & 0b111).try_into().unwrap();
        Ok(())
    }

    // TODO test
    pub fn read_raw_reg(&mut self, page: u8, reg: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.read_reg(page, reg)
    }

    fn read_reg(&mut self, page: u8, reg: u8) -> Result<u8, TLV320DAC3100Error<I2C::Error>> {
        self.select_page(page)?;
        let mut buf = [0u8];
        self.i2c.write_read(I2C_DEVICE_ADDRESS, &[reg], &mut buf).map_err(TLV320DAC3100Error::I2C)?;
        Ok(buf[0])
    }

    fn select_page(&mut self, page: u8) -> TLV320Result<I2C> {
        Ok(self.i2c.write(I2C_DEVICE_ADDRESS, &[PAGE_CONTROL, page]).map_err(TLV320DAC3100Error::I2C)?)
    }

    // TODO getter
    // TODO test
    pub fn set_bclk_n_val(&mut self, powered: bool, divider: u8) -> TLV320Result<I2C> {
        let mut reg_val = (powered as u8) << 7;
        if divider < 128 {
            set_bits(&mut reg_val, divider, 0, 0b0111_1111);
        }
        self.write_reg(0, BCLK_N_VAL, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_beep_length(&mut self, samples: u32) -> TLV320Result<I2C> {
        if samples > 16777215 { return Err(TLV320DAC3100Error::InvalidArgument) }

        self.write_reg(0, BEEP_LENGTH_MSB, ((samples >> 16) & 0xff) as u8)?;
        self.write_reg(0, BEEP_LENGTH_MSB, ((samples >> 8) & 0xff) as u8)?;
        self.write_reg(0, BEEP_LENGTH_MSB, (samples & 0xff) as u8)
    }

    // TODO getter
    // TODO test
    pub fn set_beep_cos_x(&mut self, val: u16) -> TLV320Result<I2C> {
        self.write_reg(0, BEEP_COS_X_MSB, (val >> 8) as u8)?;
        self.write_reg(0, BEEP_COS_X_LSB, (val & 0xff) as u8)
    }

    // TODO getter
    // TODO test
    pub fn set_beep_sin_x(&mut self, val: u16) -> TLV320Result<I2C> {
        self.write_reg(0, BEEP_SIN_X_MSB, (val >> 8) as u8)?;
        self.write_reg(0, BEEP_SIN_X_LSB, (val & 0xff) as u8)
    }

    // TODO getter
    pub fn set_class_d_spk_amp(&mut self, powered_up: bool) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, CLASS_D_SPEAKER_AMPLIFIER)?;
        reg_val |= (powered_up as u8) << 7;
        self.write_reg(1, CLASS_D_SPEAKER_AMPLIFIER, reg_val)
    }

    pub fn set_class_d_spk_driver(&mut self, output_stage_gain: OutputStage, driver_muted: bool) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, CLASS_D_SPK_DRIVER)?;
        set_bits(&mut reg_val, output_stage_gain as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, !driver_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, CLASS_D_SPK_DRIVER, reg_val)
    }

    pub fn set_clock_gen_muxing(&mut self, pll_clkin: PllClkin, codec_clkin: CodecClkin) -> TLV320Result<I2C> {
        let mut reg_val = (pll_clkin as u8) << 2;
        reg_val |= codec_clkin as u8;
        self.write_reg(0, CLOCK_GEN_MUXING, reg_val)
    }

    pub fn set_clkout_m_val(&mut self, powered: bool, divider: u8) -> TLV320Result<I2C> {
        if divider == 0 || divider == 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, CLKOUT_M_VAL, reg_val)
    }

    pub fn set_clkout_mux(&mut self, cdiv_clkin: CdivClkin) -> TLV320Result<I2C> {
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
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_1)?;
        set_bits(&mut reg_val, codec_interface as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, word_length as u8, 4, 0b0011_0000);
        set_bits(&mut reg_val, bclk_output as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, wclk_output as u8, 2, 0b0000_0100);
        self.write_reg(0, CODEC_INTERFACE_CONTROL_1, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_codec_interface_control_2(
        &mut self,
        bclk_inverted: bool,
        bclk_wclk_always_active: bool,
        bdiv_clkin: BdivClkin
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, CODEC_INTERFACE_CONTROL_2)?;
        set_bits(&mut reg_val, bclk_inverted as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, bclk_wclk_always_active as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, bdiv_clkin as u8, 0, 0b0000_0011);
        self.write_reg(0, CODEC_INTERFACE_CONTROL_2, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_codec_secondary_interface_control_1(
        &mut self,
        bclk_from_gpio1: bool,
        wclk_from_gpio1: bool,
        din_from_gpio1: bool
    ) -> TLV320Result<I2C> {
        let mut reg_val = (bclk_from_gpio1 as u8) << 5;
        set_bits(&mut reg_val, wclk_from_gpio1 as u8, 2, 0b1100);
        set_bits(&mut reg_val, din_from_gpio1 as u8, 0, 0b11);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_1, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_codec_secondary_interface_control_2(
        &mut self,
        secondary_bclk: bool,
        secondary_wclk: bool,
        secondary_din: bool
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_2)?;
        set_bits(&mut reg_val, secondary_bclk as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, secondary_wclk as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, secondary_din as u8, 0, 0b0000_0001);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_2, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_codec_secondary_interface_control_3(
        &mut self,
        primary_bclk: PrimaryBclkOutput,
        secondary_bclk: SecondaryBclkOutput,
        primary_wclk: PrimaryWclkOutput,
        secondary_wclk: SecondaryWclkOutput
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_3)?;
        set_bits(&mut reg_val, primary_bclk as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, secondary_bclk as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, primary_wclk as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, secondary_wclk as u8, 1, 0b0000_110);
        self.write_reg(0, CODEC_SECONDARY_INTERFACE_CONTROL_3, reg_val)
    }

    // TODO getter
    pub fn set_dac_data_path_setup(
        &mut self,
        left_powered: bool,
        right_powered: bool,
        left_data_path: LeftDataPath,
        right_data_path: RightDataPath,
        soft_stepping: SoftStepping
    ) -> TLV320Result<I2C> {
        let mut reg_val: u8 = (left_powered as u8) << 7;
        reg_val |= (right_powered as u8) << 6;
        reg_val |= (left_data_path as u8) << 4;
        reg_val |= (right_data_path as u8) << 2;
        reg_val |= soft_stepping as u8;
        self.write_reg(0, DAC_DATA_PATH_SETUP, reg_val)
    }

    pub fn set_dac_dosr_val(&mut self, osr: u16) -> TLV320Result<I2C> {
        let msb = ((0xff00 & osr) >> 8) as u8;
        if msb > 3 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let lsb: u8 = (0x00ff & osr) as u8;
        if lsb == 1 || lsb == 255 { return Err(TLV320DAC3100Error::InvalidArgument) }

        self.write_reg(0, DAC_DOSR_VAL_MSB, msb)?;
        self.write_reg(0, DAC_DOSR_VAL_LSB, lsb)
    }

    // TODO getter
    pub fn set_dac_l_and_dac_r_output_mixer_routing(
        &mut self,
        left_routing: DacLeftOutputMixerRouting,
        ain1_input_routed_left: bool,
        ain2_input_routed_left: bool,
        right_routing: DacRightOutputMixerRouting,
        ain2_input_routed_right: bool,
        hpl_output_routed_to_hpr: bool
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING)?;
        set_bits(&mut reg_val, left_routing as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, ain1_input_routed_left as u8, 5, 0b0010_0000);
        set_bits(&mut reg_val, ain2_input_routed_left as u8, 4, 0b0001_0000);
        set_bits(&mut reg_val, right_routing as u8, 2, 0b0000_1100);
        set_bits(&mut reg_val, ain2_input_routed_right as u8, 1, 0b0000_0010);
        set_bits(&mut reg_val, hpl_output_routed_to_hpr as u8, 0, 0b0000_0001);
        self.write_reg(1, DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING, reg_val)
    }

    // TODO getter
    pub fn set_dac_left_volume_control(&mut self, db: f32) -> TLV320Result<I2C> {
        self.set_dac_channel_volume_control(true, db)
    }

    pub fn set_dac_processing_block_selection(&mut self, prb: u8) -> TLV320Result<I2C> {
        if prb == 0 || prb > 25 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, DAC_PROCESSING_BLOCK_SECTION)?;
        set_bits(&mut reg_val, prb, 0, 0b0001_1111);
        self.write_reg(0, DAC_PROCESSING_BLOCK_SECTION, reg_val)
    }

    // TODO getter
    pub fn set_dac_right_volume_control(&mut self, db: f32) -> TLV320Result<I2C> {
        self.set_dac_channel_volume_control(false, db)
    }

    pub fn set_dac_mdac_val(&mut self, powered: bool, divider: u8) -> TLV320Result<I2C> {
        if divider == 0 || divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, DAC_MDAC_VAL, reg_val)
    }

    pub fn set_dac_ndac_val(&mut self, powered: bool, divider: u8) -> TLV320Result<I2C> {
        if divider == 0 || divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= divider;
        self.write_reg(0, DAC_NDAC_VAL, reg_val)
    }

    // TODO getter
    pub fn set_dac_volume_control(
        &mut self,
        left_muted: bool,
        right_muted: bool,
        control: VolumeControl
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, DAC_VOLUME_CONTROL)?;
        set_bits(&mut reg_val, left_muted as u8, 3, 0b1000);
        set_bits(&mut reg_val, right_muted as u8, 2, 0b100);
        set_bits(&mut reg_val, control as u8, 0, 0b11);
        self.write_reg(0, DAC_VOLUME_CONTROL, reg_val)
    }

    fn set_dac_channel_volume_control(&mut self, left_channel: bool, db: f32) -> TLV320Result<I2C> {
        if db < MIN_DAC_VOLUME_CONTROL_DB || db > MAX_DAC_VOLUME_CONTROL_DB {
            return Err(TLV320DAC3100Error::InvalidArgument)
        }

        let reg_val = (db * 2.0) as i8;
        self.write_reg(0, if left_channel { DAC_LEFT_VOLUME_CONTROL } else { DAC_RIGHT_VOLUME_CONTROL }, reg_val as u8)
    }

    // TODO getter
    // TODO test
    pub fn set_data_slot_offset_programmability(&mut self, offset: u8) -> TLV320Result<I2C> {
        self.write_reg(0, DATASLOT_OFFSET_PROGRAMMABILITY, offset)
    }

    // TODO getter
    // TODO test
    pub fn set_din_control(&mut self, din_control: DinControl) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, DIN_CONTROL)?;
        set_bits(&mut reg_val, din_control as u8, 1, 0b0000_0110);
        self.write_reg(0, DIN_CONTROL, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_drc_control_1(
        &mut self,
        left: bool,
        right: bool,
        threshold: u8,
        hysteresis: u8
    ) -> TLV320Result<I2C> {
        if hysteresis > 3 { return Err(TLV320DAC3100Error::InvalidArgument) }
        let mut reg_val = self.read_reg(0, DRC_CONTROL_1)?;
        set_bits(&mut reg_val, left as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, right as u8, 5, 0b0010_0000);
        set_bits(&mut reg_val, threshold, 2, 0b0001_1100);
        set_bits(&mut reg_val, hysteresis, 0, 0b0000_0011);
        self.write_reg(0, DRC_CONTROL_1, reg_val)
    }

    // TODO getter
    // TODO test
    // TODO enum instead of u8?
    pub fn set_drc_control_2(&mut self, hold_time: u8) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, DRC_CONTROL_2)?;
        set_bits(&mut reg_val, hold_time, 3, 0b0111_1000);
        self.write_reg(0, DRC_CONTROL_2, reg_val)
    }

    // TODO reg table in datasheet in strange. verify.
    // TODO getter
    // TODO test
    pub fn set_drc_control_3(&mut self, attack_rate: u8, decay_rate: u8) -> TLV320Result<I2C> {
        if attack_rate > 15 || decay_rate > 15 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, DRC_CONTROL_3)?;
        set_bits(&mut reg_val, attack_rate, 4, 0b1111_0000);
        set_bits(&mut reg_val, decay_rate, 0, 0b0000_1111);
        self.write_reg(0, DRC_CONTROL_3, reg_val)
    }

    // TODO getter
    pub fn set_gpio1_io_pin_control(&mut self, mode: Gpio1Mode) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, GPIO1_IN_OUT_PIN_CONTROL)?;
        set_bits(&mut reg_val, mode as u8, 2, 0b0011_1100);
        // TODO output value?
        self.write_reg(0, GPIO1_IN_OUT_PIN_CONTROL, reg_val)
    }

    // TODO test
    // TODO getter
    pub fn set_headphone_and_speaker_amplifier_error_control(
        &mut self,
        preserve_spk_ctrl_bits: bool,
        preserve_hp_ctrl_bits: bool
    ) -> TLV320Result<I2C> {
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
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, HEADPHONE_DRIVERS)?;
        set_bits(&mut reg_val, left_powered as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, right_powered as u8, 6, 0b0100_0000);
        set_bits(&mut reg_val, common as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, power_down_on_scd as u8, 1, 0b0000_0010);
        self.write_reg(1, HEADPHONE_DRIVERS, reg_val)
    }

    pub fn set_headset_detection(&mut self, enabled: bool, debounce: HeadsetDetectionDebounce, button_debounce: HeadsetButtonPressDebounce) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, HEADSET_DETECTION)?;
        set_bits(&mut reg_val, enabled as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, debounce as u8, 2, 0b0001_1100);
        set_bits(&mut reg_val, button_debounce as u8, 0, 0b0001_0011);
        self.write_reg(0, HEADSET_DETECTION, reg_val)
    }

    // TODO test
    // TODO getter
    pub fn set_hp_driver_control(
        &mut self,
        scd_debounce: HpScdDebounce,
        mode: HpMode,
        hpl_lineout: bool,
        hpr_lineout: bool,
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, HP_DRIVER_CONTROL)?;
        set_bits(&mut reg_val, scd_debounce as u8, 5, 0b1110_0000);
        set_bits(&mut reg_val, mode as u8, 3, 0b0001_1000);
        set_bits(&mut reg_val, hpl_lineout as u8, 2, 0b0000_0100);
        set_bits(&mut reg_val, hpr_lineout as u8, 1, 0b0000_0010);
        self.write_reg(1, HP_DRIVER_CONTROL, reg_val)
    }

    // TODO getter
    pub fn set_hp_output_drivers_pop_removal_settings(
        &mut self,
        optimize_power_down_pop: bool,
        power_on: HpPowerOn,
        ramp_up: HpRampUp,
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS)?;
        set_bits(&mut reg_val, optimize_power_down_pop as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, power_on as u8, 3, 0b0111_1000);
        set_bits(&mut reg_val, ramp_up as u8, 1, 0b0000_0110);
        self.write_reg(1, HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS, reg_val)
    }

    // TODO getter
    pub fn set_hpl_driver(&mut self, pga: u8, hpl_muted: bool) -> TLV320Result<I2C> {
        if pga > 9 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, HPL_DRIVER)?;
        set_bits(&mut reg_val, pga, 3, 0b0111_1000);
        set_bits(&mut reg_val, hpl_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, HPL_DRIVER, reg_val)
    }

    // TODO getter
    pub fn set_hpr_driver(&mut self, pga: u8, hpr_muted: bool) -> TLV320Result<I2C> {
        if pga > 9 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, HPR_DRIVER)?;
        set_bits(&mut reg_val, pga, 3, 0b0111_1000);
        set_bits(&mut reg_val, !hpr_muted as u8, 2, 0b0000_0100);
        self.write_reg(1, HPR_DRIVER, reg_val)
    }

    // TODO getter
    // TODO test
    pub fn set_i2c_bus_condition(&mut self, accept_address: bool) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(0, I2C_BUS_CONDITION)?;
        set_bits(&mut reg_val, accept_address as u8, 5, 0b0010_0000);
        self.write_reg(0, I2C_BUS_CONDITION, reg_val)
    }

    // TODO getter
    pub fn set_input_cm_settings(&mut self, ain1_floating: bool, ain2_floating: bool) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, INPUT_CM_SETTINGS)?;
        set_bits(&mut reg_val, ain1_floating as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, ain2_floating as u8, 6, 0b0100_0000);
        self.write_reg(1, INPUT_CM_SETTINGS, reg_val)
    }

    // TODO test
    pub fn set_int_control_register(
        &mut self,
        int1: bool,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> TLV320Result<I2C> {
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

    // TODO test
    // TODO getter
    pub fn set_int1_control_register(
        &mut self,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> TLV320Result<I2C> {
        self.set_int_control_register(true, headset_detect, button_press, use_drc_signal_power, short_circuit, data_overflow, multiple_pulses)
    }

    // TODO test
    // TODO getter
    pub fn set_int2_control_register(
        &mut self,
        headset_detect: bool,
        button_press: bool,
        use_drc_signal_power: bool,
        short_circuit: bool,
        data_overflow: bool,
        multiple_pulses: bool
    ) -> TLV320Result<I2C> {
        self.set_int_control_register(false, headset_detect, button_press, use_drc_signal_power, short_circuit, data_overflow, multiple_pulses)
    }

    // TODO getter
    pub fn set_left_analog_volume_to_hpl(&mut self, route_to_hpl: bool, gain: u8) -> TLV320Result<I2C> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_HPL)?;
        set_bits(&mut reg_val, route_to_hpl as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, LEFT_ANALOG_VOLUME_TO_HPL, reg_val)
    }

    // TODO test
    // TODO getter
    pub fn set_left_beep_generator(&mut self, enabled: bool, volume: u8) -> TLV320Result<I2C> {
        if volume > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_BEEP_GENERATOR)?;
        set_bits(&mut reg_val, enabled as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, volume, 0, 0b0011_1111);
        self.write_reg(0, LEFT_BEEP_GENERATOR, reg_val)
    }

    // TODO getter
    pub fn set_left_analog_volume_to_spk(&mut self, route_to_class_d: bool, gain: u8) -> TLV320Result<I2C> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_ANALOG_VOLUME_TO_SPK)?;
        set_bits(&mut reg_val, route_to_class_d as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, LEFT_ANALOG_VOLUME_TO_SPK, reg_val)
    }

    // TODO getter
    pub fn set_micbias(
        &mut self,
        power_down: bool,
        always_on: bool,
        output: MicBiasOutput
    ) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, MICBIAS)?;
        set_bits(&mut reg_val, power_down as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, always_on as u8, 3, 0b0000_1000);
        set_bits(&mut reg_val, output as u8, 0, 0b0000_0011);
        self.write_reg(1, MICBIAS, reg_val)
    }

    // TODO test
    // TODO getter
    pub fn set_output_driver_pga_ramp_down_period_control(&mut self, ramp_down: PgaRampDown) -> TLV320Result<I2C> {
        let mut reg_val = self.read_reg(1, OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL)?;
        set_bits(&mut reg_val, ramp_down as u8, 4, 0b0111_0000);
        self.write_reg(1, OUTPUT_DRIVER_PGA_RAMP_DOWN_PERIOD_CONTROL, reg_val)
    }

    pub fn set_pll_d_value(&mut self, d: u16) -> TLV320Result<I2C> {
        let msb: u8 = ((d >> 8) & 0b0011_1111) as u8;
        self.write_reg(0, PLL_D_VALUE_MSB, msb)?;
        self.write_reg(0, PLL_D_VALUE_LSB, (0xff & d) as u8)
    }

    pub fn set_pll_j_value(&mut self, j: u8) -> TLV320Result<I2C> {
        if j == 0 || j > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(0, PLL_J_VALUE)?;
        set_bits(&mut reg_val, j, 0, 0b0011_1111);
        self.write_reg(0, PLL_J_VALUE, reg_val)
    }

    pub fn set_pll_p_and_r_values(&mut self, powered: bool, p: u8, r: u8) -> TLV320Result<I2C> {
        if p == 0 || p > 8 { return Err(TLV320DAC3100Error::InvalidArgument) }
        if r == 0 || r > 16 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = (powered as u8) << 7;
        reg_val |= (if p == 0 { 0x0 } else { p }) << 4;
        reg_val |= if r == 16 { 0x0 } else { r };
        self.write_reg(0, PLL_P_AND_R_VALUES, reg_val)
    }

    // TODO getter
    pub fn set_right_analog_volume_to_hpr(&mut self, route_to_hpr: bool, gain: u8) -> TLV320Result<I2C> {
        if gain > 127 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, RIGHT_ANALOG_VOLUME_TO_HPR)?;
        set_bits(&mut reg_val, route_to_hpr as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, gain, 0, 0b0111_1111);
        self.write_reg(1, RIGHT_ANALOG_VOLUME_TO_HPR, reg_val)
    }

    // TODO test
    // TODO getter
    pub fn set_right_beep_generator(&mut self, mode: RightBeepMode, volume: u8) -> TLV320Result<I2C> {
        if volume > 63 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(1, LEFT_BEEP_GENERATOR)?;
        set_bits(&mut reg_val, mode as u8, 6, 0b1100_0000);
        set_bits(&mut reg_val, volume, 0, 0b0011_1111);
        self.write_reg(0, LEFT_BEEP_GENERATOR, reg_val)
    }

    pub fn set_software_reset(&mut self, reset: bool) -> TLV320Result<I2C> {
        self.write_reg(0, SOFTWARE_RESET, reset as u8)
    }

    // TODO getter
    pub fn set_timer_clock_mclk_divider(&mut self, external_mclk: bool, mclk_divider: u8) -> TLV320Result<I2C> {
        if mclk_divider == 0 || mclk_divider > 128 { return Err(TLV320DAC3100Error::InvalidArgument) }

        let mut reg_val = self.read_reg(3, TIMER_CLOCK_MCLK_DIVIDER)?;
        set_bits(&mut reg_val, external_mclk as u8, 7, 0b1000_0000);
        set_bits(&mut reg_val, mclk_divider, 0, 0b0111_1111);
        self.write_reg(3, TIMER_CLOCK_MCLK_DIVIDER, reg_val)
    }

    pub fn set_vol_micdet_pin_sar_adc(&mut self, pin_control: bool, use_mclk: bool, hysteresis: VolumeControlHysteresis, throughput: VolumeControlThroughput) -> TLV320Result<I2C> {
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