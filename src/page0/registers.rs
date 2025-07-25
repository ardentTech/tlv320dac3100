// TODO if registers are pub(crate) then reads can happen in the higher-level API
// TODO is there a way to declare the visibility once instead of per-register? enum.
//      need to check mem usage + asm count against const u8:
//      pub enum Registers {
//          VolMicDetPinGain = 0x75
//      }
pub(crate) const SOFTWARE_RESET: u8 = 0x01;
pub(crate) const OT_FLAG: u8 = 0x03;
pub(crate) const CLOCK_GEN_MUXING: u8 = 0x04;
pub(crate) const PLL_P_AND_R_VALUES: u8 = 0x05;
pub(crate) const PLL_J_VALUE: u8 = 0x06;
pub(crate) const PLL_D_VALUE_MSB: u8 = 0x07;
pub(crate) const PLL_D_VALUE_LSB: u8 = 0x08;
pub(crate) const DAC_NDAC_VAL: u8 = 0x0b;
pub(crate) const DAC_MDAC_VAL: u8 = 0x0c;
pub(crate) const DAC_DOSR_VAL_MSB: u8 = 0x0d;
pub(crate) const DAC_DOSR_VAL_LSB: u8 = 0x0e;
pub(crate) const CLKOUT_MUX: u8 = 0x19;
pub(crate) const CLKOUT_M_VAL: u8 = 0x1a;
pub(crate) const CODEC_INTERFACE_CONTROL_1: u8 = 0x1b;
pub(crate) const DATA_SLOT_OFFSET_PROGRAMMABILITY: u8 = 0x1c;
pub(crate) const CODEC_INTERFACE_CONTROL_2: u8 = 0x1d;
pub(crate) const BCLK_N_VAL: u8 = 0x1e;
pub(crate) const CODEC_SECONDARY_INTERFACE_CONTROL_1: u8 = 0x1f;
pub(crate) const CODEC_SECONDARY_INTERFACE_CONTROL_2: u8 = 0x20;
//pub(crate) const CODEC_SECONDARY_INTERFACE_CONTROL_3: u8 = 0x21;
pub(crate) const I2C_BUS_CONDITION: u8 = 0x22;
// pub(crate) const DAC_FLAG_REGISTER_1: u8 = 0x25;
// pub(crate) const DAC_FLAG_REGISTER_2: u8 = 0x26;
// pub(crate) const OVERFLOW_FLAGS: u8 = 0x27;
// pub(crate) const DAC_INTERRUPT_FLAGS_STICKY_BITS: u8 = 0x2c;
// pub(crate) const INTERRUPT_FLAGS_DAC: u8 = 0x2e;
// pub(crate) const INT1_CONTROL_REGISTER: u8 = 0x30;
// pub(crate) const INT2_CONTROL_REGISTER: u8 = 0x31;
// pub(crate) const GPIO1_IN_OUT_PIN_CONTROL: u8 = 0x33;
pub(crate) const DIN_CONTROL: u8 = 0x36;
pub(crate) const DAC_PROCESSING_BLOCK_SELECTION: u8 = 0x3c;
pub(crate) const DAC_DATA_PATH_SETUP: u8 = 0x3f;
//pub(crate) const DAC_VOLUME_CONTROL: u8 = 0x40;
//pub(crate) const DAC_LEFT_VOLUME_CONTROL: u8 = 0x41;
//pub(crate) const DAC_RIGHT_VOLUME_CONTROL: u8 = 0x42;
// pub(crate) const HEADSET_DETECTION: u8 = 0x43;
// pub(crate) const DRC_CONTROL_1: u8 = 0x44;
// pub(crate) const DRC_CONTROL_2: u8 = 0x45;
// pub(crate) const DRC_CONTROL_3: u8 = 0x46;
// pub(crate) const LEFT_BEEP_GENERATOR: u8 = 0x47;
// pub(crate) const RIGHT_BEEP_GENERATOR: u8 = 0x48;
// pub(crate) const BEEP_LENGTH_MSB: u8 = 0x49;
// pub(crate) const BEEP_LENGTH_MIDDLE_BITS: u8 = 0x4a;
// pub(crate) const BEEP_LENGTH_LSB: u8 = 0x4b;
// pub(crate) const BEEP_SINX_MSB: u8 = 0x4c;
// pub(crate) const BEEP_SINX_LSB: u8 = 0x4d;
// pub(crate) const BEEP_COSX_MSB: u8 = 0x4e;
// pub(crate) const BEEP_COSX_LSB: u8 = 0x4f;
// pub(crate) const VOL_MICDET_PIN_SAR_ADC_VOLUME_CONTROL: u8 = 0x74;
pub(crate) const VOL_MICDET_PIN_GAIN: u8 = 0x75;