pub fn set_bits(byte: &mut u8, data: u8, left_shift: u8, mask: u8) {
    *byte &= !mask;
    *byte |= (data << left_shift) & mask
}

#[cfg(test)]
mod tests {
    use crate::bits::set_bits;

    #[test]
    fn update_bits_ok() {
        let mut byte = 0b1010_0001;
        set_bits(&mut byte, 0b11, 2, 0b0000_1100);
        assert_eq!(0b1010_1101, byte);
        set_bits(&mut byte, 0b10, 5, 0b0110_0000);
        assert_eq!(0b1100_1101, byte);
    }
}