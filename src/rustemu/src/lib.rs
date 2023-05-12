#![no_std]

// dev profile: easier to debug panics; can put a breakpoint on `rust_begin_unwind`
#[cfg(all(debug_assertions, not(test)))]
use panic_halt as _;

// release profile: minimize the binary size of the application
#[cfg(all(not(debug_assertions), not(test)))]
use panic_abort as _;


#[no_mangle] pub extern fn zigzagEncode(value: i32) -> u32 {
    ((value << 1) ^ (value >> 31)) as u32
}

#[no_mangle] pub extern fn castFloatBytesToInt(value: f32) -> u32 {
    u32::from_ne_bytes(value.to_ne_bytes())
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zigzagEncode_test() {
        assert_eq!(zigzagEncode(0), 0);
        assert_eq!(zigzagEncode(-1), 1);
        assert_eq!(zigzagEncode(1), 2);
        assert_eq!(zigzagEncode(2147483646), 4294967292);
        assert_eq!(zigzagEncode(-2147483647), 4294967293);
        assert_eq!(zigzagEncode(2147483647), 4294967294);
        assert_eq!(zigzagEncode(-2147483648), 4294967295);
    }

    #[test]
    fn castFloatBytesToInt_test() {
        assert_eq!(castFloatBytesToInt(0.0), 0);
        assert_eq!(castFloatBytesToInt(2.0), 0x40000000);
        assert_eq!(castFloatBytesToInt(4.5), 0x40900000);
    }
}
