/// A big unsigned integer type.
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct BigUint {
    data: Vec<BigDigit>,
}
