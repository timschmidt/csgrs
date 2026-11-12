use alumina_interface_core::{BoundaryError, BoundedMeasurement, Millimetres};
use hyperreal::Rational;

/// Hash the complete exact endpoint text; fixed sentinels distinguish failures.
#[unsafe(no_mangle)]
pub extern "C" fn measurement(nominal: i64, uncertainty: i64, denominator: u64) -> u64 {
    let (Ok(n), Ok(u)) = (
        Rational::fraction(nominal, denominator),
        Rational::fraction(uncertainty, denominator),
    ) else {
        return u64::MAX;
    };
    match BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(n, u) {
        Err(BoundaryError::NegativeUncertainty) => 0,
        // The host oracle rejects this distinct status. Unexpected diagnostics
        // are also inspected as typed Error::source chains by the host tests.
        Err(_) => u64::MAX - 1,
        Ok(value) => {
            let text = format!("{} {} mm", value.lower(), value.upper());
            text.bytes().fold(0xcbf29ce484222325, |hash, byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
            })
        }
    }
}
