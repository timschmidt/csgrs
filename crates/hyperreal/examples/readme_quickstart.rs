//! Executable README introduction to exact rational arithmetic and a symbolic root.
//!
//! The assertions concern exact rational equality and a proved positive sign.
//! Only the final display requests a lossy binary64 view; its twelve printed
//! fractional digits are not an enclosure or an exact decimal representation.

use hyperreal::{Problem, Rational, Real, RealSign};

/// Check 1/10 + 2/10 = 3/10 using exact fractions, construct sqrt(2), and require
/// a positive sign with refinement floor -64 before printing a lossy float view.
/// Constructor failures propagate as Problem; mismatched assertions, absent float
/// conversion or stdout failures panic. The fixed positive radicand avoids the
/// ordinary square-root domain error. The sign floor is not a total-work bound.
fn main() -> Result<(), Problem> {
    let one_tenth = Rational::fraction(1, 10)?;
    let two_tenths = Rational::fraction(2, 10)?;
    let exact_sum = one_tenth + two_tenths;
    assert_eq!(exact_sum, Rational::fraction(3, 10)?);

    let diagonal = Real::new(Rational::new(2)).sqrt()?;
    assert_eq!(diagonal.refine_sign_until(-64), Some(RealSign::Positive));

    println!("sqrt(2) ≈ {:.12}", diagonal.to_f64_lossy().unwrap());
    Ok(())
}
