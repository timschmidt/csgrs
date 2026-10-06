use hyperreal::{Problem, Rational, Real, RealSign};

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
