use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

fn polynomials(count: usize) -> Vec<Vec<Real>> {
    (0..3).map(|axis| {
        let mut differences: Vec<Rational> = (0..count).map(|index| {
            let coordinate = match axis {
                0 => index as i64,
                1 => (index * 19 % 37) as i64 - 18,
                _ => 1,
            };
            Rational::new(coordinate * (index % 5 + 1) as i64)
        }).collect();
        let mut coefficients = Vec::with_capacity(count);
        let mut binomial = Rational::one();
        for degree in 0..count {
            coefficients.push(Real::new(&differences[0] * &binomial));
            for index in 0..count - degree - 1 {
                differences[index] = &differences[index + 1] - &differences[index];
            }
            if degree + 1 < count {
                binomial = (&binomial * Rational::new((count - degree - 1) as i64))
                    / Rational::new((degree + 1) as i64);
            }
        }
        coefficients
    }).collect()
}

fn real_horner(coefficients: &[Real], point: &Real) -> Real {
    coefficients.iter().rev().fold(Real::zero(), |value, coefficient| value * point + coefficient)
}

fn real_fused(coefficients: &[Real], point: &Real) -> Real {
    let (leading, rest) = coefficients.split_last().unwrap();
    rest.iter().rev().fold(leading.clone(), |value, coefficient| Real::mul_add(&value, point, coefficient))
}

fn rational_horner(coefficients: &[Real], point: &Real) -> Real {
    let point = point.exact_rational_ref().unwrap();
    let (leading, rest) = coefficients.split_last().unwrap();
    let value = rest.iter().rev().fold(leading.exact_rational_ref().unwrap().clone(), |value, coefficient| {
        &value * point + coefficient.exact_rational_ref().unwrap()
    });
    Real::new(value)
}

fn main() {
    let methods: [(&str, fn(&[Real], &Real) -> Real); 4] = [
        ("real_horner", real_horner), ("native", Real::eval_poly),
        ("rational_horner", rational_horner), ("real_fused", real_fused),
    ];
    for count in [4, 16, 64] {
        for fractional in [false, true] {
            let mut polynomials = polynomials(count);
            if fractional {
                for polynomial in &mut polynomials {
                    for (index, coefficient) in polynomial.iter_mut().enumerate() {
                        *coefficient = (&*coefficient / Real::from(index as u64 % 7 + 1)).unwrap();
                    }
                }
            }
            for (numerator, denominator) in [(1, 2), (1, 3), (-2, 7)] {
                let point = Real::new(Rational::fraction(numerator, denominator).unwrap());
                for (name, method) in methods {
                    for polynomial in &polynomials {
                        assert_eq!(method(polynomial, &point), real_horner(polynomial, &point), "{name}: {count}");
                    }
                }
                let iterations = if count == 64 { 2_000 } else { 10_000 };
                for sample in 0..3 {
                    for index in 0..methods.len() {
                        let (name, method) = methods[(index + sample) % methods.len()];
                        let start = Instant::now();
                        for _ in 0..iterations {
                            for polynomial in &polynomials {
                                black_box(method(black_box(polynomial), black_box(&point)));
                            }
                        }
                        println!("controls={count} fractional={fractional} x={numerator}/{denominator} sample={sample} method={name} ns_per_point={}", start.elapsed().as_nanos() as f64 / f64::from(iterations));
                    }
                }
            }
        }
    }
}
