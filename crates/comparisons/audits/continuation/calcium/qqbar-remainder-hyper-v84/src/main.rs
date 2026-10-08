use hyperreal::{Rational, Real};
use num::BigInt;
use serde_json::{Value, json};

fn ratio(r: &Rational) -> Value {
    json!([
        BigInt::from_biguint(r.sign(), r.numerator().clone()).to_string(),
        r.denominator().to_string()
    ])
}
fn bounds(x: &Real, precision: i32) -> Value {
    json!(
        x.certified_dyadic_interval(precision)
            .unwrap()
            .iter()
            .map(ratio)
            .collect::<Vec<_>>()
    )
}
fn coefficient(seed: i32, i: i32, j: i32) -> i32 {
    if seed < 8 {
        (seed + 3 * i + 5 * j) % 7 - 3
    } else if seed == 9 && i == 0 && j == 0 {
        7
    } else {
        0
    }
}
fn main() {
    let mut rows = 0;
    for exponent in [0, 30, 63, 64, 80, 256] {
        for base in [-1, 0, 1] {
            for terms in [1, 2] {
                let x = Real::from(base)
                    .powi(BigInt::from(1) << exponent as usize)
                    .unwrap()
                    + Real::from(terms - 1);
                for precision in [-128, -256] {
                    println!(
                        "{}",
                        json!({"family":"monomial","exponent":exponent,"base":base,"terms":terms,"precision":precision,"value":bounds(&x, precision),"exact":x.exact_rational_ref().map(ratio)})
                    );
                    rows += 1;
                }
            }
        }
    }
    for seed in 0..10 {
        let x = Real::from(2).sqrt().unwrap() + Real::from(seed - 4);
        let y = Real::from(3).sqrt().unwrap() + Real::from(1 - seed);
        let xp = [Real::one(), x.clone(), &x * &x];
        let yp = [Real::one(), y.clone(), &y * &y];
        let mut direct = Real::zero();
        for (i, xi) in xp.iter().enumerate() {
            for (j, yj) in yp.iter().enumerate() {
                direct += Real::from(coefficient(seed, i as i32, j as i32)) * xi * yj;
            }
        }
        let columns: Vec<_> = (0..3)
            .map(|i| {
                let coeffs: Vec<_> = (0..3)
                    .map(|j| Real::from(coefficient(seed, i, j)))
                    .collect();
                Real::eval_poly(&coeffs, &y)
            })
            .collect();
        let nested = Real::eval_poly(&columns, &x);
        for precision in [-128, -256] {
            let equal = direct.certified_eq_until(&nested, precision);
            println!(
                "{}",
                json!({"family":"normal-mpoly","seed":seed,"precision":precision,"direct":bounds(&direct, precision),"nested":bounds(&nested, precision),"equality":equal.as_bool(),"certificate":format!("{equal:?}")})
            );
            rows += 1;
        }
    }
    let x = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    for length in [0, 1, 2, 7, 8, 9, 63, 64, 65, 127, 128, 129] {
        let coeffs: Vec<_> = (0..length)
            .map(|i| Real::from((5 * i + length) % 7 - 3))
            .collect();
        let actual = Real::eval_poly(&coeffs, &x);
        let horner = coeffs
            .iter()
            .rev()
            .fold(Real::zero(), |acc, c| acc * &x + c);
        for precision in [-128, -256] {
            let equal = actual.certified_eq_until(&horner, precision);
            println!(
                "{}",
                json!({"family":"long-poly","length":length,"precision":precision,"actual":bounds(&actual, precision),"horner":bounds(&horner, precision),"equality":equal.as_bool(),"certificate":format!("{equal:?}")})
            );
            rows += 1;
        }
    }
    assert_eq!(rows, 116);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
