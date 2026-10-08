use hyperlattice::Complex;
use hyperreal::{Problem, Rational, Real};
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
fn actual(x: Real, op: usize) -> Result<Complex, Problem> {
    if op == 6 {
        return Ok(Complex::new(x.clone().cos(), x.sin()));
    }
    let r = match op {
        0 => x.sin(),
        1 => x.cos(),
        2 => x.tan()?,
        3 => x.cot()?,
        4 => x.cos().inverse()?,
        5 => x.sin().inverse()?,
        _ => unreachable!(),
    };
    Ok(Complex::new(r, Real::zero()))
}
fn output(mut row: Value, x: Real, op: usize) -> usize {
    let value = actual(x, op);
    for precision in [-128, -256] {
        row["precision"] = json!(precision);
        match &value {
            Ok(z) => {
                row["real"] = bounds(&z.re, precision);
                row["imag"] = bounds(&z.im, precision);
            }
            Err(error) => {
                row["error"] = json!(format!("{error:?}"));
            }
        }
        println!("{row}");
    }
    2
}
fn main() {
    let mut rows = 0;
    for op in 0..7 {
        for exponent in [0, 30, 60, 61, 62, 63, 80, 256] {
            for sign in [-1, 1] {
                let den = if op == 3 || op == 5 { 2 } else { 1 };
                let p = BigInt::from(sign)
                    * ((BigInt::from(1) << exponent as usize) + if den == 2 { 1 } else { 0 });
                let x = (Real::integer(p) / Real::from(den)).unwrap() * Real::pi();
                rows += output(
                    json!({"family":"large-angle","op":op,"exponent":exponent,"sign":sign,"den":den}),
                    x,
                    op,
                );
            }
        }
        for kind in 0..3 {
            for sign in [-1, 1] {
                for has_pi in [0, 1] {
                    let p = if kind == 0 { 0 } else { sign };
                    let den = if kind == 2 { 6 } else { 1 };
                    let x = (Real::from(p) / Real::from(den)).unwrap();
                    let x = if has_pi == 1 { x * Real::pi() } else { x };
                    rows += output(
                        json!({"family":"pi-factor","op":op,"kind":kind,"sign":sign,"hasPi":has_pi}),
                        x,
                        op,
                    );
                }
            }
        }
    }
    for (scale, (n, d)) in [(-7, 3), (0, 1), (1, 1), (5, 2)].into_iter().enumerate() {
        let s = (Real::from(n) / Real::from(d)).unwrap();
        for k in 0..24 {
            let angle = (Real::from(k) / Real::from(12)).unwrap();
            let z = Complex::new(&s * angle.clone().cos_pi(), &s * angle.sin_pi());
            let norm = z.norm_squared();
            let abs = norm.clone().sqrt().unwrap();
            for precision in [-128, -256] {
                let equal = norm.certified_eq_until(&(&s * &s), precision);
                println!(
                    "{}",
                    json!({"family":"complex-norm","scale":scale,"k":k,"precision":precision,
                    "norm":bounds(&norm,precision),"abs":bounds(&abs,precision),"normEquality":equal.as_bool(),"certificate":format!("{equal:?}")})
                );
                rows += 1;
            }
        }
    }
    assert_eq!(rows, 584);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
