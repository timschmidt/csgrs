use hyperreal::{Problem, Rational, Real};
use num::{BigInt, BigUint};
use serde_json::{Value, json};

fn fraction(p: i64, q: u64) -> Real {
    Real::from(Rational::fraction(p, q).unwrap())
}

fn emit_import(width: u32, bits: u64) {
    let imported = if width == 32 {
        Real::try_from(f32::from_bits(u32::try_from(bits).unwrap()))
    } else {
        Real::try_from(f64::from_bits(bits))
    };
    let mut row = json!({"family":"float","width":width,"bits":format!("{bits:016x}")});
    match imported {
        Ok(value) => {
            let rational = value
                .exact_rational_ref()
                .expect("finite import stays exact rational");
            row["num"] = json!(
                BigInt::from_biguint(rational.sign(), rational.numerator().clone()).to_string()
            );
            row["den"] = json!(rational.denominator().to_string());
            row["exportBits"] = json!(format!("{:016x}", value.as_f64_lossy().unwrap().to_bits()));
        }
        Err(error) => row["error"] = json!(format!("{error:?}")),
    }
    println!("{row}");
}

fn rounding_input(b: usize, kind: usize) -> Real {
    let base = match b {
        0 => -(BigInt::from(1) << 256_usize),
        6 => BigInt::from(1) << 256_usize,
        _ => BigInt::from([0, -3, -1, 0, 1, 3, 0][b]),
    };
    let mut x = Real::integer(base);
    if kind != 0 {
        let sign = if kind % 2 == 1 { 1 } else { -1 };
        let exponent = if kind <= 8 {
            [1, 8, 128, 1024][(kind - 1) / 2]
        } else if kind <= 16 {
            [0, 8, 128, 1024][(kind - 9) / 2]
        } else {
            [8, 128, 1024][(kind - 17) / 2]
        };
        let factor = Real::from(
            Rational::from_bigint_fraction(BigInt::from(sign), BigUint::from(1_u8) << exponent)
                .unwrap(),
        );
        x += if kind <= 8 {
            factor
        } else {
            factor * Real::from(2).sqrt().unwrap()
        };
        if kind >= 17 {
            x += fraction(1, 2);
        }
    }
    x
}

fn integer_result(result: Result<BigInt, Problem>) -> Value {
    match result {
        Ok(value) => json!({"value":value.to_string()}),
        Err(error) => json!({"error":format!("{error:?}")}),
    }
}

fn forward(op: i32, angle: Real) -> Result<Real, Problem> {
    if op == 0 {
        angle.sec_pi()
    } else {
        angle.csc_pi()
    }
}

fn derived_inverse(op: i32, value: Real) -> Result<Real, Problem> {
    let reciprocal = (Real::one() / value)?;
    if op == 0 {
        reciprocal.acos()
    } else {
        reciprocal.asin()
    }
}

fn target(op: i32, k: i32) -> Real {
    let mut k = k.rem_euclid(24);
    if op == 0 {
        if k > 12 {
            k = 24 - k;
        }
    } else {
        if k > 12 {
            k -= 24;
        }
        if k > 6 {
            k = 12 - k;
        }
        if k < -6 {
            k = -12 - k;
        }
    }
    Real::pi() * fraction(i64::from(k), 12)
}

fn main() {
    let mut rows = 0;
    for width in [32_u32, 64] {
        let significand_bits = if width == 32 { 23 } else { 52 };
        for sign in [0_u64, 1] {
            for exponent in 0..if width == 32 { 256_u64 } else { 2048_u64 } {
                for significand in [
                    0_u64,
                    1,
                    1 << (significand_bits - 1),
                    (1 << significand_bits) - 1,
                ] {
                    let bits = (sign << (width - 1)) | (exponent << significand_bits) | significand;
                    emit_import(width, bits);
                    rows += 1;
                }
            }
        }
    }
    for b in 0..7 {
        for kind in 0..23 {
            let input = rounding_input(b, kind);
            for phase in [0, 1] {
                // The second phase repeats on the same retained expression; it
                // is not the donor's explicit 1536-bit enclosure-polishing phase.
                let floor = integer_result(input.floor_certified());
                let ceil = integer_result(input.ceil_certified());
                let near = input.near_integer().to_string();
                println!(
                    "{}",
                    json!({"family":"round","b":b,"kind":kind,"phase":phase,"floor":floor,"ceil":ceil,"near":near})
                );
                rows += 1;
            }
        }
    }
    for op in [0, 1] {
        for k in 0..24 {
            for shift in [-3, 0, 5] {
                for scale in [1, 3] {
                    let input = forward(
                        op,
                        fraction(
                            i64::from((k + 24 * shift) * scale),
                            u64::try_from(12 * scale).unwrap(),
                        ),
                    );
                    for (phase, precision) in [(0, -64), (1, -256)] {
                        let mut row = json!({"family":"inverse","op":op,"k":k,"shift":shift,"scale":scale,"phase":phase,"precision":precision});
                        match &input {
                            Err(error) => row["error"] = json!(format!("{error:?}")),
                            Ok(value) => {
                                let result = derived_inverse(op, value.clone())
                                    .expect("in-domain reciprocal");
                                row["certificate"] = json!(format!(
                                    "{:?}",
                                    result.certified_eq_until(&target(op, k), precision)
                                ));
                            }
                        }
                        println!("{row}");
                        rows += 1;
                    }
                }
            }
        }
    }
    for op in [0, 1] {
        for k in 0..5 {
            let input = match k {
                0 => Real::zero(),
                1 => Real::one(),
                2 => -Real::one(),
                3 => fraction(1, 2),
                _ => fraction(-1, 2),
            };
            let mut row = json!({"family":"inverse-control","op":op,"k":k});
            match derived_inverse(op, input) {
                Err(error) => row["error"] = json!(format!("{error:?}")),
                Ok(value) => {
                    let expected = if op == 0 {
                        Real::pi() * Real::from(k - 1)
                    } else {
                        Real::pi() * fraction(if k == 1 { 1 } else { -1 }, 2)
                    };
                    row["certificate"] =
                        json!(format!("{:?}", value.certified_eq_until(&expected, -256)));
                }
            }
            println!("{row}");
            rows += 1;
        }
    }
    assert_eq!(rows, 19340);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
