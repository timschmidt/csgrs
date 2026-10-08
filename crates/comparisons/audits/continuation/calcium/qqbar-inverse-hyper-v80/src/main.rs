use hyperreal::{CertifiedRealEquality, Rational, Real};
use num::{BigInt, BigUint};
use serde_json::{Value, json};

fn fraction(p: i32, q: i32) -> Real {
    Real::from(Rational::fraction(p.into(), u64::try_from(q).unwrap()).unwrap())
}

fn sine(k: i32) -> Real {
    let mut k = k.rem_euclid(24);
    let negative = k > 12;
    if negative {
        k -= 12;
    }
    if k > 6 {
        k = 12 - k;
    }
    let value = match k {
        0 => Real::zero(),
        1 => ((Real::from(6).sqrt().unwrap() - Real::from(2).sqrt().unwrap()) / Real::from(4))
            .unwrap(),
        2 => fraction(1, 2),
        3 => (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
        4 => (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap(),
        5 => ((Real::from(6).sqrt().unwrap() + Real::from(2).sqrt().unwrap()) / Real::from(4))
            .unwrap(),
        6 => Real::one(),
        _ => unreachable!(),
    };
    if negative { -value } else { value }
}

fn forward(angle: Real, op: &str) -> Result<Real, hyperreal::Problem> {
    match op {
        "asin" => Ok(angle.sin_pi()),
        "acos" => Ok(angle.cos_pi()),
        "atan" => angle.tan_pi(),
        "acot" => angle.cot_pi(),
        _ => unreachable!(),
    }
}

fn inverse(value: Real, op: &str, zero: bool) -> Result<Real, hyperreal::Problem> {
    match op {
        "asin" => value.asin(),
        "acos" => value.acos(),
        "atan" => value.atan(),
        // Hyper has no acot API. Explicitly implement the donor branch;
        // this is a derived comparison, not a claimed Hyper public method.
        "acot" if zero => Ok(Real::pi() * fraction(1, 2)),
        "acot" => (Real::one() / value)?.atan(),
        _ => unreachable!(),
    }
}

fn radical(k: i32, op: &str) -> Real {
    match op {
        "asin" => sine(k),
        "acos" => sine(k + 6),
        "atan" => (sine(k) / (Real::one() + sine(k + 6))).unwrap(),
        "acot" if k % 24 == 12 => Real::zero(),
        "acot" => ((Real::one() + sine(k + 6)) / sine(k)).unwrap(),
        _ => unreachable!(),
    }
}

fn principal(k: i32, op: &str) -> (i32, i32) {
    let mut u = k.rem_euclid(24);
    if op == "acos" {
        if u > 12 {
            u = 24 - u;
        }
        return (u, 12);
    }
    if u > 12 {
        u -= 24;
    }
    if op == "asin" {
        if u > 6 {
            u = 12 - u;
        }
        if u < -6 {
            u = -12 - u;
        }
        (u, 12)
    } else {
        (u, 24)
    }
}

fn equality(mut row: Value, left: Real, p: i32, q: i32, precision: i32, unequal: bool) {
    let mut right = Real::pi() * fraction(p, q);
    if row["control"] == "perturbed" {
        right += fraction(1, 1024);
    }
    let certificate = left.certified_eq_until(&right, precision);
    let outcome = match &certificate {
        CertifiedRealEquality::Equal { .. } => {
            assert!(!unequal, "false equality: {row}");
            "Equal"
        }
        CertifiedRealEquality::NotEqual { .. } => {
            assert!(unequal, "false inequality: {row}");
            "NotEqual"
        }
        CertifiedRealEquality::Unknown { .. } => "Unknown",
    };
    row["p"] = json!(p);
    row["q"] = json!(q);
    row["outcome"] = json!(outcome);
    row["certificate"] = json!(format!("{certificate:?}"));
    println!("{row}");
}

fn main() {
    let mut rows = 0;
    for op in ["asin", "acos", "atan", "acot"] {
        let d = if op == "atan" || op == "acot" { 24 } else { 12 };
        for k in 0..2 * d {
            let pole = (op == "atan" && k % 24 == 12) || (op == "acot" && k % 24 == 0);
            for representation in ["stored", "radical", "huge-period"] {
                for precision in [-64, -256] {
                    for control in ["identity", "perturbed"] {
                        rows += 1;
                        let row = json!({"family":"angle","op":op,"k":k,"representation":representation,
                            "precision":precision,"control":control});
                        if pole {
                            assert!(matches!(
                                forward(fraction(k, d), op),
                                Err(hyperreal::Problem::NotANumber)
                            ));
                            let mut row = row;
                            row["outcome"] = json!("Pole");
                            println!("{row}");
                            continue;
                        }
                        let input = if representation == "radical" {
                            radical(k, op)
                        } else {
                            let mut angle = fraction(k, d);
                            if representation == "huge-period" {
                                angle += Real::from(Rational::from_bigint(
                                    BigInt::from(1) << 1025_usize,
                                ));
                            }
                            forward(angle, op).unwrap()
                        };
                        let left = inverse(input, op, op == "acot" && k % 24 == 12).unwrap();
                        let (p, q) = principal(k, op);
                        equality(row, left, p, q, precision, control == "perturbed");
                    }
                }
            }
        }
    }
    for family in ["golden", "cubic"] {
        let count = if family == "golden" { 4 } else { 6 };
        for which in 0..count {
            for op in ["asin", "acos"] {
                let (input, mut p, q) = if family == "golden" {
                    let mut x = (Real::from(5).sqrt().unwrap()
                        + Real::from(if which % 2 == 0 { -1 } else { 1 }))
                        * fraction(1, 4);
                    let sign = if which >= 2 { -1 } else { 1 };
                    if sign < 0 {
                        x = -x;
                    }
                    (x, if which % 2 == 0 { sign } else { 3 * sign }, 10)
                } else {
                    let mut x = fraction(2 * (which % 3 + 1), 7).cos_pi();
                    let sign = if which >= 3 { -1 } else { 1 };
                    if sign < 0 {
                        x = -x;
                    }
                    (x, [3, -1, -5][(which % 3) as usize] * sign, 14)
                };
                if op == "acos" {
                    p = q / 2 - p;
                }
                for precision in [-64, -256] {
                    for control in ["identity", "perturbed"] {
                        rows += 1;
                        equality(
                            json!({"family":family,"which":which,"op":op,"precision":precision,"control":control}),
                            inverse(input.clone(), op, false).unwrap(),
                            p,
                            q,
                            precision,
                            control == "perturbed",
                        );
                    }
                }
            }
        }
    }
    for op in ["asin", "acos", "atan", "acot"] {
        let q = if op == "asin" || op == "acos" { 12 } else { 24 };
        for sign in [-1, 1] {
            for precision in [-64, -256] {
                rows += 1;
                let input = forward(fraction(1, q), op).unwrap()
                    + Real::from(
                        Rational::from_bigint_fraction(
                            BigInt::from(sign),
                            BigUint::from(1_u8) << 100_usize,
                        )
                        .unwrap(),
                    );
                equality(
                    json!({"family":"near","op":op,"sign":sign,"precision":precision,"control":"near"}),
                    inverse(input, op, false).unwrap(),
                    1,
                    q,
                    precision,
                    true,
                );
            }
        }
    }
    for sign in [-1, 1] {
        for representation in ["stored", "huge-period"] {
            for precision in [-64, -256] {
                for control in ["identity", "perturbed"] {
                    rows += 1;
                    let mut angle = fraction(sign, 3360);
                    if representation == "huge-period" {
                        angle += Real::from(Rational::from_bigint(BigInt::from(1) << 1025_usize));
                    }
                    equality(
                        json!({"family":"3360","sign":sign,"representation":representation,"precision":precision,"control":control}),
                        angle.tan_pi().unwrap().atan().unwrap(),
                        sign,
                        3360,
                        precision,
                        control == "perturbed",
                    );
                }
            }
        }
    }
    for op in ["asin", "acos"] {
        for x in [-2, 2] {
            for precision in [-64, -256] {
                rows += 1;
                assert!(matches!(
                    inverse(Real::from(x), op, false),
                    Err(hyperreal::Problem::NotANumber)
                ));
                println!(
                    "{}",
                    json!({"family":"domain","op":op,"x":x,"precision":precision,"outcome":"DomainError"})
                );
            }
        }
    }
    assert_eq!(rows, 1848);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
