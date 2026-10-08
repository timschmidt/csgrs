use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
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

fn operands(k: usize, equal: bool, sign: i32, offset: i32, swap: bool) -> [Real; 2] {
    let delta =
        Rational::from_bigint_fraction(BigInt::from(i32::from(!equal)), BigUint::from(1_u8) << k)
            .unwrap();
    let x = Real::from(sign) * Real::from(2).sqrt().unwrap() + Real::from(offset);
    let y =
        Real::from(sign) * (Real::from(2) + Real::new(delta)).sqrt().unwrap() + Real::from(offset);
    if swap { [y, x] } else { [x, y] }
}

fn continuous(x: &Real, y: &Real) -> [Real; 2] {
    let sum = x + y;
    let difference = (x - y).abs();
    [
        ((&sum - &difference) / Real::from(2)).unwrap(),
        ((sum + difference) / Real::from(2)).unwrap(),
    ]
}

fn main() {
    let mut rows = 0;
    for (k, equal) in [
        (0, false),
        (32, false),
        (128, false),
        (2056, false),
        (4096, false),
        (0, true),
    ] {
        for sign in [-1, 1] {
            for offset in [0, 7] {
                for swap in [false, true] {
                    // Preserve cold bounded-selection observations before any
                    // high-precision query warms operand caches.
                    let [x, y] = operands(k, equal, sign, offset, swap);
                    let cmp = x.certified_cmp_until(&y, Real::PARTIAL_CMP_MIN_PRECISION);
                    let borrowed_min_left = std::ptr::eq(x.min(&y), &x);
                    let borrowed_max_left = std::ptr::eq(x.max(&y), &x);
                    // Fresh operands keep the formula independent of that query.
                    let [a, b] = operands(k, equal, sign, offset, swap);
                    let [min, max] = continuous(&a, &b);
                    let precision = -i32::try_from(k.max(32) + 64).unwrap();
                    println!(
                        "{}",
                        json!({"row":rows,"k":k,"equal":equal,"sign":sign,"offset":offset,"swap":swap,
                            "precision":precision,"ordering":cmp.ordering().map(|v| format!("{v:?}")),
                            "certificate":format!("{cmp:?}"),"borrowed_min_left":borrowed_min_left,
                            "borrowed_max_left":borrowed_max_left,"min":bounds(&min,precision),
                            "max":bounds(&max,precision)})
                    );
                    rows += 1;
                }
            }
        }
    }
    assert_eq!(rows, 48);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
