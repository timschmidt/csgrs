use hyperreal::{Rational, Real};
use num::BigInt;
use serde_json::{Value, json};

fn integer(c: &Value, key: &str) -> BigInt {
    c[key].as_str().unwrap().parse().unwrap()
}

fn ratio(r: &Rational) -> Value {
    json!([
        BigInt::from_biguint(r.sign(), r.numerator().clone()).to_string(),
        r.denominator().to_string()
    ])
}

fn main() {
    let inputs: Value =
        serde_json::from_str(include_str!("../../quadratic-extraction-input-v82.json")).unwrap();
    let mut rows = 0;
    for c in inputs.as_array().unwrap() {
        let d = c["d"].as_i64().unwrap();
        if d < 0 {
            continue;
        }
        let a = integer(c, "a");
        let b = integer(c, "b");
        let s = integer(c, "s");
        let q = integer(c, "q");
        let rad = BigInt::from(d) * &s * &s;
        let (scale, rest) = Rational::from_bigint(rad.clone()).extract_square_reduced();
        let root = Real::integer(rad.clone()).sqrt().unwrap();
        let value = ((Real::integer(a.clone()) + Real::integer(b.clone()) * root)
            / Real::integer(q.clone()))
        .unwrap();
        let canonical = ((Real::integer(a.clone())
            + Real::integer(&b * &s) * Real::from(d).sqrt().unwrap())
            / Real::integer(q.clone()))
        .unwrap();
        let shifted = value.clone() * Real::integer(q.clone()) - Real::integer(a.clone());
        let squared = &shifted * &shifted;
        let square_target = Real::integer(&b * &b * rad);
        for precision in [-64, -256] {
            let equality = value.certified_eq_until(&canonical, precision);
            let square = squared.certified_eq_until(&square_target, precision);
            let order = value.certified_cmp_until(&Real::zero(), precision);
            println!(
                "{}",
                json!({"id":c["id"],"precision":precision,"scale":ratio(&scale),"rest":ratio(&rest),
                    "equality":equality.as_bool(),"equalityCertificate":format!("{equality:?}"),
                    "square":square.as_bool(),"squareCertificate":format!("{square:?}"),
                    "order":order.ordering().map(|v|format!("{v:?}")),"orderCertificate":format!("{order:?}")})
            );
            rows += 1;
        }
    }
    assert_eq!(rows, 934);
    println!("{}", json!({"terminal":true,"rows":rows}));
}
