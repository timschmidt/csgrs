//! Diagnostic corpus shared verbatim by the frozen baseline and candidate.
//! Records actual private normal-form results, ordered memo entries and remaining
//! work. It contains no second implementation of the recognizer.
use super::*;
use serde_json::{Value, json};
use std::collections::HashMap;

fn raw(approximation: Approximation) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            approximation,
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

fn fixtures() -> Vec<(String, Computable)> {
    let mut fixtures = Vec::new();
    for depth in [0, 1, 2, 7, 8, 9, 31, 47, 48, 49, 63, 64, 65, 127] {
        let mut value = Computable::pi();
        for _ in 0..depth {
            value = raw(Approximation::Negate(value));
        }
        fixtures.push((format!("depth-{depth}"), value));
    }
    for degree in [6, 7, 8, 9] {
        let mut power = Computable::one();
        let mut sum = Computable::zero();
        for _ in 0..degree {
            sum = raw(Approximation::Add(sum, power.clone()));
            power = raw(Approximation::Multiply(power, Computable::pi()));
        }
        fixtures.push((format!("pi-terms-{degree}"), sum));
    }
    for count in [7, 8, 9] {
        let mut sum = Computable::zero();
        for prime in [2, 3, 5, 7, 11, 13, 17, 19, 23].into_iter().take(count) {
            let root = raw(Approximation::Sqrt(Computable::rational(Rational::new(prime))));
            sum = raw(Approximation::Add(sum, root));
        }
        fixtures.push((format!("radical-terms-{count}"), sum));
    }
    for count in [127, 128, 129, 255, 256, 257, 511, 512, 513] {
        for opaque in [false, true] {
            let mut layer: Vec<_> = (0..count)
                .map(|index| {
                    let rational = Rational::fraction(index + 1, 4099).unwrap();
                    if opaque {
                        raw(Approximation::AtanRational(rational))
                    } else {
                        raw(Approximation::Ratio(rational))
                    }
                })
                .collect();
            while layer.len() > 1 {
                let mut next = Vec::new();
                let mut values = layer.into_iter();
                while let Some(left) = values.next() {
                    next.push(match values.next() {
                        Some(right) => raw(Approximation::Add(left, right)),
                        None => left,
                    });
                }
                layer = next;
            }
            fixtures.push((format!("width-{count}-opaque-{opaque}"), layer.pop().unwrap()));
        }
    }
    for depth in [1, 2, 8, 16, 48, 64] {
        for opaque in [false, true] {
            let mut value = if opaque {
                raw(Approximation::AtanRational(Rational::fraction(3, 4).unwrap()))
            } else {
                raw(Approximation::Add(
                    Computable::pi(),
                    Computable::sqrt_rational(Rational::new(2)),
                ))
            };
            for _ in 0..depth {
                value = raw(Approximation::Add(value.clone(), value));
            }
            fixtures.push((format!("shared-{depth}-opaque-{opaque}"), value));
        }
    }
    let closed = raw(Approximation::Add(
        Computable::one(),
        Computable::sqrt_rational(Rational::new(2)),
    ));
    let atom = raw(Approximation::AtanRational(Rational::fraction(3, 4).unwrap()));
    fixtures.extend([
        ("closed-square".into(), raw(Approximation::Square(closed.clone()))),
        ("closed-inverse".into(), raw(Approximation::Inverse(closed.clone()))),
        ("closed-offset".into(), raw(Approximation::Offset(closed.clone(), -1000))),
        ("opaque-product".into(), raw(Approximation::Multiply(closed.clone(), atom.clone()))),
        ("opaque-square".into(), raw(Approximation::Square(atom.clone()))),
        ("opaque-cancellation".into(), raw(Approximation::Add(atom.clone(), raw(Approximation::Negate(atom.clone()))))),
        ("linear-combination".into(), raw(Approximation::LinearCombination3(Box::new(LinearCombination3 {
            coefficients: [closed.clone(), closed, atom],
            values: [Rational::new(3), Rational::new(-3), Rational::new(2)],
        })))),
    ]);
    fixtures
}

fn collect_nodes<'a>(value: &'a Computable, nodes: &mut Vec<&'a Computable>, ids: &mut HashMap<usize, usize>) {
    let key = Arc::as_ptr(&value.internal) as usize;
    if ids.contains_key(&key) {
        return;
    }
    ids.insert(key, nodes.len());
    nodes.push(value);
    match &value.internal.approximation {
        Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
            collect_nodes(left, nodes, ids);
            collect_nodes(right, nodes, ids);
        }
        Approximation::Negate(child) | Approximation::Square(child)
        | Approximation::Sqrt(child) | Approximation::Inverse(child)
        | Approximation::Offset(child, _) => collect_nodes(child, nodes, ids),
        Approximation::LinearCombination3(combination) => {
            for child in &combination.coefficients {
                collect_nodes(child, nodes, ids);
            }
        }
        _ => {}
    }
}

fn polynomial_receipt(value: &PiLaurentPolynomial, ids: &HashMap<usize, usize>) -> Value {
    Value::Array(value.terms.iter().map(|(power, coefficient)| json!({
        "power": power,
        "rational": coefficient.rational.to_string(),
        "radicals": coefficient.radicals.iter().map(|(radicand, scale)| [radicand.to_string(), scale.to_string()]).collect::<Vec<_>>(),
        "atoms": coefficient.atoms.iter().map(|(atom, scale)| json!([ids[&(Arc::as_ptr(&atom.internal) as usize)], scale.to_string()])).collect::<Vec<_>>(),
    })).collect())
}

#[test]
fn record_normal_form_memo_frontiers() {
    let mut rows = 0;
    let values = fixtures();
    let fixture_count = values.len();
    for (name, value) in values {
        for stage in ["fresh", "warm", "cloned"] {
            if stage == "warm" {
                let _ = value.approx(-16);
            }
            let clone = (stage == "cloned").then(|| value.clone());
            let query = clone.as_ref().unwrap_or(&value);
            let mut nodes = Vec::new();
            let mut ids = HashMap::new();
            collect_nodes(query, &mut nodes, &mut ids);
            let owners: Vec<_> = nodes.iter().map(|node| Arc::strong_count(&node.internal)).collect();
            let caches: Vec<_> = nodes.iter().map(|node| node.cached()).collect();
            for depth in [0, 1, 2, 7, 8, 47, 48, 63, 64, 65] {
                for work in [0, 1, 2, 7, 8, 31, 127, 255, 511, 512, 513] {
                    let mut remaining = work;
                    let mut memo = None;
                    let result = query.pi_laurent_polynomial_with_memo(depth, &mut remaining, &mut memo);
                    let receipt = json!({
                        "fixture": name,
                        "stage": stage,
                        "depth": depth,
                        "work": work,
                        "remaining": remaining,
                        "result": result.as_ref().map(|value| polynomial_receipt(value, &ids)),
                        "memo": memo.as_ref().map(|entries| entries.iter().map(|(key, value)| json!([ids[key], value.as_ref().map(|value| polynomial_receipt(value, &ids))])).collect::<Vec<_>>()),
                    });
                    drop(result);
                    drop(memo);
                    assert_eq!(nodes.iter().map(|node| Arc::strong_count(&node.internal)).collect::<Vec<_>>(), owners);
                    assert_eq!(nodes.iter().map(|node| node.cached()).collect::<Vec<_>>(), caches);
                    println!("LAURENT_FRONTIER {receipt}");
                    rows += 1;
                }
            }
        }
    }
    println!("LAURENT_FRONTIER_SUMMARY {}", json!({"fixtures": fixture_count, "rows": rows}));
}
