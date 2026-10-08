use hypercurve::{Rational, Real};
use hypersolve::{IsolatedRootInterval, sign_at_selected_root};
use std::cmp::Ordering;
use std::time::Instant;

fn polynomial(lines: &mut std::str::Lines<'_>, label: &str) -> Vec<Real> {
    let mut header = lines.next().unwrap().split_whitespace();
    assert_eq!(header.next(), Some(label));
    let count: usize = header.next().unwrap().parse().unwrap();
    (0..count)
        .map(|_| {
            let mut fields = lines.next().unwrap().split_whitespace();
            Real::new(
                Rational::from_bigint_fraction(
                    fields.next().unwrap().parse().unwrap(),
                    fields.next().unwrap().parse().unwrap(),
                )
                .unwrap(),
            )
        })
        .collect()
}

fn main() {
    for (id, expected) in [
        (10, Ordering::Less),
        (13, Ordering::Greater),
        (19, Ordering::Greater),
        (21, Ordering::Greater),
        (24, Ordering::Less),
        (31, Ordering::Less),
        (33, Ordering::Greater),
        (43, Ordering::Less),
    ] {
        let input = std::fs::read_to_string(format!(
            "stationary-ph-scalar-query-{id}.decimal.txt"
        ))
        .unwrap();
        let mut lines = input.lines();
        assert_eq!(lines.next().unwrap(), format!("query {id}"));
        let defining = polynomial(&mut lines, "defining");
        let predicate = polynomial(&mut lines, "predicate");
        let bounds = polynomial(&mut lines, "interval");
        let interval = IsolatedRootInterval {
            lower: bounds[0].clone(),
            upper: bounds[1].clone(),
            exact_root: None,
            distinct_root_count: 1,
        };
        let parity = (0..2).find(|parity| {
            predicate
                .iter()
                .enumerate()
                .all(|(power, value)| power % 2 == *parity || value == &Real::zero())
        });
        let started = Instant::now();
        assert_eq!(sign_at_selected_root(&defining, &predicate, &interval), Some(expected));
        let original = started.elapsed();
        let Some(parity) = parity else {
            println!("query={id} mixed-power original={original:?} expected={expected:?}");
            continue;
        };
        assert!(defining.iter().skip(1).step_by(2).all(|value| value == &Real::zero()));
        assert!(interval.lower > Real::zero());
        // Positive squaring is injective on the selected interval. P(t)=R(t²)
        // therefore transfers its singleton certificate. For Q(t)=t^r S(t²),
        // the omitted factor is strictly positive on this particular chart.
        let reduced_defining: Vec<_> = defining.iter().step_by(2).cloned().collect();
        let reduced_predicate: Vec<_> = predicate.iter().skip(parity).step_by(2).cloned().collect();
        let reduced_interval = IsolatedRootInterval {
            lower: &interval.lower * &interval.lower,
            upper: &interval.upper * &interval.upper,
            exact_root: None,
            distinct_root_count: 1,
        };
        let started = Instant::now();
        assert_eq!(
            sign_at_selected_root(&reduced_defining, &reduced_predicate, &reduced_interval),
            Some(expected),
        );
        println!("query={id} parity={parity} original={original:?} reduced={:?} expected={expected:?}", started.elapsed());
    }
}
