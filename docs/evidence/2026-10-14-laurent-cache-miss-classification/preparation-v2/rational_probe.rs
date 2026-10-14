//! Public arithmetic and optional cache-event counts; no cost measurements.
use hyperreal::{Rational, Real, dispatch_trace};
use std::hint::black_box;

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    dispatch_trace::reset();
    #[cfg(feature = "cache-events")]
    let (result, coverage) = hyperreal::construction_cache_trace::with_recording(|| {
        dispatch_trace::with_recording(body)
    });
    #[cfg(not(feature = "cache-events"))]
    let result = dispatch_trace::with_recording(body);
    #[cfg(feature = "cache-events")]
    let cache_events = Some(
        hyperreal::construction_cache_trace::EVENT_NAMES
            .into_iter()
            .zip(coverage)
            .map(|(name, count)| (name.to_owned(), serde_json::json!(count)))
            .collect::<serde_json::Map<String, serde_json::Value>>(),
    );
    #[cfg(not(feature = "cache-events"))]
    let cache_events: Option<serde_json::Map<String, serde_json::Value>> = None;
    #[cfg(feature = "cache-events")]
    let lookup_classes = Some(
        hyperreal::construction_cache_trace::LOOKUP_KIND_NAMES
            .into_iter()
            .zip(hyperreal::construction_cache_trace::lookup_counts())
            .map(|(kind, counts)| {
                let outcomes = hyperreal::construction_cache_trace::LOOKUP_OUTCOME_NAMES
                    .into_iter()
                    .zip(counts)
                    .map(|(name, count)| (name.to_owned(), serde_json::json!(count)))
                    .collect::<serde_json::Map<String, serde_json::Value>>();
                (kind.to_owned(), serde_json::Value::Object(outcomes))
            })
            .collect::<serde_json::Map<String, serde_json::Value>>(),
    );
    #[cfg(not(feature = "cache-events"))]
    let lookup_classes: Option<serde_json::Map<String, serde_json::Value>> = None;
    let snapshot = dispatch_trace::take_trace();
    let stats = snapshot.rational;
    let dispatch: Vec<_> = snapshot
        .dispatch
        .into_iter()
        .map(|entry| {
            serde_json::json!({"layer": entry.layer, "operation": entry.operation,
                               "path": entry.path, "count": entry.count})
        })
        .collect();
    println!(
        "{}",
        serde_json::json!({
            "trace_phase": name, "iterations": count, "dispatch": dispatch,
            "cache_events": cache_events,
            "lookup_classes": lookup_classes,
            "rational": {
                "temporary_rationals": stats.temporary_rationals,
                "reductions": stats.reductions, "gcds": stats.gcds,
                "peak_operand_bits": stats.peak_operand_bits,
                "common_factors": {"none": stats.common_factors.none,
                    "power_of_two": stats.common_factors.power_of_two,
                    "small": stats.common_factors.small, "medium": stats.common_factors.medium,
                    "large": stats.common_factors.large},
                "gcd_operand_widths": {"both_u64": stats.gcd_operand_widths.both_u64,
                    "mixed_u64_u128": stats.gcd_operand_widths.mixed_u64_u128,
                    "both_u128": stats.gcd_operand_widths.both_u128,
                    "arbitrary_precision": stats.gcd_operand_widths.arbitrary_precision}
            }
        })
    );
    black_box(result)
}

fn fixture(kind: usize, index: usize) -> Real {
    let root2 = Real::from(2).sqrt().unwrap();
    let root3 = Real::from(3).sqrt().unwrap();
    let pi = Real::pi();
    let k = Real::from((index % 7 + 1) as i32);
    let zero = match kind {
        0 => {
            // (sqrt(2) + k sqrt(3))^2 = 2 + 3k^2 + 2k sqrt(6).
            let sum = &root2 + &k * &root3;
            &sum * &sum
                - Real::from(2)
                - Real::from(3) * &k * &k
                - Real::from(2) * &k * Real::from(6).sqrt().unwrap()
        },
        1 => {
            // 1/(1 + sqrt(2)) = sqrt(2) - 1.
            ((Real::one() + &root2).inverse().unwrap() + Real::one() - &root2) * k
        },
        2 => {
            // -pi * ((pi-1)-pi/3) * pi / pi^2 + (pi-1) = pi/3.
            let third = (&pi / Real::from(3)).unwrap();
            let shifted = &pi - Real::one();
            let transported = (&pi * (&shifted - &third) * &pi / (&pi * &pi)).unwrap();
            (-transported + shifted - third) * k
        },
        3 => {
            // Laurent multiplication and reciprocal scaling preserve r+s.
            let sum = &root2 + &root3;
            let transported = ((&sum * &pi) / (&k * &pi)).unwrap() * &k;
            transported - sum
        },
        4 => {
            // Linear opaque atan atoms cancel through the same pi chart.
            let angle = Real::new(Rational::fraction(3, 4).unwrap()).atan().unwrap();
            let shifted = &angle + &pi;
            let transported = (&pi * (&pi - &shifted) * &pi / (&pi * &pi)).unwrap();
            (-transported - angle) * k
        },
        5 => {
            // (k+sqrt(2))(k-sqrt(2)) = k^2-2.
            (&k + &root2) * (&k - &root2) - &k * &k + Real::from(2)
        },
        _ => unreachable!(),
    };
    zero + Real::from((index % 3) as i32 - 1)
}

fn verify(name: &str, answers: &[Option<Rational>]) {
    let mut counts = [0_usize; 3];
    let mut hash = 0xcbf29ce484222325_u64;
    for (index, answer) in answers.iter().enumerate() {
        let expected = Rational::new((index % 3) as i64 - 1);
        assert_eq!(answer.as_ref(), Some(&expected), "{name}, item {index}");
        counts[index % 3] += 1;
        for byte in answer.as_ref().unwrap().to_string().bytes().chain(*b"\n") {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"negative\":{},\"zero\":{},\"positive\":{},\"canonical_rational_fnv1a64\":\"{hash:016x}\"}}",
        counts[0], counts[1], counts[2]
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for (kind, name) in [
        "multiradical_square",
        "quadratic_reciprocal",
        "pi_parameter_transport",
        "radical_laurent_transport",
        "opaque_linear_atom",
        "conjugate_product",
    ]
    .into_iter()
    .enumerate()
    {
        let values = measure(&format!("{name}_construction"), count, || {
            (0..count).map(|i| fixture(kind, i)).collect::<Vec<_>>()
        });
        let non_leaf = values.iter().filter(|v| v.exact_rational().is_none()).count();
        println!("{{\"fixture\":\"{name}\",\"non_leaf_inputs\":{non_leaf}}}");
        for phase in ["first", "warm"] {
            let query = format!("{name}_{phase}");
            let answers = measure(&query, count, || {
                values
                    .iter()
                    .map(|value| black_box(value).exact_rational_normal_form())
                    .collect::<Vec<_>>()
            });
            verify(&query, &answers);
        }
        let cloned = values.clone();
        let query = format!("{name}_cloned");
        let answers = measure(&query, count, || {
            cloned
                .iter()
                .map(|value| black_box(value).exact_rational_normal_form())
                .collect::<Vec<_>>()
        });
        verify(&query, &answers);
    }
}
