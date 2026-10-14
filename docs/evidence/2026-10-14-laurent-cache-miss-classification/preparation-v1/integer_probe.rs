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

fn integer(kind: usize, index: usize) -> Rational {
    let values: &[&str] = match kind {
        0 => &["-65", "-64", "-1", "0", "1", "64", "65"],
        1 => &[
            "-9223372036854775809",
            "-9223372036854775808",
            "-9223372036854775807",
            "9223372036854775807",
            "9223372036854775808",
            "9223372036854775809",
        ],
        2 => &[
            "-340282366920938463463374607431768211457",
            "340282366920938463463374607431768211455",
            "-57896044618658097711785492504343953926634992332820282019728792003956564819968",
            "57896044618658097711785492504343953926634992332820282019728792003956564819969",
            "-179769313486231590772930519078902473361797697894230657273430081157732675805500963132708477322407536021120113879871393357658789768814416622492847430639474124377767893424865485276302219601246094119453082952085005768838150682342462881473913110540827237163350510684586298239947245938479716304835356329624224137215",
            "179769313486231590772930519078902473361797697894230657273430081157732675805500963132708477322407536021120113879871393357658789768814416622492847430639474124377767893424865485276302219601246094119453082952085005768838150682342462881473913110540827237163350510684586298239947245938479716304835356329624224137217",
        ],
        _ => unreachable!(),
    };
    values[index % values.len()].parse().unwrap()
}

fn fixture(kind: usize, index: usize) -> Real {
    let root = Real::from(13).sqrt().unwrap();
    let scale = if index % 2 == 0 { 1 } else { -1 };
    Real::new(integer(kind, index)) + Real::from(scale) * root
}

fn expected(kind: usize, index: usize) -> Option<(Rational, Rational, Rational)> {
    Some((
        integer(kind, index),
        Rational::new(if index % 2 == 0 { 1 } else { -1 }),
        Rational::new(13),
    ))
}

fn verify(name: &str, kind: usize, answers: &[Option<(Rational, Rational, Rational)>]) {
    let mut certified = 0_usize;
    let mut hash = 0xcbf29ce484222325_u64;
    for (index, answer) in answers.iter().enumerate() {
        assert_eq!(*answer, expected(kind, index), "{name}, item {index}");
        let witness = match answer {
            Some((a, b, d)) => {
                certified += 1;
                format!("{a};{b};{d}\n")
            },
            None => "Unknown\n".to_owned(),
        };
        for byte in witness.bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"certified\":{certified},\"unknown\":{},\"quadratic_parts_fnv1a64\":\"{hash:016x}\"}}",
        answers.len() - certified
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for (kind, name) in [
        "canonical_integer_boundary",
        "signed_word_boundary",
        "wide_integer_affine",
    ]
    .into_iter()
    .enumerate()
    {
        let values = measure(&format!("{name}_construction"), count, || {
            (0..count)
                .map(|index| fixture(kind, index))
                .collect::<Vec<_>>()
        });
        let non_leaf = values.iter().filter(|v| v.exact_rational().is_none()).count();
        println!("{{\"fixture\":\"{name}\",\"non_leaf_inputs\":{non_leaf}}}");
        for phase in ["first", "warm"] {
            let query = format!("{name}_{phase}");
            let answers = measure(&query, count, || {
                values
                    .iter()
                    .map(|value| black_box(value).exact_quadratic_normal_form())
                    .collect::<Vec<_>>()
            });
            verify(&query, kind, &answers);
        }
        let cloned = values.clone();
        let query = format!("{name}_cloned");
        let answers = measure(&query, count, || {
            cloned
                .iter()
                .map(|value| black_box(value).exact_quadratic_normal_form())
                .collect::<Vec<_>>()
        });
        verify(&query, kind, &answers);
    }
}
