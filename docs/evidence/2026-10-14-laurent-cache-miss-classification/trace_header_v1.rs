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
