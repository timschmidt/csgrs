//! Development process probe for adversarial search over exact scalar recipes.
//!
//! Build with serde and dispatch-trace. `--list` reports the finite representation
//! and query inventories; a JSON input path runs all six queries, three times each.
//! Each sample constructs and inspects its own Real before resetting trace counters
//! and timing the instrumented query. Shared constants and earlier samples can warm
//! caches: fresh objects do not mean fresh-process or universally cold execution.
//! The median is search fitness, not an uninstrumented competitive benchmark.
//!
//! A fixed directed MPFR reference supplies a nonzero sign and contradiction checks.
//! Rational answers must lie in the reference, signs must agree when known, and
//! intervals must overlap it in valid endpoint order. These checks do not independently
//! prove exact-value containment. Unknown answers remain JSON null. A final ordered
//! cache exercise rejects loss of a definite fact for that sequence; it is not an
//! exhaustive cache, concurrency, eviction or policy-transition test.
//!
//! Query precision floors and the pre-read 4096-byte metadata check are not hard
//! work, read-size, memory or cancellation bounds. The external campaign owns process
//! timeouts. String failures become error JSON/status 2; invariant panics are not
//! caught. Successful output retains exact fraction text, traces and every sample,
//! but simplifies rich sign receipts to sign values for this diagnostic protocol.

use hyperreal::{CertifiedRealSign, Rational, Real, RealSign, dispatch_trace};
use serde_json::{Value, json};
use std::{path::Path, time::Instant};

#[path = "../tests/support/representations.rs"]
/// Shared finite certificate fixtures for --list and regression tests; labels do not prove constructed classes.
mod inventory;
#[path = "../tests/support/mpfr.rs"]
/// Register test-thread MPFR cache cleanup; main separately frees local/shared caches on normal return.
mod mpfr;
#[path = "directed_scalar_probe/oracle.rs"]
/// Directed 1024-bit MPFR recipe evaluation, independent of Hyperreal arithmetic.
mod oracle;
#[path = "directed_scalar_probe/recipe.rs"]
/// Deterministic eight-byte input expansion and exact Real construction shared with the benchmark.
mod recipe;
#[path = "directed_scalar_probe/wire.rs"]
/// Exact signed numerator/positive denominator output; avoids human mixed-number Display syntax.
mod wire;

use recipe::{Input, Recipe};

/// Fixed query order used for samples, cache-sensitive execution history and the external evidence schema.
/// Normal form precedes affine sign, sign floors 0/-256, and interval precisions -64/-256.
const QUERIES: [&str; 6] = [
    "normal_form",
    "affine_det3",
    "sign_zero",
    "sign_256",
    "interval_64",
    "interval_256",
];

/// Diagnostic projection of a query result before reference checking and JSON encoding.
/// Sign results omit their richer certainty/proof details; None records unresolved
/// normal-form or sign queries rather than a false or zero answer.
enum Answer {
    /// Optional exact rational normal form recognized by the scalar layer.
    Normal(
        /// Recognized rational value, or None when this query does not produce a normal form.
        Option<Rational>,
    ),
    /// Optional certified sign, without the original sign-query receipt.
    Sign(
        /// Known sign or None for an unresolved query; no guessed sign is substituted.
        Option<RealSign>,
    ),
    /// Closed interval endpoints returned by the certified scalar enclosure query.
    Interval(
        /// Intended lower/upper bounds, whose ordering and reference overlap are checked later.
        [Rational; 2],
    ),
}

/// Request a certified sign down to the supplied dyadic precision floor and retain
/// only its sign or unresolved None. The richer Known receipt is intentionally
/// projected away for diagnostic JSON. The floor is not a total-work bound.
fn sign(value: &Real, floor: i32) -> Option<RealSign> {
    match value.certified_sign_until(floor) {
        CertifiedRealSign::Known { sign, .. } => Some(sign),
        CertifiedRealSign::Unknown { .. } => None,
    }
}

/// Embed value as the first diagonal entry of an affine 3D determinant: the other
/// two diagonal entries are one and the fourth point is the origin. Its exact
/// determinant is value. Return the specialized certified sign or None, exercising
/// that predicate path without numerical perturbation or a general matrix workload.
fn affine(value: &Real) -> Option<RealSign> {
    let zero = Real::zero();
    let one = Real::one();
    Real::certified_affine_det3_sign(
        [value, &zero, &zero],
        [&zero, &one, &zero],
        [&zero, &zero, &one],
        [&zero, &zero, &zero],
    )
}

/// Dispatch one of the six names to normal-form, affine-sign, bounded-sign or
/// certified-interval queries. Unknown names return a string error. Interval None
/// is reported with the existing exhausted-message text, although the underlying
/// API returns None on abortion; this wrapper supplies no independent total-resource
/// budget or proof that one was exhausted. Normal/sign uncertainty remains None.
fn query(value: &Real, name: &str) -> Result<Answer, String> {
    Ok(match name {
        "normal_form" => Answer::Normal(value.exact_rational_normal_form()),
        "affine_det3" => Answer::Sign(affine(value)),
        "sign_zero" => Answer::Sign(sign(value, 0)),
        "sign_256" => Answer::Sign(sign(value, -256)),
        "interval_64" | "interval_256" => Answer::Interval(
            value
                .certified_rational_interval(if name == "interval_64" { -64 } else { -256 })
                .ok_or("bounded interval query exhausted")?,
        ),
        _ => return Err("unknown query".into()),
    })
}

/// Encode an answer after checking it against the supplied reference and sign.
/// A recognized rational must lie inside the reference; a known sign must match;
/// interval endpoints must be ordered and overlap the reference. Unknown normal/sign
/// answers pass as JSON null. Assume the caller supplied a valid ordered reference
/// and its correct sign; these are not cross-validated here. Overlap or rational
/// membership is a contradiction check, not proof of equality to the exact source.
fn checked_answer(
    answer: Answer,
    reference: &[Rational; 2],
    expected_sign: RealSign,
) -> Result<Value, String> {
    match answer {
        Answer::Normal(value) => {
            if value
                .as_ref()
                .is_some_and(|x| x < &reference[0] || x > &reference[1])
            {
                return Err("rational normal form contradicts independent MPFR enclosure".into());
            }
            Ok(json!({"normal_form": value.as_ref().map(wire::rational)}))
        }
        Answer::Sign(value) => {
            if value.is_some_and(|x| x != expected_sign) {
                return Err("certified sign contradicts independent MPFR enclosure".into());
            }
            Ok(json!({"sign": value.map(|x| format!("{x:?}"))}))
        }
        Answer::Interval([lo, hi]) => {
            if lo > hi || hi < reference[0] || lo > reference[1] {
                return Err(
                    "certified interval is disjoint from independent MPFR enclosure".into(),
                );
            }
            Ok(json!({"interval": [wire::rational(&lo), wire::rational(&hi)]}))
        }
    }
}

/// Construct a Real, query normal form/affine sign/sign at floor zero, then a
/// -256 interval and sign before repeating the three initial queries. Require an
/// available normal/affine fact to survive, and definite signs to survive each
/// cold-to-fine-to-warm transition. Validate the interval and final answers against
/// the reference, then return snapshots for external inspection.
///
/// Cold names the initial queries on this object, not empty process caches; construction
/// and earlier queries can already refine it. Normal form equality here is exact
/// rational equality. Unknown may strengthen. This ordered sequence does not cover
/// all cache states or compare full proof receipts; query/constructor failures
/// propagate as strings and invariant panics remain uncaught.
fn check_cache(
    recipe: &Recipe,
    reference: &[Rational; 2],
    expected_sign: RealSign,
) -> Result<Value, String> {
    let value = recipe.real()?;
    let cold_normal = value.exact_rational_normal_form();
    let cold_affine = affine(&value);
    let cold_sign = sign(&value, 0);
    let _ = checked_answer(query(&value, "interval_256")?, reference, expected_sign)?;
    let fine_sign = sign(&value, -256);
    let warm_normal = value.exact_rational_normal_form();
    let warm_affine = affine(&value);
    let warm_sign = sign(&value, 0);
    if cold_normal
        .as_ref()
        .is_some_and(|x| warm_normal.as_ref() != Some(x))
        || cold_affine.is_some_and(|x| warm_affine != Some(x))
        || cold_sign.is_some_and(|x| fine_sign != Some(x))
        || fine_sign.is_some_and(|x| warm_sign != Some(x))
    {
        return Err("cache warming weakened or contradicted a definite fact".into());
    }
    let record = json!({
        "cold": {"normal": cold_normal.as_ref().map(wire::rational),
                 "affine": cold_affine.map(|x| format!("{x:?}")), "sign": cold_sign.map(|x| format!("{x:?}"))},
        "refined_sign": fine_sign.map(|x| format!("{x:?}")),
        "warm": {"normal": warm_normal.as_ref().map(wire::rational),
                 "affine": warm_affine.map(|x| format!("{x:?}")), "sign": warm_sign.map(|x| format!("{x:?}"))},
    });
    for answer in [
        Answer::Normal(warm_normal),
        Answer::Sign(warm_affine),
        Answer::Sign(warm_sign),
    ] {
        let _ = checked_answer(answer, reference, expected_sign)?;
    }
    Ok(record)
}

/// Expand the input, compute a separately timed 1024-bit reference and its nonzero
/// sign, then run three instrumented samples of each query in fixed order. Before
/// each sample, construct a Real and verify its serialized private class. Reset
/// trace counters, time with_recording(query), then take/clear traces and validate
/// the answer outside the timer. Construction, class inspection, reference work,
/// answer encoding and value destruction are excluded from query-body timings.
///
/// Keep all answers/traces and choose the median of three u64 elapsed-nanosecond
/// values; an elapsed value not representable as u64 panics rather than enforcing
/// a time limit. Run the cache sequence after all samples. A failure aborts this
/// request without publishing the partially collected rows. Test-thread cleanup is
/// registered, but caches remain shared during the request. Debug builds are accepted;
/// this is diagnostic search, with trace overhead inside the timer.
fn probe(input: Input) -> Result<Value, String> {
    mpfr::cleanup_on_thread_exit();
    let recipe = Recipe::new(input);
    let oracle_start = Instant::now();
    let reference = oracle::bounds(&recipe)?;
    let oracle_ns = oracle_start.elapsed().as_nanos();
    let expected_sign = oracle::sign(&reference)?;
    let mut rows = Vec::new();
    for name in QUERIES {
        let mut samples = Vec::new();
        for sample in 0..3 {
            let value = recipe.real()?;
            let actual = recipe::private_class(&value)?;
            if actual != recipe.input.bucket {
                return Err(format!(
                    "unexpected representation normalization: {} -> {actual}",
                    recipe.input.bucket
                ));
            }
            dispatch_trace::reset();
            let start = Instant::now();
            let answer = dispatch_trace::with_recording(|| query(&value, name));
            let elapsed_ns = start.elapsed().as_nanos();
            let trace = dispatch_trace::take_trace();
            let answer = checked_answer(answer?, &reference, expected_sign)?;
            samples.push(json!({
                "sample": sample, "elapsed_ns": elapsed_ns, "answer": answer,
                "dispatch": trace.dispatch.iter().map(|row| json!({
                    "layer": row.layer, "operation": row.operation, "path": row.path, "count": row.count,
                })).collect::<Vec<_>>(),
                "rational": {"temporaries": trace.rational.temporary_rationals,
                             "reductions": trace.rational.reductions, "gcds": trace.rational.gcds,
                             "peak_operand_bits": trace.rational.peak_operand_bits},
            }));
        }
        let mut nanos = samples
            .iter()
            .map(|row| row["elapsed_ns"].as_u64().expect("bounded probe duration"))
            .collect::<Vec<_>>();
        nanos.sort_unstable();
        rows.push(json!({"query": name, "score_ns": nanos[1], "samples": samples}));
    }
    let cache = check_cache(&recipe, &reference, expected_sign)?;
    Ok(json!({
        "schema": 1, "status": "ok", "recipe": recipe,
        "rational_encoding": "signed-numerator/positive-denominator-v1",
        "oracle": {"precision_bits": 1024, "interval": reference.each_ref().map(wire::rational),
                   "sign": format!("{expected_sign:?}"), "diagnostic_elapsed_ns": oracle_ns},
        "queries": rows, "cache_contract": cache, "cache_contract_checked": true,
        "score_contract": "Median of three query-body elapsed times; every sample and trace is retained. This is search fitness, not a competitive benchmark.",
    }))
}

/// Accept exactly --list or one input path. Listing constructs the finite inventory
/// and reports expected public kinds plus actual serialized private labels; it does
/// not assert they agree. Input mode checks metadata length <=4096 before an uncapped
/// UTF-8 read and serde decoding, then runs the probe. Return string errors for
/// usage, filesystem, decoding or probe failures. There is no release-only gate,
/// atomic file snapshot, preemptive timeout or enforced maximum read allocation.
fn run() -> Result<Value, String> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args == ["--list"] {
        return Ok(
            json!({"representations": inventory::optimized_certificate_representatives().iter()
            .map(|case| json!({"name": case.certificate, "kind": format!("{:?}", case.public_kind),
                             "actual_class": recipe::private_class(&case.value).expect("inventory serializes")}))
            .collect::<Vec<_>>(), "queries": QUERIES}),
        );
    }
    if args.len() != 1 {
        return Err("usage: directed_scalar_probe INPUT.json | --list".into());
    }
    let path = Path::new(&args[0]);
    if path.metadata().map_err(|e| e.to_string())?.len() > 4096 {
        return Err("input exceeds 4096 bytes".into());
    }
    let input = serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| format!("invalid input: {e}"))?;
    probe(input)
}

/// Run one request and free local/shared MPFR caches after normal return in this
/// single-threaded executable. Print returned JSON on success, or schema-1 error
/// JSON and exit 2 on failure. Listing has its own inventory object shape. Uncaught
/// panics bypass this cleanup/error conversion; stdout failures can also panic.
fn main() {
    let result = run();
    // This standalone process is single-threaded; all oracle Float values have
    // dropped before releasing its local and shared MPFR caches.
    rug::float::free_cache(rug::float::FreeCache::All);
    match result {
        Ok(value) => println!("{value}"),
        Err(error) => {
            println!(
                "{}",
                json!({"schema": 1, "status": "error", "error": error})
            );
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
/// Finite regression corpus for exact transport, recipe classes, query contradictions and cache transitions.
/// These in-process tests do not establish cold-process performance or universal scalar support.
mod tests {
    use super::*;

    #[test]
    /// Retain the original eight-byte input and historical mixed-number response.
    /// Run the probe, then compare zipped old/new reference endpoints using independent
    /// Rug fraction arithmetic, including negative mixed-number subtraction. Require
    /// new text to contain / and no spaces; this is not full-response or array-length equality.
    fn original_mixed_number_failure_uses_exact_fraction_wire_output() {
        mpfr::cleanup_on_thread_exit();
        let input: Input = serde_json::from_str(include_str!(
            "directed_scalar_probe/regressions/mixed_number_v2.input.json"
        ))
        .unwrap();
        assert_eq!(input.genome, [47, 128, 223, 71, 133, 255, 155, 107]);
        let previous: Value = serde_json::from_str(include_str!(
            "directed_scalar_probe/regressions/mixed_number_v2.response.json"
        ))
        .unwrap();
        let response = probe(input).unwrap();
        for (old, new) in previous["oracle"]["interval"]
            .as_array()
            .unwrap()
            .iter()
            .zip(response["oracle"]["interval"].as_array().unwrap())
        {
            let old = old.as_str().unwrap();
            let new = new.as_str().unwrap();
            assert!(old.contains(' ') && !new.contains(' ') && new.contains('/'));
            let (whole, remainder) = old.split_once(' ').unwrap();
            let whole = whole.parse::<rug::Rational>().unwrap();
            let remainder = remainder.parse::<rug::Rational>().unwrap();
            let expected = if old.starts_with('-') {
                whole - remainder
            } else {
                whole + remainder
            };
            assert_eq!(expected, new.parse::<rug::Rational>().unwrap());
        }
    }

    /// Construct the supplied recipe and assert its private class, then obtain the
    /// reference sign, check all six queries sequentially on one value and exercise
    /// the separate cache sequence. Panics on failures. This helper does not time
    /// queries or assert dispatch counts, and callers register MPFR cleanup.
    fn check_recipe(recipe: &Recipe, expected_class: &str) {
        let value = recipe.real().unwrap();
        assert_eq!(
            recipe::private_class(&value).unwrap(),
            expected_class,
            "{recipe:?}"
        );
        let bounds = oracle::bounds(recipe).unwrap();
        let expected = oracle::sign(&bounds).unwrap();
        for name in QUERIES {
            checked_answer(query(&value, name).unwrap(), &bounds, expected).unwrap();
        }
        check_cache(recipe, &bounds, expected).unwrap();
    }

    #[test]
    /// Keep the fixture's original ConstOffset label and all-zero genome assertions,
    /// then deliberately select Irrational for the same historical near-pi expression.
    /// Require offset 31/10, scale one and the near-pi branch, and check pi-31/10 through
    /// all queries. The current ConstOffset generator has distinct positive-offset rules.
    fn original_near_pi_failure_is_preserved_with_its_actual_class() {
        mpfr::cleanup_on_thread_exit();
        let mut input: Input = serde_json::from_str(include_str!(
            "directed_scalar_probe/regressions/near_pi_v1.json"
        ))
        .unwrap();
        assert_eq!(input.bucket, "ConstOffset");
        assert_eq!(input.genome, [0; 8]);
        // The failed expectation is retained in the fixture. The same exact
        // expression belongs to Irrational under the documented constructor.
        input.bucket = "Irrational".into();
        let recipe = Recipe::new(input);
        assert_eq!(recipe.pi_offset, "31/10");
        assert_eq!(recipe.scale, "1/1");
        assert!(recipe.generic_near_pi);
        check_recipe(&recipe, "Irrational");
    }

    #[test]
    /// Check 156 near-pi recipes (78 decimal depths from both sides) and sixteen
    /// constant-offset recipes (four families times four fixed genomes). Preserve low
    /// three selector bits while assigning each offset family. Assert class, all query
    /// contradiction checks and the cache sequence; this is finite deterministic coverage.
    fn both_sides_of_all_near_pi_depths_and_all_offset_families_are_checked() {
        mpfr::cleanup_on_thread_exit();
        for depth in 0..78 {
            for side in [0, 2] {
                let mut genome = [0; 8];
                genome[5] = side;
                genome[7] = depth;
                check_recipe(
                    &Recipe::new(Input {
                        bucket: "Irrational".into(),
                        genome,
                    }),
                    "Irrational",
                );
            }
        }
        for family in 0..4 {
            for mut genome in [
                [0; 8],
                [255; 8],
                [0, 1, 127, 3, 5, 2, 128, 77],
                [4, 31, 0, 0, 0, 1, 0, 0],
            ] {
                genome[5] = (genome[5] & 7) | (family << 3);
                check_recipe(
                    &Recipe::new(Input {
                        bucket: "ConstOffset".into(),
                        genome,
                    }),
                    "ConstOffset",
                );
            }
        }
    }

    #[test]
    /// Require the 22-entry inventory, then check private class and public structural
    /// kind for each of four fixed genomes per label. Compare all six sequential
    /// queries against directed MPFR and exercise cache preservation for all 88 recipes.
    /// These selected parameter extremes are not exhaustive over eight-byte inputs.
    fn parameter_extremes_keep_every_private_class_and_agree_with_mpfr() {
        mpfr::cleanup_on_thread_exit();
        let cases = inventory::optimized_certificate_representatives();
        assert_eq!(cases.len(), 22);
        for genome in [
            [0; 8],
            [255; 8],
            [0, 1, 127, 3, 5, 2, 128, 77],
            [4, 31, 0, 0, 0, 1, 0, 0],
        ] {
            for case in &cases {
                let recipe = Recipe::new(Input {
                    bucket: case.certificate.into(),
                    genome,
                });
                let value = recipe.real().unwrap();
                assert_eq!(
                    recipe::private_class(&value).unwrap(),
                    case.certificate,
                    "{recipe:?}"
                );
                assert_eq!(value.detailed_facts().symbolic.kind, case.public_kind);
                let bounds = oracle::bounds(&recipe).unwrap();
                let expected = oracle::sign(&bounds).unwrap();
                for name in QUERIES {
                    checked_answer(query(&value, name).unwrap(), &bounds, expected).unwrap();
                }
                check_cache(&recipe, &bounds, expected).unwrap();
            }
        }
    }

    #[test]
    /// Require an unknown bucket to fail Real construction, and serde to reject an empty
    /// genome, an out-of-range byte and an extra field. Does not exercise file-size
    /// checks, every malformed JSON form or manually inconsistent expanded recipes.
    fn unknown_representations_and_malformed_genomes_are_rejected() {
        assert!(
            Recipe::new(Input {
                bucket: "not-a-class".into(),
                genome: [0; 8]
            })
            .real()
            .is_err()
        );
        for input in [
            r#"{"bucket":"Pi","genome":[]}"#,
            r#"{"bucket":"Pi","genome":[256,0,0,0,0,0,0,0]}"#,
            r#"{"bucket":"Pi","genome":[0,0,0,0,0,0,0,0],"extra":true}"#,
        ] {
            assert!(serde_json::from_str::<Input>(input).is_err());
        }
    }

    #[test]
    /// Against reference `[1,2]` and Positive, reject a negative sign, rational three and
    /// interval `[3,4]`, while accepting an unresolved sign. Uses exact literals rather
    /// than MPFR evaluation; does not assert that overlapping intervals prove containment.
    fn independent_oracle_rejects_opposite_sign_and_disjoint_intervals() {
        let bounds = [Rational::new(1), Rational::new(2)];
        assert!(
            checked_answer(
                Answer::Sign(Some(RealSign::Negative)),
                &bounds,
                RealSign::Positive
            )
            .is_err()
        );
        assert!(
            checked_answer(
                Answer::Normal(Some(Rational::new(3))),
                &bounds,
                RealSign::Positive
            )
            .is_err()
        );
        assert!(
            checked_answer(
                Answer::Interval([Rational::new(3), Rational::new(4)]),
                &bounds,
                RealSign::Positive
            )
            .is_err()
        );
        assert!(checked_answer(Answer::Sign(None), &bounds, RealSign::Positive).is_ok());
    }
}
