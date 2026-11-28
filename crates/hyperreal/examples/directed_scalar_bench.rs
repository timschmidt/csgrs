//! One-process enclosure measurement for a generated exact scalar recipe.
//!
//! Build with the serde feature in release mode without dispatch-trace, then pass
//! `hyperreal|mpfr INPUT.json`. The external competitive runner launches a fresh
//! process for each sample; this executable does not manage repeated samples,
//! affinity or timeouts. Both backends stop at the first interval whose exact
//! absolute width is at most 2^-256, using 15 precision settings from 256 to 2048.
//! Hyperreal's settings are absolute dyadic approximation precisions; MPFR's are
//! significand widths. Matching the output width does not equate internal work.
//!
//! The timer includes backend construction, interval materialization, width checks
//! and retention of intermediate steps. Input decoding, recipe derivation, target
//! construction, reference evaluation and JSON encoding are outside it. Hyperreal
//! reuses one Real across stages; MPFR reconstructs at each precision. Process-wide
//! constant caches can warm within either sequence.
//!
//! Post-timing checks verify the requested private class, a nonzero reference sign,
//! interval overlap and first-sufficient-width stopping. Overlap is a contradiction
//! check, not proof that either interval contains the exact value. The reference is
//! independent of Hyperreal arithmetic but shares MPFR and the recipe formulas with
//! the MPFR backend. Precision and input-size checks are not total-work or memory
//! limits. Errors use strings and an error JSON response; internal invariant
//! panics are not caught.

use hyperreal::Rational;
use num::BigUint;
use rug::Rational as Mpq;
use serde_json::{Value, json};
use std::{path::Path, time::Instant};

#[cfg(test)]
#[path = "../tests/support/representations.rs"]
/// Finite certificate fixtures used only by tests; no exhaustive-domain claim follows.
mod inventory;
#[path = "directed_scalar_bench/mpfr.rs"]
/// Variable-significand directed MPFR implementation measured as the native comparison.
mod mpfr;
#[cfg(test)]
#[path = "../tests/support/mpfr.rs"]
/// Test-thread MPFR cache cleanup; separate from the executable's final shared-cache cleanup.
mod mpfr_test;
#[path = "directed_scalar_probe/oracle.rs"]
/// Fixed 1024-bit directed reference used after timing; independent of Hyperreal, not of MPFR.
mod oracle;
#[path = "directed_scalar_probe/recipe.rs"]
/// Shared genome decoding and exact scalar construction; retains the original input in output.
mod recipe;
#[path = "directed_scalar_probe/wire.rs"]
/// Exact fraction encoder shared with the adversarial probe; avoids mixed-number Display syntax.
mod wire;

use recipe::{Input, Recipe};

/// Backend-specific exact rational endpoints retained until untimed validation and JSON encoding.
/// The variant records origin; it is not itself proof of enclosure or width.
enum Bounds {
    /// Hyperreal endpoints in intended lower/upper order; checked by the evaluator.
    Hyperreal(
        /// Exact Hyperreal lower and upper bounds from one stage.
        [Rational; 2],
    ),
    /// MPFR endpoints materialized as exact GMP rationals in intended lower/upper order.
    Mpfr(
        /// Exact GMP lower and upper bounds materialized from directed MPFR endpoints.
        [Mpq; 2],
    ),
}

/// One retained adaptive stage, including endpoints even when its width is insufficient.
struct Step {
    /// Positive schedule label: absolute dyadic precision magnitude for Hyperreal, significand bits for MPFR.
    bits: u32,
    /// Materialized endpoints kept alive through the end of timed adaptive evaluation.
    bounds: Bounds,
}

/// Yield the 15 settings 256, 384, ..., 2048 in increasing order.
/// This bounds the number of stages and requested precision, not work or allocation per stage.
fn precision_schedule() -> impl Iterator<Item = u32> {
    (256..=2048).step_by(128)
}

/// Construct one Real and reuse it across negative dyadic precision requests until
/// its exact interval width is at most target. Retain every attempted interval.
/// The benchmark supplies a positive target; this helper does not validate it.
/// Return strings for construction failure, aborted enclosure, reversed endpoints
/// or exhaustion of the 2048-bit schedule. There is no independent containment check
/// or total-resource bound here. The Real drops before returning the retained steps.
fn evaluate_hyperreal(recipe: &Recipe, target: &Rational) -> Result<Vec<Step>, String> {
    let value = recipe.real()?;
    let mut steps = Vec::new();
    for bits in precision_schedule() {
        let bounds = value
            .certified_rational_interval(-(bits as i32))
            .ok_or("Hyperreal enclosure evaluation aborted")?;
        let width = &bounds[1] - &bounds[0];
        if width < Rational::zero() {
            return Err("Hyperreal interval endpoints are reversed".into());
        }
        let sufficient = &width <= target;
        steps.push(Step {
            bits,
            bounds: Bounds::Hyperreal(bounds),
        });
        if sufficient {
            return Ok(steps);
        }
    }
    Err("Hyperreal absolute-width budget exhausted at 2048 bits".into())
}

/// Reconstruct directed MPFR bounds at each significand precision, materialize exact
/// GMP rationals, and stop at the first width at most target. Retain every stage.
/// Assumes the benchmark's positive target and generated recipe; propagate adapter
/// errors and reject reversed bounds or failure to meet width by 2048 bits.
/// Shared MPFR caches may warm between stages. No total-resource bound is installed.
fn evaluate_mpfr(recipe: &Recipe, target: &Mpq) -> Result<Vec<Step>, String> {
    let mut steps = Vec::new();
    for bits in precision_schedule() {
        let bounds = mpfr::bounds(recipe, bits)?;
        let width = Mpq::from(&bounds[1] - &bounds[0]);
        if width < 0 {
            return Err("MPFR interval endpoints are reversed".into());
        }
        let sufficient = &width <= target;
        steps.push(Step {
            bits,
            bounds: Bounds::Mpfr(bounds),
        });
        if sufficient {
            return Ok(steps);
        }
    }
    Err("MPFR absolute-width budget exhausted at 2048 bits".into())
}

/// Encode a GMP rational as signed numerator/positive denominator, including /1
/// for integers. This is exact transport, with no decimal conversion or mixed numbers.
fn mpq_wire(value: &Mpq) -> String {
    format!("{}/{}", value.numer(), value.denom())
}

/// Convert both backends' endpoints through exact fraction text, reject empty,
/// reversed or reference-disjoint stages, and require only the final stage to meet
/// the target width. Emit each precision label, endpoint pair and exact width.
///
/// Assumes an ordered, valid reference and a suitable target. It does not validate
/// the precision schedule, nesting, class or exact-value containment: overlap alone
/// cannot establish those claims. Generated fraction parsing is expected infallible
/// and panics if its encoder invariant fails. The competitive runner separately
/// checks the schedule in the emitted response.
fn checked_steps(
    steps: Vec<Step>,
    target: &Rational,
    reference: &[Rational; 2],
) -> Result<Vec<Value>, String> {
    if steps.is_empty() {
        return Err("missing adaptive enclosure steps".into());
    }
    let final_index = steps.len() - 1;
    steps.into_iter().enumerate().map(|(index, step)| {
        let text = match step.bounds {
            Bounds::Hyperreal(bounds) => bounds.each_ref().map(wire::rational),
            Bounds::Mpfr(bounds) => bounds.each_ref().map(mpq_wire),
        };
        let [lo, hi] = text.each_ref().map(|s| s.parse::<Rational>().expect("exact fraction encoding"));
        if lo > hi || hi < reference[0] || lo > reference[1] {
            return Err("native enclosure contradicts independent reference".into());
        }
        let width = &hi - &lo;
        let sufficient = &width <= target;
        if sufficient != (index == final_index) {
            return Err("adaptive evaluation stopped at the wrong absolute width".into());
        }
        Ok(json!({"precision_bits": step.bits, "interval": text, "width": wire::rational(&width)}))
    }).collect()
}

/// Require debug assertions disabled and dispatch tracing absent, validate backend
/// and two arguments, then decode a file whose metadata reports at most 4096 bytes.
/// The subsequent read is not capped, so that metadata check is not a hard read or
/// allocation bound. Input serde rejects unknown fields and malformed genomes.
///
/// Derive the recipe and exact width target before timing the selected adaptive
/// evaluator. Afterwards reconstruct a Real for class inspection, obtain the
/// 1024-bit reference and nonzero sign, and validate/encode all stages. Return schema-1
/// JSON or a string error; failed attempts do not publish a partial timing result.
/// Does not enforce external timeout, affinity or a fresh-process sampling protocol.
fn run() -> Result<Value, String> {
    if cfg!(feature = "dispatch-trace") || cfg!(debug_assertions) {
        return Err("benchmark requires a release build without dispatch-trace".into());
    }
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 2 || !["hyperreal", "mpfr"].contains(&args[0].as_str()) {
        return Err("usage: directed_scalar_bench hyperreal|mpfr INPUT.json".into());
    }
    let path = Path::new(&args[1]);
    if path.metadata().map_err(|e| e.to_string())?.len() > 4096 {
        return Err("input exceeds 4096 bytes".into());
    }
    let input: Input =
        serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
            .map_err(|e| format!("invalid input: {e}"))?;
    let recipe = Recipe::new(input);
    let width_text = format!("1/{}", BigUint::from(1_u8) << 256_usize);
    let target = width_text.parse::<Rational>().expect("positive width");
    let mpfr_target = width_text.parse::<Mpq>().expect("positive width");
    let start = Instant::now();
    let steps = match args[0].as_str() {
        "hyperreal" => evaluate_hyperreal(&recipe, &target),
        "mpfr" => evaluate_mpfr(&recipe, &mpfr_target),
        _ => unreachable!("validated backend"),
    };
    let elapsed_ns = start.elapsed().as_nanos();
    let steps = steps?;
    // Everything below is untimed. In particular the reference cannot warm
    // MPFR caches before its measured operation in this fresh process.
    let value = recipe.real()?;
    let actual_class = recipe::private_class(&value)?;
    if actual_class != recipe.input.bucket {
        return Err("benchmark recipe did not reach its declared class".into());
    }
    let reference = oracle::bounds(&recipe)?;
    let expected_sign = oracle::sign(&reference)?;
    let steps = checked_steps(steps, &target, &reference)?;
    Ok(
        json!({"schema": 1, "status": "ok", "backend": args[0], "recipe": recipe,
        "actual_class": actual_class, "rational_encoding": "signed-numerator/positive-denominator-v1",
        "elapsed_ns": elapsed_ns, "maximum_absolute_width": width_text, "steps": steps,
        "oracle": {"precision_bits": 1024, "interval": reference.each_ref().map(wire::rational),
                   "sign": format!("{expected_sign:?}")},
        "dispatch_trace_enabled": cfg!(feature = "dispatch-trace"), "debug_assertions": cfg!(debug_assertions),
        "timing_contract": "Fresh-process construction and adaptive enclosure to absolute width <= 2^-256; input decoding, output encoding and reference validation excluded."}),
    )
}

/// Run one request, free local/shared MPFR caches after normal return, and print
/// success JSON or an error JSON followed by exit status 2. This executable is
/// single-threaded and its MPFR values have dropped before cache cleanup. Panics
/// are not caught, so they bypass this cleanup/JSON path; stdout failure can panic.
fn main() {
    let result = run();
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
/// Finite qualification of adaptive stopping, width checks and exact fraction transport.
/// Tests share process caches and do not exercise release CLI gating or fresh-process timings.
mod tests {
    use super::*;

    #[test]
    /// For the 22 fixture labels and four fixed genomes, check both adaptive backends
    /// against the fixed reference and require their 1024-bit MPFR endpoint text to agree.
    /// Validate class/kind on the inventory representatives; this test does not separately
    /// assert the generated recipes' classes. The 88 cases are finite coverage, and
    /// agreement of the two MPFR paths is not independent arithmetic evidence.
    fn all_representation_boundaries_meet_the_same_absolute_width() {
        mpfr_test::cleanup_on_thread_exit();
        let target_text = format!("1/{}", BigUint::from(1_u8) << 256_usize);
        let target = target_text.parse::<Rational>().unwrap();
        let mpfr_target = target_text.parse::<Mpq>().unwrap();
        for case in inventory::optimized_certificate_representatives() {
            assert_eq!(
                recipe::private_class(&case.value).unwrap(),
                case.certificate
            );
            assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
            for genome in [
                [0; 8],
                [255; 8],
                [0, 1, 127, 3, 5, 2, 128, 77],
                [4, 31, 0, 0, 0, 1, 0, 0],
            ] {
                let recipe = Recipe::new(Input {
                    bucket: case.certificate.into(),
                    genome,
                });
                let reference = oracle::bounds(&recipe).unwrap();
                checked_steps(
                    evaluate_hyperreal(&recipe, &target).unwrap(),
                    &target,
                    &reference,
                )
                .unwrap();
                checked_steps(
                    evaluate_mpfr(&recipe, &mpfr_target).unwrap(),
                    &target,
                    &reference,
                )
                .unwrap();
                let direct = mpfr::bounds(&recipe, 1024).unwrap();
                assert_eq!(
                    direct.each_ref().map(mpq_wire),
                    reference.each_ref().map(wire::rational)
                );
            }
        }
    }

    #[test]
    /// Preserve the original eight-byte mixed-number regression input, require both
    /// backends to advance beyond 256 bits, then validate their retained stages.
    /// Also assert all 15 schedule entries and rejection of MPFR precisions 63 and 4097.
    /// This demonstrates adaptive refinement for one case, not a total-work bound.
    fn wide_scaled_regression_requires_refinement_and_preserves_the_full_input() {
        mpfr_test::cleanup_on_thread_exit();
        let input: Input = serde_json::from_str(include_str!(
            "directed_scalar_probe/regressions/mixed_number_v2.input.json"
        ))
        .unwrap();
        assert_eq!(input.genome, [47, 128, 223, 71, 133, 255, 155, 107]);
        let recipe = Recipe::new(input);
        let target_text = format!("1/{}", BigUint::from(1_u8) << 256_usize);
        let target = target_text.parse::<Rational>().unwrap();
        let mpfr_target = target_text.parse::<Mpq>().unwrap();
        let reference = oracle::bounds(&recipe).unwrap();
        for steps in [
            evaluate_hyperreal(&recipe, &target).unwrap(),
            evaluate_mpfr(&recipe, &mpfr_target).unwrap(),
        ] {
            assert!(steps.last().unwrap().bits > 256);
            checked_steps(steps, &target, &reference).unwrap();
        }
        assert_eq!(
            precision_schedule().collect::<Vec<_>>(),
            (0..15).map(|n| 256 + n * 128).collect::<Vec<_>>()
        );
        assert!(mpfr::bounds(&recipe, 63).is_err());
        assert!(mpfr::bounds(&recipe, 4097).is_err());
    }
}
