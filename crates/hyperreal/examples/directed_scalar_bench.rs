//! Fresh-process comparison at a matching absolute enclosure-width contract.
use hyperreal::Rational;
use num::BigUint;
use rug::Rational as Mpq;
use serde_json::{Value, json};
use std::{path::Path, time::Instant};

#[cfg(test)]
#[path = "../tests/support/representations.rs"]
mod inventory;
#[path = "directed_scalar_bench/mpfr.rs"]
mod mpfr;
#[cfg(test)]
#[path = "../tests/support/mpfr.rs"]
mod mpfr_test;
#[path = "directed_scalar_probe/oracle.rs"]
mod oracle;
#[path = "directed_scalar_probe/recipe.rs"]
mod recipe;
#[path = "directed_scalar_probe/wire.rs"]
mod wire;

use recipe::{Input, Recipe};

enum Bounds {
    Hyperreal([Rational; 2]),
    Mpfr([Mpq; 2]),
}

struct Step {
    bits: u32,
    bounds: Bounds,
}

fn precision_schedule() -> impl Iterator<Item = u32> {
    (256..=2048).step_by(128)
}

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

fn mpq_wire(value: &Mpq) -> String {
    format!("{}/{}", value.numer(), value.denom())
}

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
mod tests {
    use super::*;

    #[test]
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
