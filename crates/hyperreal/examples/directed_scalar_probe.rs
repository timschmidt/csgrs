//! Development-only process probe for representation-bucketed adversarial search.

use hyperreal::{CertifiedRealSign, Rational, Real, RealSign, dispatch_trace};
use serde_json::{Value, json};
use std::{path::Path, time::Instant};

#[path = "../tests/support/representations.rs"]
mod inventory;
#[path = "../tests/support/mpfr.rs"]
mod mpfr;
#[path = "directed_scalar_probe/oracle.rs"]
mod oracle;
#[path = "directed_scalar_probe/recipe.rs"]
mod recipe;
#[path = "directed_scalar_probe/wire.rs"]
mod wire;

use recipe::{Input, Recipe};

const QUERIES: [&str; 6] = [
    "normal_form",
    "affine_det3",
    "sign_zero",
    "sign_256",
    "interval_64",
    "interval_256",
];

enum Answer {
    Normal(Option<Rational>),
    Sign(Option<RealSign>),
    Interval([Rational; 2]),
}

fn sign(value: &Real, floor: i32) -> Option<RealSign> {
    match value.certified_sign_until(floor) {
        CertifiedRealSign::Known { sign, .. } => Some(sign),
        CertifiedRealSign::Unknown { .. } => None,
    }
}

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
mod tests {
    use super::*;

    #[test]
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
