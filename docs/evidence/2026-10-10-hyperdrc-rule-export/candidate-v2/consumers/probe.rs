//! Exercise the checked KiCad rule exporter and preserve typed failures.
use hyperdrc::Scalar;
use hyperdrc::config::{RuleConfig, RuleOverrides, effective_rules};
use hyperdrc::kicad_dru::{merge_rules_into_kicad_dru, rules_to_kicad_dru};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

fn unresolved_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

fn value(name: &str) -> Scalar {
    match name {
        "pi" => Scalar::pi(),
        "unresolved_zero" => unresolved_zero(),
        "unresolved_positive" => unresolved_zero() + "0.2".parse::<Scalar>().unwrap(),
        "unresolved_tie" => unresolved_zero() + "0.1234565".parse::<Scalar>().unwrap(),
        "unresolved_tiny" => unresolved_zero() + "1e-40".parse::<Scalar>().unwrap(),
        "aborted_pi" => {
            let mut value = Scalar::pi();
            value.abort(Arc::new(AtomicBool::new(true)));
            value
        },
        _ => name.parse().unwrap(),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    if args[1] == "matrix" {
        let input = std::fs::read_to_string(&args[3]).unwrap();
        for token in input.lines() {
            let mut rules = effective_rules(&RuleConfig::default(), RuleOverrides::default());
            rules.min_width = token.parse().unwrap();
            if args[2] == "prepared" {
                let _ = rules.min_width.certified_rational_interval(-256);
            }
            match rules_to_kicad_dru(&rules) {
                Ok(text) => {
                    let threshold = text
                        .split("(constraint track_width (min ")
                        .nth(1)
                        .map(|s| s.split("mm)").next().unwrap())
                        .unwrap_or("omitted");
                    println!("{token}\tOk\t{threshold}");
                },
                Err(error) => println!("{token}\tErr\t{error:?}"),
            }
        }
        return;
    }
    let mut rules = effective_rules(&RuleConfig::default(), RuleOverrides::default());
    if args[1] != "default" {
        rules.min_width = value(&args[1]);
    }
    eprintln!("stage: constructed");
    if args[2] == "prepared" {
        let _ = rules.min_width.certified_rational_interval(-256);
    }
    eprintln!("stage: export");
    let text = if args[3] == "render" {
        rules_to_kicad_dru(&rules)
    } else {
        let original = std::fs::read_to_string(&args[3]).unwrap();
        merge_rules_into_kicad_dru(&original, &rules)
    };
    let text = match text {
        Ok(text) => text,
        Err(error) => {
            eprintln!("stage: rejected {error:?}");
            eprintln!("{error}");
            std::process::exit(2);
        },
    };
    std::fs::write(&args[4], &text).unwrap();
    eprintln!("stage: written");
    println!("{text}");
}
