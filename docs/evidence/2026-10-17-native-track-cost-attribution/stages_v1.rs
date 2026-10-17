use hyperdrc::checks;
use hyperdrc::kicad::load_kicad_pcb;
use hyperdrc::report::Violation;
use hyperdrc::Scalar;
use hyperlimit::PredicatePolicy;
use serde_json::{Value, json};
use std::collections::BTreeMap;
use std::path::Path;
use std::sync::OnceLock;
use std::time::Instant;

static START: OnceLock<Instant> = OnceLock::new();
struct Trace;
impl log::Log for Trace {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.target().starts_with("hyperdrc::checks::layer")
    }
    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            eprintln!("TRACE {:.9} {} {}", START.get().unwrap().elapsed().as_secs_f64(),
                      record.target(), record.args());
        }
    }
    fn flush(&self) {}
}
static TRACE: Trace = Trace;

fn stage<T>(name: &str, run: impl FnOnce() -> T) -> T {
    eprintln!("START {:.9} {name}", START.get().unwrap().elapsed().as_secs_f64());
    let start = Instant::now();
    let result = hypercurve::evaluate_under(PredicatePolicy::STRICT, run).certified()
        .expect("a strict local diagnostic scope cannot consume approximate facts");
    let elapsed = start.elapsed().as_secs_f64();
    println!("{}", json!({"stage": name, "elapsed_seconds": elapsed, "local_receipt": "Certified"}));
    eprintln!("END {:.9} {name} {elapsed:.9}", START.get().unwrap().elapsed().as_secs_f64());
    result
}

fn check(name: &str, run: impl FnOnce() -> Vec<Violation>, findings: &mut Vec<Value>) {
    let result = stage(name, run);
    println!("{}", json!({"stage": name, "findings": result.len(),
        "checks": result.iter().map(|v| v.check.as_str()).collect::<Vec<_>>() }));
    findings.push(json!({"stage": name, "findings": result}));
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    START.set(Instant::now()).unwrap();
    log::set_logger(&TRACE).unwrap();
    log::set_max_level(log::LevelFilter::Trace);
    let args = std::env::args().collect::<Vec<_>>();
    assert_eq!(args.len(), 4, "probe BOARD MODE FINDINGS_JSON");
    assert!(matches!(args[2].as_str(), "groups" | "cold-neck"));
    let board = stage("load", || load_kicad_pcb(Path::new(&args[1])))?;
    let mut kinds = BTreeMap::<String, usize>::new();
    for feature in &board.copper {
        *kinds.entry(format!("{}:{:?}", feature.layer, feature.kind)).or_default() += 1;
    }
    println!("{}", json!({"source": board.source, "copper_features": board.copper.len(),
        "drills": board.drills.len(), "feature_kinds": kinds}));
    let selected = if args[2] == "cold-neck" { vec!["F.Cu".to_owned(), "B.Cu".to_owned()] } else { vec![] };
    let layers = stage("copper-layers", || board.copper_layers(&selected))?;
    println!("{}", json!({"layers": layers.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(), "selected": selected}));
    let min_area: Scalar = "1.0e-9".parse()?;
    let min_width: Scalar = "0.0762".parse()?;
    let mut findings = Vec::new();
    if args[2] == "groups" {
        for (name, region) in &layers {
            check(&format!("{name}:layer-sanity"), || checks::layer_sanity(name, region, None), &mut findings);
            check(&format!("{name}:tiny-feature"), || checks::tiny_layer_feature_readiness(name, region, &min_area), &mut findings);
            check(&format!("{name}:skinny-feature"), || checks::skinny_layer_feature_readiness(name, region, &min_width, &min_area), &mut findings);
            check(&format!("{name}:duplicate-island"), || checks::duplicate_layer_island_readiness(name, region, &min_area), &mut findings);
        }
        check("duplicate-layers", || checks::duplicate_layer_geometry_readiness(&layers, &min_area), &mut findings);
    }
    for (name, region) in &layers {
        check(&format!("{name}:min-copper-neck"), || checks::min_copper_neck_width(name, region, &min_width, &min_area), &mut findings);
    }
    std::fs::write(&args[3], serde_json::to_vec_pretty(&json!({
        "source": board.source, "mode": args[2], "findings": findings,
        "scope": "Diagnostic local receipts do not certify source intent or report heuristics."
    }))?)?;
    println!("{}", json!({"complete": true, "process_body_seconds": START.get().unwrap().elapsed().as_secs_f64()}));
    Ok(())
}
