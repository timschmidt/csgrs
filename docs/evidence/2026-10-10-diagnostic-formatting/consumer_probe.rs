//! Reproduce formatting-dependent readiness completion and aggregate display.
use hyperdrc::Scalar;
use hyperdrc::kicad::BoardModel;
use std::sync::{Arc, atomic::AtomicBool};

struct Logger;
impl log::Log for Logger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }
    fn log(&self, record: &log::Record<'_>) {
        eprintln!("stage: log-start");
        eprintln!("trace: {}", record.args());
        eprintln!("stage: log-end");
    }
    fn flush(&self) {}
}
static LOGGER: Logger = Logger;

fn value(name: &str) -> Scalar {
    match name {
        "pi" => Scalar::pi(),
        "unresolved_zero" => {
            let sine = Scalar::e().sin();
            let cosine = Scalar::e().cos();
            &sine * &sine + &cosine * &cosine - Scalar::one()
        },
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
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(if args[4] == "trace" {
        log::LevelFilter::Trace
    } else {
        log::LevelFilter::Off
    });
    let value = value(&args[2]);
    eprintln!("stage: constructed");
    if args[3] == "prepared" {
        let _ = value.certified_rational_interval(-256);
    }
    eprintln!("stage: invoke");
    let board = BoardModel {
        source: "empty-formatting-control".to_owned(),
        copper: Vec::new(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let layers = ["F.Cu".to_owned()];
    let ordinary: Scalar = "0.15".parse().unwrap();
    let findings = match args[1].as_str() {
        "switch" => {
            hyperdrc::checks::switch_node_keepout_readiness(&board, &layers, value, &ordinary)
        },
        "inductor" => hyperdrc::checks::inductor_copper_keepout_readiness(
            &board, &layers, value, &ordinary,
        ),
        "via" => hyperdrc::checks::via_in_pad_readiness(&board, &layers, &value),
        "component" => {
            hyperdrc::checks::component_spacing_readiness(&board, &layers, &value, &ordinary)
        },
        "complex" => {
            println!("{:#.6}", hyperlattice::Complex::new(value, Scalar::one()));
            Vec::new()
        },
        "vector" => {
            println!("{:#.6}", hyperlattice::Vector2::new([value, Scalar::one()]));
            Vec::new()
        },
        _ => panic!("unknown operation"),
    };
    println!(
        "findings={}; ids={:?}",
        findings.len(),
        findings.iter().map(|x| &x.id).collect::<Vec<_>>()
    );
    eprintln!("stage: complete");
}
