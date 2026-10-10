use hyperdrc::{Scalar, kicad::BoardModel};
use std::sync::{Arc, Mutex, atomic::AtomicBool};

struct Logger(Mutex<Vec<String>>);
impl log::Log for Logger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }
    fn log(&self, record: &log::Record<'_>) {
        self.0.lock().unwrap().push(format!("{}", record.args()));
    }
    fn flush(&self) {}
}
static LOGGER: Logger = Logger(Mutex::new(Vec::new()));

fn unresolved_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

#[test]
fn tracing_preserves_readiness_results_and_exposes_unavailable_values() {
    log::set_logger(&LOGGER).unwrap();
    let board = BoardModel {
        source: "diagnostic-control".to_owned(),
        copper: Vec::new(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let layers = ["F.Cu".to_owned()];
    let ordinary: Scalar = "0.15".parse().unwrap();
    let mut aborted = Scalar::pi();
    aborted.abort(Arc::new(AtomicBool::new(true)));
    for (value, expected) in [
        (unresolved_zero(), "[UnknownZero]"),
        (aborted, "[Exhausted]"),
        ("1e-40".parse().unwrap(), "1.000000"),
    ] {
        for prepared in [false, true] {
            if prepared {
                let _ = value.certified_rational_interval(-512);
            }
            for operation in 0..4 {
                for level in [log::LevelFilter::Off, log::LevelFilter::Trace] {
                    LOGGER.0.lock().unwrap().clear();
                    log::set_max_level(level);
                    let findings = match operation {
                        0 => hyperdrc::checks::switch_node_keepout_readiness(
                            &board,
                            &layers,
                            value.clone(),
                            &ordinary,
                        ),
                        1 => hyperdrc::checks::inductor_copper_keepout_readiness(
                            &board,
                            &layers,
                            value.clone(),
                            &ordinary,
                        ),
                        2 => hyperdrc::checks::via_in_pad_readiness(&board, &layers, &value),
                        _ => hyperdrc::checks::component_spacing_readiness(
                            &board, &layers, &value, &ordinary,
                        ),
                    };
                    assert!(findings.is_empty(), "{findings:?}");
                    let logs = LOGGER.0.lock().unwrap();
                    if level == log::LevelFilter::Off {
                        assert!(logs.is_empty());
                    } else {
                        assert!(logs.iter().any(|line| line.contains(expected)), "{logs:?}");
                    }
                }
            }
        }
    }
    log::set_max_level(log::LevelFilter::Off);
}
