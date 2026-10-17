"""Require load failures without endorsing the legacy stroke's unresolved fidelity."""
import json,subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;P=W/'prototype-v1'
cases=json.loads((P/'cases.json').read_text())
s='''//! Malformed track source cannot be published as a successful board.
use hypercurve::RegionPointLocation;
use hyperdrc::kicad::{BoardModel, load_kicad_pcb};
use hyperdrc::Scalar;
use std::sync::atomic::{AtomicU64, Ordering};

fn board(fields: &str) -> anyhow::Result<BoardModel> {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!("hyperdrc-track-failure-{}-{}.kicad_pcb",
        std::process::id(), NEXT.fetch_add(1,Ordering::Relaxed)));
    std::fs::write(&path,format!("(kicad_pcb (net 1 signal) (segment (layer F.Cu) (net 1) {fields}))")).unwrap();
    let result=load_kicad_pcb(&path);
    std::fs::remove_file(path).unwrap();
    result
}

fn invalid_source(fields: &str) {
    for policy in [hyperlimit::PredicatePolicy::STRICT,
        hyperlimit::PredicatePolicy::APPROXIMATE_512,hyperlimit::PredicatePolicy::STRICT] {
        let result=hypercurve::evaluate_under(policy,||board(fields)).into_unverified();
        assert!(result.is_err(),"invalid source published a board: {fields}");
    }
}

macro_rules! malformed {
    ($name:ident,$fields:literal) => {
        #[test]
        fn $name() { invalid_source($fields); }
    };
}
'''
for c in cases:
    if c['expected']=='invalid_input':s+=f'malformed!({c["name"]}, {json.dumps(c["fields"])});\n'
s+='''
#[test]
fn valid_source_controls_preserve_metadata_and_known_membership() {
    let model=board("(start 10 20) (end 12 20) (width 2)").unwrap();
    assert_eq!(model.copper.len(),1);
    let feature=&model.copper[0];
    assert_eq!(feature.layer,"F.Cu");
    assert_eq!(feature.net.as_deref(),Some("signal"));
    assert_eq!(feature.location,[Scalar::from(11),Scalar::from(20)]);
    // Both the finite legacy body and the intended round stroke contain this
    // central point and exclude this distant point. This is not full fidelity.
    for region in [feature.region.clone(),model.all_copper().unwrap(),
        model.copper_layers(&[]).unwrap().pop().unwrap().1] {
        assert_eq!(region.classify_xy(11.into(),20.into()),Ok(RegionPointLocation::Inside));
        assert_eq!(region.classify_xy(1000.into(),1000.into()),Ok(RegionPointLocation::Outside));
        assert!(region.try_into_native().is_ok());
    }
}

#[test]
fn validated_zero_width_is_empty_even_when_coordinates_exceed_binary64() {
    for fields in ["(start 10 20) (end 12 20) (width 0)",
        "(start 1e400 0) (end 1e400 2) (width 0)"] {
        let model=board(fields).unwrap();
        assert!(model.copper.is_empty());
        assert_eq!(model.all_copper().unwrap().try_is_empty(),Ok(true));
    }
}

fn deny_refinement() {
    let polynomial=hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36,1368,-11034,31728,-38280,16620].into_iter().map(Scalar::from).collect()).unwrap();
    assert!(matches!(polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
        if blocker.reason()==hypercurve::UncertaintyReason::BudgetExhausted));
}

#[test]
fn exhausted_source_load_cannot_escape_as_a_board_and_later_load_recovers() {
    let mut escaped=None;
    let denied=hypercurve::evaluate_with_budget(hyperlimit::PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,||{
            deny_refinement();
            escaped=Some(board("(start 10 20) (end 12 20) (width 2)"));
        });
    assert!(denied.result.is_err());
    assert!(escaped.unwrap().is_err());
    let retried=board("(start 10 20) (end 12 20) (width 2)").unwrap();
    assert_eq!(retried.copper[0].region.classify_xy(11.into(),20.into()),Ok(RegionPointLocation::Inside));
}
'''
out=P/'public_tests.rs';assert not out.exists();out.write_text(s)
subprocess.run(['rustfmt','--edition','2024',str(out)],check=True)
print('Prepared 35 public source-failure tests; typed-error assertions will supplement this API contract',flush=True)
