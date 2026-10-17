"""Freeze report-state regressions against the closed v3 candidate."""
import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent
O=W/'report-probe-preparation-v1';O.mkdir(exist_ok=False)
S=W/'probe-source-v4';S.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert json.loads((W/'candidate-v3-cancellation-v1/result.json').read_text())['cargo_target_closed']
original=W/'probe-source-v3/kicad_native_aggregation.rs'
added='''
#[test]
fn authored_report_decomposition_is_separate_from_exact_union_topology() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 0.0, 2.0);
    let mut expected = a.try_to_multipolygon().unwrap().0;
    expected.extend(b.try_to_multipolygon().unwrap().0);
    let board = model(vec![feature("F.Cu", a), feature("F.Cu", b)]);
    for region in profiles(&board) {
        // Input rectangles overlap in a 1 by 2 strip: exact union area is 6.
        same(&region.try_area().unwrap(), &Scalar::from(6));
        at(&region, "1.5", "1", Inside);
        let report = region.try_to_multipolygon().unwrap();
        assert_eq!(report.0.len(), 2, "authored reports are not canonical components");
        assert_eq!(report.0, expected);
        let native = region.try_into_native().unwrap();
        assert_eq!(native.material_components().unwrap().len(), 1);
    }
}

#[test]
fn failed_report_prefix_is_not_cached_and_native_composition_recovers() {
    let mut board = source("(start 0 0) (end 2 0) (width 1e-400)");
    board.copper.insert(0, feature("F.Cu", square(4.0, 0.0, 1.0)));
    for region in profiles(&board) {
        let radius = s("5e-401");
        let area = Scalar::one() + s("2e-400") + Scalar::pi() * &radius * &radius;
        for _ in 0..2 {
            assert!(region.try_to_multipolygon().is_err(), "no successful finite prefix may escape");
            same(&region.try_area().unwrap(), &area);
            at(&region, "1", "0", Inside);
            at(&region, "4.5", "0.5", Inside);
        }
    }
    board.copper[1].region.replace_native(square(0.0, 0.0, 1.0).try_into_native().unwrap());
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(2));
        assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
    }
}

#[test]
fn report_sources_are_owned_scope_checked_and_isolated_from_mutation() {
    let mut board = model(vec![feature("F.Cu", square(0.0, 0.0, 1.0)), feature("F.Cu", square(3.0, 0.0, 1.0))]);
    let mut region = board.all_copper().unwrap();
    board.copper.clear();
    let original = region.clone();
    let threads = (0..2).map(|_| {
        let owned = region.clone();
        std::thread::spawn(move || {
            let mut view = owned.try_to_multipolygon().unwrap();
            assert_eq!(view.0.len(), 2);
            view.0.clear();
            assert_eq!(owned.try_to_multipolygon().unwrap().0.len(), 2);
            same(&owned.try_area().unwrap(), &Scalar::from(2));
        })
    }).collect::<Vec<_>>();
    for thread in threads { thread.join().unwrap(); }
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512, PredicatePolicy::STRICT] {
        assert!(hypercurve::evaluate_under(policy, || {
            assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
            same(&region.try_area().unwrap(), &Scalar::from(2));
        }).certified().is_some());
    }
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(PredicatePolicy::STRICT, hypercurve::WorkBudget::ZERO, || {
        let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect()).unwrap();
        assert!(polynomial.isolate_unit_interval_roots().is_err());
        escaped = Some(region.try_to_multipolygon());
    });
    assert!(denied.result.is_err());
    assert!(escaped.unwrap().unwrap_err().detail.contains("budget"));
    assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
    region.replace_native(CurveRegion2::empty());
    assert!(region.try_to_multipolygon().unwrap().0.is_empty());
    same(&region.try_area().unwrap(), &Scalar::zero());
    assert_eq!(original.try_to_multipolygon().unwrap().0.len(), 2);
    same(&original.try_area().unwrap(), &Scalar::from(2));
}
'''
test=S/'kicad_native_aggregation.rs';test.write_text(original.read_text()+added)
runner=(W/'aggregation_reproduce_v2.py').read_text()
runner=runner.replace("O = W / 'aggregation-reproduction-v2'","O = W / 'aggregation-reproduction-v3'")
runner=runner.replace("S = W / 'candidate-source-v1'","S = W / 'candidate-source-v3'")
runner=runner.replace("TEST = W / 'probe-source-v3/kicad_native_aggregation.rs'","TEST = W / 'probe-source-v4/kicad_native_aggregation.rs'")
runner=runner.replace("assert read(W / 'candidate-validation-v1/result.json')['unexpected_exits'] == ['source-failures']",
    "assert read(W / 'candidate-v3-cancellation-v1/result.json')['cargo_target_closed']")
runner=runner.replace('candidate-preparation-v1/','candidate-preparation-v3/')
runner=runner.replace('candidate-validation-v1/native-build.stdout','candidate-validation-v3/native-build.stdout')
runner=runner.replace('candidate-validation-v1/commands.json','candidate-validation-v3/commands.json')
runner=runner.replace('candidate-validation-v1/result.json','candidate-v3-cancellation-v1/result.json')
runner=runner.replace("W / 'aggregation-provenance-contract-v1.md', build,","W / 'aggregation-provenance-contract-v1.md', W / 'report-source-contract-v1.md', build,")
with (W/'aggregation_reproduce_v3.py').open('x') as f:f.write(runner)
(O/'result.json').write_text(json.dumps({'complete':True,'runner_sha256':sha(__file__),
    'original_test_sha256':sha(original),'new_test_sha256':sha(test),'original_tests_preserved_as_exact_prefix':True,
    'diagnostic_runner_sha256':sha(W/'aggregation_reproduce_v3.py')},indent=2)+'\n')
