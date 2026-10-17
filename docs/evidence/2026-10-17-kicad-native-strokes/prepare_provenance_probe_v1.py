"""Add a public source-history regression without modifying earlier probes."""
import hashlib, json
from pathlib import Path
W=Path(__file__).resolve().parent
O=W/'provenance-probe-preparation-v1'; O.mkdir(exist_ok=False)
S=W/'probe-source-v3'; S.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
original=W/'probe-source-v2/kicad_native_aggregation.rs'
added='''
#[test]
fn aggregation_preserves_source_nonfinite_history_after_exact_intersection() {
    let circle = polygon_to_profile(
        hyperdrc::geometry::circle_polygon([1e308, 0.0], 1e308, 16), None);
    let clipped = circle.try_intersection(&square(1.0, 0.0, 1.0)).unwrap();
    same(&clipped.try_area().unwrap(), &Scalar::one());
    let diagnosed = |region: &PcbRegion| {
        hyperdrc::checks::layer::layer_sanity("F.Cu", region, None).iter().any(|v|
            v.message.as_deref().is_some_and(|s| s.contains("non-finite coordinates")))
    };
    assert!(diagnosed(&clipped), "the source-history diagnostic is a precondition");
    let board = model(vec![feature("F.Cu", clipped)]);
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::one());
        assert!(diagnosed(&region), "aggregation must retain source history");
    }
}
'''
test=S/'kicad_native_aggregation.rs'
test.write_text(original.read_text()+added)
runner=(W/'aggregation_reproduce_v1.py').read_text()
runner=runner.replace("O = W / 'aggregation-reproduction-v1'","O = W / 'aggregation-reproduction-v2'")
runner=runner.replace("TEST = W / 'probe-source-v2/kicad_native_aggregation.rs'","TEST = W / 'probe-source-v3/kicad_native_aggregation.rs'")
runner=runner.replace("W / 'aggregation-contract-v1.md', build,","W / 'aggregation-contract-v1.md', W / 'aggregation-provenance-contract-v1.md', build,")
with (W/'aggregation_reproduce_v2.py').open('x') as f:f.write(runner)
(O/'result.json').write_text(json.dumps({'complete':True,'runner_sha256':sha(__file__),
    'original_test_sha256':sha(original),'new_test_sha256':sha(test),
    'original_tests_preserved_as_exact_prefix':True,'diagnostic_runner_sha256':sha(W/'aggregation_reproduce_v2.py')},indent=2)+'\n')
