"""Connect the unmodified index to actual public selected-point producers."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-producer-baseline'
OUT.mkdir()
WORK = ROOT / 'target/contact-index-range'
source_name = 'crates/hypercurve/src/curve_region_boolean.rs'
source = subprocess.check_output(['git', 'show', 'HEAD:'+source_name], cwd=ROOT, text=True)
start = source.index('#[derive(Default)]\nstruct ContactPointIndex {')
body = source[start:source.index('\nfn replace_topology_vertex(', start)]
assert body in (ROOT / source_name).read_text()
adapter = r'''
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
    Curve2, CurveGeometry2, CurveParameterRange2, CurvePoint2, Point2, PredicatePolicy,
    RationalBezier2, Real, evaluate_under,
};
// The public STRICT bounds wrapper delegates to the production private
// bounds_with_policy path. The scalar enclosure helper is unchanged.
fn certified_f64_enclosure(value: &Real) -> Option<[f64; 2]> {
    if let Some(exact) = value.exact_rational_ref() { return exact.to_f64_enclosure(); }
    let [lower, upper] = value.certified_rational_interval(-64)?;
    Some([lower.to_f64_enclosure()?[0], upper.to_f64_enclosure()?[1]])
}
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    if let Some(point) = point.coordinates() {
        let [x_low, x_high] = certified_f64_enclosure(point.x())?;
        let [y_low, y_high] = certified_f64_enclosure(point.y())?;
        return Some([x_low, x_high, y_low, y_high]);
    }
    let bounds = evaluate_under(PredicatePolicy::STRICT, || point.bounds()).certified()?.ok()?;
    Some([
        certified_f64_enclosure(bounds.min().x())?[0],
        certified_f64_enclosure(bounds.max().x())?[1],
        certified_f64_enclosure(bounds.min().y())?[0],
        certified_f64_enclosure(bounds.max().y())?[1],
    ])
}
'''
main = r'''
fn selected_point(lower: Real, polynomial: &BezierParameterPolynomial) -> CurvePoint2 {
    let interval = BezierParameterInterval::try_new(lower, Real::one()).unwrap();
    let parameter = BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval).unwrap();
    let line = RationalBezier2::try_new(vec![Point2::from_values(0, 0), Point2::from_values(1, 0)], vec![Real::one(); 2]).unwrap();
    let range = CurveParameterRange2::try_new(parameter.into(), Real::from(2).into()).unwrap();
    Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range).unwrap().start()
}
fn main() {
    evaluate_under(PredicatePolicy::STRICT, || {
        let ulp = (Real::one() / Real::from(1_u64 << 52)).unwrap();
        let delta = (&ulp / Real::from(4)).unwrap();
        // p(t)=t^5+t-(2-2^-54) is strictly increasing on R. Its unique
        // root is strictly between 1-2^-53 and 1, and both boxes enclose it.
        let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![delta-Real::from(2), Real::one(), Real::zero(), Real::zero(), Real::zero(), Real::one()]).unwrap();
        let wide = selected_point(-Real::one()-&ulp, &polynomial);
        let narrow = selected_point(Real::one()-(&ulp/Real::from(2)).unwrap(), &polynomial);
        let wide_box = exact_contact_point_box(&wide).unwrap();
        let narrow_box = exact_contact_point_box(&narrow).unwrap();
        let equal = wide.coincides_with(&narrow).unwrap();
        println!("{{\"wide_box\":{wide_box:?},\"narrow_box\":{narrow_box:?},\"same_point_certified\":{equal}}}");
        assert!(equal);
        let mut index = ContactPointIndex::default();
        index.push(0, Some(&wide), 0);
        index.push(1, None, 1);
        index.merge_vertex(1, 0);
        assert_eq!(index.incidences(0), &[0, 1]);
        let candidates = index.candidates(Some(&narrow)).unwrap();
        println!("{{\"candidates\":{candidates:?},\"stored_width_bits\":{},\"search_low_bits\":{},\"same_point_retained\":{}}}", index.max_width.to_bits(), (narrow_box[0]-index.max_width).to_bits(), candidates.contains(&0));
        assert!(candidates.contains(&0), "a certified identical public selected point must remain a topology candidate");
    }).certified().unwrap();
}
'''
(OUT / 'probe.rs').write_text(adapter+body+main)
library = json.loads((EVIDENCE / 'ray-filter-production-cost/manifest.json').read_text())['libraries']['candidate']
lib = Path(library['path'])
assert hashlib.sha256(lib.read_bytes()).hexdigest() == library['sha256']
commands = []
for label, command in [('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT/'probe.rs'), '-L', 'dependency='+str(lib.parent), '-L', 'dependency='+str(lib.parent/'deps'), '--extern', 'hypercurve='+str(lib), '-o', str(WORK/'producer-baseline')]), ('run', ['timeout', '120', str(WORK/'producer-baseline')])]:
    start_time = time.monotonic()
    with (OUT/(label+'.log')).open('w') as log, (OUT/(label+'.jsonl')).open('w') as rows:
        result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
    commands.append({'name':label, 'command':command, 'exit_code':result.returncode, 'wall_seconds':time.monotonic()-start_time})
    (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    if label=='build': assert result.returncode==0
assert hashlib.sha256(lib.read_bytes()).hexdigest() == library['sha256']
record = {'scope':'Actual public selected-point producers and certified equality feed the verbatim private contact index. Only its private bounds wrapper is replaced by the public STRICT wrapper of the same production method. This does not yet execute a complete public Boolean operation.', 'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(), 'implementation_sha256':hashlib.sha256(body.encode()).hexdigest(), 'library':library, 'probe_sha256':hashlib.sha256((OUT/'probe.rs').read_bytes()).hexdigest(), 'commands':commands, 'main_runtime_unchanged':True}
(OUT/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT/'reproduce.py').write_bytes(Path(__file__).read_bytes())
print('Actual producer/index probe exit:',result.returncode,flush=True)
