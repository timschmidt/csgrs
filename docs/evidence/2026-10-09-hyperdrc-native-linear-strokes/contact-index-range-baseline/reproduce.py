"""Exercise the verbatim contact index with valid conservative point boxes."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-range-baseline'
OUT.mkdir()
WORK = ROOT / 'target/contact-index-range'
WORK.mkdir(exist_ok=True)
name = 'crates/hypercurve/src/curve_region_boolean.rs'
source = subprocess.check_output(['git', 'show', 'HEAD:'+name], cwd=ROOT, text=True)
start = source.index('#[derive(Default)]\nstruct ContactPointIndex {')
body = source[start:source.index('\nfn replace_topology_vertex(', start)]
assert body in (ROOT / name).read_text()
adapter = r'''
// Adapter supplies valid conservative enclosures for known exact binary64
// rational points. The complete index implementation below is unmodified.
struct CurvePoint2 { exact: [f64; 2], bounds: [f64; 4] }
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    let [lx, hx, ly, hy] = point.bounds;
    assert!(point.bounds.into_iter().chain(point.exact).all(f64::is_finite));
    assert!(lx <= point.exact[0] && point.exact[0] <= hx);
    assert!(ly <= point.exact[1] && point.exact[1] <= hy);
    Some(point.bounds)
}
'''
main = r'''
fn main() {
    let mut failures = 0;
    let mut cases = 0;
    let mut check = |name: String, low: f64, high: f64, query: f64, overlaps: bool| {
        let stored = CurvePoint2 { exact: [high, 0.0], bounds: [low, high, 0.0, 0.0] };
        let query = CurvePoint2 { exact: [query, 0.0], bounds: [query, query, 0.0, 0.0] };
        let mut index = ContactPointIndex::default();
        index.push(0, Some(&stored), 0);
        index.push(1, None, 1);
        index.merge_vertex(1, 0);
        assert_eq!(index.incidences(0), &[0, 1]);
        let candidates = index.candidates(Some(&query)).unwrap();
        assert!(candidates.contains(&1), "unboxed contacts remain candidates");
        let retained = candidates.contains(&0);
        assert_eq!(overlaps, low <= query.exact[0] && query.exact[0] <= high);
        if overlaps { assert_eq!(stored.exact, query.exact); }
        failures += usize::from(retained != overlaps);
        cases += 1;
        println!("{{\"case\":\"{name}\",\"box_low_bits\":{},\"box_high_bits\":{},\"query_bits\":{},\"stored_width_bits\":{},\"search_low_bits\":{},\"expected_overlap\":{overlaps},\"retained\":{retained},\"candidates\":{candidates:?}}}", low.to_bits(), high.to_bits(), query.exact[0].to_bits(), index.max_width.to_bits(), (query.exact[0]-index.max_width).to_bits());
    };
    let fraction = f64::from_bits((1023_u64-54) << 52);
    for exponent in [-1000, -600, -53, 0, 53, 600, 1000] {
        let scale = f64::from_bits(((1023+exponent) as u64) << 52);
        check(format!("cross_zero_width_{exponent}"), -scale*fraction, scale, scale, true);
    }
    check("exact_point_control".into(), 1.0, 1.0, 1.0, true);
    check("wide_control".into(), -1.0, 1.0, 1.0, true);
    check("negative_endpoint_control".into(), -2.0, -1.0, -1.0, true);
    check("strictly_disjoint_control".into(), -fraction, 1.0, 1.0_f64.next_up(), false);
    assert_eq!(failures, 0, "{failures} of {cases} exact contact-box witnesses were contradicted");
}
'''
(OUT / 'probe.rs').write_text(adapter+body+main)
commands = []
for label, command in [('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-o', str(WORK / 'baseline')]), ('run', [str(WORK / 'baseline')])]:
    start_time = time.monotonic()
    with (OUT / (label+'.log')).open('w') as log, (OUT / (label+'.jsonl')).open('w') as rows:
        result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
    commands.append({'name': label, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic()-start_time})
    if label == 'build': assert result.returncode == 0
rows = [json.loads(line) for line in (OUT / 'run.jsonl').read_text().splitlines()]
failures = [row for row in rows if row['expected_overlap'] != row['retained']]
assert len(rows) == 11 and len(failures) == 7 and commands[-1]['exit_code'] == 101
record = {
    'scope': 'The complete committed private index is extracted verbatim. An explicit adapter supplies valid conservative finite enclosures for exact dyadic points, and checks containment before the index uses them. This isolates the index contract; it is not yet a production point-box producer or public Boolean counterexample.',
    'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    'source_file': name, 'implementation_sha256': hashlib.sha256(body.encode()).hexdigest(),
    'probe_sha256': hashlib.sha256((OUT / 'probe.rs').read_bytes()).hexdigest(),
    'compiler': subprocess.check_output(['rustc', '-Vv'],text=True),
    'commands': commands, 'cases': len(rows), 'false_candidate_omissions': len(failures),
    'main_runtime_unchanged': True,
    'next_step': 'Trace actual certified point-box producers and establish their range contract before choosing or admitting a rounding repair.',
}
(OUT / 'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT / 'reproduce.py').write_bytes(Path(__file__).read_bytes())
print(f'Verbatim contact index omits {len(failures)} of {len(rows)} independent valid-box cases; public reachability remains unverified.')
