"""Compare the index with an exhaustive interval-overlap oracle."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-range-matrix'
OUT.mkdir()
WORK = ROOT / 'target/contact-index-range'
committed = subprocess.check_output(['git', 'show', 'HEAD:crates/hypercurve/src/curve_region_boolean.rs'], cwd=ROOT, text=True)
start = committed.index('#[derive(Default)]\nstruct ContactPointIndex {')
baseline = committed[start:committed.index('\nfn replace_topology_vertex(', start)]
width_before = 'self.max_width = self.max_width.max(bounds[1] - bounds[0]);'
width_after = 'self.max_width = self.max_width.max((bounds[1] - bounds[0]).next_up());'
assert baseline.count(width_before) == 1
width_only = baseline.replace(width_before, width_after)
candidate = width_only.replace('let bits = value.to_bits();', 'let bits = if value == 0.0 { 0 } else { value.to_bits() };')
adapter = '''
struct CurvePoint2 { bounds: [f64; 4] }
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    assert!(point.bounds.into_iter().all(f64::is_finite));
    assert!(point.bounds[0] <= point.bounds[1] && point.bounds[2] <= point.bounds[3]);
    Some(point.bounds)
}
'''
main = r'''
fn main() {
    let mut values = vec![-f64::MAX, -1.0, -0.0, 0.0, 1.0, f64::MAX];
    for exponent in [-1022, -1000, -600, -54, -53, 0, 53, 600, 1000, 1023] {
        let power = f64::from_bits(((1023+exponent) as u64) << 52);
        for value in [power.next_down(), power, power.next_up()] {
            values.extend([value, -value]);
        }
    }
    for bits in [1, 2, 3, (1_u64 << 52)-1] {
        let value = f64::from_bits(bits);
        values.extend([value, -value]);
    }
    values.sort_by(f64::total_cmp);
    values.dedup_by(|a,b| a.to_bits() == b.to_bits());
    let mut intervals = Vec::new();
    let mut index = ContactPointIndex::default();
    for &low in &values {
        for &high in &values {
            if low <= high {
                let bounds = [low, high, 0.0, 0.0];
                let i = intervals.len();
                index.push(i, Some(&CurvePoint2 { bounds }), i);
                intervals.push(bounds);
            }
        }
    }
    let unboxed = intervals.len();
    index.push(unboxed, None, unboxed);
    index.merge_vertex(unboxed, 0);
    assert_eq!(index.incidences(0), &[0, unboxed]);
    assert!(index.candidates(None).is_none());
    let mut cases = 0_u64;
    let mut omissions = 0_u64;
    let mut extras = 0_u64;
    // A saturated-width shared index checks upper keys and full overlap filtering.
    for &bounds in &intervals {
        let candidates = index.candidates(Some(&CurvePoint2 { bounds })).unwrap();
        assert!(candidates.contains(&unboxed));
        assert!(candidates.windows(2).all(|w| w[0] < w[1]));
        for (i, stored) in intervals.iter().enumerate() {
            let expected = stored[0] <= bounds[1] && stored[1] >= bounds[0];
            let actual = candidates.binary_search(&i).is_ok();
            cases += 1;
            omissions += u64::from(expected && !actual);
            extras += u64::from(!expected && actual);
        }
    }
    // Each isolated index exercises its own finite or overflowing rounded width.
    let mut isolated_omissions = 0_u64;
    for &stored in &intervals {
        let mut one = ContactPointIndex::default();
        one.push(0, Some(&CurvePoint2 { bounds: stored }), 0);
        for &query in &values {
            let bounds = [query, query, 0.0, 0.0];
            let expected = stored[0] <= query && stored[1] >= query;
            let actual = one.candidates(Some(&CurvePoint2 { bounds })).unwrap().contains(&0);
            cases += 1;
            isolated_omissions += u64::from(expected && !actual);
            extras += u64::from(!expected && actual);
        }
    }
    println!("{{\"values\":{},\"intervals\":{},\"overlap_comparisons\":{cases},\"shared_omissions\":{omissions},\"isolated_omissions\":{isolated_omissions},\"extras\":{extras}}}",values.len(),intervals.len());
    assert_eq!((omissions, isolated_omissions, extras), (0,0,0));
}
'''
commands = []
for version, body in [('baseline', baseline), ('width-only', width_only), ('candidate', candidate)]:
    source = OUT / (version + '.rs')
    source.write_text(adapter + body + main)
    binary = WORK / ('range-matrix-' + version)
    for phase, command in [('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(source), '-o', str(binary)]), ('run', ['timeout', '120', str(binary)])]:
        name = version + '-' + phase
        start_time = time.monotonic()
        with (OUT / (name + '.log')).open('w') as log, (OUT / (name + '.jsonl')).open('w') as rows:
            result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
        commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic()-start_time})
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        if phase == 'build': assert result.returncode == 0
    print(version, result.returncode, flush=True)
record = {'scope':'Exhaustive direct interval-overlap oracle for finite endpoint corpus, with explicit adapters supplying valid boxes. Not public Boolean coverage or a benchmark.', 'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(), 'baseline_implementation_sha256':hashlib.sha256(baseline.encode()).hexdigest(), 'changes': {'width-only': 'Round every stored width upward by one binary64 step.', 'candidate': 'Also canonicalize signed zeros in order keys, consistent with mathematical endpoint comparisons.'}, 'commands':commands, 'main_runtime_unchanged':True, 'compiler':subprocess.check_output(['rustc','-Vv'],text=True), 'source_hashes':{path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in OUT.glob('*.rs')}}
(OUT / 'manifest.json').write_text(json.dumps(record, indent=2)+'\n')
(OUT / 'reproduce.py').write_bytes(Path(__file__).read_bytes())
