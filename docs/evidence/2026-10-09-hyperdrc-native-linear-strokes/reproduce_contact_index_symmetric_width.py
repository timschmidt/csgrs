"""Check whether nonzero endpoints still cause unnecessary range saturation."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-symmetric-width'
OUT.mkdir()
cost = json.loads((EVIDENCE / 'contact-index-exact-widths-cost/manifest.json').read_text())
adapter = (EVIDENCE / 'contact-index-producer-baseline-corrected/probe.rs').read_text().split('#[derive(Default)]\nstruct ContactPointIndex {')[0]
main = r'''
fn main() {
    evaluate_under(PredicatePolicy::STRICT, || {
        let ulp = (Real::one()/Real::from(1_u64 << 52)).unwrap();
        let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
            (&ulp/Real::from(4)).unwrap()-Real::from(2), Real::one(), Real::zero(), Real::zero(), Real::zero(), Real::one(),
        ]).unwrap();
        let interval = BezierParameterInterval::try_new(-Real::one(), Real::one()).unwrap();
        let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval).unwrap();
        let largest = Real::try_from(f64::MAX).unwrap();
        let half = (&largest/Real::from(2)).unwrap();
        let line = RationalBezier2::try_new(vec![Point2::from_values(0,0), Point2::new(half.clone(), Real::zero())], vec![Real::one();2]).unwrap();
        let range = CurveParameterRange2::try_new(BezierParameter2::Algebraic(parameter).into(), Real::from(2).into()).unwrap();
        let wide = Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range).unwrap().start();
        let bounds = exact_contact_point_box(&wide).unwrap();
        assert_eq!(bounds, [-f64::MAX*0.5, f64::MAX*0.5, 0.0, 0.0]);
        let mut index = ContactPointIndex::default();
        for i in 0..64_u32 {
            let x = (-&largest * Real::from(65+i)/Real::from(128)).unwrap();
            let point = CurvePoint2::from(Point2::new(x, Real::zero()));
            index.push(usize::try_from(i).unwrap(), Some(&point), usize::try_from(i).unwrap());
        }
        index.push(64, Some(&wide), 64);
        index.merge_vertex(0, 0);
        assert_eq!(index.incidences(0), &[0]);
        let query = CurvePoint2::from(Point2::new(half, Real::zero()));
        let query_bounds = exact_contact_point_box(&query).unwrap();
        let low = ContactPointIndex::key(query_bounds[0]-index.max_width);
        let high = ContactPointIndex::key(query_bounds[1]);
        let visited: usize = index.by_x_low.range(low..=high).map(|(_,entries)| entries.len()).sum();
        let candidates = index.candidates(Some(&query)).unwrap();
        assert_eq!(candidates, vec![64]);
        println!("{{\"width_is_finite\":{},\"width_bits\":{},\"visited_indices\":{visited},\"candidates\":{candidates:?},\"expected_finite_width_bits\":{}}}",index.max_width.is_finite(),index.max_width.to_bits(),f64::MAX.to_bits());
        assert_eq!(index.max_width, f64::MAX, "the exactly representable symmetric width must not disable range pruning");
    }).certified().unwrap();
}
'''
commands = []
probes = {}
for variant, source in [('baseline',ROOT/'target/ray-filter-finite-guard/source'),('candidate',ROOT/'target/contact-index-exact-widths/source')]:
    text = (source/'crates/hypercurve/src/curve_region_boolean.rs').read_text()
    begin = text.index('#[derive(Default)]\nstruct ContactPointIndex {')
    body = text[begin:text.index('\nfn replace_topology_vertex(',begin)]
    probe = OUT / (variant+'.rs')
    probe.write_text(adapter+body+main)
    library = cost['libraries'][variant]
    lib = Path(library['path'])
    assert hashlib.sha256(lib.read_bytes()).hexdigest() == library['sha256']
    binary = ROOT / 'target/contact-index-exact-widths' / ('symmetric-width-'+variant)
    for phase,command in [('build',['rustc','--edition=2024','-O','-D','warnings',str(probe),'-L','dependency='+str(lib.parent),'-L','dependency='+str(lib.parent/'deps'),'--extern','hypercurve='+str(lib),'-o',str(binary)]),('run',['timeout','120',str(binary)])]:
        name = variant+'-'+phase
        start = time.monotonic()
        with (OUT/(name+'.log')).open('w') as log,(OUT/(name+'.jsonl')).open('w') as rows:
            result = subprocess.run(command,cwd=ROOT,stdout=rows,stderr=log,check=False)
        commands.append({'name':name,'command':command,'exit_code':result.returncode,'wall_seconds':time.monotonic()-start})
        (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        if phase=='build':assert result.returncode==0
    probes[variant] = {'library':library,'implementation_sha256':hashlib.sha256(body.encode()).hexdigest(),'probe_sha256':hashlib.sha256(probe.read_bytes()).hexdigest(),'result':json.loads((OUT/(variant+'-run.jsonl')).read_text())}
assert [command['exit_code'] for command in commands] == [0,0,0,101]
assert probes['baseline']['result']['visited_indices'] == 1
assert probes['candidate']['result']['visited_indices'] == 65
record = {'scope':'Actual production point constructors and public STRICT bounds feed the verbatim index. Both variants return the same correct conservative candidate [64], but the refined candidate needlessly visits 64 additional disjoint entries. This is a structural scan-count regression, not a matched timing measurement or a public Boolean wrong answer.', 'independent_width':'MAX/2 - (-MAX/2) is exactly MAX in both binary64 and real arithmetic.', 'commands':commands,'probes':probes,'main_runtime_unchanged':True}
(OUT/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT/'reproduce.py').write_bytes(Path(__file__).read_bytes())
print('Symmetric production box: baseline visits 1 entry; exact-endpoint exception candidate visits 65.',flush=True)
