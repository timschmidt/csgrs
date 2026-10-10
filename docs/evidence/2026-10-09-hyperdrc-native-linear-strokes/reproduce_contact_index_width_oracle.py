"""Check a proposed directed width with exact Fraction arithmetic, independently."""
from collections import Counter
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-width-oracle'
OUT.mkdir()
helper = (EVIDENCE/'contact_index_width_upper_bound.rs').read_text()
main = r'''
use std::collections::BTreeMap;
fn add(cases: &mut BTreeMap<(u64,u64), &'static str>, a: f64, b: f64, bucket: &'static str) {
    if !a.is_finite() || !b.is_finite() { return; }
    let (low,high) = if a > b { (b,a) } else { (a,b) };
    cases.entry((low.to_bits(),high.to_bits())).or_insert(bucket);
}
fn main() {
    let mut values = vec![-f64::MAX,-1.0,-0.0,0.0,1.0,f64::MAX];
    for exponent in [-1022,-1000,-600,-54,-53,0,53,600,1000,1023] {
        let power = f64::from_bits(((1023+exponent) as u64)<<52);
        for value in [power.next_down(),power,power.next_up()] { values.extend([value,-value]); }
    }
    for bits in [1,2,3,(1_u64<<52)-1] { let value=f64::from_bits(bits); values.extend([value,-value]); }
    let mut cases = BTreeMap::new();
    for &a in &values { for &b in &values { add(&mut cases,a,b,"critical_grid"); } }
    for exponent in 1..2047_u64 {
        let value=f64::from_bits(exponent<<52);
        for (a,b) in [
            (value.next_down(),value.next_up()),(-value,value),(-value.next_up(),value),
            (-value,value.next_up()),(-value*2_f64.powi(-54),value),(0.0,value),(-value,0.0),
            (-value,-value.next_down()),(value,f64::MAX),(-f64::MAX,value),(value*0.5,value),
            (value,value),(-value,-value),
        ] { add(&mut cases,a,b,"exponent_edges"); }
    }
    let half=f64::MAX*0.5;
    for (a,b) in [(-half,half),(-half.next_down(),half),(-half.next_up(),half),(0.0,f64::MAX),(-f64::MAX,0.0)] {
        add(&mut cases,a,b,"max_width_edges");
    }
    let mut state=0x9e3779b97f4a7c15_u64;
    for _ in 0..65536 {
        state=state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let a=f64::from_bits(state);
        state=state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let b=f64::from_bits(state);
        add(&mut cases,a,b,"deterministic_bits");
    }
    for ((low_bits,high_bits),bucket) in cases {
        let low=f64::from_bits(low_bits); let high=f64::from_bits(high_bits);
        let upper=width_upper_bound(low,high);
        println!("{{\"bucket\":\"{bucket}\",\"low_bits\":{low_bits},\"high_bits\":{high_bits},\"upper_bits\":{}}}",upper.to_bits());
    }
}
'''
(OUT/'probe.rs').write_text(helper+main)
binary=ROOT/'target/contact-index-range/width-oracle'
commands=[]
for name,command in [('build',['rustc','--edition=2024','-O','-D','warnings',str(OUT/'probe.rs'),'-o',str(binary)]),('run',['timeout','120',str(binary)])]:
    start=time.monotonic()
    with (OUT/(name+'.log')).open('w') as log,(OUT/(name+'.jsonl')).open('w') as rows:
        result=subprocess.run(command,cwd=ROOT,stdout=rows,stderr=log,check=False)
    commands.append({'name':name,'command':command,'exit_code':result.returncode,'wall_seconds':time.monotonic()-start})
    (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    assert result.returncode==0

decode=lambda bits:struct.unpack('>d',struct.pack('>Q',bits))[0]
maximum=Fraction.from_float(decode(0x7fefffffffffffff))
raw=(OUT/'run.jsonl').read_bytes()
failures=[]
buckets=Counter()
exponents=set()
infinite=0
for number,line in enumerate(raw.splitlines(),1):
    row=json.loads(line)
    low,high,upper=[decode(row[key]) for key in ['low_bits','high_bits','upper_bits']]
    assert math.isfinite(low) and math.isfinite(high) and low<=high
    exact=Fraction.from_float(high)-Fraction.from_float(low)
    if math.isinf(upper):
        safe=upper>0
        tight=exact>maximum
        infinite+=1
    elif math.isnan(upper) or upper<0:
        safe=tight=False
    else:
        safe=Fraction.from_float(upper)>=exact
        tight=(upper==0 and exact==0) or (upper>0 and Fraction.from_float(math.nextafter(upper,-math.inf))<exact)
    if not (safe and tight):failures.append(row|{'row':number,'safe':safe,'tight':tight})
    buckets[row['bucket']]+=1
    exponents.update((row[key]>>52)&2047 for key in ['low_bits','high_bits'])
assert exponents==set(range(2047))
(OUT/'failures.json').write_text(json.dumps(failures,indent=2)+'\n')
compressed=gzip.compress(raw,mtime=0)
assert gzip.decompress(compressed)==raw
(OUT/'run.jsonl.gz').write_bytes(compressed)
(OUT/'run.jsonl').unlink()
record={
    'scope':'A proposed private floating width helper only; no production source change. Rust optimized output is checked independently using exact Python Fraction arithmetic on the bit-exact input endpoints.',
    'commands':commands,
    'cases':sum(buckets.values()),'buckets':dict(buckets),'encoded_endpoint_exponents':len(exponents),
    'correct_infinite_upper_bounds':infinite,'failures':len(failures),
    'oracle':'Every finite output must enclose the exact rational difference, and its immediately preceding binary64 value must not. Infinity is permitted only when the exact width exceeds MAX. Zero is checked exactly.',
    'helper_sha256':hashlib.sha256(helper.encode()).hexdigest(),
    'probe_sha256':hashlib.sha256((OUT/'probe.rs').read_bytes()).hexdigest(),
    'raw_rows_sha256':hashlib.sha256(raw).hexdigest(),'compressed_rows_sha256':hashlib.sha256(compressed).hexdigest(),
    'raw_rows_bytes':len(raw),'compressed_rows_bytes':len(compressed),
    'primary_reference':'https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf, Two-Diff following Theorem 7.',
    'range_argument':'The helper returns overflowed positive widths directly and declines any tighter certificate with a nonfinite residual. For finite intermediates, gradual underflow is exact for additions/subtractions of binary64 operands: every operand is an integer multiple of the least subnormal, and any exact result in the subnormal range is representable. Thus no subnormal rounding error is hidden by a zero residual.',
    'limitations':'This deterministic corpus and algebraic argument are not exhaustive enumeration of all binary64 pairs or a machine-checked proof. Full contact-index and production validation remain pending.',
}
(OUT/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT/'reproduce.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps({'cases':record['cases'],'buckets':dict(buckets),'exponents':len(exponents),'correct_infinite_upper_bounds':infinite,'failures':len(failures)},indent=2),flush=True)
assert not failures
