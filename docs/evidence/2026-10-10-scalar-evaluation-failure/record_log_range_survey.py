"""Freeze the logarithm reachability survey without claiming a runtime repair."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = lambda p: json.loads(gzip.decompress(p.read_bytes()))

previous = E/'typed-api-prototype-v1576-state.json'
assert sha(previous) == '5e3cbb79eb74375ef87d589961bcb1e7cf01d6c24ce82da79490add7ac21ed20'
old = hashes(E/read(previous)['source_record'])
record = E/'typed-api-prototype-v1580-log-range-retry-build/source-hashes.json.gz'
source = hashes(record)
changed = sorted(k for k in old.keys() | source.keys() if old.get(k) != source.get(k))
assert changed == ['crates/hyperreal/src/computable/node.rs', 'crates/hyperreal/src/computable/node/log_range_probe_tests.rs']
assert (SOURCE/changed[0]).read_text() == (E/'typed-api-prototype-v1573-restored-build/inputs'/changed[0]).read_text() + 'include!("node/log_range_probe_tests.rs");\n'
assert (SOURCE/changed[1]).read_text().startswith('#[cfg(all(test, feature = "serde"))]')
assert all(sha(SOURCE/k) == v for k,v in source.items())
guard = hashes(E/'candidate-preparation/main-hashes.json.gz')
assert all(sha(ROOT/k) == v for k,v in guard.items())
original = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
status = subprocess.check_output(['git','status','--short','--untracked-files=all'],cwd=ROOT,text=True).splitlines()
assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT))+'/')] == original
assert all(s.startswith('?? ') for s in status if s[3:].startswith(str(E.relative_to(ROOT))+'/'))
phases = []
for filename in ['log-range-survey-caps-v1579.json','log-range-retry-caps-v1583.json']:
    for phase,row in read(E/filename)['tests'].items():
        assert row['result']['exit_code'] == row['cap']['exit_code'] == 0
        assert not row['cap']['wall_timeout']
        assert row['result']['source_unchanged'] and row['result']['main_unchanged']
        phases.append(phase)
for phase in ['typed-api-prototype-v1577-log-range-survey-build','typed-api-prototype-v1580-log-range-retry-build']:
    r=read(E/phase/'result.json');assert r['exit_code']==0 and r['source_unchanged'] and r['main_unchanged']
    phases.append(phase)
observations=[];retries=[]
for phase in phases:
    for line in (E/phase/'stderr.log').read_text().splitlines():
        if line.startswith('log range survey '): observations.append(line)
        if line.startswith('log range retry '): retries.append(line)
assert len(observations)==241 and len(retries)==13
low=[r for r in observations if ', units=1000000,' not in r]
high=[r for r in observations if ', units=1000000,' in r]
assert len(low)==228 and all('answer=Err(Exhausted)' in r for r in low)
assert sum('answer=Ok(' in r for r in high)==11
assert sum('answer=Err(Exhausted)' in r for r in high)==2
assert all('limited=Err(Exhausted)' in r and 'sign=Some(Positive)' in r for r in retries)
out=E/'log-range-survey-v1583.json';assert not out.exists()
out.write_text(json.dumps({
    'status':'Measured isolated survey; retained production unchanged; no runtime admission',
    'previous_checkpoint':previous.name,'previous_checkpoint_sha256':sha(previous),
    'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),
    'source_changes':changed,'production_unchanged':True,
    'main_guarded_files':len(guard),'original_worktree_entries_preserved':len(original),
    'phases':phases,'observations':observations,'successful_retries':retries,
    'low_allowance_queries':228,'low_allowance_successes':0,
    'million_unit_queries':13,'million_unit_successes':11,'million_unit_exhaustions':2,
    'unlimited_retries_after_exhaustion':13,'zero_work_cached_queries_after_retry':52,
    'oracle':'Directed MPFR ln(2) and ln(3), converted to exact rationals; enclosure checked against each successful scaled result.',
    'findings':[
        'Positive rational domain recognition pays bounded coefficient-reading costs before consulting a sign. Widths just inside the 4096-bit recognition limit can exhaust earlier than oversized canonical values which skip that recognition.',
        'Stored LnDeferred(2^4097/3) reaches repeated smooth-factor extraction under the public query wrapper: the cold eight-unit observation requests 1068400 Rust bytes before exhaustion.',
        'The public asinh fixture is a rational ln1p child through exponent 516, and an unfactored expression graph at exponents 2050 and 8192. The public Real serialized payload remains a rational ln1p child.',
        'All thirteen exhausted fine queries recover on unlimited retry, satisfy directed enclosures, and retain positive signs through zero-work cached queries.'
    ],
    'limitations':[
        'The shared_setup comment overstates its warmup: ln(4/3) takes direct rational ln1p reduction, so this call does not establish that shared ln(2)/ln(3) caches are warm. The exact executed setup and test order are retained; later queries may warm constants for subsequent cases.',
        'The original 228 allowance cutpoints all exhaust. Their conditional numerical-oracle branches were not executed; numerical accuracy coverage comes from the successful retry tests.',
        'The Real route reads its public serialized Computable payload; it is not a native Real method accepting EvaluationBudget. Stored LnDeferred expressions are public deserialization inputs, distinct from ordinary ln construction.',
        'Allocation windows exclude fixtures, warmup and oracle construction; the Rust allocator omits native allocations, RSS and peak live memory. Single observation timings are probes, not matched benchmarks.',
        'EvaluationBudget excludes individual-operation byte/time bounds and unmetered algebraic helpers. These allocations are not evidence of a budget contract violation.',
        'Production and all prior tests are unchanged. Full property, native-prototype, formal-proof and full-board admission remain open.'
    ],
    'next_step':'Measure a cheap retained/leaf-sign domain certificate ahead of bounded rational recognition, preserving independent child existence and every prior certificate.'
},indent=2)+'\n')
print(json.dumps({'record':out.name,'queries':len(observations),'retries':len(retries),'source_sha256':sha(record)}))
