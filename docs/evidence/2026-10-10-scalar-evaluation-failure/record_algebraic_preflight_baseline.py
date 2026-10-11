#!/usr/bin/env python3
"""Freeze early algebraic allocations before changing their preflight order."""
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


def main():
    out = E/'algebraic-preflight-baseline-v1085.json'
    assert not out.exists()
    previous = read(E/'typed-api-prototype-v1074-state.json')
    for name,value in previous['artifacts_sha256'].items():
        assert sha(E/name) == value,name
    old_record = E/previous['source_record']
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    record = E/'typed-api-prototype-v1075-algebraic-preflight-baseline-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    for name,value in source.items():
        assert sha(SOURCE/name) == value,name
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    assert set(changed) == {prefix+'node.rs',prefix+'node/algebraic_preflight_tests.rs'}
    node = prefix+'node.rs'
    assert (SOURCE/node).read_text().replace('include!("node/algebraic_preflight_tests.rs");\n','') == (old_record.parent/'inputs'/node).read_text()
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    guard = json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name,value in guard.items():assert sha(ROOT/name) == value,name
    expected = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    caps = read(E/'algebraic-preflight-baseline-caps-v1085.json')
    assert len(caps['tests']) == 10 and caps['source_record_sha256'] == sha(record)
    observations = {}
    codes = []
    for phase,row in caps['tests'].items():
        result = read(E/phase/'result.json')
        assert result == row['result'] and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(record)
        cap = row['cap']
        assert cap['address_space_bytes'] == 192*1024*1024
        assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
        assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        codes.append(result['exit_code'])
        observations[row['test']] = [s for s in (E/phase/'stderr.log').read_text().splitlines()
            if s.startswith(('lazy ','empty metadata '))]
    assert codes == [101]*5+[0]*5
    assert [len(v) for v in observations.values()] == [8,12,12,24,5,0,0,0,6,0]
    costs = {}
    for family,p in read(E/'structural-comparison-paired-cost-v1042/summary.json')['provenance'].items():
        binary = Path(p['binaries']['candidate'])
        assert sha(binary) == p['binary_sha256_at_paired_run']['candidate']
        assert binary.stat().st_size == p['executable_bytes']['candidate']
        assert sha(E/p['phases']['candidate']/'summary.json') == p['original_summary_sha256']['candidate']
        costs[family] = {'phase':p['phases']['candidate'],'binary':str(binary),
            'sha256':sha(binary),'bytes':binary.stat().st_size}
    out.write_text(json.dumps({
        'status':'Frozen algebraic lazy-preflight and zero-work vector regressions before runtime repair; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1074-state.json',
        'previous_verification':{'artifacts':len(previous['artifacts_sha256']),'candidate_sources':len(before),
            'main_guarded':len(guard),'original_worktree_entries':len(expected)},
        'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),
        'test_only_changes':changed,'prior_named_test_bodies_and_assertions_unchanged':True,
        'baseline_caps':'algebraic-preflight-baseline-caps-v1085.json',
        'outcomes':{'assertion_failures':5,'passing_guards':5},'observations':observations,
        'findings':[
            'The metadata leaf, dependency-candidate and dependency-base size checks call canonicalizing numerator/denominator accessors before their work debit. Each million-bit fixture requests 2500392 Rust allocation bytes before declining for insufficient work.',
            'The coefficient-work estimator also canonicalizes first. Two million-bit lazy operands request 5000752 Rust allocation bytes while charging one unit; the same premature reduction occurs with generous work.',
            'The metadata parser allocates its initial 16-entry vector before its first node debit. All five zero-work fixtures request one 768-byte allocation before returning Unknown.',
            'Generous exact reductions, cheap canonical/negative/degree rejections, both logarithm-size boundaries, every measured metadata cutpoint, and public completed-zero proof reuse already pass.',
            'Metadata growth already has local guards of 16 generators, 256 parser nodes, field degree 2^20 and 4096 dependency combinations. These limits do not prevent the earlier measured lazy reduction or zero-work allocation.'
        ],
        'allocation_scope':'Opt-in thread-local Rust allocation requests only during the named synchronous helper call. Fixture setup, BudgetScope setup, assertions, printing, native allocations and other threads are excluded. Not RSS or peak-live memory.',
        'coverage':{'test_functions':10,'stored_factor_bits':[65,4097,65537,1000001],
            'dependency_degrees':[2,3,4],'metadata_signs':['positive','negative'],
            'coefficient_work_allowances':[1,18446744073709551615],
            'zero_work_nodes':['One','Int','Ratio','unsupported SinDeferred','eighty Offset nodes'],
            'canonical_logarithm_boundary':'2^131072 and 2^131072+1, numerator/denominator, both signs, with unreduced factors of 65 and 4097 bits.',
            'metadata_cutpoints':'Every allowance through successful work plus two at factor widths 65,256,4097 and depths zero/eighty.',
            'public_retry_schedules':81,'public_factor_bits':[65,4097,65537],
            'public_allowances':[0,8,128],'public_growing_retry_allowance':1000000000},
        'retained_cost_baselines':costs,
        'limits':['No production runtime source changed in this baseline.',
            'The public identity test checks exact answers and proof reuse, not allocation bounds for the complete public call graph.',
            'All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their previous scope and status.']
    },indent=2)+'\n')
    print(json.dumps({'record':out.name,'source_sha256':sha(record),
        'artifacts_verified':len(previous['artifacts_sha256']),'failing_regressions':5,'passing_guards':5}))


if __name__ == '__main__':
    main()
