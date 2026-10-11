#!/usr/bin/env python3
"""Bind rational/domain preflight repair to frozen regressions and cost inputs."""
import difflib
import gzip
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = '2f5184c00b655901e7a89b44ccb63824a7aad7ce3790a11f7e82e7ca5da5192f'
read, sha, write = evidence.read, evidence.sha, evidence.write


def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        assert a.keys() == b.keys(), path
        return [d for k in a for d in differences(a[k], b[k], path+'/'+k)]
    if isinstance(a, list) and isinstance(b, list):
        assert len(a) == len(b), path
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,path+'/'+str(i))]
    return [] if a == b else [{'path':path,'before':a,'after':b}]


def main():
    prior = read(E/'typed-api-prototype-v1133-state.json')
    baseline = read(E/'rational-preflight-baseline-v1146.json')
    record = E/'typed-api-prototype-v1151-rational-preflight-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    runtime = {prefix+'node/'+n+'.rs' for n in ['symbolic_work','structural_analysis','domain_certificates','domain_validation']}
    test = prefix+'node/rational_preflight_tests.rs'
    assert set(changed) == runtime | {test,prefix+'node.rs'}
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    assert set(n for n,v in source.items() if initial.get(n) != v) == runtime
    assert source[test] == initial[test]
    patch = E/'rational-preflight-runtime-diff-v1161.patch'
    assert not patch.exists()
    hunks = []
    for name in sorted(runtime):
        old = initial_record.parent/'inputs'/name
        assert sha(old) == initial[name]
        hunks.extend(difflib.unified_diff(old.read_text().splitlines(True),
            (SOURCE/name).read_text().splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    patch.write_text(''.join(hunks))
    passes = {}
    for version,suffix in [(1147,'repair-build'),(1148,'repair-cap'),(1149,'unit-tests'),(1150,'clippy'),(1151,'build')]:
        phase = f'typed-api-prototype-v{version}-rational-preflight-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    units = parse_phase(E/'typed-api-prototype-v1149-rational-preflight-unit-tests')
    assert units['top_level_passed'] == 1037 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v1150-rational-preflight-clippy/stderr.log').read_text().splitlines())
    cap_phase = E/'typed-api-prototype-v1148-rational-preflight-repair-cap'
    log = (cap_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['exit_code'] == 0 and not cap['wall_timeout']
    assert cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
    assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
    assert '10 passed; 0 failed;' in log
    observations = [s for s in (cap_phase/'stderr.log').read_text().splitlines()
        if s.startswith(('rational ', 'child domain ', 'literal domain ', 'public domain '))]
    private = [s for s in observations if 'allocations=' in s and not s.startswith('public ')]
    public = [s for s in observations if s.startswith('public ')]
    cutpoints = [s for s in observations if s.startswith('rational cutpoints ')]
    assert len(private) == 80 and all('calls: 0, bytes: 0, largest: 0' in s for s in private)
    assert len(public) == 56
    for line in public:
        allocations = re.findall(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}',line)
        assert allocations == [('1','40','40')]*2,line
    assert len(cutpoints) == 9
    original_cutpoints = baseline['observations']['rational_preflight_every_recognition_cutpoint_preserves_retry']
    work = []
    for old,new in zip(original_cutpoints,cutpoints):
        pattern = r'rational cutpoints bits=(\d+), kind=(\d+), needed=(\d+)'
        a,b = re.fullmatch(pattern,old),re.fullmatch(pattern,new)
        assert a and b and a.groups()[:2] == b.groups()[:2]
        assert int(b[3]) <= int(a[3])
        work.append({'factor_bits':int(a[1]),'kind':int(a[2]),'before':int(a[3]),'after':int(b[3])})
    replays = {}
    for a,b,suffix,count in [(1091,1152,'inverse',384),(1096,1157,'evaluation',384),(1097,1158,'algebraic',288),(1098,1159,'wide',144),(1099,1160,'normal-form',384)]:
        old = f'typed-api-prototype-v{a}-algebraic-preflight-{suffix}-replay'
        new = f'typed-api-prototype-v{b}-rational-preflight-{suffix}-replay'
        data = evidence.check_artifacts(new)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert data.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/old/name) == sha(E/new/name)
        old_rows,new_rows = read(E/old/'receipts.json'),read(E/new/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        strip = lambda rows:[{k:v for k,v in r.items() if k != 'command'} for r in rows]
        diff = differences(strip(old_rows),strip(new_rows))
        assert all(d['path'].rsplit('/',1)[-1] in ['consumed','remaining'] for d in diff),diff
        replays[new] = {'cases':count,'summary_sha256':sha(E/new/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,
            'answers_numeric_observations_and_retained_proofs_identical':True,
            'receipts_identical_except_commands':not diff,'cooperative_work_count_changes':diff}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,suffix,checksum in [
        ('inverse-trig',1092,1153,'inverse-cost','certified'),
        ('normal-form',1093,1154,'normal-form-cost','certified'),
        ('default',1094,1155,'default-cost','checksums'),
        ('algebraic',1095,1156,'algebraic-cost','certified'),
    ]:
        name = 'rational-preflight-'+family+'-cost-comparison-v1161.json'
        costs[family] = evidence.cost(f'typed-api-prototype-v{a}-algebraic-preflight-{suffix}',
            f'typed-api-prototype-v{b}-rational-preflight-{suffix}',name,checksum)
        records.append(name)
    paired = read(E/'rational-preflight-paired-cost-v1161/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for name,value in paired['artifacts_sha256'].items(): assert sha(E/'rational-preflight-paired-cost-v1161'/name) == value
    confirmation = read(E/'rational-preflight-targeted-confirmation-v1161/summary.json')
    assert confirmation['source_record_sha256'] == EXPECTED and confirmation['processes'] == 216 and confirmation['guards_unchanged']
    for name,value in confirmation['artifacts_sha256'].items(): assert sha(E/'rational-preflight-targeted-confirmation-v1161'/name) == value
    for family in ['default','algebraic']:
        assert confirmation['provenance'][family]['binary_sha256_at_paired_run'] == paired['provenance'][family]['binary_sha256_at_paired_run']
    write('rational-preflight-implementation-v1161.json',{
        'status':'Scoped rational/domain lazy-preflight repair; full checkpoint pending; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1133-state.json','baseline_commit':'f00bfc5cd',
        'baseline_record':'rational-preflight-baseline-v1146.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,'changed_sources':changed,
        'runtime_diff':patch.name,'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_ten_baseline_tests_values_caps_and_assertions_unchanged':True,
        'changes':[
            'Share the existing algebraic rational preflight through SymbolicWork and use it for bounded rational recognition. Debit raw stored widths before lazy reduction, then retain the paid canonical view.',
            'Preserve canonical 4096-bit recognition limits, cheap canonical oversize rejection and sufficiently funded wide unreduced exact witnesses.',
            'Charge rational literal domain comparisons before canonicalizing components. Preserve allocation-free positive-sign logarithm domain shortcuts.',
            'After completed exact-sign and numeric-cache certificates have been checked, stop an exhausted sign request before allocating a new traversal.',
            'Stop an exhausted finite-real domain preflight before allocating the fallback traversal.',
            'Preserve exact answers, typed domain errors, completed certificates and zero-work proof reuse.'
        ],
        'unit_coverage':baseline['coverage'],'cap':cap,'observations':observations,
        'allocation_observations':{'private':private,'public':public},
        'measured_work_observations':work,
        'work_count_explanation':'Returning the paid canonical view avoids charging its original wide unreduced storage again in subsequent inverse/offset arithmetic. Every allowance through each measured successful cost plus two retains its exact success/exhaustion contract and unlimited retry.',
        'allocation_scope':baseline['allocation_scope'],'passing_validation':passes,'unit_summary':units,
        'replays':replays,'cost_records':records,'original_trial_percent_changes':costs,
        'paired_cost':'rational-preflight-paired-cost-v1161/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'targeted_confirmation':'rational-preflight-targeted-confirmation-v1161/summary.json',
        'confirmation_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in confirmation['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':[
            'Cooperative units do not bound individual exact-operation CPU, native allocation, elapsed time or total memory.',
            'Allocation evidence counts Rust requests in specific synchronous calls, excluding fixture setup and native allocation; it is not RSS or peak-live memory.',
            'The 56 public approximation allocation cases and 42 public retry schedules cover fourteen frozen shapes, not the complete public call graph.',
            'Finite tests and replays do not exhaust concurrent, reconstructed or cross-root warming, constructor arithmetic or every unshared symbolic expression.',
            'Changed runtime remains outside the conditional Verus target; all thirteen whole-crate proof groups remain open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their previous scope and status.'
        ]})
    print(json.dumps({'source':EXPECTED,'replay_cases':1584,'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'work_count_changes':{k:len(v['cooperative_work_count_changes']) for k,v in replays.items()}}))


if __name__ == '__main__':
    main()
