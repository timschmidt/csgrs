#!/usr/bin/env python3
"""Bind the fallible constructor repair to frozen regression and cost evidence."""
import difflib
import gzip
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence
from record_rational_preflight_implementation import differences
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = 'ae855c9cd84a4f04b1cee1e34918da8c44059038d190f7248e0801d75cdaabe9'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    prior = read(E/'typed-api-prototype-v1195-state.json')
    baseline = read(E/'inverse-constructor-corrected-baseline-v1232.json')
    record = E/'typed-api-prototype-v1238-inverse-constructor-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    prefix = 'crates/hyperreal/src/computable/'
    runtime = {prefix+'node/'+n+'.rs' for n in ['algebra','symbolic_work','structural_analysis']}
    assert {n for n,v in source.items() if initial.get(n) != v} == runtime
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert len(changed) == 6
    patch = ''.join(line for name in sorted(runtime) for line in difflib.unified_diff(
        (initial_record.parent/'inputs'/name).read_text().splitlines(True),
        (SOURCE/name).read_text().splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    passes = {}
    for version,suffix in [(1234,'repair-build'),(1235,'repair-cap'),(1236,'unit-tests'),(1237,'clippy'),(1238,'build')]:
        phase = f'typed-api-prototype-v{version}-inverse-constructor-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    units = parse_phase(E/'typed-api-prototype-v1236-inverse-constructor-unit-tests')
    assert units['top_level_passed'] == 1049 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v1237-inverse-constructor-clippy/stderr.log').read_text().splitlines())
    cap_phase = E/'typed-api-prototype-v1235-inverse-constructor-repair-cap'
    log = (cap_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['exit_code'] == 0 and not cap['wall_timeout']
    assert cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
    assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
    assert '11 passed; 0 failed;' in log
    observations = [s for s in (cap_phase/'stderr.log').read_text().splitlines() if s.startswith('constructor ')]
    allocations = [s for s in observations if 'allocations=' in s]
    assert len(allocations) == 84
    measured = []
    for line in allocations:
        rows = re.findall(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}',line)
        assert len(rows) == 2 and int(rows[1][1]) <= int(rows[0][1]),line
        assert int(rows[1][2]) <= 256,line
        measured.append({'fixture':line.split(', answer=')[0], 'control':list(map(int,rows[0])), 'candidate':list(map(int,rows[1]))})
    groups = {}
    for row in measured:
        key = re.sub(r'bits=\d+', 'bits=*',row['fixture'])
        groups.setdefault(key,[]).append(row['candidate'])
    assert len(groups) == 28 and all(len(v)==3 and v[0]==v[1]==v[2] for v in groups.values())
    old_caps = read(E/baseline['baseline_caps'])
    old_observations = [s for phase in old_caps['tests'] for s in (E/phase/'stderr.log').read_text().splitlines() if s.startswith('constructor ')]
    old_allocations = {s.split(', answer=')[0]:s for s in old_observations if 'allocations=' in s}
    assert set(old_allocations) == {r['fixture'] for r in measured}
    for row in measured:
        old = re.findall(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}',old_allocations[row['fixture']])
        row['baseline'] = list(map(int,old[1]))
    cuts = [s for s in observations if 'cutpoints ' in s]
    old_cuts = {s.split(', needed=')[0]:int(s.split(', needed=')[1]) for s in old_observations if 'cutpoints ' in s}
    assert len(cuts)==len(old_cuts)==18
    cutpoints=[{'schedule':s.split(', needed=')[0], 'before':old_cuts[s.split(', needed=')[0]], 'after':int(s.split(', needed=')[1])} for s in cuts]
    replays = {}
    for a,b,suffix,count in [(1152,1239,'inverse',384),(1157,1244,'evaluation',384),(1158,1245,'algebraic',288),(1159,1246,'wide',144),(1160,1247,'normal-form',384)]:
        old=f'typed-api-prototype-v{a}-rational-preflight-{suffix}-replay'
        new=f'typed-api-prototype-v{b}-inverse-constructor-{suffix}-replay'
        data=evidence.check_artifacts(new)
        assert data['cases']==count and data['incomplete_cases']==0 and data['source_hashes_sha256']==EXPECTED
        for k in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert data.get(k,0)==0
        for name in ['probe.rs','reproduce.py']: assert sha(E/old/name)==sha(E/new/name)
        old_rows,new_rows=read(E/old/'receipts.json'),read(E/new/'receipts.json')
        assert [evidence.contract(r) for r in old_rows]==[evidence.contract(r) for r in new_rows]
        strip=lambda rows:[{k:v for k,v in r.items() if k!='command'} for r in rows]
        diff=differences(strip(old_rows),strip(new_rows))
        assert all(d['path'].rsplit('/',1)[-1] in ['consumed','remaining'] for d in diff),diff
        replays[new]={'cases':count,'summary_sha256':sha(E/new/'summary.json'),
            'inputs_harness_order_and_caps_preserved':True,'answers_numeric_observations_and_retained_proofs_identical':True,
            'receipts_identical_except_commands':not diff,'cooperative_work_count_changes':diff}
    paired=read(E/'inverse-constructor-paired-cost-v1248/summary.json')
    assert paired['source_record_sha256']==EXPECTED and paired['processes']==432 and paired['guards_unchanged']
    for name,value in paired['artifacts_sha256'].items(): assert sha(E/'inverse-constructor-paired-cost-v1248'/name)==value
    evidence.EXPECTED=EXPECTED
    costs,records={},[]
    for family,a,b,suffix,checksum in [
        ('inverse-trig',1153,1240,'inverse-cost','certified'),('normal-form',1154,1241,'normal-form-cost','certified'),
        ('default',1155,1242,'default-cost','checksums'),('algebraic',1156,1243,'algebraic-cost','certified')]:
        name='inverse-constructor-'+family+'-cost-comparison-v1248.json'
        costs[family]=evidence.cost(f'typed-api-prototype-v{a}-rational-preflight-{suffix}',f'typed-api-prototype-v{b}-inverse-constructor-{suffix}',name,checksum)
        records.append(name)
    patch_name='inverse-constructor-runtime-diff-v1248.patch'
    assert not (E/patch_name).exists();(E/patch_name).write_text(patch)
    write('inverse-constructor-implementation-v1248.json',{
        'status':'Isolated constructor allowance repair; full checkpoint and broad performance admission pending',
        'previous_checkpoint':'typed-api-prototype-v1195-state.json','baseline_commits':['e30d8e06c','5f4024e2b'],
        'baseline_record':'inverse-constructor-corrected-baseline-v1232.json','source_record':str(record.relative_to(E)),
        'source_record_sha256':EXPECTED,'changed_sources':changed,'runtime_diff':patch_name,
        'preexisting_tests_unchanged':True,'all_eleven_corrected_baseline_tests_and_caps_unchanged':True,
        'changes':[
            'Share negation, addition, rational scaling and binary-offset construction through fallible private specializations for bounded proof queries. Public constructors retain the infallible specialization and exact leaf folding.',
            'Route inverse-trig argument differences and acos endpoint differences through the fallible constructors.',
            'Debit stored rational/integer widths before cloning or materializing leaves. Debit affine parsing, structural comparisons and coefficient arithmetic before work, preserving local visit/size limits.',
            'Stop after an exhausted optional recognizer rather than continuing to copy coefficients. Share constant-term and constant-bound construction with the same request work switch.',
            'Preserve completed root proofs and zero-work reuse; do not cache failure or change the accepted P4 strengthening contract.'
        ],
        'retained_preparation_failure':{'phase':'typed-api-prototype-v1233-inverse-constructor-repair-build','reason':'Nine existing affine-parser tests call the old private entry point. Restored that unlimited test entry as a wrapper; no preexisting test body changed.'},
        'cap':cap,'observations':observations,'allocation_observations':measured,'cutpoints':cutpoints,
        'coverage':{'public_allocation_cases':60,'internal_deferred_integer_allocation_cases':24,'public_rational_atan_width_controls':24,
            'allocation_widths':[4097,65537,1000001],'cutpoint_schedules':18,'public_retry_schedules':18,'domain_warming_schedules':12,
            'exact_public_cancellation_widths':[65,4097,65537],'infallible_exact_leaf_widths':[65,4097,65537,1000001]},
        'allocation_scope':'Rust allocation requests during synchronous queries, excluding fixture construction and native allocations. Not RSS, peak-live memory or a whole-call-graph bound.',
        'passing_validation':passes,'unit_summary':units,'replays':replays,'cost_records':records,'original_trial_percent_changes':costs,
        'paired_cost':'inverse-constructor-paired-cost-v1248/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':['Cooperative work is not an instruction, native-memory or elapsed-time bound.',
            'Only the listed proof constructor paths and fixtures are covered; broader constructors, unshared form growth, concurrent/reconstructed and cross-root warming remain open.',
            'The two new log-offset guard expectations were corrected before runtime changes, as separately recorded. Earlier failed sources and receipts remain archived.',
            'Changed runtime is outside the conditional Verus target; thirteen whole-crate proof groups remain open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work are preserved.']
    })
    print(json.dumps({'record':'inverse-constructor-implementation-v1248.json','allocation_cases':len(measured),'replay_cases':sum(r['cases'] for r in replays.values()),'separate_cost_percent_changes':costs,'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
