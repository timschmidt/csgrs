#!/usr/bin/env python3
"""Record constructor allowance validation and remaining admission work."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase
from record_evaluation_budget_checkpoint import case_commands, command_contract

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = 'ae855c9cd84a4f04b1cee1e34918da8c44059038d190f7248e0801d75cdaabe9'
read = lambda p:json.loads(p.read_text())
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(name,data):
    path = E/name
    assert not path.exists(),path
    path.write_text(json.dumps(data,indent=2)+'\n')


def completed(phase):
    path = E/phase
    result = read(path/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged'],(phase,result)
    assert sha(path/'source-hashes.json.gz') == EXPECTED
    return path


def validations(prior,implementation):
    summaries = {}
    for key,version,suffix in [('debug_scalar',1251,'debug-scalar-unit-tests'),('five_crate',1252,'stack-tests'),('default_scalar',1253,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed']+12
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(1235,'inverse-constructor-repair',11),(1274,'rational-preflight',10),(1273,'algebraic-preflight',10),(1272,'structural-comparison',14),(1271,'inverse-dag',10),(1270,'algebraic-leaf',6),(1257,'private-kernel',7),(1258,'magnitude',5),(1259,'publication',7),(1260,'fractional',5),(1261,'scaled-lower',1),(1262,'trig-range',5),(1263,'trig-lattice',5),(1264,'trig-residual',6),(1265,'trig-operand',6),(1266,'evaluation-budget',7),(1267,'algebraic-budget',4),(1268,'normal-form-budget',8),(1269,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 1271 else [])+[(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        assert receipt['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        caps[phase] = receipt|{'passed':count,'child_passed':int(version == 1271),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 136
    replays = {}
    for version,old_version,suffix,count in [(1275,1187,'replay',415),(1276,1188,'precision-replay',60),(1277,1189,'private-kernel-replay',78),(1278,1190,'private-trig-replay',600),(1279,1191,'private-trig-wide-replay',384),(1280,1192,'symbolic-trig-scale-replay',168),(1281,1193,'symbolic-trig-residual-replay',456),(1282,1194,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name),(phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1275:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1276:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1277:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1280:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(path/'summary.json')}
    for phase,value in implementation['replays'].items():
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == value['cases']
        assert data['incomplete_cases'] == 0 and value['inputs_harness_order_and_caps_preserved']
        assert sha(E/phase/'summary.json') == value['summary_sha256']
        replays[phase] = {'cases':value['cases'],'summary_sha256':value['summary_sha256']}
    assert sum(r['cases'] for r in replays.values()) == 4129
    return summaries,caps,replays


def main():
    assert len(sys.argv)==2, 'implementation commit required'
    prior=read(E/'typed-api-prototype-v1195-state.json')
    implementation=read(E/'inverse-constructor-implementation-v1248.json')
    assert implementation['source_record_sha256']==EXPECTED
    changes=[d for r in implementation['replays'].values() for d in r['cooperative_work_count_changes']]
    assert len(changes)==28
    consumed=[d for d in changes if d['path'].endswith('/consumed')]
    remaining_units=[d for d in changes if d['path'].endswith('/remaining')]
    assert len(consumed)==len(remaining_units)==14
    assert all(d['after']-d['before']==9 for d in consumed)
    assert all(d['after']-d['before']==-9 for d in remaining_units)
    proof=read(E/'inverse-constructor-proof-reuse-v1283.json')
    supplemental=read(E/'inverse-constructor-laurent-review-v1283.json')
    assert supplemental['source_record_sha256']==EXPECTED
    assert proof['source_record_sha256']==EXPECTED and proof['verified_units']==638 and proof['errors']==0
    assert len(proof['proof_and_guard_inputs_sha256'])==44 and proof['reused_rejected_mutants']==213
    assert proof['fresh_acceptance_tests']==10 and proof['open_obligation_groups']==13
    source_record='typed-api-prototype-v1249-budget-verus/source-hashes.json.gz'
    source=json.loads(gzip.decompress((E/source_record).read_bytes()))
    before=json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n)==v for n,v in source.items())
    changed={n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n)!=v}
    assert changed==implementation['changed_sources'] and len(changed)==6
    assert all(before[n]==source[n] for n in before if 'test' in n)
    assert implementation['preexisting_tests_unchanged']
    summaries,caps,replays=validations(prior,implementation)
    formatted=read(completed('typed-api-prototype-v1256-included-fmt')/'command.json')['command']
    formatted=formatted[formatted.index('--check')+1:]
    original=json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted==sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n)!=v)
    warnings=lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited=warnings(completed('typed-api-prototype-v1252-stack-tests')/'stderr.log')
    assert inherited==warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited)==14
    for phase in ['typed-api-prototype-v1237-inverse-constructor-clippy','typed-api-prototype-v1254-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    orchestration=read(E/'inverse-constructor-orchestration-v1283.json')
    assert orchestration['exit_code']==0 and orchestration['source_record_sha256']==EXPECTED
    assert orchestration['log_sha256']==sha(E/'inverse-constructor-orchestration-v1283.log')
    limits=implementation['limits']+[
        'The 78 private-kernel replay cases check result shapes. The 84 new measured allocation cases and 24 rational-atan controls do not establish whole-call-graph resource bounds.',
        'All 1584 focused replay answers, numerical observations and retained proofs agree. Fourteen inverse-trig cases consume nine additional units and retain nine fewer; other focused receipts agree except executable commands.',
        'Local timings and standalone probe sizes are scoped evidence. Broad performance admission remains open.',
        'The retained v3 parser and its earlier six-test correction remain authoritative; raw logs are unchanged.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review=read(E/'rational-preflight-resource-followup-v1195.json')
    next_candidates=[r for r in previous_review['next_candidates'] if r.get('operations')!=['inverse-trig argument differences through general constructors']]
    next_candidates.append({
        'path':'crates/hyperreal/src/computable/node/algebra.rs',
        'operations':['remaining constructor call sites and generalized coefficient shapes'],
        'finding':'The two inverse-trig proof differences now use shared fallible constructors. The frozen public pi/sqrt/log offsets and internal deferred integers cover this repair. Broader public construction is intentionally infallible; other bounded query paths and lazy/scaled coefficient shapes need separate reachability and allocation evidence before any claim.'})
    remaining=[r for r in previous_review['remaining_work'] if not r.startswith('Measure the retained inverse-trig constructor path')]
    remaining.insert(0,'Continue from the measured constructor repair: review additional bounded-query constructor call sites, lazy/scaled coefficients and acos endpoint allocation cutpoints without changing ordinary constructor exactness.')
    reviewed={
        'algebra.rs':'Inverse-trig sign recognition through the two argument/endpoint differences; complete negate, shift_left, multiply_rational, rational_offset_parts, add, canonical_sin_pi_term and sin_pi_rational_sum_sign bodies; complete runtime diff reviewed. Not a whole-file audit.',
        'symbolic_work.rs':'Complete shared work helpers, including width charges, fallible leaf materialization and exhaustion checks.',
        'structural_analysis.rs':'shared_constant_term, bound_from_strict_interval and constant-rational sum bounds, plus the runtime diff. Not a whole-file audit.',
        'domain_certificates.rs':'Complete finite-real certificate visitor and its helper branches, reviewed to interpret cold log-offset recognition.',
        'primitive_constructors.rs':'Structural equality entry points and integer/rational payload comparison preflight, plus rational leaf construction; not a whole-file audit.',
        'logarithms.rs':'Public ln entry, try_reduced_ln domain/range planning through final prescaled kernel construction; not a whole-file audit.',
        'inverse_constructor_preflight_tests.rs':'Complete final eleven regression/preservation tests and their fixture helpers.',
        'inverse_constructor_probe_tests.rs':'Complete 160-case public construction/sign survey and fixture helper.'}
    slices={'crates/hyperreal/src/computable/node/'+n:{'sha256':source['crates/hyperreal/src/computable/node/'+n],'scope':scope} for n,scope in reviewed.items()}
    followup={'status':'Constructor allowance repair validated in isolation; remaining admission work open',
        'previous_review':'rational-preflight-resource-followup-v1195.json','source_record':source_record,'source_record_sha256':EXPECTED,
        'supplemental_source_review':'inverse-constructor-laurent-review-v1283.json',
        'reviewed_source_slices':slices,'closed_scoped_gaps':implementation['changes'],
        'public_reachability_evidence':'Sixty public sign queries and twenty-four internal deferred-integer queries show width-independent allocation after repair across 4097, 65537 and 1000001-bit integers. Twenty-four ordinary public rational-atan controls remain preflighted. Full fixture and allowance axes are retained in the tests and implementation record.',
        'warming_contract_evidence':'Eighteen cutpoint schedules, eighteen wide public retry schedules and twelve domain-warming schedules preserve sound answers and completed proofs at zero work. The ln(2) baseline first warms child facts; requiring equal first/subsequent certainty was an invalid new test assumption, corrected before runtime edits and preserved in the evidence.',
        'next_candidates':next_candidates,'remaining_work':remaining,'limits':limits}
    write('inverse-constructor-resource-followup-v1283.json',followup)
    write('inverse-constructor-regression-coverage-v1283.json',{
        'status':'Isolated constructor allowance repair validated; no runtime admission','source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['coverage'],
        'retained_failure_record':'inverse-constructor-baseline-v1216.json','corrected_baseline_record':'inverse-constructor-corrected-baseline-v1232.json',
        'allocation_observations':implementation['allocation_observations'],'cutpoints':implementation['cutpoints'],
        'allocation_scope':implementation['allocation_scope'],'final_source_replays':replays,'prior_tests_unchanged':True,
        'all_eleven_corrected_baseline_tests_and_caps_unchanged':True,'focused_replay_comparison':implementation['replays'],
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'limits':limits})
    write('typed-api-prototype-v1283-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v1195-state.json','implementation_commits':[sys.argv[1]],
        'cost_baseline_commit':'e30d8e06c','baseline_correction_commit':'5f4024e2b','implementation_records':['inverse-constructor-implementation-v1248.json'],
        'status':'isolated fallible inverse-trig constructor repair; no runtime admission','contract_decision':'p4-contract-decision-v173.json',
        'changes':implementation['changes'],'retained_regression_record':'inverse-constructor-regression-coverage-v1283.json',
        'retained_failure_record':'inverse-constructor-baseline-v1216.json','test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'inverse-constructor-proof-reuse-v1283.json','formal_validation':proof,
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'retention_record':'inverse-constructor-retention-v1248.json',
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'inverse-constructor-resource-followup-v1283.json',
        'supplemental_source_review':'inverse-constructor-laurent-review-v1283.json',
        'retained_parser_correction':'algebraic-preflight-parser-correction-v1133.json','completed_orchestration':'inverse-constructor-orchestration-v1283.json',
        'remaining_requirements':remaining+limits})
    print(json.dumps({'tests':summaries,'capped_tests':136,'allocation_child_tests':1,'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__=='__main__':
    main()
