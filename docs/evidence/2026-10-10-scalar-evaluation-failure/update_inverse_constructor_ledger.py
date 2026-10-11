#!/usr/bin/env python3
"""Update scoped P4 evidence while retaining the complete plan and statuses."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())


def main():
    metadata=read(E/'typed-api-prototype-v1283-metadata.json')
    implementation=read(E/'inverse-constructor-implementation-v1248.json')
    assert metadata['previous_checkpoint']=='typed-api-prototype-v1195-state.json'
    assert sum(r['cases'] for r in metadata['final_source_replays'].values())==4129
    assert sum(r['passed'] for r in metadata['capped_unit_regressions'].values())==136
    path=ROOT/'docs/properties-status.json'
    original=path.read_bytes();before=json.loads(original);after=deepcopy(before)
    assert len(after['properties'])==39 and len(after['targets'])==14
    prop=next(r for r in after['properties'] if r['id']=='P4')
    assert prop['status']=='in-progress'
    prefix=str(E.relative_to(ROOT))+'/'
    for name in ['inverse-constructor-baseline-v1216.json','inverse-constructor-corrected-baseline-v1232.json',
                 'inverse-constructor-test-correction-v1232.patch','inverse-constructor-implementation-v1248.json',
                 'inverse-constructor-runtime-diff-v1248.patch','inverse-constructor-retention-v1248.json',
                 'inverse-constructor-paired-cost-v1248/summary.json','inverse-constructor-regression-coverage-v1283.json',
                 'inverse-constructor-proof-reuse-v1283.json','inverse-constructor-resource-followup-v1283.json',
                 'inverse-constructor-laurent-review-v1283.json',
                 'typed-api-prototype-v1283-state.json']:
        assert prefix+name not in prop['evidence'];prop['evidence'].append(prefix+name)
    marker='The v1195 isolated checkpoint retains'
    assert prop['remaining'].count(marker)==1
    retained=(
        'The v1283 isolated checkpoint retains all v1195 scalar/cache/domain/trig, EvaluationBudget, algebraic, '
        'Laurent/normal-form, inverse-trig DAG, structural comparison and raw rational preflight repairs. '
        'The accepted P4 contract permits Unknown to strengthen into Certified and reuse prior proofs at zero '
        'allowance; warming may neither contradict nor weaken a certificate. '
        'Inverse-trig argument and acos endpoint differences now use fallible private specializations of the '
        'existing add, negate, rational scaling and binary-offset constructors. Shared affine parsing, '
        'constant bounds, structural comparison, leaf materialization and coefficient arithmetic charge before '
        'work and stop on exhaustion. Public constructors remain infallible and preserve exact leaf folding, '
        'including inside an already exhausted private scope. Existing local recognition limits are unchanged. '
        'Three frozen allocation regressions fail before repair and pass afterward under unchanged assertions '
        'and resource caps. All eleven corrected baseline tests pass. The baseline correction preserves two '
        'failed log-offset expectations: cold funded and cold unlimited queries both return Unknown, then '
        'a subsequent funded query certifies after child facts warm. Requiring equal certainty was an invalid '
        'new test assumption; all eight earlier test bodies and allocation assertions remained unchanged. '
        'Sixty public and twenty-four internal allocation observations are width-independent across 4097, '
        '65537 and 1000001-bit inputs after repair. Million-bit pi-offset requests fall from 251176 to 872 '
        'Rust bytes; square-root offsets at 48/64 units fall from 251800/251920 to 1112; raw deferred integer '
        'cases fall from up to 876688 to 800. Log controls remain width-independent. Twenty-four ordinary '
        'public rational-atan controls stay preflighted. Eighteen cutpoint schedules, eighteen wide retries '
        'and twelve domain-warming schedules retain soundness, work conservation and zero-work completed '
        'proof reuse. These are synchronous Rust allocation requests excluding fixture setup/native allocation, '
        'not RSS, peak-live memory or whole-call-graph bounds. '
    )
    tests=metadata['test_summaries']
    validation=(
        f"Final-source checks pass {tests['five_crate']['passed']} five-crate and {tests['default_scalar']['passed']} "
        f"default-scalar tests/doctests, each with {tests['five_crate']['child_passed']} separately counted child tests; "
        f"{tests['debug_scalar']['passed']} debug scalar units plus one allocation child; strict Clippy; workspace "
        f"and {metadata['included_rust_fragments_checked']} included-file format checks; 136 capped tests plus their "
        'allocation child; and all 4129 frozen replay cases. Fourteen inherited Cargo filename warnings are '
        'unchanged. All 1584 focused replay answers, numerical observations and retained proofs agree. Fourteen '
        'inverse-trig cases charge nine additional units and retain nine fewer; other focused receipts agree '
        'except executable commands. Fresh pinned Verus verification passes 638 conditional units and ten '
        'acceptance tests. The 213 earlier negative checks reuse 44 unchanged proof/guard inputs and identical '
        'tools/dependencies. Changed runtime remains outside that target; all thirteen whole-crate proof groups '
        'remain open. The retained v3 parser and earlier six-test correction preserve raw logs. '
    )
    paired=[v for rows in implementation['paired_percent_changes'].values() for v in rows]
    sizes=[100*(r['candidate']/r['baseline']-1) for r in implementation['executable_sizes'].values()]
    cost=(
        'Ordinary-cost fixtures, inputs, iterations, checksums and certificates are preserved. Separate and '
        f'alternating trials remain archived. Alternating medians across 24 fixtures range from {min(paired):+.2f} '
        f'to {max(paired):+.2f} percent; standalone probe sizes grow {min(sizes):.3f} to {max(sizes):.3f} percent. '
        'The separate pi-wide +20.13 percent observation measures +0.51 percent paired; algebraic reciprocals '
        'measure +10.39 separately and -1.46 paired. This supports the scoped allowance/allocation repair, '
        'not a uniform speedup or proof of no ordinary-path overhead. Broad performance admission remains open. '
    )
    remaining=(
        'Next review other bounded-query constructor call sites, lazy/scaled coefficients and acos endpoint '
        'allocation cutpoints. The Laurent vector has an eight-term limit and its visit is paid; coefficient '
        'copies preflight their own storage. Allocation before a later failed debit alone is not a resource '
        'violation. If measured, classify bounded paid work honestly rather than inventing a zero-allocation '
        'requirement. See inverse-constructor-laurent-review-v1283.json. Continue general '
        'unshared form growth, algebraic metadata/vector copies and individual exact-operation resource use. '
        'Direct Real inverse-trig/radical/quadratic-tower reachability from Computable allowance wrappers '
        'remains unestablished. Continue kernel-specific cutpoints, concurrent/reconstructed and cross-root '
        'warming, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype '
        'combination and unchanged full-board admission. Main retains the original scalar failure; all 49 '
        'original worktree entries are preserved. See inverse-constructor-resource-followup-v1283.json and '
        'typed-api-prototype-v1283-state.json.'
    )
    prop['remaining']=prop['remaining'][:prop['remaining'].index(marker)]+retained+validation+cost+remaining
    after['next_actions'][0]=(
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1283-state.json before '
        'production admission. Preserve all original tests, signed 96-addition graphs, 740-term domain '
        'fixtures, 192000-bit series and the original medium coarse-cache regression. '
        +retained+validation+cost+remaining+
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification remain in scope.')
    for key in before:
        if key not in {'properties','next_actions'}:assert before[key]==after[key],key
    for old,new in zip(before['properties'],after['properties']):
        if old['id']!='P4':assert old==new
        else:assert old['requirement']==new['requirement'] and old['status']==new['status']
    assert before['next_actions'][1:]==after['next_actions'][1:]
    out=E/'inverse-constructor-ledger-update-v1283.json';assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated scoped P4 evidence; complete plan and statuses preserved.')


if __name__=='__main__':main()
