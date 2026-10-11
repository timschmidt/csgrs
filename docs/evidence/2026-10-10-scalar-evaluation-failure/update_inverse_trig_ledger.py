#!/usr/bin/env python3
"""Update P4's evidence without changing the accepted contract or plan scope."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def main():
    m = json.loads((E/'typed-api-prototype-v834-metadata.json').read_text())
    assert m['previous_checkpoint'] == 'typed-api-prototype-v729-state.json'
    assert sum(r['cases'] for r in m['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in m['capped_unit_regressions'].values()) == 75
    path = ROOT/'docs/properties-status.json'
    original_bytes = path.read_bytes()
    before = json.loads(original_bytes)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    p = next(r for r in after['properties'] if r['id'] == 'P4')
    assert p['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['inverse-trig-regression-coverage-v834.json','inverse-trig-budget-proof-reuse-v834.json','inverse-trig-resource-followup-v834.json','inverse-trig-inline-v804.json','inverse-trig-paired-cost-v804/summary.json','inverse-trig-source-diff-v760.patch','inverse-trig-inline-diff-v804.patch','typed-api-prototype-v834-state.json']:
        assert prefix+name not in p['evidence']
        p['evidence'].append(prefix+name)
    marker = 'The v729 isolated checkpoint retains'
    assert p['remaining'].count(marker) == 1
    retained = (
        'The v834 isolated checkpoint retains every v729 scalar/cache/domain/trig repair, shared EvaluationBudget, algebraic dependency and normal-form accounting change. The accepted P4 contract allows Unknown to strengthen into Certified and prior proofs to remain available at zero allowance; warming may neither contradict nor weaken a certificate. '
        'Inverse-trig linear and multi-argument recognition, coefficient arithmetic, rational argument copying and fallible exact argument comparisons now debit the shared allowance. Retained pi multiples, Laurent coefficient projection, canonical sine recognition and supported power-of-two coefficients are also metered. Unsupported shifts are rejected before allocation debit. Finite-domain requirements remain independent, and interrupted sign searches cannot publish a root certificate. '
        'Six accounting regressions were retained, including a depth-28 shared DAG that hit the original CPU cap; two existing retry/domain checks already passed. Nine final capped tests cover all measured parser cutpoints (successful costs 38, 44 and 26), every cached-domain sign cutpoint through cost 168, coefficient widths 16/256/1024 (costs 44/152/860), depth-512 comparisons, the unchanged shared DAG, unsupported extreme shifts, rational argument copying, undefined cancellation and 28 public retry schedules. '
        'The new public inverse-trig corpus has 384 cases, 960 independent numeric observations and 384 retained zero-budget proofs. Newly accounted work changes some small-allowance answers to Unknown; every definite answer agrees with the exact oracle, every unlimited retry certifies, and completed proofs remain reusable. All 1584 focused replay answers, samples, work counts and proof-reuse results are identical before and after the six inlining annotations. '
    )
    validation = (
        f"The v834 final-source checkpoint passes {m['test_summaries']['five_crate']['passed']} five-crate and {m['test_summaries']['default_scalar']['passed']} default-scalar tests/doctests, with two separately counted child tests in each; {m['test_summaries']['debug_scalar']['passed']} debug scalar units; strict Clippy; workspace and {m['included_rust_fragments_checked']} included-file format checks; 75 capped tests; and all 4129 replay cases. Fourteen inherited Cargo filename warning lines are unchanged. "
        'Fresh pinned Verus validation passes 638 conditional units and ten acceptance tests. The 213 earlier negative checks reuse 44 byte-identical proof/guard inputs and identical tool/dependency identities. The changed runtime remains outside that target, and all thirteen whole-crate proof groups remain open. '
    )
    cost = (
        'The ordinary path uses private const specialization, and six temporary-form methods permit inlining of fallible returns. Initial alternating runs showed roughly 12-15 percent slowdowns in two inverse-trig fixtures; final paired observations across 24 workloads have a largest median increase of about 4.5 percent. Original separate-process observations remain archived and include larger increases; no uniform speedup is claimed. All inputs, iterations and checksums/certificates are preserved. Four standalone probe executables grew about 0.44-0.50 percent. These are local timing and probe-size observations, not allocation, deployed-size or full-board admission evidence. '
    )
    remaining = (
        'Next inspect algebraic rational-root leaf copying before dependency-size rejection. Distinguish direct Real argument/radical/quadratic-tower helpers from Computable budget reachability. Default shared-DAG expansion and individual exact-operation time/allocation remain separate gaps; the new capped DAG test covers active allowances only. Continue kernel-specific cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission. Main retains the original scalar failure; all 49 original worktree entries are preserved. See inverse-trig-resource-followup-v834.json and typed-api-prototype-v834-state.json.'
    )
    p['remaining'] = p['remaining'][:p['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v834-state.json before production admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, 192000-bit series and original medium coarse-cache regression. '
        +retained+validation+cost+remaining+
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}: assert before[key] == after[key], key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'inverse-trig-ledger-update-v834.json'
    assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original_bytes).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated only P4 evidence, remaining work and its next action; all scope and statuses preserved.')


if __name__ == '__main__':
    main()
