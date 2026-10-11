#!/usr/bin/env python3
"""Update P4's evidence and next action while retaining the complete plan scope."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def main():
    m = json.loads((E/'typed-api-prototype-v578-metadata.json').read_text())
    assert m['previous_checkpoint'] == 'typed-api-prototype-v504-state.json'
    assert sum(r['cases'] for r in m['final_source_replays'].values()) == 3361
    assert sum(r['passed'] for r in m['capped_unit_regressions'].values()) == 58
    path = ROOT/'docs/properties-status.json'
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    p = next(r for r in after['properties'] if r['id'] == 'P4')
    assert p['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['algebraic-budget-regression-coverage-v578.json','algebraic-budget-proof-reuse-v578.json','algebraic-wide-budget-coverage-v549.json','algebraic-budget-resource-followup-v578.json','algebraic-budget-cost-comparison-v525.json','algebraic-budget-default-cost-comparison-v525.json','typed-api-prototype-v578-state.json']:
        assert prefix+name not in p['evidence']
        p['evidence'].append(prefix+name)
    marker = 'The v504 isolated checkpoint retains all earlier magnitude/cache'
    assert p['remaining'].count(marker) == 1
    validation = (
        f"The v578 final-source checkpoint passes {m['test_summaries']['five_crate']['passed']} five-crate and {m['test_summaries']['default_scalar']['passed']} default-scalar tests/doctests, plus two separately counted child tests in each; {m['test_summaries']['debug_scalar']['passed']} debug scalar units; strict Clippy; workspace and {m['included_rust_fragments_checked']} included-file formatting checks; 58 capped tests; and all 3361 replay cases. Fourteen inherited Cargo filename warning lines are unchanged. "
        "The fresh pinned Verus target passes 638 conditional units and ten acceptance tests. The 213 negative guards reuse 44 identical proof/guard inputs plus identical tool and dependency identities. This budget runtime remains outside that target; thirteen whole-crate proof groups remain open. "
    )
    retained = (
        "The v578 isolated checkpoint retains the v504 caller-owned shared EvaluationBudget implementation and all earlier magnitude/cache, inverse-series, coarse-asinh, exact-pole, serialized range, compressed lattice, rational-residual and finite non-rational operand repairs. Unknown may strengthen into Certified and every available proof remains reusable, including high-precision proofs at zero allowance. "
        "Algebraic separation metadata now debits shared work for parsing, generator merging, fallible structural comparisons and rational-root dependency combinations. Selected coefficient operations are charged by operand size before multiplication, division or exact-root extraction. Exhaustion declines metadata without treating an interrupted search as inequality or independence. Ordinary queries skip accounting when no allowance is active. "
        "Four new regressions cover metadata allowance boundaries, 16/256/1024-bit dependency coefficients, 512-deep generator comparisons and 180 public exact perturbation/reciprocal schedules. Three tests reproduced the prior accounting gaps before the repair. The frozen external replay adds 288 cases, 576 independent rational numerical observations and 288 retained zero-budget proofs; all answers, samples and work counters match between the metered implementation and its ordinary-query fast path. An additional 144 wide cube/fourth-root cases exercise the public coefficient-accounting path at 65536-bit precision, retaining the 64/256/1024-bit inputs and adding 288 numerical observations and 144 zero-budget proof checks. The original 2929 replay cases and all prior named test sources remain unchanged. "
    )
    cost = (
        "The initial accounting trial's cube/fourth-root medians rose about 17/20 percent and remain archived. The ordinary-query fast path reduced those measured changes to about 4.5/1.1 percent. Across six algebraic workloads final median changes span about -10.9 to +7.0 percent; six general workloads span -5.4 to +0.2 percent against the prior budget implementation. Each uses nine separate-process repetitions with identical certificates/checksums and iteration counts. These are local timings, not allocation, binary-size or full-board admission evidence. "
    )
    remaining = (
        "Next meter Laurent normal-form recognition and bounded rational recognition on the public budgeted sign path: their local traversal limits, opaque-atom equality and coefficient arithmetic still leave shared-accounting gaps. Algebraic leaf cloning and individual GCD/root operations are not fully metered; the coefficient weight is neither an elapsed-time nor a byte bound. Retain radical/quadratic tower gaps and map their Real API reachability separately. Continue kernel-specific cutpoints, public concurrent/reconstructed schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, complete resource accounting, native-prototype combination and unchanged full-board admission evidence. Main runtime still has the original scalar failure, and all 49 original worktree entries remain preserved. See algebraic-budget-resource-followup-v578.json and typed-api-prototype-v578-state.json."
    )
    p['remaining'] = p['remaining'][:p['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v578-state.json before production admission. Preserve P4's accepted Unknown-to-Certified strengthening and prior proofs without contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Keep all v504 repairs, signed 96-addition graphs, 740-term domain fixtures, 192000-bit series, concurrent publication/lower-exponent fixtures, every prior test and replay, and the unchanged medium coarse-cache regression that caught the reducer cycle. "
        + retained + validation + cost + remaining +
        " All 39 properties, 14 targets, crate future work and the complete pre-push verification requirements remain in scope."
    )
    for key in before:
        if key not in {'properties','next_actions'}:
            assert before[key] == after[key], key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'algebraic-budget-ledger-update-v578.json'
    assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)), 'before_sha256':hashlib.sha256(before_bytes).hexdigest(), 'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(), 'changed_scope':['P4 evidence and remaining work','next_actions[0]'], 'properties_preserved':39, 'targets_preserved':14, 'all_statuses_and_future_work_preserved':True, 'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated only P4 evidence/remaining work and its next action; complete plan and existing statuses preserved.')


if __name__ == '__main__':
    main()
