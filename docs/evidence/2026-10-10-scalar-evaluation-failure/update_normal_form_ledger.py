#!/usr/bin/env python3
"""Update only P4's retained evidence and next work; preserve the full plan."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def main():
    m = json.loads((E/'typed-api-prototype-v729-metadata.json').read_text())
    assert m['previous_checkpoint'] == 'typed-api-prototype-v578-state.json'
    assert sum(r['cases'] for r in m['final_source_replays'].values()) == 3745
    assert sum(r['passed'] for r in m['capped_unit_regressions'].values()) == 66
    path = ROOT/'docs/properties-status.json'
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    p = next(r for r in after['properties'] if r['id'] == 'P4')
    assert p['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['normal-form-regression-coverage-v729.json','normal-form-budget-proof-reuse-v729.json','normal-form-resource-followup-v729.json','normal-form-specialization-v700.json','normal-form-paired-cost-v700/summary.json','normal-form-source-diff-v700.patch','typed-api-prototype-v729-state.json']:
        assert prefix+name not in p['evidence']
        p['evidence'].append(prefix+name)
    marker = 'The v578 isolated checkpoint retains'
    assert p['remaining'].count(marker) == 1
    retained = (
        'The v729 isolated checkpoint retains the v578 shared EvaluationBudget and algebraic dependency accounting, plus every prior scalar/cache/domain/trig repair. Unknown may strengthen into Certified, and prior proofs remain reusable without contradictions or weaker available facts, including high-precision proofs at zero allowance. '
        'Laurent normal-form traversal, memo copies, opaque-atom comparisons, coefficient arithmetic, shifts and selected surd projections now debit the active shared allowance. Bounded rational recognition also meters every visit and selected coefficient work. Interrupted temporary forms cannot become opaque fallbacks, permanent negative facts or root sign certificates. '
        'The public replay exposed a tentative Unknown retained before a stronger proof exhausted its allowance; budgeted sign searches now retain completed proofs only, preserving unlimited retries. Another preserved trig replay exposed allocation charging before an existing shift-size rejection; that rejection now precedes the debit. Both failures became deterministic regressions without shrinking the original serialized inputs. '
        'Eight new capped regressions cover allowance sweeps, coefficient widths 16/256/1024, 512-deep atom comparison, 42 nested squares, million-bit shifts, unsupported i32 extrema, undefined cancellation and 22 public retry schedules. The new public normal-form corpus has 384 cases, 960 independent numeric observations and 384 retained zero-budget proofs. All 1200 budgeted replay outcomes, samples and work counts match between the dynamic and specialized implementations; all 3361 prior cases remain preserved. '
    )
    validation = (
        f"The v729 final-source checkpoint passes {m['test_summaries']['five_crate']['passed']} five-crate and {m['test_summaries']['default_scalar']['passed']} default-scalar tests/doctests, with two separately counted child tests in each; {m['test_summaries']['debug_scalar']['passed']} debug scalar units; strict Clippy; workspace and {m['included_rust_fragments_checked']} included-file format checks; 66 capped tests; and all 3745 replay cases. Fourteen inherited Cargo filename warning lines are unchanged. "
        'The fresh pinned Verus target passes 638 conditional units and ten acceptance tests. The 213 negative guards reuse 44 identical proof/guard inputs and identical tool/dependency identities. This runtime remains outside that target; thirteen whole-crate proof groups remain open. '
    )
    cost = (
        'The ordinary Laurent path is privately specialized to remove dynamic accounting checks. Original timing sets and alternating-order comparisons remain archived. The earlier paired wide-pi/reciprocal regressions were about 17/14 percent; final paired increases across 18 scoped workloads are below 7 percent, while original separate trials include larger outliers. Each comparison preserves inputs, iteration counts and checksums/certificates; no universal speedup is claimed. Standalone probe executables grew about 0.7 percent. These are local timing and probe-size observations, not allocation, deployed binary-size or full-board admission evidence. '
    )
    remaining = (
        'Next review and meter inverse-trig linear forms and retained pi-recognition helpers reached before or around the ordinary sign walk. Selected coefficient clones and arithmetic remain unmetered there; preserve finite-domain requirements, fallible comparisons and cheap recognition-limit rejection before allocation charges. Algebraic leaf cloning and individual GCD/root operations still lack complete resource accounting. Retain radical/quadratic-tower gaps and distinguish their Real API reachability. Continue kernel-specific cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission. Main runtime retains the original scalar failure, and all 49 original worktree entries remain preserved. See normal-form-resource-followup-v729.json and typed-api-prototype-v729-state.json.'
    )
    p['remaining'] = p['remaining'][:p['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v729-state.json before production admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, 192000-bit series and original medium coarse-cache regression. '
        + retained + validation + cost + remaining +
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}: assert before[key] == after[key], key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'normal-form-ledger-update-v729.json'; assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),'before_sha256':hashlib.sha256(before_bytes).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated only P4 evidence, remaining work and its next action; all scope and statuses preserved.')


if __name__ == '__main__':
    main()
