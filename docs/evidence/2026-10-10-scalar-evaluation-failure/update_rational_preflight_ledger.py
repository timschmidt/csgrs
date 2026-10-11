#!/usr/bin/env python3
"""Update scoped P4 evidence without changing any property or target status."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p:json.loads(p.read_text())


def main():
    metadata = read(E/'typed-api-prototype-v1195-metadata.json')
    implementation = read(E/'rational-preflight-implementation-v1161.json')
    retention = read(E/'rational-preflight-retention-v1161.json')
    assert metadata['previous_checkpoint'] == 'typed-api-prototype-v1133-state.json'
    assert sum(r['cases'] for r in metadata['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in metadata['capped_unit_regressions'].values()) == 125
    path = ROOT/'docs/properties-status.json'
    original = path.read_bytes()
    before = json.loads(original)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    prop = next(r for r in after['properties'] if r['id'] == 'P4')
    assert prop['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['rational-preflight-baseline-v1146.json',
                 'rational-preflight-implementation-v1161.json',
                 'rational-preflight-runtime-diff-v1161.patch','rational-preflight-retention-v1161.json',
                 'rational-preflight-paired-cost-v1161/summary.json',
                 'rational-preflight-targeted-confirmation-v1161/summary.json',
                 'rational-preflight-regression-coverage-v1195.json',
                 'rational-preflight-proof-reuse-v1195.json',
                 'rational-preflight-supplemental-review-v1195.json',
                 'rational-preflight-resource-followup-v1195.json','typed-api-prototype-v1195-state.json']:
        assert prefix+name not in prop['evidence']
        prop['evidence'].append(prefix+name)
    marker = 'The v1133 isolated checkpoint retains'
    assert prop['remaining'].count(marker) == 1
    retained = (
        'The v1195 isolated checkpoint retains all v1133 scalar/cache/domain/trig repairs, shared EvaluationBudget, '
        'algebraic dependency and borrowed-leaf checks, Laurent/normal-form accounting, depth-aware inverse-trig DAG '
        'memoization, bounded structural payload comparison and algebraic raw-storage preflight. The accepted P4 '
        'contract permits Unknown to strengthen into Certified and preserves prior proofs at zero allowance; '
        'warming may neither contradict nor weaken a certificate. '
        'Bounded rational recognition now shares the algebraic preflight and charges stored widths before lazy '
        'reduction. The paid canonical view remains available for subsequent arithmetic, preserving the 4096-bit '
        'canonical recognition limit, cheap canonical oversize rejection and funded wide reducible witnesses. '
        'Six rational literal domain branches charge component comparisons before canonicalization; positive-sign '
        'logarithm domain shortcuts remain cheap. Exhausted sign/domain fallbacks stop before allocating another '
        'traversal, after completed sign certificates have remained available. '
        'Five frozen allocation/accounting regressions fail before repair; all ten baseline tests pass afterward '
        'with unchanged inputs, caps and assertions. Eighty private low-work cases allocate zero Rust bytes after '
        'repair, including million-bit lazy operands that previously requested up to 2500656 bytes. All 56 public '
        'approximation cases still return Exhausted, with a fixed 40-byte BudgetScope request equal to their fresh '
        'canonical controls, versus up to 2500712 bytes before repair. These synchronous measurements exclude '
        'fixture setup and native allocation; they do not bound RSS, peak-live memory or the complete call graph. '
        'Every measured recognition cutpoint preserves facts and a successful retry. Keeping the paid canonical '
        'view reduces repeated raw-width charges in inverse/offset arithmetic. Canonical component boundaries, '
        '42 public retry schedules, 39 typed invalid-domain cases and six exact root identities retain their '
        'contracts, including zero-work approximation/sign proof reuse. '
    )
    tests = metadata['test_summaries']
    validation = (
        f"Final-source checks pass {tests['five_crate']['passed']} five-crate and {tests['default_scalar']['passed']} "
        f"default-scalar tests/doctests, each with {tests['five_crate']['child_passed']} separately counted child tests; "
        f"{tests['debug_scalar']['passed']} debug scalar units plus one allocation child; strict Clippy; workspace "
        f"and {metadata['included_rust_fragments_checked']} included-file format checks; 125 capped top-level tests "
        'plus their allocation child; and all 4129 frozen replay cases. Fourteen inherited Cargo filename warning '
        'lines are unchanged. All 1584 focused replay answers, numeric observations and retained proofs are '
        'identical. Eight inverse-trig cases consume six additional units for charged domain comparisons; their '
        'remaining allowance decreases by six. All other focused receipts match except executable commands. '
        'The retained v3 evidence parser preserves raw logs and its earlier six-test correction record. Fresh '
        'pinned Verus validation passes 638 conditional units and ten acceptance tests. The 213 earlier negative '
        'checks reuse 44 unchanged proof/guard inputs and identical tools/dependencies. Changed runtime remains '
        'outside that target, and all thirteen whole-crate proof groups remain open. '
    )
    paired = [v for rows in implementation['paired_percent_changes'].values() for v in rows]
    sizes = [100*(r['candidate']/r['baseline']-1) for r in implementation['executable_sizes'].values()]
    cost = (
        'Ordinary-cost fixtures, iterations, checksums and certificates are preserved. Separate, alternating '
        'and targeted default/algebraic confirmation observations remain archived. '
        f'Alternating medians across 24 fixtures range from {min(paired):+.2f} to {max(paired):+.2f} percent in '
        f'query time; standalone probe sizes change by {min(sizes):+.3f} to {max(sizes):+.3f} percent. '
        +retention['ordinary_cost_uncertainty']+' See rational-preflight-retention-v1161.json. '
    )
    remaining = (
        'Next measure the retained static inverse-trig path into Computable constructors, including differing '
        'integer widths, and bounded Laurent coefficient-copy allocation. The supplemental review proposes '
        'atan(n+pi)-atan(m+pi) to distinguish public deferred-argument construction from the already-metered '
        'exact-rational atan extraction path. Verify construction shape and cold root facts before measurement. '
        'These remain candidates, not measured '
        'public resource failures. Continue general unshared form growth, constructor-time comparison, subsequent '
        'algebraic metadata copies/vector growth and individual exact-operation resource use. Direct Real '
        'inverse-trig/radical/quadratic-tower reachability from public Computable allowance wrappers remains '
        'unestablished; do not infer an active-budget gap in those separate APIs. Continue kernel-specific '
        'cutpoints, concurrent/reconstructed and cross-root warming, implementation-connected proofs, composed '
        'ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission. '
        'Main retains the original scalar failure; all 49 original worktree entries are preserved. See '
        'rational-preflight-resource-followup-v1195.json and typed-api-prototype-v1195-state.json.'
    )
    prop['remaining'] = prop['remaining'][:prop['remaining'].index(marker)]+retained+validation+cost+remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1195-state.json before production '
        'admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment '
        'remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, '
        '192000-bit series and original medium coarse-cache regression. '
        +retained+validation+cost+remaining+
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}: assert before[key] == after[key],key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'rational-preflight-ledger-update-v1195.json'
    assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated P4 evidence; all contracts, statuses and plan scope preserved.')


if __name__ == '__main__':
    main()
