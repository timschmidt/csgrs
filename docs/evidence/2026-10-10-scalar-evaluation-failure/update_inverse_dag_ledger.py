#!/usr/bin/env python3
"""Record the scoped DAG repair without changing property contracts or status."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())


def main():
    m = read(E/'typed-api-prototype-v1011-metadata.json')
    implementation = read(E/'inverse-dag-switch-v980.json')
    assert m['previous_checkpoint'] == 'typed-api-prototype-v927-state.json'
    assert sum(r['cases'] for r in m['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in m['capped_unit_regressions'].values()) == 91
    path = ROOT/'docs/properties-status.json'
    original = path.read_bytes()
    before = json.loads(original)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    p = next(r for r in after['properties'] if r['id'] == 'P4')
    assert p['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['inverse-dag-baseline-v941.json','inverse-dag-implementation-v965.json','inverse-dag-initial-retention-v965.json','inverse-dag-switch-v980.json','inverse-dag-retention-v980.json','inverse-dag-switch-diff-v980.patch',
                 'inverse-dag-paired-cost-completion-v980/summary.json','inverse-dag-inverse-confirmation-v980/summary.json','inverse-dag-paired-failure-v980.json','inverse-dag-runtime-diff-v965.patch',
                 'inverse-dag-allocation-isolation-v965.patch','inverse-dag-regression-coverage-v1011.json',
                 'inverse-dag-proof-reuse-v1011.json','inverse-dag-resource-followup-v1011.json','inverse-dag-comparison-review-v1011.json',
                 'typed-api-prototype-v1011-state.json']:
        assert prefix+name not in p['evidence']
        p['evidence'].append(prefix+name)
    marker = 'The v927 isolated checkpoint retains'
    assert p['remaining'].count(marker) == 1
    retained = (
        'The v1011 isolated checkpoint retains all v927 scalar/cache/domain/trig repairs, shared EvaluationBudget, algebraic dependency and borrowed-leaf checks, and Laurent/normal-form and inverse-trig accounting. The accepted P4 contract permits Unknown to strengthen into Certified and retains proofs at zero allowance; warming may neither contradict nor weaken a certificate. '
        'Three inverse-trig parsers now memoize completed forms for shared compound descendants within one query. Keys include node identity and remaining depth, preserving path-specific limits, exact coefficients and argument order. Copies debit the active allowance before memo reuse or insertion; interruption discards the local table. '
        'Four frozen ordinary DAG cases hit the original two-second CPU cap before repair, and another exhausted 10000 work units. They now pass unchanged: depth-28 doubling requires 331, 415 and 331 units for atan-linear, atan-terms and acos-linear. Ten new tests also cover depth-24 unequal paths, both depth-limit traversal orders, every depth-five interruption point, unshared reconstructions and undefined cancellation. '
        'The allocation fixture retains depths 6 and 12 and its eightfold growth limit. A full-suite failure revealed interference from global rational caches; running that same fixture in a fresh child process reproduces the original failure and passes the repair. Multi-argument depth-12 Rust allocation requests decrease from 98304 to 2220 bytes. Linear parsers use additional memo storage. The measurement excludes fixtures and native allocation and does not bound RSS or peak-live memory. '
    )
    tests = m['test_summaries']
    validation = (
        f"Final-source checks pass {tests['five_crate']['passed']} five-crate and {tests['default_scalar']['passed']} default-scalar tests/doctests, each with {tests['five_crate']['child_passed']} separately counted child tests; {tests['debug_scalar']['passed']} debug scalar units plus one allocation child; strict Clippy; workspace and {m['included_rust_fragments_checked']} included-file format checks; 91 capped top-level tests plus their allocation child; and all 4129 frozen replay cases. Fourteen inherited Cargo filename warning lines are unchanged. "
        'Fresh pinned Verus validation passes 638 conditional units and ten acceptance tests. The 213 earlier negative checks reuse 44 unchanged proof/guard inputs and identical tool/dependency identities. The changed runtime remains outside that target and all thirteen whole-crate proof groups remain open. '
    )
    paired = [v for rows in implementation['paired_percent_changes'].values() for v in rows]
    sizes = [100*(r['candidate']/r['baseline']-1) for r in implementation['executable_sizes'].values()]
    cost = (
        'Ordinary-cost fixtures, iterations, checksums and certificates are preserved. The first memoization attempt caused repeated ordinary-path slowdowns; its observations remain archived. The final dispatch inlines the unshared-node path without changing memo keys, charges or recognition. '
        f'Alternating observations across 24 fixtures range from {min(paired):+.2f} to {max(paired):+.2f} percent in median query time; separate trials are also archived. Standalone probe-size changes range from {min(sizes):+.3f} to {max(sizes):+.3f} percent. '
        'The additional inverse-trig confirmation is retained. One unchanged-baseline process exited unsuccessfully during the paired campaign; its failed attempt remains recorded, and the unfinished comparisons passed the original limits while preserving all earlier successful observations. These local observations do not establish a uniform speedup or whole-application admission. '
    )
    remaining = (
        'Next freeze wide reconstructed-argument structural comparisons: traversal already has bounded/memoized handling, while its literal-comparison callback charges node visits without coefficient-width accounting. Trace direct Real inverse-trig/radical/quadratic-tower reachability from public Computable allowance wrappers. General unshared form growth, constructor-time structural comparison, algebraic metadata vector allocation and individual exact operations remain separate resource gaps. Continue kernel-specific cutpoints, concurrent/reconstructed and cross-root warming, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission. Main retains the original scalar failure; all 49 original worktree entries are preserved. See inverse-dag-resource-followup-v1011.json and typed-api-prototype-v1011-state.json.'
    )
    p['remaining'] = p['remaining'][:p['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1011-state.json before production admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, 192000-bit series and original medium coarse-cache regression. '
        + retained + validation + cost + remaining +
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}: assert before[key] == after[key], key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'inverse-dag-ledger-update-v1011.json'
    assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated P4 evidence; all contracts, statuses and plan scope preserved.')


if __name__ == '__main__':
    main()
