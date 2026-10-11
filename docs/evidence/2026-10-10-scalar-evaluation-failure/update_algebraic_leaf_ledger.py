#!/usr/bin/env python3
"""Update P4's scoped evidence while preserving its accepted contract and status."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p:json.loads(p.read_text())


def main():
    m = read(E/'typed-api-prototype-v927-metadata.json')
    implementation = read(E/'algebraic-leaf-switch-v897.json')
    assert m['previous_checkpoint'] == 'typed-api-prototype-v834-state.json'
    assert sum(r['cases'] for r in m['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in m['capped_unit_regressions'].values()) == 81
    path = ROOT/'docs/properties-status.json'
    original = path.read_bytes()
    before = json.loads(original)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    p = next(r for r in after['properties'] if r['id'] == 'P4')
    assert p['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['algebraic-leaf-baseline-v837.json','algebraic-leaf-implementation-v852.json',
                 'algebraic-leaf-initial-retention-v852.json','algebraic-leaf-inverse-confirmation-v852/summary.json',
                 'algebraic-leaf-switch-v897.json','algebraic-leaf-paired-cost-v897/summary.json',
                 'algebraic-leaf-runtime-diff-v852.patch','algebraic-leaf-switch-diff-v897.patch',
                 'algebraic-leaf-regression-coverage-v927.json','algebraic-leaf-proof-reuse-v927.json',
                 'algebraic-leaf-resource-followup-v927.json','algebraic-leaf-reachability-review-v927.json',
                 'typed-api-prototype-v927-state.json']:
        assert prefix+name not in p['evidence']
        p['evidence'].append(prefix+name)
    marker = 'The v834 isolated checkpoint retains'
    assert p['remaining'].count(marker) == 1
    retained = (
        'The v927 isolated checkpoint retains all v834 scalar/cache/domain/trig repairs, shared EvaluationBudget, algebraic dependency, Laurent/normal-form and inverse-trig accounting. The accepted P4 contract allows Unknown to strengthen into Certified and prior proofs to remain available at zero allowance; warming may neither contradict nor weaken a certificate. '
        'Algebraic dependency recognition now rejects incompatible root degrees, nonpositive radicands and unsupported coefficient widths from borrowed metadata before rational conversion. Supported integer copies and rational reads debit the allowance, and shared exhaustion propagates separately from declined local recognition. Integer separation metadata uses the borrowed magnitude; integer/rational scans reject impossible logarithm widths before work and preserve the exact power-of-two boundary. '
        'Five failing regressions and one already-passing generator-interruption guard were retained before repair. Six final tests preserve every original assertion and exercise rejected roots/bases through 1000001 bits, interrupted copies through 4096 bits, signed integer metadata through 1000001 bits, the 131073-bit logarithm boundary, and every integer allowance through the measured successful dependency and metadata costs plus two. '
        'The scoped Rust allocator records zero coefficient-storage requests for rejected roots and interrupted copies. The million-bit metadata query decreases from 125904 to 768 requested bytes; supported integer metadata requests 776 bytes across the tested widths. Fixture/budget creation, native memory and other threads are excluded, so these are neither RSS nor whole-application bounds. '
        'Reusing the parser budget-state switch leaves all 1584 focused answers, numeric samples, work counts, retained proofs and measured allocation observations identical to the initial repair. '
    )
    validation = (
        f"The v927 final-source checkpoint passes {m['test_summaries']['five_crate']['passed']} five-crate and {m['test_summaries']['default_scalar']['passed']} default-scalar tests/doctests, with two separately counted child tests in each; {m['test_summaries']['debug_scalar']['passed']} debug scalar units; strict Clippy; workspace and {m['included_rust_fragments_checked']} included-file format checks; 81 capped tests; and all 4129 replay cases. Fourteen inherited Cargo filename warning lines are unchanged. "
        'Fresh pinned Verus validation passes 638 conditional units and ten acceptance tests. The 213 earlier negative checks reuse 44 byte-identical proof/guard inputs and identical tool/dependency identities. The changed runtime remains outside that target, and all thirteen whole-crate proof groups remain open. '
    )
    paired = [v for rows in implementation['paired_percent_changes'].values() for v in rows]
    sizes = [100*(r['candidate']/r['baseline']-1) for r in implementation['executable_sizes'].values()]
    cost = (
        'All ordinary-cost fixtures, iterations and checksums/certificates are preserved. Initial timings included a repeatable small inverse-trig slowdown and are retained alongside the follow-up candidate that reuses parser state. '
        f'Final alternating observations across 24 fixtures range from {min(paired):+.2f} to {max(paired):+.2f} percent in median query time; separate trials remain archived. Standalone probe-size changes range from {min(sizes):+.3f} to {max(sizes):+.3f} percent. '
        'These local timing and executable-size observations do not establish a uniform speedup or deployed/full-board admission. '
    )
    remaining = (
        'Next trace direct Real inverse-trig/radical/quadratic-tower helper reachability from public Computable allowance wrappers. Default shared-DAG expansion, algebraic metadata vector allocation and individual exact-operation resource use remain separate gaps. Continue kernel-specific cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission. Main retains the original scalar failure; all 49 original worktree entries are preserved. See algebraic-leaf-resource-followup-v927.json and typed-api-prototype-v927-state.json.'
    )
    p['remaining'] = p['remaining'][:p['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v927-state.json before production admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, 192000-bit series and original medium coarse-cache regression. '
        +retained+validation+cost+remaining+
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}:assert before[key] == after[key],key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4':assert old == new
        else:assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'algebraic-leaf-ledger-update-v927.json';assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated P4 evidence and next action; all contracts, statuses and plan scope preserved.')


if __name__ == '__main__':
    main()
