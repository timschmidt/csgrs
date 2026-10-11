#!/usr/bin/env python3
"""Update scoped P4 evidence while preserving every property and target status."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())


def main():
    metadata = read(E/'typed-api-prototype-v1133-metadata.json')
    implementation = read(E/'algebraic-preflight-implementation-v1100.json')
    retention = read(E/'algebraic-preflight-retention-v1100.json')
    assert metadata['previous_checkpoint'] == 'typed-api-prototype-v1074-state.json'
    assert sum(r['cases'] for r in metadata['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in metadata['capped_unit_regressions'].values()) == 115
    path = ROOT/'docs/properties-status.json'
    original = path.read_bytes()
    before = json.loads(original)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    prop = next(r for r in after['properties'] if r['id'] == 'P4')
    assert prop['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['algebraic-preflight-baseline-v1085-corrected.json',
                 'algebraic-preflight-implementation-v1100.json',
                 'algebraic-preflight-runtime-diff-v1100.patch','algebraic-preflight-retention-v1100.json',
                 'algebraic-preflight-paired-cost-v1100/summary.json',
                 'algebraic-preflight-targeted-confirmation-v1100/summary.json',
                 'algebraic-preflight-regression-coverage-v1133.json',
                 'algebraic-preflight-proof-reuse-v1133.json',
                 'algebraic-preflight-resource-followup-v1133.json','typed-api-prototype-v1133-state.json']:
        assert prefix+name not in prop['evidence']
        prop['evidence'].append(prefix+name)
    marker = 'The v1074 isolated checkpoint retains'
    assert prop['remaining'].count(marker) == 1
    retained = (
        'The v1133 isolated checkpoint retains all v1074 scalar/cache/domain/trig repairs, shared EvaluationBudget, '
        'algebraic dependency and borrowed-leaf checks, Laurent/normal-form accounting, depth-aware inverse-trig DAG '
        'memoization and bounded structural payload comparison. The accepted P4 contract permits Unknown to '
        'strengthen into Certified and preserves prior proofs at zero allowance; warming may neither contradict '
        'nor weaken a certificate. '
        'Algebraic metadata and dependency preflight now debit raw rational storage before lazy reduction, and '
        'coefficient cost estimation reads stored widths without canonicalizing. Existing canonical recognition '
        'limits and cheap sign/degree/oversize rejections remain; sufficiently funded wide unreduced values can '
        'still produce supported canonical proofs. The metadata parser allocates its initial 16-entry vector '
        'only after a successful charged visit. '
        'Five frozen allocation/accounting regressions fail before repair; all ten baseline tests pass afterward '
        'with unchanged inputs, caps and assertions. Million-bit metadata/candidate/base preflight drops from '
        '2500392 Rust allocation bytes to zero when work is insufficient. Cost estimation for two million-bit '
        'lazy operands drops from 5000752 bytes to zero, and all five zero-work parser fixtures drop from one '
        '768-byte allocation to zero. These synchronous measurements exclude setup and native allocation and '
        'do not bound RSS or peak-live memory. '
        'Every measured metadata cutpoint preserves facts and retry behavior. Canonical logarithm boundaries '
        'and 27 public retry schedules preserve exact answers and completed zero proofs at zero work. The '
        'public schedules check proof retention, not whole-call allocation bounds. '
    )
    tests = metadata['test_summaries']
    validation = (
        f"Final-source checks pass {tests['five_crate']['passed']} five-crate and {tests['default_scalar']['passed']} "
        f"default-scalar tests/doctests, each with {tests['five_crate']['child_passed']} separately counted child tests; "
        f"{tests['debug_scalar']['passed']} debug scalar units plus one allocation child; strict Clippy; workspace "
        f"and {metadata['included_rust_fragments_checked']} included-file format checks; 115 capped top-level tests "
        'plus their allocation child; and all 4129 frozen replay cases. Fourteen inherited Cargo filename warning '
        'lines are unchanged. All 1584 focused replay receipts match except executable commands, preserving '
        'answers, observations, certificates, work counts, inputs, allowances and process caps. '
        'Fresh pinned Verus validation passes 638 conditional units and ten acceptance tests. The 213 earlier '
        'negative checks reuse 44 unchanged proof/guard inputs and identical tools/dependencies. The changed '
        'runtime remains outside that target and all thirteen whole-crate proof groups remain open. '
    )
    paired = [v for rows in implementation['paired_percent_changes'].values() for v in rows]
    sizes = [100*(r['candidate']/r['baseline']-1) for r in implementation['executable_sizes'].values()]
    cost = (
        'Ordinary-cost fixtures, iterations, checksums and certificates are preserved. Separate, alternating '
        'and targeted default-family confirmation observations remain archived. '
        f'Alternating medians across 24 fixtures range from {min(paired):+.2f} to {max(paired):+.2f} percent in '
        f'query time; standalone probe sizes change by {min(sizes):+.3f} to {max(sizes):+.3f} percent. '
        + retention['ordinary_cost_uncertainty'] + ' See algebraic-preflight-retention-v1100.json. '
    )
    remaining = (
        'Next review general unshared form growth, constructor-time comparison, subsequent algebraic metadata '
        'copies/vector growth and individual exact-operation resource use. Direct Real inverse-trig/radical/'
        'quadratic-tower reachability from public Computable allowance wrappers remains unestablished; do not '
        'infer an active-budget gap in those separate APIs. Continue kernel-specific cutpoints, concurrent/'
        'reconstructed and cross-root warming, implementation-connected proofs, composed ln/exp/inverse-hyperbolic '
        'planning, native-prototype combination and unchanged full-board admission. Main retains the original '
        'scalar failure; all 49 original worktree entries are preserved. See algebraic-preflight-resource-followup-v1133.json '
        'and typed-api-prototype-v1133-state.json.'
    )
    prop['remaining'] = prop['remaining'][:prop['remaining'].index(marker)] + retained + validation + cost + remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1133-state.json before production '
        'admission. Preserve the accepted P4 strengthening contract; the strict cold/warm equality experiment '
        'remains rejected. Keep all original tests, signed 96-addition graphs, 740-term domain fixtures, '
        '192000-bit series and original medium coarse-cache regression. '
        + retained + validation + cost + remaining +
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification requirements remain in scope.'
    )
    for key in before:
        if key not in {'properties','next_actions'}: assert before[key] == after[key], key
    for old,new in zip(before['properties'],after['properties']):
        if old['id'] != 'P4': assert old == new
        else: assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'algebraic-preflight-ledger-update-v1133.json'
    assert not out.exists()
    path.write_text(json.dumps(after,indent=2)+'\n')
    out.write_text(json.dumps({'path':str(path.relative_to(ROOT)),
        'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope':['P4 evidence and remaining work','next_actions[0]'],'properties_preserved':39,'targets_preserved':14,
        'all_statuses_and_future_work_preserved':True,'other_property_rows_and_remaining_next_actions_unchanged':True},indent=2)+'\n')
    print('Updated P4 evidence; all contracts, statuses and plan scope preserved.')


if __name__ == '__main__':
    main()
