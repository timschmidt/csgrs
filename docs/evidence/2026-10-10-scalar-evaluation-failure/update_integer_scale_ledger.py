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
    metadata = read(E/'typed-api-prototype-v1404-metadata.json')
    implementation = read(E/'integer-scale-implementation-v1367.json')
    assert metadata['previous_checkpoint'] == 'typed-api-prototype-v1283-state.json'
    assert sum(r['cases'] for r in metadata['final_source_replays'].values()) == 4129
    assert sum(r['passed'] for r in metadata['capped_unit_regressions'].values()) == 141
    path = ROOT/'docs/properties-status.json'
    original = path.read_bytes()
    before = json.loads(original)
    after = deepcopy(before)
    assert len(after['properties']) == 39 and len(after['targets']) == 14
    prop = next(r for r in after['properties'] if r['id'] == 'P4')
    assert prop['status'] == 'in-progress'
    prefix = str(E.relative_to(ROOT))+'/'
    for name in ['integer-scale-baseline-v1298.json', 'integer-scale-test-correction-v1298.patch',
        'integer-scale-implementation-v1314.json', 'integer-scale-paired-cost-v1314/summary.json',
        'integer-scale-implementation-v1367.json', 'integer-scale-runtime-diff-v1367.patch',
        'integer-scale-retention-v1367.json', 'integer-scale-paired-cost-v1367/summary.json',
        'integer-scale-regression-coverage-v1404.json', 'integer-scale-proof-validation-v1404.json',
        'integer-scale-resource-followup-v1404.json', 'integer-scale-deferred-review-v1404.json',
        'typed-api-prototype-v1404-state.json']:
        assert prefix+name not in prop['evidence']
        prop['evidence'].append(prefix+name)
    marker = 'The v1283 isolated checkpoint retains'
    assert prop['remaining'].count(marker) == 1
    retained = (
        'The v1404 isolated checkpoint retains all v1283 scalar/cache/domain/trig, EvaluationBudget, '
        'algebraic, normal-form, inverse-trig DAG, structural comparison, rational preflight and fallible '
        'inverse-constructor repairs. The accepted P4 contract permits Unknown to strengthen into Certified '
        'and reuse prior proofs at zero allowance; warming may neither contradict nor weaken a certificate. '
        'Exact integer leaves and cache rescaling now share a borrowed rounded-right-shift helper. It copies '
        'only retained limbs and returns exact zero before num-bigint can clone a completely discarded input. '
        'Signed halfway rounding remains toward positive infinity; fine clone/left-shift arithmetic uses the '
        'existing verified word plan directly, and already-owned results keep their owned scaling path. '
        'All five corrected frozen regressions and all prior test bodies are unchanged. Two allocation '
        'regressions fail before repair and pass afterward. Sixty-four measured queries span public one-unit '
        'wrappers and internal dispatch, both signs, four widths from 65 to 1000001 bits and four coarse '
        'precisions. Public requests now allocate 40-56 Rust bytes and direct dispatch 0-16, independent of '
        'input width; million-bit baseline copies requested at least 125008 bytes before public scope '
        'overhead. Independent division/remainder oracles cover 1568 signed limb/tie cases per route, and '
        'sixty schedules preserve zero/one/three-unit queries, exact retries and retained signs. The new '
        'baseline sign test was corrected before runtime edits to establish a proof before demanding '
        'zero-work reuse; its failed expectation remains archived. This is an avoidable-copy optimization, '
        'not an allowance-contract violation: EvaluationBudget explicitly excludes individual-operation '
        'bytes/time. Measurements exclude fixture/oracle creation, native allocation, RSS and peak-live memory. '
    )
    tests = metadata['test_summaries']
    validation = (
        f"Final-source checks pass {tests['five_crate']['passed']} five-crate and {tests['default_scalar']['passed']} "
        'default-scalar tests/doctests, each with three separately counted child tests; '
        f"{tests['debug_scalar']['passed']} debug scalar units plus one child; strict Clippy; workspace and "
        f"{metadata['included_rust_fragments_checked']} included-input format checks; 141 capped tests plus one "
        'child; and all 4129 frozen replays. All 1584 focused receipts agree except executable commands, '
        'including work counts, numerical observations and retained proofs. Fourteen inherited Cargo '
        'filename warnings are unchanged. Fresh pinned Verus checks pass 638 conditional units, ten '
        'acceptance tests, 213 rejected mutants and four passing guard baselines. Negative checks were '
        'rerun because the listed representation input changed. The changed scaling/cache implementation '
        'is conditionally excluded; all thirteen whole-crate proof groups remain open. '
    )
    cost = (
        'The first optimization attempt is unretained: an extra nested helper call accompanied paired '
        '+9.65 percent small-integer identity, +13.46 percent atan-wide and +14.28 percent pi-wide observations. '
        'The revised helper removes that call. Its corresponding paired observations are +0.29, -3.34 and '
        '-11.15 percent. Wide coarse, zero-output and halfway fixtures improve 97.28, 99.54 and 54.73 '
        'percent; small fine scaling measures +3.91 percent. The 24 ordinary paired medians range from '
        '-11.15 to +3.87 percent. Standalone probes grow 0.0168-0.0197 percent. All separate trials remain '
        'retained, including revised +28.58 percent terms-wide, +24.42 percent pi-small and +21.08 percent '
        'atan-wide observations; their paired medians are -1.38, -0.01 and -3.34 percent. These local '
        'observations support the scoped optimization, not a uniform speedup or absence of overhead. '
    )
    remaining = (
        'Continue remaining bounded-query constructor call sites, lazy/scaled coefficients, acos endpoint '
        'allocation cutpoints, general unshared form growth, algebraic metadata/vector copies and '
        'individual exact-operation resource use. Fine/output-sized integer allocation and negative-shift '
        'bit scans remain separate costs. Paid bounded Laurent vector work alone is not a resource '
        'violation; preserve the v1283 source review and require measured public reachability. Continue '
        'kernel-specific cutpoints, concurrent/reconstructed and cross-root warming, implementation-connected '
        'proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged '
        'full-board admission. Main retains the original scalar failure and all 49 original worktree '
        'entries. The supplemental deferred-path review identifies cold/coarse-warmed rational acos, '
        'numeric atan, log-range fallbacks and retained quadratic-tower copies for measurement; it '
        'establishes no new failure or byte/time bound. See integer-scale-deferred-review-v1404.json, '
        'integer-scale-resource-followup-v1404.json and typed-api-prototype-v1404-state.json.'
    )
    prop['remaining'] = prop['remaining'][:prop['remaining'].index(marker)]+retained+validation+cost+remaining
    after['next_actions'][0] = (
        'Continue the isolated fallible scalar evaluator from typed-api-prototype-v1404-state.json before '
        'production admission. Preserve all original tests, signed 96-addition graphs, 740-term domain '
        'fixtures, 192000-bit series and the original medium coarse-cache regression. '+retained+validation+cost+remaining+
        ' All 39 properties, 14 targets, crate future work and complete pre-push verification remain in scope.')
    for key in before:
        if key not in {'properties', 'next_actions'}:
            assert before[key] == after[key], key
    for old, new in zip(before['properties'], after['properties']):
        if old['id'] != 'P4':
            assert old == new
        else:
            assert old['requirement'] == new['requirement'] and old['status'] == new['status']
    assert before['next_actions'][1:] == after['next_actions'][1:]
    out = E/'integer-scale-ledger-update-v1404.json'
    assert not out.exists()
    path.write_text(json.dumps(after, indent=2)+'\n')
    out.write_text(json.dumps({'path': str(path.relative_to(ROOT)),
        'before_sha256': hashlib.sha256(original).hexdigest(), 'after_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'changed_scope': ['P4 evidence and remaining work', 'next_actions[0]'], 'properties_preserved': 39,
        'targets_preserved': 14, 'all_statuses_and_future_work_preserved': True,
        'other_property_rows_and_remaining_next_actions_unchanged': True}, indent=2)+'\n')
    print('Updated scoped scalar evidence; complete plan and statuses preserved.')


if __name__ == '__main__':
    main()

