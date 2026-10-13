"""Reject both scaling variants using the closed native measurements."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    native = json.loads((E / 'native-assessment-v1.json').read_text())
    controls = json.loads((E / 'native-controls-assessment-v1.json').read_text())
    scalar = json.loads((E / 'scalar-decision-v1.json').read_text())
    parent = Path(preparation['parent_source'])
    original = json.loads(gzip.decompress((E / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    assert all(sha(parent / name) == digest for name, digest in original.items())
    assert sources() == preparation['main_source_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in preparation['pending_native_sha256'].items())
    for assessment in [native, controls]:
        assert all(sha(E / name) == digest for name, digest in assessment['artifacts_sha256'].items())
    assert all(sha(Path(row['path'])) == row['sha256'] for row in
               json.loads((E / 'native-validation-v1/binaries.json').read_text()).values())
    for row in preparation['variants'].values():
        assert all(sha(Path(row['source']) / name) == digest
                   for name, digest in (original | row['source_changes_sha256']).items())
    for directory in ['cost-v1', 'native-controls-v1']:
        libraries = json.loads((E / directory / 'libraries.json').read_text())
        rows = libraries.values() if directory == 'cost-v1' else [row for entries in libraries.values() for row in entries.values()]
        assert all(sha(Path(row['path'])) == row['sha256'] for row in rows)
        assert all(sha(Path(row['path'])) == row['sha256']
                   for row in json.loads((E / directory / 'binaries.json').read_text()).values())
    assert controls['broad_and_arc_variant_witnesses_identical']
    assert controls['historical_comparison']['all_231_historical_witnesses_identical']
    assert controls['final_coordinate_abort']['fresh_processes'] == 9
    assert controls['final_coordinate_abort']['complete_receipts_identical']
    assert not (E / 'report-replay-v1').exists()
    variants = {}
    for variant in ['right', 'both']:
        row = controls['variants'][variant]
        gates = [gate for gate in native['native_gates'] if gate['variant'] == variant]
        assert len(gates) == 2 and not gates[0]['passed'] and gates[1]['passed']
        assert gates[0]['label'] == 'full-board' and gates[0]['limit_seconds'] == 120
        assert not any(item['faster_blocks'] == 5 for item in row['arc_timing'])
        assert all(item['block_deltas'][field] == [0] * 5 for item in row['arc_memory']
                   for field in ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta'])
        increases = row['memory_increases_in_any_sample']
        assert len(increases) == 4
        assert all(item['block_deltas']['peak_bytes_delta'] == [24] * 5 for item in increases)
        assert all(delta >= 0 for item in row['arc_memory'] for delta in item['block_deltas']['peak_bytes_delta'])
        variants[variant] = {
            'decision': 'rejected-for-runtime-admission',
            'native_tests': [test for test in native['test_counts'] if test['variant'] == variant],
            'original_native_gates': gates,
            'arc_all_five_slower': [{'name': item['name'], 'median_block_ratio': item['median_block_ratio']}
                                    for item in row['arc_slower_in_all_five_blocks']],
            'arc_all_five_faster_count': 0,
            'arc_slower_median_count': sum(item['median_block_ratio'] > 1 for item in row['arc_timing']),
            'broad_all_five_slower': [{'name': item['name'], 'median_block_ratio': item['median_block_ratio']}
                                      for item in row['broad_slower_in_all_five_blocks']],
            'broad_all_five_faster': [{'name': item['name'], 'median_block_ratio': item['median_block_ratio']}
                                      for item in row['broad_timing'] if item['faster_blocks'] == 5],
            'broad_slower_median_count': sum(item['median_block_ratio'] > 1 for item in row['broad_timing']),
            'native_arc_memory': 'All 48 rows have identical allocation calls, requested bytes and retained requested-byte deltas in every sample. The four opaque-translated rows each add 24 peak requested bytes in all five blocks; no row reduces a memory field.',
            'peak_increase_rows': [item['name'] for item in increases],
            'probe_binary_sizes': row['probe_binary_sizes'],
            'scalar_allocation_rows_with_reduction': scalar['variants'][variant]['allocation_rows_with_any_reduction'],
        }
    algebra = parent / 'crates/hyperreal/src/computable/node/algebra.rs'
    ops = parent / 'crates/hyperreal/src/rational/arithmetic/ops.rs'
    queries = parent / 'crates/hyperreal/src/rational/arithmetic/queries_conversion.rs'
    zero_mul = ops.read_text().split('impl<T: AsRef<Rational>> Mul<T> for &Rational {', 1)[1].split('impl<T: AsRef<Rational>> Mul<T> for Rational {', 1)[0]
    assert zero_mul.index('if sign == NoSign') < zero_mul.index('self.is_internally_unreduced()') < zero_mul.index('.retained_product(other)')
    next_step = {
        'parent': str(parent),
        'parent_evidence': 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/decision-v1.json',
        'question': 'In the original coefficient multiplier, how many actually executed linear radical/opaque products are multiplied by an already stored zero rational constant and then discarded?',
        'source_basis': 'The original multiplier forms both linear radical contributions before radical cross-products and later forms opaque linear contributions. insert_radical/insert_atom immediately drop zero coefficients. Rational multiplication returns zero before canonicalization, product-cache lookup or publication when either stored sign is NoSign.',
        'read_source_sha256': {str(path.relative_to(parent)): sha(path) for path in [algebra, ops, queries]},
        'diagnostic_only': 'Fork the unchanged endpoint parent. Use fixed integer counters for accepted coefficient shapes and stored-zero flags, plus actual entry counts at each of the four existing linear-term loops. Count zero scales from Rational::sign/is_zero only, which read the immutable stored sign. Preserve every original operation, early return, owner lifetime, insertion order, cache publication and query. Do not infer executed loop counts solely from input lengths, because later insertion can decline early.',
        'qualification': 'Calibrate reset/disabled behavior, all counter bins and joined worker aggregation; run existing scalar cache frontiers and endpoint regressions, strict Clippy and formatting. Observe the unchanged complete-board assertion once with instrumentation, reporting overhead and every original failure. Counts are coverage, not runtime shares or predicted savings.',
        'runtime_followup': 'Only if actual coverage warrants it, preregister a separate zero-contribution experiment that preserves nonzero arithmetic and original insertion order. Qualify zero/negative/mixed/opaque inputs, owner release, eight/nine-term frontiers, mandatory original retry, cold/warm/cloned facts and abort/budget receipts. Require fixed scalar/native cost controls and original gates before any retention; complete report identity remains an admission requirement.',
        'exclusions': 'Do not inherit either rejected scaling branch, the rejected coefficient merge or the rejected eight-node prepass. The endpoint parent remains unadmitted and all its prior timing and memory tradeoffs remain open.',
    }
    decision = {
        'decision': 'reject-both-scaling-variants-on-native-costs-and-original-board-failures',
        'assessor_sha256': sha(Path(__file__)), 'variants': variants,
        'why': 'Focused scalar allocation savings do not establish native benefit. The fixed native endpoint rows have no allocation savings and no all-five faster timing row for either variant, alongside persistent endpoint and broad-control slowdowns and four peak increases. Both original complete-board worker limits fail. The governing performance-before-memory ladder does not support runtime retention.',
        'preserved_evidence': 'All scalar and native samples, including faster broad rows, remain recorded. Every endpoint witness, all 231 historical broad witnesses and all nine 210-row final-abort receipt sets match. Functional tests, strict Clippy and docs pass. This rejection is a conservative decision on the measured workload, not a proof that every workload is slower or that scalar exactness failed.',
        'complete_reports': {'executed': False, 'prepared_runner': 'replay_reports_v1.py',
                             'reason': 'The closed native costs and original board failures settle rejection before runtime admission. Complete report replay and production CLI size are therefore left unexecuted for these rejected sources. Native finding counts are not claimed as complete report identity.'},
        'sample_set_closed': True, 'sample_extension_planned': False,
        'next_step': next_step,
        'evidence_sha256': {name: sha(E / name) for name in
                           ['correctness-v1.json', 'cost-assessment-v1.json', 'scalar-decision-v1.json',
                            'native-assessment-v1.json', 'native-controls-assessment-v1.json',
                            'native-controls-v1/result.json', 'native-controls-v1/trials.json', 'replay_reports_v1.py']},
        'source_parent_main_and_artifacts_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    ledger = ROOT / 'docs/properties-status.json'
    status = json.loads(ledger.read_text())
    status['current_work']['isolated_coefficient_scaling'].update({
        'status': decision['decision'], 'evidence': str(destination.relative_to(ROOT)),
        'native_cost_evidence': str((E / 'native-controls-assessment-v1.json').relative_to(ROOT)),
        'native_cost_summary': 'Right/both have 10/13 endpoint rows and 18/15 broad rows slower in all five blocks. Neither has an all-five faster endpoint row or endpoint allocation savings. Four opaque-translated phases add 24 peak requested bytes in every sample. Faster broad controls and all scalar savings remain recorded; both original board caps fail.',
        'report_replay': decision['complete_reports'], 'next_step': next_step,
        'runtime_admitted': False,
    })
    status['current_work']['pending_native_geometry']['next_step'] = next_step
    ledger.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'reports_executed': False,
                      'next_parent': str(parent), 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
