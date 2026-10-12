"""Record the frozen atan baseline before changing production arithmetic."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    previous = read(E/'typed-api-prototype-v1500-state.json')
    build = E/'typed-api-prototype-v1513-atan-baseline-build'
    source_record = build/'source-hashes.json.gz'
    sources = hashes(source_record)
    old_sources = hashes(E/previous['source_record'])
    changed = sorted(name for name in sources.keys() | old_sources.keys()
                     if sources.get(name) != old_sources.get(name))
    assert changed == ['crates/hyperreal/src/computable/node.rs',
                       'crates/hyperreal/src/computable/node/atan_kernel_tests.rs',
                       'crates/hyperreal/src/computable/node/atan_numeric_probe_tests.rs']
    old_facade = E/'typed-api-prototype-v1497-restored-build/inputs'/changed[0]
    assert (SOURCE/changed[0]).read_text() == old_facade.read_text() + (
        '\ninclude!("node/atan_numeric_probe_tests.rs");\n'
        '\ninclude!("node/atan_kernel_tests.rs");\n')
    for name in changed[1:]:
        assert (SOURCE/name).read_text().startswith('#[cfg(test)]\n')
    assert all(sha(SOURCE/name) == digest for name, digest in sources.items())
    guarded = hashes(E/'candidate-preparation/main-hashes.json.gz')
    assert all(sha(ROOT/name) == digest for name, digest in guarded.items())
    original = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--short', '--untracked-files=all'], cwd=ROOT, text=True).splitlines()
    assert [row for row in status if not row[3:].startswith(str(E.relative_to(ROOT))+'/')] == original
    assert all(row[:2] == '??' for row in status if row[3:].startswith(str(E.relative_to(ROOT))+'/'))

    validations = {}
    for version, label in [(1501, 'atan-numeric-survey-build'),
                           (1507, 'atan-kernel-baseline-build'),
                           (1512, 'atan-baseline-other-unit-tests'),
                           (1513, 'atan-baseline-build')]:
        phase = f'typed-api-prototype-v{version}-{label}'
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['main_unchanged'] and result['source_unchanged']
        validations[phase] = result

    survey_caps = read(E/'atan-numeric-survey-caps-v1506.json')
    survey = []
    for phase, row in survey_caps['tests'].items():
        assert row['result']['exit_code'] == row['cap']['exit_code'] == 0
        assert not row['cap']['wall_timeout']
        lines = [line for line in (E/phase/'stderr.log').read_text().splitlines()
                 if line.startswith('atan numeric survey ')]
        assert len(lines) == 64
        survey.extend(lines)
    assert len(survey) == 320
    warm = [line for line in survey if ', warm=true,' in line]
    assert len(warm) == 160
    assert all(', preparation=Some(Ok(0)),' in line for line in warm)

    kernel_caps = read(E/'atan-kernel-baseline-caps-v1511.json')
    failures = []
    allocations = []
    enclosure_rows = []
    for phase, row in kernel_caps['tests'].items():
        expected = 101 if 'allocations' in row['test'] else 0
        assert row['result']['exit_code'] == row['cap']['exit_code'] == expected
        assert not row['cap']['wall_timeout']
        if expected:
            failures.append({'phase': phase, 'test': row['test'], 'exit_code': expected})
        log = (E/phase/'stderr.log').read_text().splitlines()
        allocations.extend(line for line in log if line.startswith('atan zero-term '))
        enclosure_rows.extend(line for line in log if line.startswith('atan kernel enclosure '))
    assert len(failures) == 2 and len(allocations) == 24 and len(enclosure_rows) == 164
    units = parse_phase(E/'typed-api-prototype-v1512-atan-baseline-other-unit-tests')
    assert units['passed'] == 1061 and units['failed'] == 0 and units['child_process_passed'] == 1
    cost = read(E/'typed-api-prototype-v1514-atan-zero-term-baseline-cost/summary.json')
    assert cost['source_hashes_sha256'] == sha(source_record)
    assert cost['guards_unchanged'] and cost['processes'] == 108
    assert all(len(row['elapsed_ns']) == 9 and len(row['checksums']) == 1
               and len(row['references']) == 1 and len(row['output_bytes']) == 1
               for row in cost['cases'].values())
    assert sha(ROOT/'target/typed-api-prototype-v1514-atan-zero-term-baseline-cost/probe') == cost['binary_sha256']
    assert all(sha(E/'typed-api-prototype-v1514-atan-zero-term-baseline-cost'/name) == digest
               for name, digest in cost['artifacts_sha256'].items())
    out = E/'atan-zero-term-baseline-v1515.json'
    assert not out.exists()
    out.write_text(json.dumps({
        'status': 'Measured isolated baseline; production arithmetic unchanged; no runtime admission',
        'previous_checkpoint': 'typed-api-prototype-v1500-state.json',
        'previous_checkpoint_sha256': sha(E/'typed-api-prototype-v1500-state.json'),
        'source_record': str(source_record.relative_to(E)), 'source_record_sha256': sha(source_record),
        'source_changes': changed, 'production_behavior_unchanged': True,
        'main_guarded_files': len(guarded), 'original_worktree_entries_preserved': len(original),
        'survey_record': 'atan-numeric-survey-caps-v1506.json', 'survey_schedules': survey,
        'warm_preparations': len(warm), 'warm_preparations_all_ok_zero': True,
        'kernel_record': 'atan-kernel-baseline-caps-v1511.json',
        'expected_allocation_failures': failures, 'allocation_rows': allocations,
        'independent_enclosure_rows': enclosure_rows,
        'domain_precision_and_stopped_cases': 32,
        'other_scalar_tests': units,
        'cost_record': 'typed-api-prototype-v1514-atan-zero-term-baseline-cost/summary.json',
        'cost_binary_sha256': cost['binary_sha256'], 'cost_binary_bytes': cost['binary_bytes'],
        'cost_cases': cost['cases'], 'validations': validations,
        'candidate': 'After all existing precision, domain and endpoint guards, return zero when the borrowed denominator magnitude strictly exceeds scaled_one. Its truncated quotient is zero, so the existing loop and final scaling already return zero; avoid the unused square without changing nonzero-path operation order.',
        'limitations': [
            'EvaluationBudget counts cooperative work units; these allocation observations do not establish or violate a per-operation byte/time bound.',
            'The raw IntegralAtan and AtanDeferred integer routes are diagnostic; public rational atan constructs AtanRational instead.',
            'Successful cache-hit retries are checked. Exhausted uncached survey queries are not claimed to have independent unlimited retries.',
            'The full baseline unit run explicitly skips the two newly frozen allocation guards, whose failing runs remain separate evidence.',
            'Cost references detect behavior changes but are not independent mathematical oracles. Exact rational alternating-series enclosures provide separate numeric checks.',
            'The 39 properties, 14 targets and crate future work remain open; no full-stack claim follows from these scalar measurements.'
        ]}, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'schedules': len(survey), 'allocation_rows': len(allocations),
        'enclosures': len(enclosure_rows), 'cost_processes': cost['processes']}))


if __name__ == '__main__':
    main()
