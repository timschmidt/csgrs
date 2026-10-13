"""Check existing-bound-sign observations without interpreting counts as costs."""
import hashlib
import json
from pathlib import Path
import re

from diagnose_v2 import parse_counts, verify

E = Path(__file__).resolve().parent
OUT = E / 'diagnostic-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'assessment-v2.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v2/protocol.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    result = json.loads((OUT / 'result.json').read_text())
    binary = json.loads((OUT / 'binary.json').read_text())
    verify(preparation, protocol, binary)
    assert result['complete'] and result['source_parent_main_and_binary_unchanged']
    phases = result['phases']
    assert phases == json.loads((OUT / 'commands.json').read_text())
    assert [row['name'] for row in phases] == [row[0] for row in protocol['commands']] + ['full-board']
    assert len(phases) == 14
    for phase in phases:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    tests = []
    for (name, command, expected), phase in zip(protocol['commands'], phases):
        assert phase['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command]
        assert phase['cwd'] == preparation['reference_source' if name == 'parent-reference' else 'source']
        assert phase['exit_code'] == 0
        if expected is not None:
            raw = (OUT / (name + '.stdout')).read_text()
            assert re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', raw) == [(str(expected), '0', '0')]
            tests.append({'name': name, 'passed': expected, 'failed': 0, 'ignored': 0})
    assert tests == result['tests'] == json.loads((OUT / 'tests.json').read_text())
    assert Path(binary['path']).is_relative_to(Path(preparation['target']))
    assert Path(binary['manifest_path']) == Path(preparation['source']) / 'crates/hyperdrc/Cargo.toml'
    assert binary['target']['kind'] == ['lib'] and binary['profile']['test']
    assert phases[-1]['command'] == ['timeout', '--signal=TERM', '--kill-after=10s', '1800', binary['path'], protocol['gate_test'], '--exact', '--test-threads=1', '--nocapture']
    assert phases[-1]['cwd'] == preparation['source']
    both = (OUT / 'full-board.stdout').read_text() + (OUT / 'full-board.stderr').read_text()
    counts = parse_counts(both)
    assert counts == result['counts'] == json.loads((OUT / 'counts.json').read_text())
    assert protocol['original_worker_cpu_limit_seconds'] == result['original_worker_cpu_limit_seconds'] == 120
    assert both.count('hyperdrc: finished emit report') == 2
    assert [int(x) for x in re.findall(r'finished run \((\d+) active finding', both)] == [12, 197]
    elapsed = re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', both)
    assert result['original_gate_passed'] == (phases[-1]['exit_code'] == 0)
    if not result['original_gate_passed']:
        assert phases[-1]['exit_code'] == 101 and elapsed
        assert float(elapsed[1]) == result['instrumented_worker_cpu_seconds'] >= 120
    summary = {k: v for k, v in counts.items() if k != 'rows'}
    summary['eligible_percent_of_started'] = 100 * counts['eligible_known_sign'] / counts['started']
    assessment = {
        'status': 'bound-sign-fallback-coverage-observed-await-review',
        'assessor_sha256': sha(Path(__file__)), 'summary': summary, 'counts': counts,
        'tests': tests, 'format_strict_clippy_and_docs_passed': True,
        'original_gate_passed': result['original_gate_passed'],
        'instrumented_worker_cpu_seconds': result['instrumented_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120, 'finding_counts': [12, 197],
        'limits': [
            'These are counts of existing operations under instrumentation, not time shares, allocations or projected savings.',
            'The only new fact inspection reads the already-stored root bound. It performs no cheap_bound traversal, scalar comparison or new recognizer. The original fallback and result publication still execute. Nonzero without a sign is not eligible; anomalous nonzero/zero-sign metadata is retained separately.',
            'The full-board assertion retains its original worker CPU metric and limit. Instrumentation changes execution cost and is not a benchmark of an optimization.',
            'The 12/197 finding counts do not establish complete report-byte identity.',
            'Focused test totals overlap the full scalar library. This is not the complete workflow/feature/target matrix.',
            'The parent remains unadmitted. All native tradeoffs and original limits, all 35 pending entries and the entire properties/targets/future-work plan remain open.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'source_parent_main_and_binary_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'summary': summary, 'gate_passed': result['original_gate_passed'], 'tests': tests}))


if __name__ == '__main__':
    main()
