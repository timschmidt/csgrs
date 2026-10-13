"""Assess the completed correctness run and unchanged native assertions."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'validation-v1'
SOURCE = ROOT / 'target/endpoint-coordinate-reuse/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    preparation = json.loads((E / 'preparation.json').read_text())
    assert result['complete']
    assert sha(E / 'validate_v1.py') == protocol['runner_sha256']
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    current = original | preparation['source_changes_sha256']
    assert all(sha(SOURCE / name) == digest for name, digest in current.items())
    assert sources() == preparation['main_source_sha256']
    identity = json.loads((OUT / 'binary.json').read_text())
    assert sha(Path(identity['path'])) == identity['sha256']
    phases = {row['name']: row for row in result['phases']}
    for name, phase in phases.items():
        assert phase['sources_unchanged']
        if name not in ['full-board', 'copper-neck']:
            assert phase['exit_code'] == 0
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (name + '.' + stream)) == phase[stream + '_sha256']
    counts = {}
    for name in ['endpoint-regressions', 'curve-tests', 'drc-regressions']:
        matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (OUT / (name + '.stdout')).read_text())
        assert matches
        counts[name] = {'summaries': len(matches), 'passed': sum(int(row[0]) for row in matches),
                        'failed': sum(int(row[1]) for row in matches), 'ignored': sum(int(row[2]) for row in matches)}
    assert counts['endpoint-regressions']['passed'] == 5
    assert counts['curve-tests']['passed'] == 2494
    assert counts['drc-regressions']['passed'] == 26
    gates = []
    for gate in protocol['gates']:
        label = gate['label']
        stderr = (OUT / (label + '.stderr')).read_text()
        if label == 'full-board':
            assert stderr.count('finished emit report') == 2
            assert [int(value) for value in re.findall(r'finished run \((\d+) active finding', stderr)] == [12, 197]
            match = re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', stderr)
        else:
            match = re.search(r'complex project copper neck regression fixture took ([0-9.]+)s', stderr)
        passed = phases[label]['exit_code'] == 0
        gates.append(gate | {'passed': passed, 'observed_assertion_seconds': float(match[1]) if match else None,
                             'process_wall_seconds': phases[label]['seconds'],
                             'timing_note': 'Process wall time includes setup and is not the asserted metric. Passing assertions need not print the measured interval.'})
        if not passed:
            assert match and float(match[1]) > gate['limit_seconds']
    assessment = {
        'decision': 'isolated-correctness-passed-cost-and-report-qualification-pending',
        'source_delta': 'A private tri-state point helper preserves the original lazy coordinate facts. Endpoint classification reuses only certified negatives in exact non-preview contexts; all unknown retries, approximate/preview retries and later whole endpoint calls remain. Global PartialEq is unchanged. Five regressions and accurate inherited two-pass recognition-budget documentation are added; no layouts or limits change.',
        'test_counts': counts,
        'count_overlap': 'The five endpoint regressions are included again in the full curve suite; do not add them to its total.',
        'strict_curve_and_drc_clippy_passed': True,
        'warning_denied_scalar_and_curve_docs_passed': True,
        'workspace_formatting_passed': True,
        'native_gates': gates,
        'all_original_native_gates_passed': all(gate['passed'] for gate in gates),
        'limits': [
            'The complete curve suite and 26 focused DRC regressions ran. This is not the complete DRC suite or every workflow combination.',
            'Observed original-board counts are 12 and 197. Complete report bytes are not compared by these test runs.',
            'These standalone original-cap results are not a matched application-speedup comparison.',
            'Five regression functions check named coordinate, endpoint, policy, clone, abort and shared-budget scenarios. They do not establish universal cache or equality completeness.',
        ],
        'next_step': 'Run the fixed paired public arc-cost probe, all 231 historical capability witnesses, effective final-coordinate abort receipts and complete unnormalized report replay from separate source/target directories. Preserve all original limits, failed gates and control timing tradeoffs.',
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'validation-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    work = status['current_work']
    work['isolated_endpoint_coordinate_reuse'] = {
        'status': assessment['decision'],
        'evidence': str((E / 'validation-assessment-v1.json').relative_to(ROOT)),
        'validation': counts,
        'native_gates': gates,
        'next_step': assessment['next_step'],
        'runtime_admitted': False,
    }
    work['pending_native_geometry']['next_step'] = assessment['next_step']
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'counts': counts, 'gates': gates}))


if __name__ == '__main__':
    main()
