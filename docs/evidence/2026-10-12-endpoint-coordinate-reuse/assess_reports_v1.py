"""Verify complete report identity and production executable sizes."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'report-replay-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    preparation = json.loads((E / 'preparation.json').read_text())
    assert result['complete'] and result['inputs_and_sources_unchanged']
    assert sha(E / 'replay_reports_v1.py') == protocol['runner_sha256']
    assert sources() == preparation['main_source_sha256']
    assert all(sha(Path(value['path'])) == value['sha256'] for value in binaries.values())
    commands = json.loads((OUT / 'commands.json').read_text())
    processes = []
    for command in commands:
        name = command['name']
        stderr = OUT / (name + '.stderr')
        assert sha(stderr) == command['stderr_sha256']
        if name.startswith('build-'):
            assert command['exit_code'] == 0
            assert sha(OUT / (name + '.stdout')) == command['stdout_sha256']
            continue
        data = gzip.decompress((OUT / (name + '.json.gz')).read_bytes())
        assert hashlib.sha256(data).hexdigest() == command['stdout_sha256']
        assert len(data) == command['stdout_bytes']
        report = json.loads(data)
        assert command['exit_code'] == 1
        assert report['violation_count'] == len(report['violations'])
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', stderr.read_text())
        assert rss
        processes.append({'name': name, 'process_seconds': command['seconds'],
                          'maximum_rss_kib': int(rss[1]), 'findings': report['violation_count']})
    assert len(processes) == 4
    for comparison in result['comparisons']:
        label = comparison['fixture']
        baseline = gzip.decompress((OUT / (label + '-baseline.json.gz')).read_bytes())
        candidate = gzip.decompress((OUT / (label + '-candidate.json.gz')).read_bytes())
        assert (baseline == candidate) == comparison['complete_bytes_identical']
        assert (json.loads(baseline) == json.loads(candidate)) == comparison['complete_json_values_identical']
    assessment = {
        'decision': 'complete-report-replay-recorded',
        'complete_reports_byte_identical': result['all_reports_byte_identical'],
        'complete_report_values_identical': result['all_report_values_identical'],
        'comparisons': result['comparisons'],
        'production_binaries': binaries,
        'production_binary_byte_delta': binaries['candidate']['bytes'] - binaries['baseline']['bytes'],
        'process_observations': processes,
        'limits': [
            'Both complete original boards use shared absolute paths and all original input bytes. No report fields are normalized or omitted.',
            'Active findings explain CLI exit code 1; JSON structure, complete bytes and counts are checked independently.',
            'Four standalone CLI executions are report verification, not a fixed repeated application benchmark or a substitute for either original runtime assertion.',
            'Process RSS includes fixture import, setup and reports; it does not isolate constructor allocations or prove graph reclamation.',
            'Different replay-directory paths change report bytes relative to historical replay hashes; comparisons here use the same paths for both variants.',
        ],
        'artifacts_sha256': {str(path.relative_to(E)): sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'report-assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in ['complete_reports_byte_identical',
                      'production_binary_byte_delta', 'comparisons', 'process_observations']}))


if __name__ == '__main__':
    main()
