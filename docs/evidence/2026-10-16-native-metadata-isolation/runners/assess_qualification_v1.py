"""Classify every completed qualification command without masking failures."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert not (E / 'qualification-assessment-v1.json').exists()
phases = {}
for name in ['baseline-v1', 'candidate-v1']:
    folder = E / name
    protocol = json.loads((folder / 'protocol.json').read_text())
    result = json.loads((folder / 'result.json').read_text())
    specification = json.loads((E / (name + '.json')).read_text())
    assert protocol['commands'] == specification
    assert [row['name'] for row in result['commands']] == [row[0] for row in specification]
    assert all(row['sources_unchanged'] for row in result['commands'])
    assert sha(E / 'validate_v1.py') == protocol['wrapper_sha256']
    with gzip.open(folder / 'source-hashes.json.gz', 'rt') as stream:
        assert all(sha(ROOT / path) == digest for path, digest in json.load(stream).items())
    commands = []
    for row in result['commands']:
        for stream in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + stream)) == row[stream + '_sha256']
        stdout = (folder / (row['name'] + '.stdout')).read_text()
        summaries = re.findall(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored;', stdout)
        commands.append({
            'name': row['name'], 'exit_code': row['exit_code'],
            'whole_command_seconds': row['seconds'],
            'test_summary_count': len(summaries),
            'tests_passed': sum(int(s[1]) for s in summaries),
            'tests_failed': sum(int(s[2]) for s in summaries),
            'tests_ignored': sum(int(s[3]) for s in summaries),
        })
    phases[name] = {'passed': result['passed'], 'commands': commands,
                    'result_sha256': sha(folder / 'result.json')}
gates = {}
for label, metric, limit, pattern in [
    ('full-board', 'worker CPU seconds', 120,
     r'complex project KiCad smoke suite took ([0-9.]+)s of worker time'),
    ('copper-neck', 'check-body wall seconds', 60,
     r'complex project copper neck regression fixture took ([0-9.]+)s'),
    ('gerber', 'original smoke assertions', None, None),
]:
    folder = E / 'candidate-v1'
    row = next(row for row in phases['candidate-v1']['commands'] if row['name'] == label)
    stdout = (folder / (label + '.stdout')).read_text()
    stderr = (folder / (label + '.stderr')).read_text()
    assert 'skipping complex project' not in stdout + stderr
    assert re.search(r'test result: (?:ok|FAILED)\. (?:1 passed; 0 failed|0 passed; 1 failed)', stdout)
    match = re.search(pattern, stderr) if pattern else None
    gates[label] = {'passed': row['exit_code'] == 0, 'exit_code': row['exit_code'],
                    'metric': metric, 'original_limit': limit,
                    'reported_assertion_seconds': float(match[1]) if match else None,
                    'whole_command_seconds': row['whole_command_seconds']}
    if label == 'full-board':
        assert stderr.count('hyperdrc: finished emit report') == 2
        gates[label]['reported_findings'] = [int(n) for n in re.findall(
            r'hyperdrc: finished run \((\d+) active finding\(s\)', stderr)]
        assert gates[label]['reported_findings'] == [12, 197]
preparation = json.loads((E / 'preparation-v1.json').read_text())
assert all(sha(ROOT / p) == h for p, h in preparation['pending_native_sha256'].items())
record = {
    'status': 'current isolated qualification recorded; cost and full report gates separate',
    'phases': phases, 'original_native_gates': gates,
    'pending_main_native_files_unchanged': 35,
    'count_scope': 'Counts describe each command. Feature configurations and focused subsets overlap; do not add them as distinct tests.',
    'timing_scope': 'Compilation and setup are included in whole-command durations, which are not the original internal assertion metrics. Standalone runs do not establish a matched application speedup.',
    'semantic_scope': 'Findings and test counts alone are not independent geometry oracles. The retained tests and complete report comparison have separate claims.',
    'runtime_admitted': False, 'full_plan_complete': False,
}
(E / 'qualification-assessment-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'phases': {k: v['passed'] for k, v in phases.items()},
                  'gates': gates, 'runtime_admitted': False}), flush=True)
