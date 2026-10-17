"""Compare full deterministic CLI reports with the same complete project inputs."""
import gzip
import hashlib
import json
import resource
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'project-reports-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

q = W / 'candidate-qualification-v2'
assert read(q / 'result.json')['command_failures'] == [] and read(q / 'result.json')['cargo_target_closed']
old = ROOT / 'docs/evidence/2026-10-17-native-stroke-completion/project-reports-v1/protocol.json'
prior = read(old)
baseline = ROOT / 'target/properties/native-stroke-completion/consumer-target-v1/release/hyperdrc'
assert sha(baseline) == prior['input_sha256'][str(baseline)]
commands = read(q / 'commands.json')
cli = next(r for r in commands if r['name'] == 'native-cli')
matches = [b for b in cli['binaries'].values() if b['target']['name'] == 'hyperdrc' and not b['profile']['test']]
assert len(matches) == 1
candidate = Path(matches[0]['retained'])
assert sha(candidate) == matches[0]['sha256']
source_protocol = read(ROOT / 'docs/evidence/2026-10-17-native-track-cost-attribution/stage-observations-v1/protocol.json')
old_inputs = source_protocol['sources']['baseline']['inputs']
with gzip.open(W / 'baseline-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    current_inputs = json.load(f)
differences = {p: {'baseline': h, 'current': current_inputs[p]} for p, h in old_inputs.items() if current_inputs[p] != h}
admitted = read(ROOT / 'docs/evidence/2026-10-17-native-stroke-completion/checkpoint-v1.json')['implementation_sha256']
expected = {p for p in admitted if p.startswith('crates/hypercircuit/tests/fixtures/')}
assert len(expected) == 7 and set(differences) == expected | {'docs/properties-status.json'}
assert all(differences[p]['current'] == admitted[p] for p in expected)
assert all(sha(p) == record['sha256'] for p, record in prior['project_files'].items())
boards = {}
for fixture in prior['original_archives']:
    label = 'cparti' if 'CPArti' in fixture['entry'] else 'hvp109a'
    paths = [Path(p) for p in prior['project_files'] if Path(p).name == fixture['entry']]
    assert len(paths) == 1 and sha(paths[0]) == fixture['entry_sha256']
    boards[label] = paths[0]
O.mkdir(exist_ok=False)
pins = {str(p): sha(p) for p in [Path(__file__), old, baseline, candidate, q / 'result.json', q / 'commands.json']}
dump(O / 'protocol.json', {'input_sha256': pins, 'project_files': prior['project_files'],
    'baseline_source_differences': differences, 'native_baseline_implementation_inputs_identical': True,
    'cache': 'Independent process per complete project and variant; every original project file is read-only.',
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'performance_comparison_claimed': False, 'purpose': 'Preserve report values, order, serialized bytes and checksums; this is not an independent source-fidelity oracle.'})
rows = []
reports = {}

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

for label, board in boards.items():
    for variant, binary in [('baseline', baseline), ('candidate', candidate)]:
        name = label + '-' + variant
        command = [str(binary), '--kicad-pcb', str(board), '--check', 'layer-sanity', '--check', 'board-outline-sanity',
                   '--check', 'min-copper-neck', '--check', 'drill-spacing', '--min-width', '0.0762', '--format', 'json', '--allow-findings']
        print('START ' + name, flush=True)
        start = time.monotonic()
        with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
            result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')), 'timeout', '--kill-after=20s', '1800s', *command],
                                    cwd=O, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
        rows.append({'name': name, 'command': command, 'exit_code': result.returncode, 'elapsed_seconds': time.monotonic() - start,
                     **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}})
        dump(O / 'commands.json', rows)
        assert result.returncode == 0
    a, b = [O / (label + '-' + v + '.stdout') for v in ['baseline', 'candidate']]
    assert read(a) == read(b) and a.read_bytes() == b.read_bytes()
    reports[label] = {'all_values_and_order_identical': True, 'serialized_bytes_identical': True,
                      'sha256': sha(a), 'bytes': a.stat().st_size, 'violation_count': read(a)['violation_count']}
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(p) == r['sha256'] for p, r in prior['project_files'].items())
dump(O / 'result.json', {'complete': True, 'command_failures': [], 'all_report_bytes_identical': True,
                       'reports': reports, 'read_only_inputs_unchanged': True, 'performance_comparison_claimed': False})
print(json.dumps(reports), flush=True)
