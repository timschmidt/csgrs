"""Run the new source-fidelity contract against the unchanged admitted baseline."""
import gzip, hashlib, json, os, re, resource, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'baseline-acceptance-v1'
SOURCE = W / 'candidate-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(W / 'candidate-preparation-v1/result.json')['complete']
baseline = read(W / 'reproduction-v1/protocol.json')
readonly = baseline['read_only_library_and_cli_sha256']
assert all(sha(p) == h for p, h in readonly.items())
with gzip.open(W / 'candidate-preparation-v1/source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
libraries = {n: Path(p) for n, p in baseline['libraries'].items()}
deps = libraries['hypercurve'].parent
cli = next(p for p in readonly if Path(p).name == 'hyperdrc')
O.mkdir(exist_ok=False)
env = os.environ | {'CARGO_BIN_EXE_hyperdrc': cli, 'TMPDIR': str(O)}
pins = {str(p): sha(p) for p in [Path(__file__), W / 'contract-v1.md', W / 'candidate-preparation-v1/result.json',
    W / 'candidate-preparation-v1/source-hashes.json.gz', W / 'reproduction-v1/protocol.json']}
dump(O / 'protocol.json', {'input_sha256': pins, 'source_inputs_sha256': inputs,
    'libraries': baseline['libraries'], 'read_only_library_and_cli_sha256': readonly,
    'timeout_seconds_per_command': 1800, 'address_space_bytes': 8 * 1024**3,
    'test_source': str(SOURCE), 'implementation': 'Unchanged a843bf79 qualified libraries, read-only.',
    'completed_targets_reopened': False})
commands = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command):
    begun = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
            'timeout', '--kill-after=20s', '1800s', *command], cwd=O, env=env,
            stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
        'elapsed_seconds': time.monotonic() - begun,
        **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    commands.append(row)
    dump(O / 'commands.json', commands)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return result.returncode

flags = ['rustc', '--edition=2024', '--test', '-D', 'warnings', '-C', 'opt-level=3', '-L', 'dependency=' + str(deps)]
for name, path in libraries.items():
    flags += ['--extern', name + '=' + str(path)]
counts = {}
for name in ['kicad_native_strokes', 'kicad_track_failures']:
    assert run(name + '-compile', [*flags, str(SOURCE / 'crates/hyperdrc/tests' / (name + '.rs')), '-o', str(O / name)]) == 0
    if name == 'kicad_native_strokes':
        rc = run('zero-budget-fresh-process', [str(O / name), 'fresh_zero_budget_load_retains_native_rational_direction_certificates', '--exact', '--nocapture', '--test-threads=1'])
        assert rc in [0, 101, 124]
    rc = run(name, [str(O / name), '--nocapture', '--test-threads=1'])
    assert rc in [0, 101, 124]
    output = (O / (name + '.stdout')).read_text()
    counts[name] = {'exit_code': rc,
        'counts': re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;', output),
        'failed_names': re.findall(r'^test (\S+) \.\.\. FAILED$', output, re.M), 'binary_sha256': sha(O / name)}
assert all(sha(p) == h for p, h in readonly.items())
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
dump(O / 'result.json', {'complete': True, 'counts': counts, 'sources_and_read_only_artifacts_unchanged': True,
    'scope': 'Baseline evidence stops at each test first failure; no later assertion is credited.',
    'main_sources_modified': False, 'full_plan_complete': False})
print(json.dumps({'complete': True, 'counts': {n: r['counts'] for n, r in counts.items()}}), flush=True)
