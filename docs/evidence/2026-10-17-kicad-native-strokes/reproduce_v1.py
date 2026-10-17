"""Reproduce both current source contracts and the unmodified archived frontier."""
import gzip, hashlib, json, os, re, resource, shutil, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = ROOT / 'target/properties/native-stroke-completion'
E = ROOT / 'docs/evidence/2026-10-17-native-stroke-completion'
BASE = 'a843bf79dfe329fb51ababaaf803752bb74a0f64'
O = W / 'reproduction-v1'
P = W / 'probe-source-v1'
SOURCE = W / 'baseline-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def gz(p):
    with gzip.open(p, 'rt') as f:
        return json.load(f)

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
parent = read(E / 'checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in parent['evidence_files_sha256'].items())
inputs = gz(E / 'final-input-hashes-v1.json.gz')
inputs['docs/properties-status.json'] = parent['ledger_after_sha256']
assert len(inputs) == 2835 and all(sha(ROOT / p) == h for p, h in inputs.items())
compiled = gz(OLD / 'candidate-preparation-v5/source-hashes.json.gz')
assert inputs.keys() == compiled.keys()
delta = {p for p in inputs if inputs[p] != compiled[p]}
assert delta == {'docs/properties-status.json'} | set(read(OLD / 'candidate-preparation-v6/result.json')['delta_from_v5'])
assert all(p == 'docs/properties-status.json' or p.startswith('crates/hypercircuit/tests/fixtures/') for p in delta)
assert all(sha(OLD / 'candidate-source-v5' / p) == h for p, h in compiled.items())
native = read(OLD / 'consumer-validation-v1/result.json')
assert native['complete'] and native['unexpected_exits'] == ['circuit-all']
build = OLD / 'consumer-validation-v1/native-cli.stdout'
build_row = next(r for r in read(OLD / 'consumer-validation-v1/commands.json') if r['name'] == 'native-cli')
assert build_row['exit_code'] == 0 and sha(build) == build_row['stdout_sha256']
artifacts = [json.loads(s) for s in build.read_text().splitlines() if s.startswith('{')]
libraries = {name: next(Path(p) for row in artifacts if row.get('reason') == 'compiler-artifact'
    and row['target']['name'] == name for p in row['filenames'] if p.endswith('.rlib'))
    for name in ['hypercurve', 'hyperlimit', 'hyperreal', 'hyperdrc', 'anyhow']}
cli = next(Path(row['executable']) for row in artifacts if row.get('reason') == 'compiler-artifact'
    and row['target']['name'] == 'hyperdrc' and row.get('executable'))
deps = libraries['hypercurve'].parent
paths = {cli, *libraries.values(), *deps.glob('*.rlib'), *deps.glob('*.so')}
expected = gz(ROOT / parent['artifact_expectations'])['added_physical_artifacts_sha256']
readonly = {str(p): sha(p) for p in paths}
assert all(expected[p] == h for p, h in readonly.items())
for directory in [O, P, SOURCE]:
    directory.mkdir(exist_ok=False)
for name, h in inputs.items():
    dest = SOURCE / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, dest)
    assert sha(dest) == h
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f:
    json.dump(inputs, f, sort_keys=True)
archive = ROOT / parent['candidate_snapshot']
tests = {'archived-frontier': archive / 'crates/hyperdrc/tests/native_linear_strokes.rs',
    'current-source-failures': ROOT / 'crates/hyperdrc/tests/kicad_track_failures.rs'}
for name, path in tests.items():
    shutil.copy2(path, P / (name + '.rs'))
    assert sha(path) == sha(P / (name + '.rs'))
pins = {str(p): sha(p) for p in [Path(__file__), W / 'contract-v1.md', *P.iterdir(),
    E / 'checkpoint-v1.json', OLD / 'consumer-validation-v1/result.json', build]}
env = os.environ.copy()
env['CARGO_BIN_EXE_hyperdrc'] = str(cli)
env['TMPDIR'] = str(O)
dump(O / 'protocol.json', {'base_commit': BASE, 'source': str(SOURCE),
    'source_hashes_sha256': sha(O / 'source-hashes.json.gz'), 'input_sha256': pins,
    'read_only_library_and_cli_sha256': readonly, 'libraries': {n: str(p) for n, p in libraries.items()},
    'compiled_source_differences': sorted(delta), 'differences_outside_native_dependency_code': True,
    'qualification': 'Completed native-cli build, all current source inputs identical except the ledger and seven Hypercircuit expected artifacts. Retained physical hashes verified before read-only reuse.',
    'timeout_seconds_per_command': 1800, 'address_space_bytes': 8 * 1024**3,
    'completed_targets_reopened': False, 'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True)})
rows = []

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
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return result.returncode

flags = ['rustc', '--edition=2024', '--test', '-D', 'warnings', '-C', 'opt-level=3', '-L', 'dependency=' + str(deps)]
for name, path in libraries.items():
    flags += ['--extern', name + '=' + str(path)]
counts = {}
for name in tests:
    assert run(name + '-compile', [*flags, '--crate-name', name.replace('-', '_'), str(P / (name + '.rs')), '-o', str(O / name)]) == 0
    rc = run(name, [str(O / name), '--nocapture', '--test-threads=1'])
    assert rc in [0, 101, 124]
    output = (O / (name + '.stdout')).read_text()
    matches = re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;', output)
    counts[name] = {'exit_code': rc, 'counts': matches,
        'passed_names': re.findall(r'^test (\S+) \.\.\. ok$', output, re.M),
        'failed_names': re.findall(r'^test (\S+) \.\.\. FAILED$', output, re.M),
        'binary_sha256': sha(O / name)}
assert all(sha(p) == h for p, h in readonly.items())
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(ROOT / p) == sha(SOURCE / p) == h for p, h in inputs.items())
dump(O / 'result.json', {'complete': True, 'base_commit': BASE, 'counts': counts,
    'source_inputs': len(inputs), 'all_sources_and_read_only_artifacts_unchanged': True,
    'test_sources_unmodified': True, 'main_sources_modified': False,
    'scope': 'Each regression stops at its first failure; later assertions are not credited.',
    'full_plan_complete': False})
print(json.dumps({'complete': True, 'counts': {n: r['counts'] for n, r in counts.items()}}), flush=True)
