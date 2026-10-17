"""Compile matched private-kernel probes without reopening completed Cargo targets."""
import gzip
import hashlib
import json
import os
import resource
import shutil
import subprocess
import sys
import time
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'cost-build-v3'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

assert read(W / 'preflight-comparison-v1/result.json')['all_ordered_exact_rows_equal']
assert read(W / 'baseline-validation-v1/result.json')['cargo_target_closed']
assert read(W / 'candidate-validation-v1/result.json')['cargo_target_closed']
O.mkdir(exist_ok=False)
deps_dir = Path('/tmp/csgrs-circle-support-baseline-v1/release/deps')
dependency_pins = {str(p): sha(p) for p in sorted(deps_dir.iterdir()) if p.is_file() and p.suffix in ['.rlib', '.rmeta', '.so']}
artifacts = [json.loads(s) for s in (W / 'baseline-validation-v1/build.stdout').read_text().splitlines() if s.startswith('{')]
externs = {}
for name in ['hyperreal', 'hypersolve', 'hyperlimit', 'num', 'geo', 'proptest']:
    files = {p for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name
             for p in a['filenames'] if p.endswith('.rlib')}
    assert len(files) == 1
    externs[name] = files.pop()
sources = {}
for label in ['baseline', 'candidate']:
    prep = read(W / (label + '-preparation-v1/result.json'))
    manifest = W / (label + '-preparation-v1/source-hashes.json.gz')
    assert sha(manifest) == prep['source_hashes_sha256']
    with gzip.open(manifest, 'rt') as f:
        prior = json.load(f)
    src = O / (label + '-source')
    src.mkdir()
    for p, h in prior.items():
        if not p.startswith('crates/hypercurve/'):
            continue
        original = W / (label + '-source-v1') / p
        assert sha(original) == h
        dest = src / p
        dest.parent.mkdir(exist_ok=True, parents=True)
        shutil.copy2(original, dest)
    shutil.copy2(W / 'cost_probe_v3.rs', src / 'crates/hypercurve/src/circle_support_cost_probe.rs')
    source_pins = {str(p.relative_to(src)): sha(p) for p in sorted(src.rglob('*')) if p.is_file()}
    assert source_pins['crates/hypercurve/src/curve_intersection.rs'] == prior['crates/hypercurve/src/curve_intersection.rs']
    dump(O / (label + '-source-hashes.json'), source_pins)
    sources[label] = {'path': str(src), 'sha256': source_pins}
pins = {str(p): sha(p) for p in [Path(__file__), W / 'cost_probe_v3.rs', W / 'preflight-comparison-v1/result.json']}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'read_only_dependencies_sha256': dependency_pins, 'externs': externs,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True), 'sources': sources,
    'features': [], 'same_dependency_libraries_for_both_variants': True,
    'completed_cargo_targets_reopened': False, 'production_source_bytes_unchanged_from_v1': True,
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'diagnostic_not_a_cargo_release_build': True,
})
rows = []
binaries = {}

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

for label, source in sources.items():
    src = Path(source['path'])
    binary = O / (label + '-tests')
    cmd = ['rustc', '--crate-name', 'hypercurve', '--edition=2024', '--test', str(src / 'crates/hypercurve/src/lib.rs'),
           '-C', 'opt-level=3', '-C', 'codegen-units=16', '-C', 'embed-bitcode=no', '-D', 'warnings',
           '--remap-path-prefix', str(src) + '=/csgrs-circle-support-cost-v3',
           '-L', 'dependency=' + str(deps_dir), '-o', str(binary)]
    for name, path in externs.items():
        cmd += ['--extern', name + '=' + path]
    print('START ' + label + '-compile', flush=True)
    start = time.monotonic()
    env = os.environ.copy()
    for k in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER']:
        env.pop(k, None)
    env['TMPDIR'] = str(O)
    with (O / (label + '.stdout')).open('wb') as out, (O / (label + '.stderr')).open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (label + '.resources')), 'timeout', '--kill-after=20s', '1800s', *cmd],
                                cwd=src, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    rows.append({'name': label, 'command': cmd, 'exit_code': result.returncode, 'elapsed_seconds': time.monotonic() - start,
                 **{s + '_sha256': sha(O / (label + '.' + s)) for s in ['stdout', 'stderr', 'resources']}})
    dump(O / 'commands.json', rows)
    print(json.dumps({k: rows[-1][k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    if result.returncode == 0:
        binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
assert all(sha(p) == h for p, h in dependency_pins.items())
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(Path(s['path']) / p) == h for s in sources.values() for p, h in s['sha256'].items())
dump(O / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
                       'binaries': binaries, 'source_and_read_only_dependencies_unchanged': True,
                       'native_runtime_admitted': False})
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
