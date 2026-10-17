"""Attribute unchanged full projects against closed, hash-pinned libraries."""
import gzip
import hashlib
import json
import os
import resource
import subprocess
import time
import zipfile
from pathlib import Path, PurePosixPath

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = ROOT / 'target/properties/kicad-native-strokes'
E = ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes'
PARENT = ROOT / 'docs/evidence/2026-10-17-native-stroke-completion'
OUT = W / 'stage-observations-v1'
PROBE = W / 'stages_v1.rs'

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

assert not OUT.exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == '7906f9da08ad55ad2f36719a9375ed702f896707'
checkpoint = read(E / 'checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in checkpoint['evidence_files_sha256'].items())
assert sha(ROOT / 'docs/properties-status.json') == checkpoint['ledger_after_sha256']
assert read(E / 'candidate-validation-v4/result.json')['complete']
sources = {
    'baseline': (ROOT / 'target/properties/native-stroke-completion/candidate-source-v5',
                 PARENT / 'candidate-preparation-v5/source-hashes.json.gz',
                 PARENT / 'consumer-validation-v1/native-cli.stdout'),
    'candidate': (OLD / 'candidate-source-v4', E / 'candidate-preparation-v4/source-hashes.json.gz',
                  E / 'candidate-validation-v4/native-cli.stdout')
}
libraries, pins, source_pins = {}, {}, {}
for label, (source, manifest, build) in sources.items():
    inputs = gz(manifest)
    assert all(sha(source / p) == h for p, h in inputs.items())
    source_pins[label] = {'source': str(source), 'manifest': str(manifest), 'manifest_sha256': sha(manifest), 'inputs': inputs}
    artifacts = [json.loads(line) for line in build.read_text().splitlines() if line.startswith('{')]
    libraries[label] = {}
    for name in ['hyperdrc', 'hypercurve', 'hyperlimit', 'serde_json', 'log']:
        paths = {Path(p) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name
                 for p in a['filenames'] if p.endswith('.rlib')}
        assert len(paths) == 1, (label, name, paths)
        libraries[label][name] = paths.pop()
    deps = libraries[label]['hypercurve'].parent
    pins.update({str(p): sha(p) for p in deps.iterdir() if p.suffix in ['.rlib', '.so']})
    pins.update({str(p): sha(p) for p in [*libraries[label].values(), manifest, build]})
pins.update({str(p): sha(p) for p in [PROBE, Path(__file__), W / 'contract-v1.md', E / 'checkpoint-v1.json']})
OUT.mkdir()
projects = OUT / 'projects'
projects.mkdir()
fixtures = []
for label, zip_name, board_name in [('cparti', 'CPArti FPGA dev board.zip', 'CPArti FPGA dev board.kicad_pcb'),
                                   ('hvp109a', 'HVP109A.zip', 'HVP109A.kicad_pcb')]:
    archive = ROOT / 'crates/hyperdrc/docs' / zip_name
    pins[str(archive)] = sha(archive)
    directory = projects / label
    directory.mkdir()
    with zipfile.ZipFile(archive) as z:
        for entry in z.infolist():
            path = PurePosixPath(entry.filename)
            assert not path.is_absolute() and '..' not in path.parts and '\\' not in entry.filename
            dest = directory / str(path)
            if entry.is_dir():
                dest.mkdir(parents=True, exist_ok=True)
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(z.read(entry))
        board_bytes = z.read(board_name)
    board = directory / board_name
    assert board.read_bytes() == board_bytes
    fixture = {'label': label, 'archive': str(archive), 'archive_sha256': sha(archive), 'board': str(board),
               'board_sha256': sha(board), 'board_bytes': len(board_bytes),
               'project_files_sha256': {str(p): sha(p) for p in directory.rglob('*') if p.is_file()}}
    fixtures.append(fixture)
    pins.update(fixture['project_files_sha256'])
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(key, None)
env.update({'TMPDIR': str(OUT), 'RUST_BACKTRACE': '0'})
dump(OUT / 'protocol.json', {
    'main_commit': '7906f9da08ad55ad2f36719a9375ed702f896707', 'read_only_sha256': pins,
    'sources': source_pins, 'libraries': {k: {n: str(p) for n,p in v.items()} for k,v in libraries.items()},
    'fixtures': fixtures, 'cargo_targets_reopened': False, 'main_sources_modified': False,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'address_space_bytes': 8 * 1024**3, 'external_timeout_seconds': 1800,
    'scope': 'Per-board fresh-process stage attribution. Different source sets; not a performance-only comparison, full gate requalification or independent correctness oracle.'
})
rows = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command):
    print('START ' + name, flush=True)
    begin = time.monotonic()
    with (OUT / (name + '.stdout')).open('wb') as out, (OUT / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
            'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
            cwd=OUT, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': r.returncode, 'elapsed_seconds': time.monotonic() - begin,
           **{s + '_sha256': sha(OUT / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(OUT / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return r.returncode

compiled = {}
for label, libs in libraries.items():
    binary = OUT / (label + '-stages')
    flags = ['rustc', '--edition=2024', '-D', 'warnings', '-C', 'opt-level=3', '-C', 'embed-bitcode=no',
             '-L', 'dependency=' + str(libs['hypercurve'].parent)]
    for name, library in libs.items():
        flags += ['--extern', name + '=' + str(library)]
    rc = run(label + '-compile', [*flags, str(PROBE), '-o', str(binary)])
    if not rc:
        compiled[label] = binary
        pins[str(binary)] = sha(binary)
for fixture in fixtures:
    for label, binary in compiled.items():
        name = label + '-' + fixture['label'] + '-groups'
        run(name, [str(binary), fixture['board'], 'groups', str(OUT / (name + '.findings.json'))])
assert all(sha(p) == h for p, h in pins.items())
for record in source_pins.values():
    assert all(sha(Path(record['source']) / p) == h for p, h in record['inputs'].items())
dump(OUT / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'source_and_read_only_inputs_unchanged': True, 'compiled_binary_sha256': {str(p): sha(p) for p in compiled.values()},
    'diagnostic_only': True, 'native_runtime_admitted': False, 'full_plan_complete': False})
print(json.dumps({'complete': True, 'failures': [r['name'] for r in rows if r['exit_code']]}), flush=True)
