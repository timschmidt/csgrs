"""Compile and run public STL topology regressions against the completed physical baseline libraries."""
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
PREP = Path(__file__).resolve().parent
E = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
DEPS = Path('/dev/shm/csgrs-physical-build-requalification/format-target-v1/release/deps')
OUT = PREP / 'baseline-tests-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


assert json.loads((E / 'format-reproduction-v1/result.json').read_text())['complete']
OUT.mkdir(exist_ok=False)
inputs = {str(p): sha(p) for p in DEPS.iterdir() if p.suffix in ['.rlib', '.rmeta', '.so']}
source = PREP / 'probe_v1.rs'
expected = json.loads((PREP / 'test-preparation.json').read_text())['sha256']
assert sha(source) == expected
binary = OUT / 'stl_import_contracts'
command = ['rustc', '--edition=2024', '--crate-name', 'stl_import_contract_baseline_v1', '--test', str(source),
           '-C', 'opt-level=3', '-C', 'debug-assertions=off', '--cfg', 'feature="stl-io"',
           '-L', 'dependency=' + str(DEPS), '-o', str(binary)]
for name in ['csgrs', 'hypermesh', 'hyperlattice']:
    paths = list(DEPS.glob('lib' + name + '.rlib')) if name == 'csgrs' else list(DEPS.glob('lib' + name + '-*.rlib'))
    assert len(paths) == 1, (name, paths)
    command.extend(['--extern', name + '=' + str(paths[0])])
protocol = {'command': command, 'source_sha256': expected, 'runner_sha256': sha(Path(__file__)),
            'library_inputs_sha256': inputs, 'libraries_from_completed_physical_build': True,
            'baseline_result_sha256': sha(E / 'format-reproduction-v1/result.json'),
            'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
            'no_completed_target_modified': True, 'runtime_admitted': False}
dump(OUT / 'protocol.json', protocol)


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


rows = []
for name, argv in [('compile', command), ('test', [str(binary), '--test-threads=1', '--nocapture'])]:
    with (OUT / (name + '.stdout')).open('wb') as stdout, (OUT / (name + '.stderr')).open('wb') as stderr:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / (name + '.resources')),
                                 'timeout', '--signal=TERM', '--kill-after=10s', '120s', *argv],
                                cwd=OUT, stdout=stdout, stderr=stderr, preexec_fn=limits, start_new_session=True)
    rows.append({'name': name, 'command': argv, 'exit_code': result.returncode,
                 **{suffix + '_sha256': sha(OUT / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}})
    if name == 'compile' and result.returncode:
        break
assert sha(source) == expected and all(sha(p) == h for p, h in inputs.items())
record = {'complete': len(rows) == 2, 'commands': rows,
          'inputs_unchanged': True, 'binary_sha256': sha(binary) if binary.exists() else None,
          'runtime_admitted': False}
dump(OUT / 'result.json', record)
print(json.dumps({'complete': record['complete'], 'exits': [r['exit_code'] for r in rows]}), flush=True)
raise SystemExit(rows[-1]['exit_code'])
