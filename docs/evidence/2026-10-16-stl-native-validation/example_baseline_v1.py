"""Reproduce the existing example feature mismatch before changing its manifest."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-stl-native-validation'
WORK = ROOT / 'target/properties/stl-native-validation'
OUT = WORK / 'example-baseline-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


OUT.mkdir(exist_ok=False)
baseline = json.loads((WORK / 'baseline-v1/protocol.json').read_text())
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == baseline['head']
inputs = {str(ROOT / name): sha(ROOT / name) for name in ['Cargo.toml', 'examples/basic_shapes.rs']}
for row in baseline['bindings'].values():
    assert sha(row['retained']) == row['sha256']
command = ['rustc', '--edition=2024', '-C', 'opt-level=3', '-C', 'debug-assertions=off',
           '-L', 'dependency=' + baseline['dependencies']]
for crate, row in baseline['bindings'].items():
    command += ['--extern', crate + '=' + row['retained']]
command += [str(ROOT / 'examples/basic_shapes.rs'), '-o', str(OUT / 'basic-shapes')]
result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '120s', *command], cwd=ROOT, capture_output=True)
(OUT / 'compile.stdout').write_bytes(result.stdout)
(OUT / 'compile.stderr').write_bytes(result.stderr)
assert result.returncode == 1 and b'cannot find function `torus` in module `solid`' in result.stderr
assert all(sha(path) == digest for path, digest in inputs.items())
assert all(sha(row['retained']) == row['sha256'] for row in baseline['bindings'].values())
record = {'runner_sha256': sha(__file__), 'head': baseline['head'], 'input_hashes': inputs,
          'bindings': baseline['bindings'], 'command': command, 'exit_code': result.returncode,
          'stdout_sha256': sha(OUT / 'compile.stdout'), 'stderr_sha256': sha(OUT / 'compile.stderr'),
          'reproduced_existing_failure': 'basic_shapes requires curve for solid::torus, but Cargo declares only stl-io.',
          'main_sources_modified': False, 'completed_targets_modified': False}
(OUT / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(record['reproduced_existing_failure'], flush=True)
