"""Bind the example's executed bytes and replay all five complete outputs."""
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/properties/stl-native-validation'
OUT = WORK / 'example-replay-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


assert json.loads((E / 'application-v3.json').read_text())['format_exit_code'] == 0
OUT.mkdir(exist_ok=False)
source = WORK / 'candidate-target-v2/release/examples/basic_shapes'
binary = OUT / 'basic-shapes'
shutil.copy2(source, binary)
digest = sha(binary)
assert digest == sha(source)
run = OUT / 'run'
run.mkdir()
with (OUT / 'stdout').open('wb') as stdout, (OUT / 'stderr').open('wb') as stderr:
    result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / 'resources'),
                             'timeout', '--signal=TERM', '--kill-after=10s', '120s', str(binary)],
                            cwd=run, env=os.environ | {'TMPDIR': str(OUT)},
                            stdout=stdout, stderr=stderr, preexec_fn=limits, start_new_session=True)
assert result.returncode == 0
assert sha(source) == sha(binary) == digest
outputs = {}
for path in sorted((run / 'stl/examples/basic_shapes').glob('*.stl')):
    original = WORK / 'candidate-source-v2/stl/examples/basic_shapes' / path.name
    assert path.read_bytes() == original.read_bytes() == (E / 'basic-shapes-output-v2' / path.name).read_bytes()
    outputs[path.name] = {'sha256': sha(path), 'bytes': path.stat().st_size}
assert set(outputs) == {'cube.stl', 'sphere.stl', 'cylinder.stl', 'ellipsoid.stl', 'torus.stl'}
record = {'runner_sha256': sha(__file__), 'binary': str(binary), 'binary_sha256': digest,
          'binary_source': str(source), 'exit_code': 0, 'complete_output_bytes_equal': True,
          'outputs': outputs, 'completed_target_modified': False,
          'scope': 'Cargo run logs use backticks, outside the initial test-executable parser. This fresh replay binds the retained executable and all five complete outputs without claiming a hash recorded at the earlier execution.'}
(OUT / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
destination = E / 'example-replay-v1'
destination.mkdir()
for name in ['stdout', 'stderr', 'resources', 'result.json']:
    shutil.copy2(OUT / name, destination / name)
    assert sha(OUT / name) == sha(destination / name)
print('Replayed the retained example binary; all five complete STL outputs match.', flush=True)
