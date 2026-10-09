"""Validate only Point2 documentation and independent regressions over admitted production source."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

repo = Path.cwd()
root = repo / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
baseline = Path('/tmp/drc-normal-form-ownership-isolated')
base_record = root / 'normal-form-ownership-isolated/sources.json'
base = json.loads(base_record.read_text())
work = Path('/tmp/drc-point-equality-regressions')
out = root / 'point-equality-regressions'
work.mkdir(exist_ok=False)
out.mkdir(exist_ok=False)
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(digest(baseline / 'source' / p) == h for p, h in base['source_hashes'].items())
for folder in ['source', 'target']:
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline / folder), str(work / folder)], check=True)
source, target = work / 'source', work / 'target'
changes = ['crates/hypercurve/src/point.rs', 'crates/hypercurve/src/point/equality_tests.rs']
for name in changes:
    text = (repo / name).read_text()
    assert 'cached_rational_interval' not in text
    destination = source / name
    destination.parent.mkdir(exist_ok=True, parents=True)
    destination.write_text(text)
assert subprocess.check_output(['git', 'show', 'HEAD:' + changes[0]]) == (baseline / 'source' / changes[0]).read_bytes()
paths = sorted(set(base['source_hashes']) | set(changes))
hashes = lambda: {p:digest(source / p) for p in paths}
initial = hashes()
assert {p for p, h in initial.items() if base['source_hashes'].get(p) != h} == set(changes)
doc = '///\n/// Equality proves matching coordinates; `false` can also mean that symbolic\n/// equality could not be established. Convert to [`CurvePoint2`](crate::CurvePoint2)\n/// and use [`coincides_with`](crate::CurvePoint2::coincides_with) for a fallible\n/// geometric comparison.\n'
hook = '#[cfg(test)]\nmod equality_tests;\n\n'
point_text = (source / changes[0]).read_text()
assert point_text.count(doc) == 1 and point_text.count(hook) == 1
assert point_text.replace(doc, '').replace(hook, '') == (baseline / 'source' / changes[0]).read_text()
record = dict(scope='Documentation and tests only; executable production source matches the admitted baseline exactly after removing the documentation and cfg(test) declaration. No optimization or application performance admission is claimed.', non_test_executable_source_unchanged=True, production_baseline_revision='73d162ed0', base_source_record=str(base_record),
              only_modified_sources=changes, source_hashes=initial, unchanged=True)
(out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(out / 'source-delta.tar.gz', 'w:gz') as archive:
    for name in changes:
        archive.add(source / name, arcname=name)
shutil.copyfile(__file__, out / 'reproduce.py')
environment = {**os.environ, 'CARGO_HOME':'/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE':'true',
               'CARGO_TARGET_DIR':str(target), 'CCACHE_DIR':'/tmp/csgrs-properties-ccache', 'RUSTDOCFLAGS':'-D warnings'}
record['environment'] = {key:environment[key] for key in ['CARGO_HOME', 'CARGO_NET_OFFLINE', 'CARGO_TARGET_DIR', 'CCACHE_DIR', 'RUSTDOCFLAGS']}
status = []
def run(name, command):
    assert hashes() == initial
    start = time.monotonic()
    with (out / (name + '.log')).open('w') as log:
        code = subprocess.run(command, cwd=source, env=environment, stdout=log, stderr=subprocess.STDOUT).returncode
    unchanged = hashes() == initial
    status.append(dict(name=name, command=command, exit_code=code, elapsed_seconds=time.monotonic() - start))
    (out / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    record['unchanged'] = unchanged
    (out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
    assert unchanged
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
commands = [
    ('format', ['cargo', 'fmt', '--all', '--', '--check']),
    ('point-tests', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'point::', '--', '--test-threads=2']),
    ('public-curve-point', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_curve_point', '--', '--test-threads=2']),
    ('curve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercurve', '--release', '--all-targets', '--', '-D', 'warnings']),
    ('curve-doc', ['cargo', 'doc', '--locked', '-p', 'hypercurve', '--release', '--lib', '--no-deps']),
]
for name, command in commands:
    run(name, command)
assert all(digest(baseline / 'source' / p) == h for p, h in base['source_hashes'].items())
assert all((repo / p).read_bytes() == (source / p).read_bytes() for p in changes)
print('isolated point documentation and regression validation passed; executable source unchanged', flush=True)
