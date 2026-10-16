"""Compare the same file identities using independent physical copies as the lower layer."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]
OUT = E / 'closure-diagnostic-v1/flat-probe-v1'
WORK = Path('/dev/shm/csgrs-native-metadata-isolation/closure-flat-probe-v1')
LOWER = WORK / 'flat-source'
files = {
    'release/libcsgrs.rlib': Path('/dev/shm/csgrs-distribution-contracts/cost-build-v2/upper/release/libcsgrs.rlib'),
    'release/deps/libcsgrs.rlib': Path('/dev/shm/csgrs-polyhedral-construction/candidate-v1/upper/release/deps/libcsgrs.rlib'),
}


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


OUT.mkdir(exist_ok=False)
WORK.mkdir(exist_ok=False)
runner = sha(Path(__file__))
tool = Path(shutil.which('fuse-overlayfs'))
expected = {name: sha(path) for name, path in files.items()}
assert expected == {
    'release/libcsgrs.rlib': '8131217ef1e1b510d615d0963966bfe849f3fc99f3f53fdfd096a4a7f5756e53',
    'release/deps/libcsgrs.rlib': 'a47fb08cd44b228677076ccc2434af55caff700c6beb3f219775e565aa7183bb',
}
LOWER.mkdir()
for name, source in files.items():
    destination = LOWER / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open('rb') as src, destination.open('xb') as dst:
        shutil.copyfileobj(src, dst)
    assert sha(destination) == expected[name]
    assert (source.stat().st_dev, source.stat().st_ino) != (destination.stat().st_dev, destination.stat().st_ino)
assert len({(p.stat().st_dev, p.stat().st_ino) for p in (LOWER / name for name in files)}) == len(files)
assert all(sha(LOWER / name) == digest for name, digest in expected.items())
protocol = {'runner_sha256': runner, 'tool': str(tool), 'tool_sha256': sha(tool),
            'lower': str(LOWER), 'expected_sha256': expected,
            'physical_sources': {name: str(path) for name, path in files.items()},
            'contract': str(E / 'closure-diagnostic-v1/probe-contract-v1.md'),
            'existing_targets_modified': False, 'lower_strategy': 'Independent byte-verified physical copies; no nested lower mount.', 'runtime_admitted': False}
dump(OUT / 'protocol.json', protocol)
observations = []
for label, order in [('root-first', list(files)), ('deps-first', list(reversed(files)))]:
    base = WORK / label
    base.mkdir()
    for name in ['upper', 'work']:
        (base / name).mkdir()
    target = ROOT / 'target/properties/native-metadata-isolation' / ('closure-flat-read-' + label + '-v1')
    target.mkdir(exist_ok=False)
    command = [str(tool), '-o', 'ro,lowerdir=' + str(LOWER) + ',upperdir=' + str(base / 'upper') + ',workdir=' + str(base / 'work'), str(target)]
    with (OUT / (label + '.mount.stdout')).open('wb') as stdout, (OUT / (label + '.mount.stderr')).open('wb') as stderr:
        result = subprocess.run(command, stdout=stdout, stderr=stderr, timeout=30)
    mount = {'label': label, 'command': command, 'exit_code': result.returncode, 'target': str(target),
             'is_mount': target.is_mount(), 'read_only': bool(os.statvfs(target).f_flag & os.ST_RDONLY)}
    dump(OUT / (label + '.mount.json'), mount)
    assert result.returncode == 0 and mount['is_mount'] and mount['read_only']
    for repeat in range(2):
        for name in order:
            path = target / name
            before = path.stat()
            data = path.read_bytes()
            after = path.stat()
            digest = hashlib.sha256(data).hexdigest()
            blob = WORK / (digest + '.rlib')
            if not blob.exists():
                with blob.open('xb') as stream:
                    stream.write(data)
            else:
                assert sha(blob) == digest
            row = {'mount': label, 'repeat': repeat, 'relative_path': name, 'expected_sha256': expected[name],
                   'sha256': digest, 'matches_expected': digest == expected[name], 'bytes_read': len(data),
                   'stat': {key: getattr(before, key) for key in ['st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns']},
                   'identity_stable': (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns,before.st_ctime_ns) == (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns,after.st_ctime_ns),
                   'retained_bytes_path': str(blob)}
            observations.append(row)
            dump(OUT / 'observations.json', observations)
            print(json.dumps(row), flush=True)
assert sha(Path(__file__)) == runner and sha(tool) == protocol['tool_sha256']
assert all(sha(path) == expected[name] for name, path in files.items())
result = {'complete': True, 'observations': len(observations),
          'all_views_match_physical_sources': all(row['matches_expected'] for row in observations),
          'physical_sources_unchanged': True, 'runtime_admitted': False}
dump(OUT / 'result.json', result)
print(json.dumps(result), flush=True)
assert result['all_views_match_physical_sources']
