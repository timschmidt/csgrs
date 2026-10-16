"""Exercise file identity and executable loading on a fresh physical build filesystem."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
WORK = Path('/home/tim/Documents/GitHub/workspace/csgrs/target/properties/wasm-format-failures')
OUT = E / 'storage-v1.json'
assert not OUT.exists()
WORK.mkdir(exist_ok=False)
BASE = WORK / 'storage-v1'
BASE.mkdir()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


rows = []
for label, order in [('root-first', ['root', 'deps']), ('deps-first', ['deps', 'root'])]:
    folder = BASE / label
    folder.mkdir()
    root, deps, replacement = [folder / name for name in ['root', 'deps', 'replacement']]
    root.write_bytes(b'original library\x00' * 4097)
    os.link(root, deps)
    assert root.stat().st_ino == deps.stat().st_ino
    replacement.write_bytes(b'replaced dependency\xff' * 1539)
    expected = {'root': digest(root), 'deps': digest(replacement)}
    os.replace(replacement, deps)
    assert root.stat().st_ino != deps.stat().st_ino
    for repeat in range(2):
        for name in order:
            path = folder / name
            actual = digest(path)
            rows.append({'order': label, 'repeat': repeat, 'path': str(path),
                         'expected_sha256': expected[name], 'sha256': actual})
            assert actual == expected[name]
    deps.write_bytes(b'truncated dependency')
    assert digest(root) == expected['root'] and deps.read_bytes() == b'truncated dependency'
# Copy a system executable to test this directory's actual execution permissions.
program = BASE / 'copied-true'
shutil.copyfile('/usr/bin/true', program)
program.chmod(0o755)
execution = subprocess.run([str(program)], capture_output=True, timeout=10)
assert execution.returncode == 0
mount = subprocess.check_output(['findmnt', '-T', str(BASE), '-o', 'TARGET,FSTYPE,OPTIONS'], text=True)
assert 'ext4' in mount and 'fuse' not in mount and 'overlay' not in mount
record = {'runner_sha256': digest(Path(__file__)), 'directory': str(BASE), 'mount': mount,
          'reads': rows, 'all_eight_identity_reads_passed': len(rows) == 8,
          'hard_link_replacement_and_truncation_passed': True,
          'executable_path': str(program), 'executable_sha256': digest(program),
          'executable_exit_code': execution.returncode, 'new_target_strategy': 'Direct physical ext4 directory; no overlay or inherited build artifacts.',
          'runtime_admitted': False, 'full_plan_complete': False}
OUT.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['all_eight_identity_reads_passed', 'hard_link_replacement_and_truncation_passed', 'executable_exit_code', 'new_target_strategy']}), flush=True)
