"""Run the two admitted DRC integration binaries after the library validation."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'laurent-root-production'
OUT = EVIDENCE / 'laurent-root-production-integration'
SOURCE = ROOT / 'target/laurent-root-production/source'
TARGET = ROOT / 'target/laurent-root-production/target'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(PRODUCTION / 'sources.json')
native = read(Path(source['main_source_record']))
hashes = read(Path(source['base_source_record']))['source_hashes'] | source['changed_source_hashes'] | source['additional_source_hashes']
status = read(EVIDENCE / 'laurent-root-production-validation/status.json')
assert len(status) == 13 and all(row['exit_code'] == 0 for row in status)
assert sorted(path.stem for path in (SOURCE / 'crates/hyperdrc/tests').glob('*.rs')) == ['kicad_pad_sources', 'readme']
OUT.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')


def verify():
    assert all(digest(SOURCE / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())


verify()
command = ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'kicad_pad_sources', '--test', 'readme', '--', '--test-threads=2']
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(TARGET), 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true'}
print('start DRC integration tests', flush=True)
start = time.monotonic()
with (OUT / 'tests.log').open('w') as log:
    result = subprocess.run(command, cwd=SOURCE, env=env, stdout=log, stderr=subprocess.STDOUT)
(OUT / 'status.json').write_text(json.dumps([{'name': 'drc-integration-tests', 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}], indent=2) + '\n')
verify()
assert result.returncode == 0
rows = re.findall(r'test result: ok\. (\d+) passed; 0 failed; (\d+) ignored', (OUT / 'tests.log').read_text())
assert len(rows) == 2 and sum(int(row[0]) for row in rows) == 35 and all(int(row[1]) == 0 for row in rows)
(OUT / 'source-verification.json').write_text(json.dumps({'candidate_files_verified': len(hashes), 'main_files_verified': len(native['source_hashes']), 'all_source_hashes_unchanged': True}, indent=2) + '\n')
print('DRC integration tests passed: 35 tests in two binaries.', flush=True)
