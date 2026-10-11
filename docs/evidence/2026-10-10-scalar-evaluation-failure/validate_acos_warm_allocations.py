#!/usr/bin/env python3
"""Run each frozen endpoint allocation regression in a capped process."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

build = E/sys.argv[1]
start = int(sys.argv[2])
result = read(build/'result.json')
assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)', (build/'stderr.log').read_text())
assert len(paths) == 1
binary = Path(paths[0])
binary_hash = sha(binary)
rows = {}
for version, sign in enumerate(['positive', 'negative'], start=start):
    phase = f'typed-api-prototype-v{version}-acos-warm-allocation-cap'
    assert not (E/phase).exists()
    selected = f'computable::node::acos_warm_probe_tests::acos_warm_{sign}_allocation_regression'
    subprocess.run([sys.executable, str(E/'validate_candidate_v2.py'), phase, '--exec',
        sys.executable, str(E/'run_limited_unit_test.py'), str(binary), binary_hash,
        selected, '--exact', '--nocapture', '--test-threads=1'], cwd=ROOT, check=True)
    result = read(E/phase/'result.json')
    assert result['source_unchanged'] and result['main_unchanged']
    assert sha(E/phase/'source-hashes.json.gz') == sha(build/'source-hashes.json.gz')
    log = (E/phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['command'] == [str(binary.resolve()), selected, '--exact', '--nocapture', '--test-threads=1']
    assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
    assert sha(E/phase/'runner.py') == sha(E/'validate_candidate_v2.py')
    assert cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
    assert cap['binary_sha256'] == binary_hash
    rows[phase] = {'test': selected, 'result': result, 'cap': cap}
    print(phase, 'exit', result['exit_code'], flush=True)
out = E/f'acos-warm-allocation-caps-v{start+1}.json'
assert not out.exists()
out.write_text(json.dumps({'source_record': str((build/'source-hashes.json.gz').relative_to(E)),
    'source_record_sha256': sha(build/'source-hashes.json.gz'), 'tests': rows}, indent=2)+'\n')
