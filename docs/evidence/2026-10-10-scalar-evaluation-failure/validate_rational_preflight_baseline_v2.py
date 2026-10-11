#!/usr/bin/env python3
"""Capture rational lazy-preflight regressions before runtime changes."""
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


def main():
    build = E/'typed-api-prototype-v1136-rational-preflight-baseline-build'
    result = read(build/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
    paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)', (build/'stderr.log').read_text())
    assert len(paths) == 1
    binary = Path(paths[0])
    binary_hash = sha(binary)
    tests = re.findall(r'    fn (rational_preflight_\w+)\(\)',
        (build/'inputs/crates/hyperreal/src/computable/node/rational_preflight_tests.rs').read_text())
    assert len(tests) == 10
    rows = {}
    for version, test in enumerate(tests, start=1137):
        phase = f'typed-api-prototype-v{version}-rational-preflight-baseline-cap'
        assert not (E/phase).exists()
        selected = 'computable::node::rational_preflight_tests::'+test
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
        rows[phase] = {'test': test, 'result': result, 'cap': cap}
        print(phase, 'exit', result['exit_code'], flush=True)
    out = E/'rational-preflight-baseline-caps-v1146.json'
    assert not out.exists()
    out.write_text(json.dumps({'source_record': str((build/'source-hashes.json.gz').relative_to(E)),
        'source_record_sha256': sha(build/'source-hashes.json.gz'), 'tests': rows}, indent=2)+'\n')


if __name__ == '__main__':
    main()
