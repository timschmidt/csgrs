#!/usr/bin/env python3
"""Retain the parser failure, focused tests, and unchanged successful Cargo run."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p:json.loads(p.read_text())
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
out = E/'algebraic-preflight-parser-correction-v1133.json'
assert not out.exists()
old = E/'typed-api-prototype-v1104-stack-tests'
original_logs = {n:sha(old/n) for n in ['stdout.log','stderr.log','result.json']}
assert read(old/'result.json')['exit_code'] == 0
failed = E/'algebraic-preflight-orchestration-v1133.json'
assert read(failed)['exit_code'] == 1
assert 'assert not stack' in (E/'algebraic-preflight-orchestration-v1133.log').read_text()
command = [sys.executable,str(E/'test_parse_test_results_v3.py'),'-v']
stdout = E/'algebraic-preflight-parser-tests-v1133.stdout.log'
stderr = E/'algebraic-preflight-parser-tests-v1133.stderr.log'
assert not stdout.exists() and not stderr.exists()
started = time.monotonic()
with stdout.open('w') as a,stderr.open('w') as b:
    result = subprocess.run(command,cwd=ROOT,stdout=a,stderr=b)
test_result = {'command':command,'exit_code':result.returncode,'seconds':time.monotonic()-started,
    'stdout_sha256':sha(stdout),'stderr_sha256':sha(stderr)}
assert result.returncode == 0, test_result
assert 'Ran 6 tests' in stderr.read_text() and stderr.read_text().rstrip().endswith('OK')
assert {n:sha(old/n) for n in original_logs} == original_logs
data = parse_phase(old)
assert data['top_level_passed'] == 5811 and data['child_process_passed'] == 3
out.write_text(json.dumps({
    'status':'Evidence parser repaired; original Cargo success and logs preserved',
    'failed_orchestration':failed.name,'failed_orchestration_sha256':sha(failed),
    'successful_cargo_phase':old.name,'original_phase_artifacts_sha256':original_logs,
    'cause':'Parent and child libtest output interleaved: the child summary marker was prefixed by a parent ok token and separated from its counters by a newline. The line-anchored v2 parser missed it and retained an unmatched child batch.',
    'repair':'v3 normalizes complete result markers in memory, preserving raw logs, batch nesting and failed counts. It additionally requires each batch declared count to equal passed plus failed plus ignored.',
    'parser':'parse_test_results_v3.py','parser_sha256':sha(E/'parse_test_results_v3.py'),
    'tests':'test_parse_test_results_v3.py','test_source_sha256':sha(E/'test_parse_test_results_v3.py'),
    'test_result':test_result,'test_functions':6,'historical_summaries_identical':4,
    'stack_counts':{k:data[k] for k in ['top_level_passed','child_process_passed','top_level_ignored','top_level_failed','child_process_failed']},
    'resumed_validation_helper':'validate_algebraic_preflight_checkpoint_v2.py',
    'resumed_orchestration':'algebraic-preflight-orchestration-completion-v1133.json',
    'runtime_source_and_test_workloads_unchanged':True,
    'scope':'The 981-second successful Cargo phase is reused with byte-identical logs; no test result is inferred from the parser exception and no workload is omitted.'
},indent=2)+'\n')
print(json.dumps({'parser_tests':6,'historical_summaries_identical':4,'stack_passed':data['top_level_passed'],'child_passed':data['child_process_passed']}))
