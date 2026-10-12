"""Freeze every new domain-bound test result before changing production code."""
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
build = E / 'typed-api-prototype-v1673-domain-bound-baseline-build'
result = read(build / 'result.json')
assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)', (build / 'stderr.log').read_text())
assert len(paths) == 1
binary = Path(paths[0])
binary_hash = sha(binary)
source_hash = sha(build / 'source-hashes.json.gz')
tests = [
    ('domain_bound_retained_unit_proofs_precede_recognition', 101),
    ('domain_bound_retained_lower_proofs_precede_recognition', 101),
    ('domain_bound_required_domain_reuses_available_proofs', 101),
    ('domain_bound_upper_estimates_do_not_prove_lower_bounds', 0),
    ('domain_bound_endpoints_and_invalid_descendants_remain_typed', 0),
    ('domain_bound_public_asin_cutpoints', 0),
    ('domain_bound_public_acos_cutpoints', 0),
    ('domain_bound_public_atanh_cutpoints', 0),
    ('domain_bound_public_fine_retries_preserve_cached_facts', 0),
]
out = E / 'domain-bound-baseline-caps-v1682.json'
assert not out.exists()
rows = []
for version, (test, expected_code) in enumerate(tests, 1674):
    phase = f'typed-api-prototype-v{version}-domain-bound-baseline-cap'
    command = [sys.executable, str(E / 'run_limited_unit_test.py'), str(binary), binary_hash,
               'computable::node::domain_bound_tests::' + test, '--exact', '--nocapture', '--test-threads=1']
    subprocess.run([sys.executable, str(E / 'validate_candidate_v2.py'), phase, '--exec', *command], cwd=ROOT, check=True)
    p = E / phase
    result = read(p / 'result.json')
    stdout = (p / 'stdout.log').read_text()
    cap = json.loads(stdout[stdout.rfind('{"command":'):])
    assert result['source_unchanged'] and result['main_unchanged']
    assert sha(p / 'source-hashes.json.gz') == source_hash
    rows.append({'phase': phase, 'test': test, 'expected_exit_code': expected_code,
                 'result': result, 'cap': cap})
    out.write_text(json.dumps({'source_record': str((build / 'source-hashes.json.gz').relative_to(E)),
                              'source_record_sha256': source_hash, 'complete': False, 'tests': rows}, indent=2) + '\n')
    print(test, 'exit', result['exit_code'], 'expected', expected_code, flush=True)
out.write_text(json.dumps({'source_record': str((build / 'source-hashes.json.gz').relative_to(E)),
                          'source_record_sha256': source_hash, 'complete': True, 'tests': rows}, indent=2) + '\n')
assert all(row['result']['exit_code'] == row['cap']['exit_code'] == row['expected_exit_code']
           and not row['cap']['wall_timeout'] for row in rows)
