"""Observe an unchanged obsolete assertion failing; not a passing test claim."""
import subprocess
import sys
result = subprocess.run([sys.argv[1], '--exact', 'every_real_certificate_crosses_solver_analysis_and_proof_surfaces', '--nocapture'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
print(result.stdout, end='')
print(result.stderr, end='', file=sys.stderr)
print('observed_original_test_exit_code=' + str(result.returncode))
assert result.returncode == 101
assert 'observed counts: sparse=22, sparse_unknown=0, pattern=22, pattern_unknown=0' in result.stderr
assert 'assertion failed: sparse_explicitly_unknown > 0' in result.stderr
