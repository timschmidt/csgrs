"""Summarize completed broad checks without widening their proof/admission scope."""
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase
E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source = sha(E / 'typed-api-prototype-v1924-cache-state-merge-verus/source-hashes.json.gz')
validation = read(E / 'cache-state-merge-candidate-validation-v1935.json')
assert validation['source_record_sha256'] == source and len(validation['phases']) == 9
summaries = {}
for name, row in validation['phases'].items():
    p = E / name
    r = read(p / 'result.json')
    assert r == row['result'] and r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(p / 'source-hashes.json.gz') == source
    summaries[name] = parse_phase(p)
checks = {
    'typed-api-prototype-v1927-cache-state-merge-scalar-tests': (1074, 1, 0),
    'typed-api-prototype-v1932-cache-state-merge-stack-tests': (5858, 3, 33),
    'typed-api-prototype-v1933-cache-state-merge-default-tests': (1059, 3, 0),
    'typed-api-prototype-v1934-cache-state-merge-debug-tests': (1074, 1, 0),
}
for name, expected in checks.items():
    tests = summaries[name]
    assert tuple(tests[k] for k in ['top_level_passed','child_process_passed','top_level_ignored']) == expected
    assert tests['top_level_failed'] == tests['child_process_failed'] == 0
stack = E / 'typed-api-prototype-v1932-cache-state-merge-stack-tests'
prior = E / 'typed-api-prototype-v1769-domain-predicate-stack-tests'
warning_lines = lambda p: sorted(line for line in (p / 'stderr.log').read_text().splitlines() if line.startswith('warning:'))
warnings = warning_lines(stack)
assert warnings == warning_lines(prior) and len(warnings) == 14
assert summaries[stack.name]['ignored_test_lines'] == parse_phase(prior)['ignored_test_lines']
for name in ['typed-api-prototype-v1928-cache-state-merge-clippy','typed-api-prototype-v1935-cache-state-merge-minimal-clippy']:
    assert not warning_lines(E / name)
format_command = read(E / 'typed-api-prototype-v1931-cache-state-merge-included-fmt/command.json')['command']
assert len([part for part in format_command if part.endswith('.rs')]) == 108
out = E / 'cache-state-merge-broad-validation-v1935.json'
assert not out.exists()
data = {
    'status':'All nine fresh scalar, consumer, build, Clippy and format checks passed; cost and replay gates remain separate',
    'source_record_sha256':source,'orchestration_record':'cache-state-merge-candidate-validation-v1935.json',
    'test_summaries':summaries,'included_rustfmt_files':108,
    'inherited_stack_warning_lines':warnings,'inherited_stack_warning_baseline':prior.name,
    'ignored_test_names_unchanged':True,'strict_clippy_warning_lines':0,
    'limits':'The stack includes hyperreal, hyperlimit, hyperlattice, hypercurve and hyperdrc; it is not an all-workspace/CI or full-board admission claim. Thirty-three ignored cases and fourteen inherited Cargo example-filename warnings remain explicit. Default feature counts exclude the serde-gated new module. No fresh whole-crate formal proof is claimed.'
}
out.write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps({'stack_passed':5858,'stack_child_passed':3,'stack_ignored':33,
                  'scalar_passed':1074,'default_passed':1059,'debug_passed':1074,
                  'inherited_warnings':14,'included_rustfmt_files':108}))
