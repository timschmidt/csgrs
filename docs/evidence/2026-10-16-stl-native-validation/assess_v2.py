"""Assess the narrow STL contract from actual tests and complete paired costs."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


assert not (E / 'decision-v2.json').exists()
application = read(E / 'application-v2.json')
assert application['format_exit_code'] == 0
qualification = read(E / 'candidate-validation-v2/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
commands = read(E / 'candidate-validation-v2/commands.json')
assert len(commands) == 13
assert all(row['exit_code'] == row['expected_exit_code'] for row in commands)
for row in commands:
    for suffix in ['stdout', 'stderr', 'resources']:
        assert sha(E / 'candidate-validation-v2' / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
for name, counts in [('minimal-regression', '8 passed; 0 failed; 0 ignored;'),
                     ('topology-probe', '4 passed; 0 failed; 0 ignored;'),
                     ('format-probe', '9 passed; 3 failed; 0 ignored;')]:
    assert counts in (E / 'candidate-validation-v2' / (name + '.stdout')).read_text()
test_counts = {}
for name in ['root-all', 'stl-only']:
    text = (E / 'candidate-validation-v2' / (name + '.stdout')).read_text()
    counts = [tuple(map(int, match)) for match in re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', text)]
    assert counts and all(failed == 0 for _, failed, _ in counts)
    test_counts[name] = {'passed': sum(row[0] for row in counts), 'failed': 0,
                         'ignored': sum(row[2] for row in counts), 'test_binaries_and_doctest_groups': len(counts)}
assert test_counts['root-all']['passed'] == 420 and test_counts['root-all']['ignored'] == 1
cost = read(E / 'cost-v2/result.json')
assert cost['complete'] and cost['runtime_processes'] == 60 and cost['sample_rows'] == 120
assert cost['complete_input_and_output_comparisons_equal'] and cost['library_inputs_unchanged']
assert len(cost['summaries']) == 12
status = 'kept: STL import uses strict native closed-PWN validation and preserves typed topology failures'
record = {
    'status': status,
    'owning_layers': ['CSGRS STL format boundary', 'Hypermesh topology validation and retained facts', 'CSGRS example feature configuration'],
    'properties': ['P6','P8','P9','P18','P19','P21','P22','P24','P25','P26','P35','P36','P37'],
    'base_commit': application['main_parent_commit'],
    'implementation_sha256': application['implementation_sha256'],
    'baseline': {'original_format_probe': {'passed': 7, 'failed': 5},
                 'original_topology_probe': {'passed': 2, 'failed': 2},
                 'new_acceptance_probe': {'passed': 3, 'failed': 5}},
    'candidate': {'original_format_probe': {'passed': 9, 'failed': 3},
                  'original_topology_probe': {'passed': 4, 'failed': 0},
                  'new_acceptance_probe': {'passed': 8, 'failed': 0}, 'cargo_tests': test_counts,
                  'commands': 13, 'unexpected_exits': []},
    'original_format_diagnostic': {'remaining_failures': qualification['known_unfixed_format_failures'],
                                   'source_unchanged': True},
    'numeric_claim': 'Finite binary32 coordinates are promoted exactly; strict native topology decisions are certified.',
    'topology_claim': 'Canonical empty or nonempty closed PWN: valid indices, nondegenerate triangles and balanced geometric edge multiplicities.',
    'fidelity_limit': 'Proof concerns parsed coordinates, not the original exact construction or analytic surface; STL retains no such provenance.',
    'error_migration': 'Topology errors previously flattened into StdIo(InvalidData) now retain IoError::Construction -> ValidationError::Mesh -> HypermeshError. Parser IO failures remain parser IO failures.',
    'cache_evidence': 'Strict -> approximate -> strict reuses certified facts; mutation of position or triangle storage invalidates those facts and fails before empty composition.',
    'cost_result': 'cost-v2/result.json', 'cost_comparison': cost['summaries'],
    'cost_limits': 'Instrumented focused comparison of complete imports, native validation and union with empty. It covers empty, unit and 256 disconnected cubes in both encodings; allocator counters exclude allocator metadata and non-Rust heap use. No uninstrumented or application performance claim.',
    'resource_limits': 'No public total-work, heap-byte or cancellation budget added. Tests and measurements use external elapsed/address-space containment; parser allocations remain unbounded internally.',
    'stl_import_fix_admitted': True, 'example_feature_requirement_corrected': True, 'optimization_claimed': False,
    'first_candidate_matrix': {'unexpected_exits': ['stl-only', 'stl-clippy'], 'cause': 'Pre-existing missing curve requirement on basic_shapes, reproduced with unchanged baseline libraries.'},
    'example_contract': 'Declare curve and stl-io; preserve and execute all five original primitives with their original resolutions. Both execution and warning-denied Clippy pass for these minimal declared features.',
    'pending_native_files_unchanged': 35, 'native_runtime_admitted': False,
    'historical_artifact_audit_passed': False, 'historical_preservation_complete': False,
    'published_package_pipeline_executed': False, 'full_pre_push_matrix_executed': False,
    'push_performed': False, 'full_plan_complete': False,
    'next': ['Correct the separately reproduced OBJ/ASCII STL coordinate formatting losses and binary STL quantization collapse without weakening the fixtures.',
             'Keep the 35 pending native changes unadmitted until the original full-board and copper-neck runtime gates pass.',
             'Continue the remaining properties, clean external distribution and full pre-push validation.'],
}
(E / 'decision-v2.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'status': status, 'tests': test_counts, 'cost_processes': 60}), flush=True)
