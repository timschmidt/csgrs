"""Assess exact projection exports against final native and real binding evidence."""
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


assert not (E / 'decision-v1.json').exists()
application = read(E / 'application-v1.json')
assert application['format_exit_code'] == 0
qualification = read(E / 'candidate-validation-v2/result.json')
assert qualification['complete'] and not qualification['unexpected_exits']
commands = read(E / 'candidate-validation-v2/commands.json')
assert len(commands) == 15 and all(row['exit_code'] == row['expected_exit_code'] == 0 for row in commands)
for row in commands:
    for suffix in ['stdout','stderr','resources']:
        assert sha(E / 'candidate-validation-v2' / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
for name, counts in [('minimal-regression','9 passed; 0 failed; 0 ignored;'),
                     ('topology-probe','4 passed; 0 failed; 0 ignored;'),
                     ('format-probe','12 passed; 0 failed; 0 ignored;')]:
    assert counts in (E / 'candidate-validation-v2' / (name + '.stdout')).read_text()
test_counts = {}
for name, expected in [('root-all', (429,1)),('stl-only',(151,0)),('obj-only',(150,0))]:
    text = (E / 'candidate-validation-v2' / (name + '.stdout')).read_text()
    counts = [tuple(map(int, match)) for match in re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', text)]
    assert counts and all(failed == 0 for _, failed, _ in counts)
    test_counts[name] = {'passed': sum(row[0] for row in counts), 'failed': 0,
                        'ignored': sum(row[2] for row in counts), 'test_binaries_and_doctest_groups': len(counts)}
    assert (test_counts[name]['passed'], test_counts[name]['ignored']) == expected
wasm = read(E / 'wasm-validation-v1/result.json')
wasm_commands = read(E / 'wasm-validation-v1/commands.json')
assert wasm['all_commands_passed'] and len(wasm_commands) == 4 and all(row['exit_code'] == 0 for row in wasm_commands)
for row in wasm_commands:
    for suffix in ['stdout','stderr','resources']:
        assert sha(E / 'wasm-validation-v1' / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
assert all(sha(path) == record['sha256'] for path, record in wasm['artifacts'].items())
assert 'WASM export projection, failure, cache and composition contracts passed' in (E / 'wasm-validation-v1/real-js-and-typescript-consumers.stdout').read_text()
cost = read(E / 'cost-v1/result.json')
assert cost['complete'] and cost['runtime_processes'] == 90 and cost['sample_rows'] == 180
assert cost['complete_input_and_output_comparisons_equal'] and cost['library_inputs_unchanged']
assert len(cost['summaries']) == 18
fractional = read(E / 'fractional-cost-v1/result.json')
assert fractional['complete'] and fractional['runtime_processes'] == 5 and fractional['sample_rows'] == 40
assert fractional['library_inputs_unchanged'] and fractional['all_repeated_complete_outputs_equal']
status = 'kept: exact decimal spelling of finite binary64 projections and strict validation of binary32 STL faces'
record = {
    'status': status,
    'owning_layers': ['CSGRS format quantization and spelling boundary','Hypermesh/Hyperlimit native triangle certification','CSGRS WASM public consumer tests'],
    'properties': ['P6','P8','P9','P18','P19','P24','P25','P26','P27','P30','P35','P36','P37'],
    'base_commit': application['main_parent_commit'],
    'implementation_sha256': application['implementation_sha256'],
    'baseline': {'original_format_probe': {'passed':9,'failed':3},
                 'original_topology_probe': {'passed':4,'failed':0},
                 'new_acceptance_probe': {'passed':3,'failed':6}},
    'candidate': {'original_format_probe': {'passed':12,'failed':0},
                  'original_topology_probe': {'passed':4,'failed':0},
                  'new_acceptance_probe': {'passed':9,'failed':0},'cargo_tests':test_counts,
                  'commands':15,'unexpected_exits':[]},
    'original_format_diagnostic': {'remaining_failures':[], 'source_unchanged':True},
    'numeric_claim': 'OBJ and ASCII STL spell each selected finite binary64 field as its exact terminating decimal. Binary STL certifies nondegeneracy of actual projected binary32 coordinates under the strict native policy before publishing or caching bytes.',
    'topology_claim': 'Binary export preserves open nondegenerate surfaces; it does not assert global closed-PWN or manifold topology. Native reimport/fixture validation establish their own explicit claims.',
    'fidelity_limit': 'The existing finite-float projections remain deliberate lossy boundaries for other native exact values. ASCII STL import still quantizes to binary32. No source analytic intent, original exact geometry or inherited provenance is certified by reimport.',
    'serialization_migration': 'Fixed 17-fractional-place output is retained when exact. Other OBJ/ASCII STL values now use enough fractional places to represent their finite binary64 projection exactly, at most 1074. Corrected text/checksums and lengths intentionally change; for example binary64 1.1 is no longer emitted as the different exact decimal 1.10000000000000009. Regenerate affected golden artifacts deliberately. Previously valid binary outputs and full-resolution basic_shapes outputs remain byte-identical.',
    'failure_migration': 'Binary STL projection collapse now returns IoError::Geometry before publishing bytes, including three distinct collinear projected points. Native predicate failure retains IoError::Construction with its cause. Invalid exports do not populate the byte cache.',
    'cache_evidence': 'Cold, warm, evicted, returned-buffer mutation, source Arc mutation and valid recovery scenarios preserve definite results; the same shared Rust implementation is exercised through actual JS exports.',
    'wasm': {'commands':4,'all_passed':True,'typescript':'5.7.3','wasm_bindgen':'0.2.126',
             'existing_consumer_preserved':True,'new_consumer':'tests/wasm/export_projection.cjs',
             'evidence':'wasm-validation-v1/result.json',
             'scope': 'Real generated JS checks independent exact BigInt ratios, projected failure diagnostics, no result on failure, cache buffer ownership, and strict composition of independently decoded tiny STL geometry. Existing certainty and TypeScript consumers also pass. JS has no STL importer; the test decodes binary32 fields and calls public fromArrays.',
             'typed_io_receipt_remains_open':True,
             'tooling_warnings':'The pinned wasm-bindgen CLI install reports third-party future-incompatibility warnings for buf_redux 0.8.4 and multipart 0.18.0; retained stderr records them. Affected project Clippy builds pass with -D warnings.'},
    'validation_partition': application['validation_partition'],
    'cost_result':'cost-v1/result.json','cost_comparison':cost['summaries'],
    'fractional_cost_result':'fractional-cost-v1/result.json','fractional_cost_observations':fractional['summaries'],
    'fractional_cost_scope':'Five candidate-only processes record absolute complete export costs and output sizes for ordinary and extreme fractional coordinates. No comparison credits the old incorrect spelling as the same successful output.',
    'cost_limits':'Allocator-instrumented focused comparison of complete exports, empty/unit/256 disconnected integer cubes across all three encodings. Untimed independent semantic and topology checks plus complete-byte comparisons qualify the samples. First binary cache miss is separated from warm calls. Requested allocations are not actual heap usage. No optimization or whole-application performance claim.',
    'resource_limits':'Per-coordinate decimal precision is bounded at 1074 fractional places. No public total-work, heap-byte or cancellation budget is added. External time/address-space limits contain tests; retained export caches still have entry-count limits only.',
    'first_candidate_matrix': {'unexpected_exits':['strict-clippy','stl-clippy'],
        'cause':'New test decoder triggered chunks_exact_to_as_chunks. v2 uses the suggested fixed-size slice iteration with every assertion and fixture unchanged; v1 evidence remains intact.'},
    'export_projection_fix_admitted':True,'optimization_claimed':False,
    'pending_native_files_unchanged':35,'native_runtime_admitted':False,
    'historical_artifact_audit_passed':False,'historical_preservation_complete':False,
    'published_package_pipeline_executed':False,'full_pre_push_matrix_executed':False,
    'push_performed':False,'full_plan_complete':False,
    'next':['Preserve typed format failures and their claim scope across WASM, where export errors currently flatten into strings.',
            'Keep the 35 pending native changes unadmitted until their original full-board and copper-neck runtime gates pass.',
            'Continue the remaining properties, clean external distribution and full pre-push validation.'],
}
(E / 'decision-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'status':status,'tests':test_counts,'cost_processes':90,'real_wasm_consumers_passed':True}),flush=True)
