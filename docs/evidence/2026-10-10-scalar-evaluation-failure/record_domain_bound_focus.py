"""Freeze the scoped implementation, proof limits and completed focused checks."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
SOURCE = E.parents[2] / 'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
baseline = read(E / 'domain-bound-baseline-v1683.json')
caps = read(E / 'domain-bound-candidate-caps-v1693.json')
comparison = read(E / 'domain-bound-query-comparison-v1693.json')
source_record = E / 'typed-api-prototype-v1696-domain-bound-release-build/source-hashes.json.gz'
assert sha(source_record) == caps['source_record_sha256'] == comparison['source_record_sha256']
source = json.loads(gzip.decompress(source_record.read_bytes()))
old_record = E / baseline['source_record']
old = json.loads(gzip.decompress(old_record.read_bytes()))
changed = [name for name in source if old.get(name) != source[name]]
expected = ['crates/hyperreal/src/computable/node/domain_certificates.rs',
            'crates/hyperreal/src/computable/node/domain_validation.rs']
assert changed == expected
for name, digest in source.items():
    assert sha(SOURCE / name) == digest
patch = E / 'domain-bound-retaining.patch'
assert not patch.exists()
parts = []
for name in changed:
    parts.append(f'diff --git a/{name} b/{name}\n')
    parts.extend(difflib.unified_diff((old_record.parent / 'inputs' / name).read_text().splitlines(keepends=True),
                                    (SOURCE / name).read_text().splitlines(keepends=True),
                                    fromfile='a/'+name, tofile='b/'+name))
patch.write_text(''.join(parts))
phases = {}
for version in range(1694, 1699):
    paths = [p for p in E.glob(f'typed-api-prototype-v{version}-*') if p.is_dir()]
    assert len(paths) == 1
    p = paths[0]
    r = read(p / 'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(p / 'source-hashes.json.gz') == caps['source_record_sha256']
    phases[p.name] = r
units = parse_phase(E / 'typed-api-prototype-v1694-domain-bound-scalar-tests')
assert units['top_level_passed'] == 1074 and units['child_process_passed'] == 1
prior_proof = read(E / 'domain-sign-proof-scope-v1630.json')
inputs = prior_proof['unchanged_proof_and_guard_inputs_sha256']
assert len(inputs) == 44
for name, digest in inputs.items():
    assert sha(SOURCE / name) == digest
proof = E / 'domain-bound-proof-scope-v1698.json'
out = E / 'domain-bound-focused-implementation-v1698.json'
assert not proof.exists() and not out.exists()
proof.write_text(json.dumps({
    'status':'Prior conditional proof inputs unchanged; new magnitude-domain runtime helper is outside that target',
    'source_record_sha256':caps['source_record_sha256'],
    'prior_record':'integer-scale-proof-validation-v1404.json',
    'prior_record_sha256':sha(E / 'integer-scale-proof-validation-v1404.json'),
    'unchanged_proof_and_guard_inputs_sha256':inputs,
    'prior_verified_units':638,'prior_errors':0,'fresh_verus_run':False,
    'open_whole_crate_obligation_groups':13,
    'limits':'Identical prior proof inputs preserve only the earlier conditional proof and guard scope. They do not prove the new available_bound_proves_domain helper, independent runtime existence prerequisites, general cache concurrency or whole-crate correctness.'
},indent=2)+'\n')
data = {
    'status':'Isolated implementation passes frozen focused tests, scalar units, strict Clippy, build and formatting; broad tests, costs and old replays pending',
    'baseline_commit':'079ddf437','baseline_record':'domain-bound-baseline-v1683.json',
    'source_record':str(source_record.relative_to(E)),'source_record_sha256':caps['source_record_sha256'],
    'changed_production_sources':changed,'production_patch':patch.name,'production_patch_sha256':sha(patch),
    'frozen_test_sources_preserved':True,'focused_caps':'domain-bound-candidate-caps-v1693.json',
    'query_comparison':'domain-bound-query-comparison-v1693.json',
    'private_cheap_completions_before_after':[4,56],'public_completions_before_after':[415,453],
    'public_completion_losses':0,'earlier_successful_samples_and_definite_signs_preserved':True,
    'negative_and_endpoint_controls_unchanged':48,'fine_retries_unchanged':36,'zero_work_cached_retry_queries':144,
    'scalar_tests':units,'completed_phases':phases,
    'new_behavior':'After independently proving operand existence, domain checks use already-cached upper or exact lower magnitude bounds before rational recognition. No new graph walk or coefficient scan is introduced on the fast path.',
    'proof_arguments':[
        'A zero bound or upper MSD e < 0 proves |x| < 1, which suffices for both unit intervals and x > -1. Probability additionally requires a positive sign.',
        'A positive sign and exact binade exponent e >= 0 prove x >= 2^e >= 1. The explicit upper-only controls forbid using an inexact estimate as that lower bound.',
        'Independent child existence still precedes every use. Stored invalid descendants and endpoints keep their typed results.',
        'An unavailable or insufficient cached bound falls through to the same rational recognition and numerical refinement paths. Existing literal sign publication remains unchanged.'
    ],
    'proof_scope':proof.name,'prepared_cost_protocol':'domain-bound-cost-protocol-v1703.json',
    'limits':[
        'This record precedes completed consumer/default/debug validation, cost comparisons and the retained 4129-case replay corpus. It is not a final retention decision or runtime admission.',
        'The public survey uses ordinary constructors and round-trips. Diagnostic domain helper queries and the explicit internal lower-bound existence preparation are not public APIs.',
        'Pi is explicitly prewarmed in each public survey; other shared constants may evolve in the frozen process order. No globally cold-process timing claim follows.',
        'The earlier misleading log-warmup comment is now corrected with all test behavior unchanged; prior archived sources and measurements remain immutable.',
        'No byte/time work guarantee, universal completeness or new Verus coverage is claimed. Main retains its original scalar failure and all 49 preexisting entries.',
        'All 39 properties, 14 targets and crate future work remain in scope.'
    ]
}
out.write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps({'record':out.name,'source_record_sha256':caps['source_record_sha256'],
                  'scalar_tests':1074,'new_public_successes':38,'new_private_completions':52}))
