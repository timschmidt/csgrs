"""Audit the actual integrated proof, fresh rejection sweep and native validation."""
import ast
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
CRATE = SOURCE / 'crates/hyperreal'
OUT = E / 'dependency-integration-audit-v2497'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint = E / 'typed-api-prototype-v2440-state.json'
    assert sha(checkpoint) == '1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0'
    prior = read(checkpoint)
    integration = E / 'dependency-integration-v2480'
    applied, protocol = read(integration / 'applied.json'), read(integration / 'protocol.json')
    assert applied['protocol_sha256'] == sha(integration / 'protocol.json')
    assert all(sha(integration / 'candidate' / n) == h for n, h in protocol['overlay_files_sha256'].items())
    assert applied['source_record_sha256'] == sha(integration / 'working-source-hashes.json.gz')
    correction = E / 'dependency-integration-correction-v2489'
    correction_protocol = read(correction / 'protocol.json')
    assert read(correction / 'applied.json')['protocol_sha256'] == sha(correction / 'protocol.json')
    source_record = correction / 'working-source-hashes.json.gz'
    current, before = snapshot(source_record), snapshot(E / prior['source_record'])
    combined_overlay = protocol['overlay_files_sha256'] | correction_protocol['overlay_files_sha256']
    assert current == before | combined_overlay
    spec = importlib.util.spec_from_file_location('catalog_v2488', E / 'validate_candidate_v3.py')
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    original = snapshot(E / 'candidate-preparation/source-hashes.json.gz')
    assert catalog.source_hashes(SOURCE, original) == current
    main_guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    expected = read(E.parent / '2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [line for line in status if not line[3:].startswith(str(E.relative_to(ROOT)) + '/')] == expected
    phase_names = [
        'typed-api-prototype-v2492-integrated-verus',
        'typed-api-prototype-v2493-integrated-acceptance',
        'typed-api-prototype-v2496-integrated-rejections',
        'typed-api-prototype-v2490-integrated-workspace-fmt',
        'typed-api-prototype-v2491-integrated-included-fmt',
        'typed-api-prototype-v2494-integrated-clippy',
        'typed-api-prototype-v2495-integrated-scalar-tests',
    ]
    phases = {}
    for name in phase_names:
        phase = E / name
        result = read(phase / 'result.json')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged'], name
        assert result['source_catalog_rechecked'] and sha(phase / 'source-hashes.json.gz') == sha(source_record)
        phases[name] = {'result': result, 'result_sha256': sha(phase / 'result.json'),
                        'command': read(phase / 'command.json')['command'],
                        'stdout_sha256': sha(phase / 'stdout.log'), 'stderr_sha256': sha(phase / 'stderr.log')}
    sys.path.insert(0, str(E))
    from parse_test_results_v3 import parse_phase
    native_tests = parse_phase(E / phase_names[6])
    assert native_tests['top_level_passed'] == 1074 and native_tests['top_level_failed'] == 0
    assert 'warning:' not in (E / phase_names[5] / 'stderr.log').read_text()
    proof = E / phase_names[0] / 'proof-reports'
    report, evidence = read(proof / 'verus.json'), read(proof / 'evidence.json')
    verified = report['verification-results']
    assert verified['verified'] == 737 and verified['errors'] == 0 and verified['success']
    assert verified['is-verifying-entire-crate'] and not verified['encountered-error'] and not verified['encountered-vir-error']
    assert not (proof / 'verus.stderr').read_bytes()
    scope = read(CRATE / 'verification/scope.json')
    assert len(scope['verified_executable_contracts']) == 99 and len(scope['verified_model_theorems']) == 178
    assert len(scope['obligations']) == 13 and all(row['status'] != 'complete' for row in scope['obligations'])
    assert evidence['scope'] == scope and evidence['verification_results'] == verified
    assert 'hyperreal_proofs::computable_bounds::approximation::Computable::bound_from_approx' in report['func-details']
    for group in ['verified_executable_contracts', 'verified_model_theorems']:
        assert all('hyperreal_proofs::' + name in report['func-details'] for name in scope[group])
    assert 'Ran 51 tests' in (E / phase_names[1] / 'stderr.log').read_text()
    guards = E / phase_names[2] / 'proof-checks'
    guard = read(guards / 'tooling.json')
    assert guard['exit_code'] == 0 and guard['tool_unchanged']
    calls = [json.loads(line) for line in gzip.decompress((guards / 'verus-invocations.jsonl.gz').read_bytes()).splitlines()]
    assert len(calls) == guard['verus_invocations'] == 326
    assert sum(call['exit_code'] == 0 for call in calls) == guard['baseline_successes'] == 8
    assert sum(call['exit_code'] != 0 for call in calls) == guard['rejected_invocations'] == 318
    assert all('--no-cheating' in call['command'] for call in calls)
    previous_calls = [json.loads(line) for line in gzip.decompress((E /
        'typed-api-prototype-v2483-integrated-rejections/proof-checks/verus-invocations.jsonl.gz').read_bytes()).splitlines()]
    assert len(previous_calls) == len(calls)
    import_name = 'verification/computable_bounds.rs'
    import_before = sha(correction / 'original/crates/hyperreal' / import_name)
    import_after = sha(CRATE / import_name)

    def library_text(root, cache):
        if cache:
            modules = ['computable_bounds', 'computable_cache', 'magnitude_model',
                       'integer_approximation_model', 'real_approximation_model']
            return ('#![feature(proc_macro_hygiene)]\nmod structural;\nmod verified {\n'
                f'#[path = "{root / "word.rs"}"] pub(crate) mod word;\n}}\n'
                + ''.join(f'#[path = "verification/{name}.rs"] mod {name};\n' for name in modules))
        return ('#![feature(proc_macro_hygiene)]\nmod verified {\n'
            f'#[path = "{root / "word.rs"}"] pub(crate) mod word;\n}}\n'
            'mod structural;\nmod magnitude_model;\nmod integer_approximation_model;\nmod real_approximation_model;\n'
            '#[path = "verification/computable_bounds.rs"] mod computable_bounds;\n')

    source_reconstructions = 0
    for old, new in zip(previous_calls, calls, strict=True):
        assert (old['exit_code'] == 0) == (new['exit_code'] == 0)
        assert old['rust_inputs_sha256'].keys() == new['rust_inputs_sha256'].keys()
        for name, old_hash in old['rust_inputs_sha256'].items():
            new_hash = new['rust_inputs_sha256'][name]
            if old_hash == new_hash:
                continue
            if name == import_name:
                assert (old_hash, new_hash) == (import_before, import_after)
            else:
                assert name == 'lib.rs', name
                cache = 'src/computable/node/representation.rs' in old['rust_inputs_sha256']
                for call, expected_hash in [(old, old_hash), (new, new_hash)]:
                    text = library_text(Path(call['input_root']), cache)
                    assert hashlib.sha256(text.encode()).hexdigest() == expected_hash
            source_reconstructions += 1
        if old['exit_code'] == 0:
            assert old['command'].count('--verify-only-module') == new['command'].count('--verify-only-module')
        else:
            # Structural controls already contain only the structural module.
            assert '--verify-only-module' in new['command'] or set(new['rust_inputs_sha256']) == {'lib.rs', 'structural.rs'}
    tree = ast.parse((CRATE / 'verification/test_approximation_bound_rejections.py').read_text())
    mutations = next(ast.literal_eval(node.value) for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'MUTATIONS' for target in node.targets))
    assert len(mutations) == 12
    producer_calls = calls[-13:]
    baseline = producer_calls[0]
    shared = 'src/computable/node/approximation_bound.rs'
    shared_text = (CRATE / shared).read_text()
    assert baseline['rust_inputs_sha256'][shared] == sha(CRATE / shared)
    expected_rust = {str(path.relative_to(CRATE)): sha(path) for path in (CRATE / 'src').rglob('*.rs')}
    expected_rust.update({str(path.relative_to(CRATE)): sha(path) for path in (CRATE / 'verification').glob('*.rs')})
    assert baseline['rust_inputs_sha256'] == expected_rust
    assert baseline['exit_code'] == 0 and json.loads(baseline['stdout'])['verification-results']['verified'] == 1
    for (name, kind, old, new), call in zip(mutations, producer_calls[1:], strict=True):
        assert shared_text.count(old) == 1 and call['command'] == baseline['command']
        expected_hash = hashlib.sha256(shared_text.replace(old, new).encode()).hexdigest()
        assert call['rust_inputs_sha256'] == expected_rust | {shared: expected_hash}, name
        failure = json.loads(call['stdout'])['verification-results']
        assert call['exit_code'] != 0 and failure['errors'] > 0 and not failure['encountered-vir-error'], name
        assert any(message in call['stderr'] for message in ['postcondition not satisfied', 'assertion failed',
            'precondition not satisfied', 'requires not satisfied', 'possible arithmetic underflow/overflow']), name
    dependencies = read(proof / 'dependencies.json')
    assert dependencies == evidence['dependencies'] and len(dependencies['bindings']) == 8
    assert len(guard['dependency_builds']) == 3
    sys.path.insert(0, str(CRATE / 'scripts'))
    import verus_dependencies
    os.environ.update(read(E / phase_names[0] / 'command.json')['environment'])
    verus_dependencies.check_dependency_identity(dependencies)
    compiler_paths = [Path(dependencies['verified_metadata']['config']).parent]
    for build in guard['dependency_builds']:
        directory = guards / build['directory']
        assert all(sha(directory / n) == h for n, h in build['files_sha256'].items())
        dep = read(directory / 'dependencies.json')
        assert dep['verified_metadata'] == build['verified_metadata']
        verus_dependencies.check_dependency_identity(dep)
        compiler_paths.append(directory)
    compiler_results = []
    for directory in compiler_paths:
        result = read(directory / 'compiler-result.json')
        assert result['gate_passed'] and result['exit_code'] == 0 and result['verified'] == 453 and result['warnings'] == 0
        compiler_results.append({'directory': str(directory), 'result_sha256': sha(directory / 'compiler-result.json')})
    OUT.mkdir()
    compiler_out = OUT / 'positive-proof-compiler'
    compiler_out.mkdir()
    for name in ['compiler-config.json', 'compiler-invocation.json', 'compiler-result.json',
                 'compiler.stdout.json', 'compiler.stderr.log']:
        shutil.copy2(compiler_paths[0] / name, compiler_out / name)
    assert catalog.source_hashes(SOURCE, original) == current
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    result = {'status': 'Validated isolated dependency and shared-producer integration; no main/runtime admission',
        'runner_sha256': sha(Path(__file__)), 'previous_checkpoint_sha256': sha(checkpoint),
        'integration_protocol_sha256': sha(integration / 'protocol.json'),
        'source_record': str((E / phase_names[0] / 'source-hashes.json.gz').relative_to(E)),
        'source_record_sha256': sha(source_record), 'source_files': len(current),
        'changed_since_checkpoint': len(combined_overlay),
        'correction_protocol_sha256': sha(correction / 'protocol.json'),
        'control_outcomes_compared': len(calls), 'control_input_differences_reconstructed': source_reconstructions,
        'rejection_run_seconds': {
            'before_module_selection': read(E / 'typed-api-prototype-v2483-integrated-rejections/result.json')['seconds'],
            'after_module_selection': phases[phase_names[2]]['result']['seconds'],
            'comparison_is_matched_performance_benchmark': False,
            'fits_current_15_minute_ci_job_locally': False,
        },
        'phases': phases, 'scalar_verified_units': 737, 'scalar_errors': 0,
        'listed_production_contracts': 99, 'listed_model_theorems': 178,
        'dependency_verified_units_per_build': 453, 'dependency_compiler_warnings': 0,
        'fresh_dependency_builds': len(compiler_results), 'compiler_results': compiler_results,
        'compiler_receipts_sha256': {p.name: sha(p) for p in compiler_out.iterdir()},
        'fresh_distinct_acceptance_tests': 51, 'fresh_guard_baselines': 8, 'fresh_rejected_controls': 318,
        'fresh_scalar_native_tests': native_tests,
        'producer_mutant_inputs_reconstructed': 12, 'producer_full_rust_inputs_checked': len(expected_rust),
        'source_catalog_rechecked': True, 'main_files_unchanged': len(main_guard),
        'original_worktree_entries_preserved': len(expected),
        'existing_worktree_changes_in_scope': True,
        'retained_runtime_reference': 'typed-api-prototype-v2335-state.json',
        'runtime_reference_applies_to_current_candidate': False, 'current_candidate_runtime_admitted': False,
        'whole_crate_obligation_groups_open': 13, 'main_runtime_admitted': False,
        'all_39_properties_14_targets_and_crate_future_work_in_scope': True,
        'limits': [
            'The scalar proof verifies the shared static associated function, with a fieldless proof owner; expression-graph state and its three production call sites are not proved.',
            'Metadata contracts require normalized integer representation and a u64 aggregate bit-count bound. Constructors, arithmetic, derivation of the count bound, approximation truth and publication invariants remain open.',
            'The 453-unit dependency export includes copied mathematical models; its count is reported separately and is not added to the scalar count as unique coverage. Unannotated upstream implementations remain unproved.',
            'The local host proof imports the actual Cargo library and fresh export. Earlier cross-target compile checks and two-width models do not establish a complete 32-bit dependency proof.',
            'Dependency binding covers the configured workspace and standalone proof manifests. Published packages and other Cargo roots still require separate binding and release qualification.',
            'Native compilation and scalar tests do not establish matched cost, memory, size, concurrent publication, original-cap success or full-board admission. Historical v2335 runtime artifacts remain a separate immutable reference, not evidence for this changed candidate.',
            'The two retained matching-baseline algebraic SIGXCPU failures and all earlier rejected variants remain unresolved. EvaluationBudget excludes bytes, time and unmetered exact algebraic work.',
            'The corrected rejection sweep retains all 326 input/outcome controls but still exceeds the current 15-minute CI job locally; CI runtime qualification remains open.',
        ]}
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['status', 'source_files', 'changed_since_checkpoint',
        'scalar_verified_units', 'dependency_verified_units_per_build', 'fresh_distinct_acceptance_tests',
        'fresh_guard_baselines', 'fresh_rejected_controls', 'current_candidate_runtime_admitted']}))


if __name__ == '__main__':
    main()
