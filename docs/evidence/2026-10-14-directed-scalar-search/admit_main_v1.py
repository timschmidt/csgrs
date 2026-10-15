"""Admit byte-identical development tooling after the closed search and benchmark gates."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
DEST = E / 'main-admission-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def inventory(base):
    return {str(p.relative_to(base)): sha(p) for p in base.rglob('*') if p.is_file()}


def main():
    assert not DEST.exists()
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only', '-z'], cwd=ROOT)
    roots = mapping(E / 'main-source-before.json.gz')
    assert len(roots) == 10153 and all(sha(ROOT / name) == digest for name, digest in roots.items())
    scope = read(E / 'competitive-scope-v2.json')
    source = Path(scope['source'])
    sources = mapping(E / 'competitive-source-v2.json.gz')
    delta = scope['expected_delta_sha256']
    assert len(delta) == 19 and len(sources) == 10170
    assert inventory(source) == sources == roots | delta
    assert all(name in ['crates/hyperreal/Cargo.toml', 'crates/hyperreal/fuzz/README.md'] or
               name.startswith(('crates/hyperreal/examples/', 'crates/hyperreal/fuzz/directed/')) for name in delta)
    before_manifest = tomllib.loads((ROOT / 'crates/hyperreal/Cargo.toml').read_text())
    after_manifest = tomllib.loads((source / 'crates/hyperreal/Cargo.toml').read_text())
    before_examples = {row['name']: row for row in before_manifest.pop('example')}
    after_examples = {row['name']: row for row in after_manifest.pop('example')}
    assert before_manifest == after_manifest
    assert after_examples == before_examples | {
        'directed_scalar_probe': {'name': 'directed_scalar_probe', 'required-features': ['serde', 'dispatch-trace']},
        'directed_scalar_bench': {'name': 'directed_scalar_bench', 'required-features': ['serde']},
    }
    for directory, closure_name in [('qualification-v4', 'closure-v4.json'),
                                     ('competitive-qualification-v2', 'competitive-closure-v2.json')]:
        out = E / directory
        result, protocol, closure = read(out / 'result.json'), read(out / 'protocol.json'), read(E / closure_name)
        assert not (out / 'failure.json').exists()
        assert result['complete'] and result['commands_all_zero'] == 13
        assert closure['driver_terminal_exit_code_observed'] == 0
        assert closure['qualification_result_sha256'] == sha(out / 'result.json')
        assert all(sha(out / name) == digest for name, digest in closure['trial_artifacts_sha256'].items())
        assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
        assert all(sha(E / name) == digest for name, digest in closure['driver_logs_sha256'].items())
        binaries = [*read(out / 'binaries.json').values(), *protocol['preserved_binaries']]
        assert all(sha(row['path']) == row['sha256'] for row in binaries)
        commands = read(out / 'commands.json')
        assert len(commands) == 13 and all(row['exit_code'] == 0 for row in commands)
    search = read(E / 'closure-v4.json')
    comparison = read(E / 'competitive-closure-v2.json')
    assert (search['campaign_probes'], search['selection_events'], search['retained_inputs']) == (528, 264, 59)
    assert (comparison['inputs'], comparison['native_samples']) == (59, 1298)
    for version in range(1, 5):
        old = ROOT / f'target/directed-scalar-search-v{version}/source'
        assert inventory(old) == mapping(E / f'source-hashes-v{version}.json.gz')
    assert inventory(ROOT / 'target/directed-scalar-bench-v1/source') == mapping(E / 'competitive-unexecuted-source-v1.json.gz')
    # The completed campaign's executable sources are unchanged in the admitted
    # combined prototype. Only the manifest, explanatory README and new native
    # comparison implementation differ from the qualified search source.
    v4 = mapping(E / 'source-hashes-v4.json.gz')
    assert {name: digest for name, digest in sources.items() if v4.get(name) != digest} == scope['changes_relative_to_qualified_search']
    assert len(scope['changes_relative_to_qualified_search']) == 6
    native = read(ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json')['pending_native_sha256']
    assert len(native) == 35 and not set(native).intersection(delta)
    assert all(sha(ROOT / name) == digest for name, digest in native.items())
    for name in delta:
        assert (sha(ROOT / name) == roots[name]) if name in roots else not (ROOT / name).exists()
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    original = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {k: v for k, v in original.items() if k != 'current_work'}
    assert 'directed_scalar_search' not in ledger['current_work']
    DEST.mkdir()
    (DEST / 'source-hashes-before.json.gz').write_bytes((E / 'main-source-before.json.gz').read_bytes())
    for name in delta:
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((source / name).read_bytes())
    prefix = str(E.relative_to(ROOT)) + '/'
    entry = {
        'status': 'finite-scalar-lexicase-and-native-enclosure-tooling-admitted',
        'admitted': True, 'development_tooling_only': True,
        'contract': prefix + 'contract-v4.json', 'comparison_contract': prefix + 'competitive-contract-v2.json',
        'search_qualification': prefix + 'qualification-v4/result.json',
        'search_closure': prefix + 'closure-v4.json',
        'comparison_qualification': prefix + 'competitive-qualification-v2/result.json',
        'comparison_closure': prefix + 'competitive-closure-v2.json',
        'admission': prefix + 'main-admission-v1/admission.json',
        'source_mapping': prefix + 'main-admission-v1/source-hashes-after.json.gz',
        'representations': 22, 'generations': 3, 'campaign_probes': 528, 'selection_events': 264,
        'mutated_descendants': 247, 'retained_selected_inputs': 59, 'native_comparison_samples': 1298,
        'samples_per_backend_per_input': 11, 'maximum_absolute_enclosure_width': comparison['maximum_absolute_width'],
        'python_tests': 33, 'search_rust_tests_each_configuration': 7, 'adapter_rust_tests_each_configuration': 3,
        'sanitizers': 'ASan/LSan on Rust targets and rebuilt standard library; native GMP/MPFR not claimed instrumented',
        'prior_failed_trials': [prefix + f'closure-v{v}.json' for v in [1, 2, 3]],
        'numeric_and_selection_evidence_replayed': True,
        'finite_scalar_campaign_complete': True, 'finite_scalar_interval_comparison_complete': True,
        'limits': search['limits'][:4] + comparison['limits'],
        'remaining': ['Directed representation populations in the remaining stack layers',
                      'Exhaustive coverage and additional compositions, degeneracies and policies',
                      'Remaining formal verification, production runtime and native geometry admission obligations'],
        'P38_complete': False, 'plan_complete': False,
    }
    ledger['current_work']['directed_scalar_search'] = entry
    legacy = ledger['current_work']['scalar_adversarial_selection']
    legacy['status'] = 'legacy-timing-promotion-described-accurately-standalone-scalar-search-admitted'
    legacy['legacy_selection_algorithm_changed'] = legacy.pop('selection_algorithm_changed')
    legacy['independent_representation_populations_implemented'] = True
    legacy['standalone_directed_scalar_search'] = entry['admission']
    ledger['current_work']['plan_complete'] = False
    dump(ledger_path, ledger)
    after = roots | delta | {'docs/properties-status.json': sha(ledger_path)}
    assert len(after) == 10170 and all(sha(ROOT / name) == digest for name, digest in after.items())
    assert all(sha(ROOT / name) == digest for name, digest in native.items())
    assert {k: v for k, v in read(ledger_path).items() if k != 'current_work'} == {k: v for k, v in original.items() if k != 'current_work'}
    (DEST / 'source-hashes-after.json.gz').write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    dump(DEST / 'admission.json', {
        'complete': True, 'admitted': True, 'admitted_files_sha256': delta,
        'all_admitted_files_byte_identical_to_qualified_source': True,
        'production_arithmetic_geometry_public_apis_and_dependency_graph_changed': False,
        'manifest_change': 'Two development examples, with existing serde and dispatch-trace feature requirements',
        'pending_native_files_preserved': 35, 'non_current_work_ledger_unchanged': True,
        'search_qualification_result_sha256': sha(E / 'qualification-v4/result.json'),
        'search_closure_sha256': sha(E / 'closure-v4.json'),
        'comparison_qualification_result_sha256': sha(E / 'competitive-qualification-v2/result.json'),
        'comparison_closure_sha256': sha(E / 'competitive-closure-v2.json'),
        'source_mapping_before_sha256': sha(DEST / 'source-hashes-before.json.gz'),
        'source_mapping_after_sha256': sha(DEST / 'source-hashes-after.json.gz'),
        'admission_script_sha256': sha(Path(__file__)),
        'admission_method': 'Copy the exact fully qualified source bytes; verify full main source mapping, manifest semantics and preserved native edits. Closed numerical trials are not rerun or pooled.',
        'limits': entry['limits'], 'P38_complete': False, 'plan_complete': False,
    })
    print(json.dumps({'admitted': True, 'files': len(delta), 'source_files_after': len(after),
                      'admission_sha256': sha(DEST / 'admission.json'), 'pending_native_preserved': len(native)}))


if __name__ == '__main__':
    main()
