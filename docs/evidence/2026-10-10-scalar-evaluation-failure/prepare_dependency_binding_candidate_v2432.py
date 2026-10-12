"""Freeze and apply the qualified dependency identity infrastructure to SOURCE only."""
import ast
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'dependency-binding-baseline-v2432'
PREVIEW = E / 'dependency-binding-candidate-v2424'
PREFIX = 'crates/hyperreal/'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

OLD = '''The verifier builds `verification/dependencies` with its pinned Rust compiler
and checks every registry dependency against the production lockfile. A
transparent type declaration imports the actual `num::bigint::Sign` variants;
it assumes no equality or arithmetic implementation. Dependency source and
artifact hashes are checked before and after proof checking and retained in
the evidence. Derived sign equality and unsupported standard-library methods
remain proof obligations. Conditional method markers use the pinned Verus
attribute implementation and disappear from normal builds.
'''
NEW = '''The verifier builds `verification/dependencies` with its pinned Rust compiler.
Registry pins must agree with the production lockfile. Every resolved proof
dependency, including local path packages, must also be reachable from the
production crate and use the same canonical manifest path, name, version and
source. An unrelated workspace package with matching names is insufficient.
The runner hashes all dependency input files and compiled artifacts, records
Cargo configuration, and rechecks both resolutions after building and proof
checking. Added or removed inputs and changed paths invalidate the evidence.
Source directory symlinks are rejected rather than hiding an unrecorded tree.

A transparent type declaration imports the actual `num::bigint::Sign` variants;
it assumes no equality or arithmetic implementation. Dependency identity checks
do not prove those implementations. Derived sign equality and unsupported
standard-library methods remain proof obligations. Conditional method markers
use the pinned Verus attribute implementation and disappear from normal builds.
'''


def main():
    action = sys.argv[1]
    assert action in {'prepare', 'apply'}
    checkpoint_path = E / 'typed-api-prototype-v2386-state.json'
    assert sha(checkpoint_path) == '6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9'
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint['source_record'])
    guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    if action == 'prepare':
        assert not OUT.exists()
        units = E / 'dependency-binding-units-v2425'
        assert read(units / 'result.json')['nineteen_tests_passed']
        proposed = read(units / 'protocol.json')['preview_files_sha256']
        assert all(sha(PREVIEW / n) == h for n, h in proposed.items())
        actual = E / 'dependency-binding-cargo-v2427/result.json'
        assert read(actual)['complete']
        workspace = E / 'workspace-dependency-binding-v2431/result.json'
        assert read(workspace)['source_unchanged'] and len(read(workspace)['bindings']) == 8
        catalogue = E / 'candidate-snapshot-validation-v2430.json'
        assert read(catalogue)['thirteen_tests_passed'] and read(catalogue)['retained_source_snapshot_identical']
        data = {PREFIX + n: (PREVIEW / n).read_bytes() for n in proposed}
        name = PREFIX + 'verification/README.md'
        text = (SOURCE / name).read_text()
        assert text.count(OLD) == 1
        data[name] = text.replace(OLD, NEW).encode()
        OUT.mkdir()
        rows = {}
        patches = []
        for name, content in sorted(data.items()):
            original = (SOURCE / name).read_bytes()
            before, after = OUT / 'original' / name, OUT / 'candidate' / name
            before.parent.mkdir(parents=True, exist_ok=True)
            after.parent.mkdir(parents=True, exist_ok=True)
            before.write_bytes(original)
            after.write_bytes(content)
            if name.endswith('.py'):
                ast.parse(content)
            rows[name] = {'before_sha256': sha(before), 'after_sha256': sha(after)}
            patches.extend(difflib.unified_diff(original.decode().splitlines(True), content.decode().splitlines(True),
                fromfile='before/' + name, tofile='after/' + name, n=0))
        (OUT / 'candidate.patch').write_text(''.join(patches))
        old_dependencies = E / 'typed-api-prototype-v2380-word-scan-verus/proof-reports/dependencies.json'
        dependency_artifacts = read(old_dependencies)['artifacts']
        assert all(sha(Path(n)) == h for n, h in dependency_artifacts.items())
        protocol = {
            'status': 'Frozen before applying dependency identity infrastructure to SOURCE',
            'preparation_sha256': sha(Path(__file__)), 'retained_checkpoint': checkpoint_path.name,
            'retained_checkpoint_sha256': sha(checkpoint_path), 'source_record_sha256': checkpoint['source_record_sha256'],
            'files': rows, 'patch_sha256': sha(OUT / 'candidate.patch'),
            'unit_result_sha256': sha(units / 'result.json'), 'cargo_fixture_result_sha256': sha(actual),
            'workspace_binding_result_sha256': sha(workspace), 'snapshot_validation_sha256': sha(catalogue),
            'previous_proof_dependency_report_sha256': sha(old_dependencies),
            'previous_proof_dependency_artifacts_sha256': dependency_artifacts,
            'validation_requirements': ['Full pinned --no-cheating proof target with the new checker',
                'Nineteen dependency identity unit tests against the applied source',
                'Acceptance guards and both existing formatting commands',
                'Exact proof-input/dependency/tool audit before reusing prior negative guards or runtime evidence'],
            'scope': 'Two verification Python files and their README description only. No Rust, Cargo configuration, dependency package, proof contract/model or runtime source changes. The BigUint dependency preview remains outside SOURCE. Retained proof counts and all thirteen whole-crate groups stay unchanged pending the fresh infrastructure validation.',
            'all_39_properties_14_targets_and_crate_future_work_in_scope': True,
        }
        (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
        print(json.dumps({'frozen_files': len(rows), 'source_applied': False}))
    else:
        protocol = read(OUT / 'protocol.json')
        assert not (OUT / 'application.json').exists()
        assert protocol['preparation_sha256'] == sha(Path(__file__))
        assert protocol['source_record_sha256'] == checkpoint['source_record_sha256']
        assert protocol['patch_sha256'] == sha(OUT / 'candidate.patch')
        for name, row in protocol['files'].items():
            assert sha(SOURCE / name) == row['before_sha256']
            assert sha(OUT / 'original' / name) == row['before_sha256']
            assert sha(OUT / 'candidate' / name) == row['after_sha256']
        for name in protocol['files']:
            (SOURCE / name).write_bytes((OUT / 'candidate' / name).read_bytes())
        expected = sources | {n: r['after_sha256'] for n, r in protocol['files'].items()}
        assert all(sha(SOURCE / n) == h for n, h in expected.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / 'source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(expected, sort_keys=True) + '\n').encode(), mtime=0))
        result = {'protocol_sha256': sha(OUT / 'protocol.json'), 'source_applied': True,
            'source_record_sha256': sha(OUT / 'source-hashes.json.gz'), 'source_files': len(expected),
            'changed_source_files': list(protocol['files']), 'main_unchanged': True,
            'runtime_source_files_changed': 0, 'retained_proof_counts_changed': False}
        (OUT / 'application.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))


if __name__ == '__main__':
    main()
