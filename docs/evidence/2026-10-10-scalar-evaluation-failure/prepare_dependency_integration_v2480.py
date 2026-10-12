"""Freeze and apply the qualified dependency/producer integration to the isolated candidate."""
import ast
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'dependency-integration-v2480'
PREFIX = 'crates/hyperreal/'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))
SPEC = importlib.util.spec_from_file_location('catalog_v2480', E / 'validate_candidate_v3.py')
catalog = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(catalog)


def producer_guard():
    tree = ast.parse((E / 'check_approximation_bound_guards_v2462.py').read_text())
    mutations = next(ast.literal_eval(node.value) for node in tree.body
                     if isinstance(node, ast.Assign) and any(
                         isinstance(target, ast.Name) and target.id == 'MUTATIONS' for target in node.targets))
    prefix = '''#!/usr/bin/env python3
"""Reject incorrect, weakened and premise-free production approximation bounds."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_verus import ROOT, PIN, REPORTS, verifier  # noqa: E402
from verus_dependencies import prepare_dependencies, check_dependency_identity  # noqa: E402

'''
    suffix = '''

def run_rejections(verus, dependency_args):
    with tempfile.TemporaryDirectory(prefix='hyperreal-approximation-rejections-') as temporary:
        root = Path(temporary)
        shutil.copytree(ROOT / 'src', root / 'src')
        (root / 'verification').mkdir()
        for path in (ROOT / 'verification').glob('*.rs'):
            shutil.copy2(path, root / 'verification' / path.name)
        path = root / 'src/computable/node/approximation_bound.rs'
        original = path.read_text()
        command = [str(verus), '--edition=2024', '--crate-type=lib', '--no-cheating', '--output-json',
                   *dependency_args, '--verify-only-module', 'computable_bounds::approximation',
                   '--verify-function', 'bound_from_approx', str(root / 'verification/lib.rs')]
        variants = [('baseline', 'baseline', original)]
        for name, kind, before, after in MUTATIONS:
            if original.count(before) != 1:
                raise RuntimeError('Producer mutation anchor is not unique: ' + name)
            variants.append((name, kind, original.replace(before, after)))
        for name, kind, text in variants:
            path.write_text(text)
            result = subprocess.run(command, capture_output=True, text=True)
            path.write_text(original)
            report = json.loads(result.stdout).get('verification-results', {})
            if kind == 'baseline':
                passed = (result.returncode == 0 and report.get('verified') == 1 and report.get('errors') == 0
                          and all(not line.strip() or line.startswith('note: verifying ')
                                  for line in result.stderr.splitlines()))
            else:
                failures = ['postcondition not satisfied', 'assertion failed', 'precondition not satisfied',
                            'requires not satisfied', 'possible arithmetic underflow/overflow']
                passed = (result.returncode != 0 and report.get('errors', 0) > 0
                          and not report.get('encountered-vir-error', True)
                          and any(message in result.stderr for message in failures))
            if not passed:
                raise RuntimeError('Unexpected producer control result for ' + name + ':\\n'
                                   + result.stdout + result.stderr)
            print(('Verified ' if kind == 'baseline' else 'Rejected ') + 'approximation producer: ' + name,
                  flush=True)


def main():
    verus = verifier()
    arguments, evidence = prepare_dependencies(ROOT, PIN, REPORTS, verus_path=verus)
    run_rejections(verus, arguments)
    check_dependency_identity(evidence)


if __name__ == '__main__':
    main()
'''
    return prefix + 'MUTATIONS = ' + repr(mutations) + '\n' + suffix


def main():
    checkpoint = E / 'typed-api-prototype-v2440-state.json'
    assert sha(checkpoint) == '1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0'
    state = read(checkpoint)
    before = snapshot(E / state['source_record'])
    original = snapshot(E / 'candidate-preparation/source-hashes.json.gz')
    main_guard = snapshot(E / 'candidate-preparation/main-hashes.json.gz')
    assert catalog.source_hashes(SOURCE, original) == before
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    if sys.argv[1] == 'apply':
        protocol = read(OUT / 'protocol.json')
        assert protocol['runner_sha256'] == sha(Path(__file__))
        assert not (OUT / 'applied.json').exists()
        assert all(sha(OUT / 'candidate' / n) == h for n, h in protocol['overlay_files_sha256'].items())
        for name in protocol['overlay_files_sha256']:
            destination = SOURCE / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(OUT / 'candidate' / name, destination)
        after = catalog.source_hashes(SOURCE, original)
        assert after == before | protocol['overlay_files_sha256']
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        record = OUT / 'working-source-hashes.json.gz'
        record.write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
        result = {'protocol_sha256': sha(OUT / 'protocol.json'), 'source_record_sha256': sha(record),
                  'source_files': len(after), 'changed_since_checkpoint': len(protocol['overlay_files_sha256']),
                  'main_unchanged': True, 'retained_checkpoint_changed': False, 'runtime_admitted': False}
        (OUT / 'applied.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
        return
    assert sys.argv[1] == 'prepare' and not OUT.exists()
    proposal = E / 'dependency-wiring-proposal-v2476'
    preview = E / 'approximation-bound-candidate-v2460'
    package = E / 'bigint-portable-package-v2466'
    receipts = {}
    for phase in ['dependency-wiring-units-v2477', 'dependency-wiring-cargo-v2478',
                  'dependency-wiring-producer-v2479']:
        result = read(E / phase / 'result.json')
        assert result['gate_passed']
        assert result['protocol_sha256'] == sha(E / phase / 'protocol.json')
        receipts[phase] = sha(E / phase / 'result.json')
    wiring = read(E / 'dependency-wiring-cargo-v2478/protocol.json')
    assert all(sha(proposal / n) == h for n, h in wiring['proposal_files_sha256'].items())
    qualified = read(preview / 'protocol.json')
    assert all(sha(preview / n) == h for n, h in qualified['preview_files_sha256'].items())
    packaged = read(package / 'protocol.json')
    assert all(sha(package / 'vendor/num-bigint' / n) == h for n, h in packaged['vendor_files_sha256'].items())
    files = {}

    def put(name, data):
        files[name] = data.encode() if isinstance(data, str) else data

    def edit(name, old, new):
        text = files.get(name, (SOURCE / name).read_bytes()).decode()
        assert text.count(old) == 1, (name, old)
        put(name, text.replace(old, new))

    edit('Cargo.toml', '  "crates/hyperreal/verification/dependencies",',
         '  "crates/hyperreal/verification/dependencies",\n  "crates/hyperreal/vendor/num-bigint",')
    edit('Cargo.toml', '[patch.crates-io]\n',
         '[patch.crates-io]\nnum-bigint = { path = "crates/hyperreal/vendor/num-bigint" }\n')
    dependency_manifest = PREFIX + 'verification/dependencies/Cargo.toml'
    put(dependency_manifest, (SOURCE / dependency_manifest).read_text()
        + '\n[patch.crates-io]\nnum-bigint = { path = "../../vendor/num-bigint" }\n')
    for name in ['Cargo.lock', PREFIX + 'verification/dependencies/Cargo.lock']:
        edit(name, 'name = "num-bigint"\nversion = "0.4.6"\n'
             'source = "registry+https://github.com/rust-lang/crates.io-index"\n'
             'checksum = "a5e44f723f1133c9deac646763579fdb3ac745e418f2a7af9cd0c431da1f20b9"\n',
             'name = "num-bigint"\nversion = "0.4.6"\n')
    for name in packaged['vendor_files_sha256']:
        put(PREFIX + 'vendor/num-bigint/' + name, (package / 'vendor/num-bigint' / name).read_bytes())
    put(PREFIX + 'vendor/num-bigint/Cargo.toml', (E / 'bigint-portable-repackage-v2473/Cargo.toml').read_bytes())
    for path in (preview / 'candidate').rglob('*'):
        if path.is_file():
            put(str(path.relative_to(preview / 'candidate')), path.read_bytes())
    for name in ['verus_bigint.py', 'verus_rustc_wrapper.py', 'verus_dependencies.py']:
        put(PREFIX + 'scripts/' + name, (proposal / name).read_bytes())
    export_test = (proposal / 'test_verified_export.py').read_text()
    export_test = export_test.replace('import tempfile\n', 'import tempfile\nimport sys\n')
    export_test = export_test.replace('import verus_bigint as binding',
        "sys.dont_write_bytecode = True\nsys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))\n\nimport verus_bigint as binding")
    put(PREFIX + 'verification/test_verified_export.py', export_test)
    edit(PREFIX + 'verification/test_acceptance.py',
         'from test_dependency_identity import DependencyIdentityTests  # noqa: F401\n',
         'from test_dependency_identity import DependencyIdentityTests  # noqa: F401\n'
         'from test_verified_export import VerifiedExportTests  # noqa: F401\n')
    edit(PREFIX + 'scripts/check_verification_acceptance.py',
         '"src/**/*", "tests/**/*", "benches/**/*", "examples/**/*",',
         '"src/**/*", "tests/**/*", "benches/**/*", "examples/**/*", "vendor/**/*",')
    edit(PREFIX + 'verification/test_acceptance.py',
         '"verification/dependencies/src/lib.rs"]:',
         '"verification/dependencies/src/lib.rs", "vendor/num-bigint/src/lib.rs",\n'
         '                     "vendor/num-bigint/verification/provenance.json"]:')
    edit(PREFIX + 'scripts/verify_verus.py', 'ROOT / "scripts/verus_dependencies.py",',
         'ROOT / "scripts/verus_dependencies.py", ROOT / "scripts/verus_bigint.py",\n'
         '                    ROOT / "scripts/verus_rustc_wrapper.py",')
    edit(PREFIX + 'scripts/verify_verus.py', 'prepare_dependencies(ROOT, PIN, REPORTS)',
         'prepare_dependencies(ROOT, PIN, REPORTS, verus_path=path)')
    bounds = (SOURCE / PREFIX / 'verification/computable_bounds.rs').read_text()
    start, end = bounds.index('verus! {'), bounds.index('include!(')
    bounds = bounds[:start] + bounds[end:]
    put(PREFIX + 'verification/computable_bounds.rs', bounds.replace('//!', '//'))
    edit(PREFIX + 'verification/lib.rs', 'mod computable_bounds;',
         'mod computable_bounds {\n    include!("computable_bounds.rs");\n'
         '    mod approximation {\n        include!("approximation_bound.rs");\n    }\n}')
    put(PREFIX + 'verification/approximation_bound.rs',
        '// Verify the shared static producer; expression graph state remains outside this proof.\n'
        'use super::*;\nuse num::bigint::BigInt;\n\n#[verus_verify]\nstruct Computable;\n'
        'include!("../src/computable/node/approximation_bound.rs");\n')
    put(PREFIX + 'verification/test_approximation_bound_rejections.py', producer_guard())
    edit(PREFIX + 'verification/test_rejections.py', '    reject_word_scan_mutations()\n',
         '    reject_word_scan_mutations()\n    reject_approximation_bound_mutations()\n')
    script = PREFIX + 'verification/test_rejections.py'
    text = files[script].decode()
    anchor = 'from test_bound_rejections import main as reject_bound_mutations'
    assert text.count(anchor) == 1
    put(script, text.replace(anchor,
        'from test_approximation_bound_rejections import main as reject_approximation_bound_mutations\n' + anchor))
    scope = read(SOURCE / PREFIX / 'verification/scope.json')
    scope['verified_executable_contracts'].append('computable_bounds::approximation::Computable::bound_from_approx')
    assert len(scope['verified_executable_contracts']) == 99 and len(scope['obligations']) == 13
    put(PREFIX + 'verification/scope.json', json.dumps(scope, indent=2) + '\n')
    doc = PREFIX + 'verification/README.md'
    edit(doc, 'not verify the BigInt bit-pattern producer.',
         'not alone verify the BigInt bit-pattern producer. The shared producer is now\n'
         'checked separately against the actual imported BigInt metadata contracts,\n'
         'with normalized representation and a representable storage-bit count as\n'
         'explicit premises. Its caller must still establish approximation validity.')
    edit(doc, 'A transparent type declaration imports the actual `num::bigint::Sign` variants;\n'
         'it assumes no equality or arithmetic implementation. Dependency identity checks\n'
         'do not prove those implementations.',
         'The local `vendor/num-bigint` package supplies the actual `Sign`, `BigInt`\n'
         'and `BigUint` proof imports. A fresh Cargo build verifies and compiles its\n'
         'metadata methods with the pinned verifier, then binds the VIR export to\n'
         'that same Cargo library. Source catalogs, copied model provenance, compiler\n'
         'arguments and receipts, library bytes and export bytes are checked before\n'
         'and after the scalar proof. Missing, changed or unrelated exports fail closed.\n'
         'This does not prove unannotated dependency arithmetic or constructors.')
    edit(doc, 'from the actual verified helper. BigUint storage and normalization, whole\n'
         "digit scans, metadata range checks and the bound producer's caller remain\n"
         'outside this conditional model boundary; runtime evaluation order is unchanged.',
         'from the actual verified helper. The vendored BigUint storage views and\n'
         'metadata scans now refine these models. Normalization and count-range\n'
         "premises still require caller proofs; runtime evaluation order is unchanged.")
    edit(doc, 'do not verify the dependency\'s storage, actual iterator scan, count range,\n'
         'sign import or producer/caller connection. Runtime evaluation order is unchanged.',
         'remain conditional. The imported dependency now connects its storage and\n'
         'metadata scans to these models and its sign/magnitude methods to the shared\n'
         'bound producer. The constructor and producer-caller premises remain open.')
    edit(doc, 'Actual storage imports, normalization/iterator execution and producer calls\n'
         'still need implementation refinement proofs.',
         'The local dependency now verifies storage-backed metadata and suffix trimming\n'
         'through its actual shared bodies. Its other normalization policy, arithmetic,\n'
         'allocation-derived count bounds and callers remain open obligations.')
    edit(doc, 'These are conditional model theorems. The actual BigInt metadata/import,\n'
         'bound_from_approx caller, valid approximation and publication invariant\n'
         'remain outside this proof boundary. No runtime code or metadata evaluation\n'
         'order changes, and all thirteen whole-crate obligation groups stay open.',
         'The shared production `bound_from_approx` body now uses the imported BigInt\n'
         'and BigUint contracts to establish the precise output and its conditional\n'
         'denotation. Magnitudes at most one remain Unknown; exponent overflow drops\n'
         'only magnitude information; all-ones and power-of-two boundaries keep their\n'
         'original behavior. The trailing-zero scan remains lazy after a representable\n'
         'exponent. Twelve mutation controls reject contradictory or weakened outputs\n'
         'and removal of the normalization/count premises. The approximation-truth,\n'
         'producer-caller and publication invariants remain open. All thirteen\n'
         'whole-crate obligation groups stay open; runtime admission is separate.')
    OUT.mkdir()
    for name, data in sorted(files.items()):
        assert before.get(name) != hashlib.sha256(data).hexdigest(), name
        destination = OUT / 'candidate' / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        if name.endswith('/verus_rustc_wrapper.py'):
            destination.chmod(0o755)
        if name in before:
            original_path = OUT / 'original' / name
            original_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(SOURCE / name, original_path)
    protocol = {'status': 'Frozen isolated integration before application and validation',
        'runner_sha256': sha(Path(__file__)), 'checkpoint_sha256': sha(checkpoint),
        'previous_source_record_sha256': state['source_record_sha256'], 'qualified_result_sha256': receipts,
        'overlay_files_sha256': {n: hashlib.sha256(data).hexdigest() for n, data in sorted(files.items())},
        'prior_files_sha256': {n: before[n] for n in sorted(files) if n in before},
        'existing_runtime_checkpoint': 'typed-api-prototype-v2335-state.json',
        'expected_scalar_verified_units': 737, 'expected_dependency_verified_units': 453,
        'expected_production_contracts': 99, 'expected_model_theorems': 178, 'open_obligation_groups': 13,
        'scope': 'Apply only to the isolated candidate. Workspace and standalone proof resolve the same local dependency. The shared producer preserves runtime statements and lazy metadata order. Conditional static proof, guard and native validation are required before retention; this is not main/runtime admission, a caller invariant proof or completion of properties.md.'}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'overlay_files': len(files), 'prior_files': len(protocol['prior_files_sha256']),
                      'protocol_sha256': sha(OUT / 'protocol.json')}))


if __name__ == '__main__':
    main()
