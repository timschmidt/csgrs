"""Finish metadata after the verified EOF correction's record writer failed."""
import ast
import gzip
import hashlib
import json
import marshal
from pathlib import Path
import platform
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
DEST = E / 'main-admission-v3'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not DEST.exists() and not (E / 'main-admission-v2/admission.json').exists()
    failure_log = E / 'main-admission-driver-v2.stderr'
    assert "AttributeError: 'str' object has no attribute 'read_bytes'" in failure_log.read_text()
    before_map = mapping(E / 'main-admission-v2/source-hashes-after.json.gz')
    assert len(before_map) == 10170 and all(sha(ROOT / name) == digest for name, digest in before_map.items())
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only', '-z'], cwd=ROOT)
    original = read(E / 'main-admission-v1/admission.json')
    name = 'crates/hyperreal/fuzz/directed/lexicase.py'
    source = ROOT / 'target/directed-scalar-bench-v2/source'
    before, after = (source / name).read_text(), (ROOT / name).read_text()
    assert before == after + '\n'
    before_ast, after_ast = [ast.dump(ast.parse(value), include_attributes=True) for value in [before, after]]
    before_code, after_code = [marshal.dumps(compile(value, name, 'exec')) for value in [before, after]]
    assert before_ast == after_ast and before_code == after_code
    delta = original['admitted_files_sha256'] | {name: sha(ROOT / name)}
    assert len(delta) == 19 and all(sha(ROOT / relative) == digest for relative, digest in delta.items())
    for version in range(1, 5):
        base = ROOT / f'target/directed-scalar-search-v{version}/source'
        assert all(sha(base / relative) == digest for relative, digest in mapping(E / f'source-hashes-v{version}.json.gz').items())
    assert all(sha(source / relative) == digest for relative, digest in mapping(E / 'competitive-source-v2.json.gz').items())
    native = read(ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json')['pending_native_sha256']
    assert len(native) == 35 and all(sha(ROOT / relative) == digest for relative, digest in native.items())
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {k: v for k, v in baseline.items() if k != 'current_work'}
    DEST.mkdir()
    (DEST / 'source-hashes-before.json.gz').write_bytes((E / 'main-admission-v2/source-hashes-after.json.gz').read_bytes())
    prefix = str(E.relative_to(ROOT)) + '/'
    entry = ledger['current_work']['directed_scalar_search']
    entry['admission'] = prefix + 'main-admission-v3/admission.json'
    entry['source_mapping'] = prefix + 'main-admission-v3/source-hashes-after.json.gz'
    ledger['current_work']['scalar_adversarial_selection']['standalone_directed_scalar_search'] = entry['admission']
    dump(ledger_path, ledger)
    final_map = before_map | {'docs/properties-status.json': sha(ledger_path)}
    assert all(sha(ROOT / relative) == digest for relative, digest in final_map.items())
    (DEST / 'source-hashes-after.json.gz').write_bytes(gzip.compress((json.dumps(final_map, sort_keys=True) + '\n').encode(), mtime=0))
    report = {
        'complete': True, 'admitted': True, 'admitted_files_sha256': delta,
        'qualified_admission': prefix + 'main-admission-v1/admission.json',
        'qualified_admission_sha256': sha(E / 'main-admission-v1/admission.json'),
        'format_only_file': name, 'bytes_removed': 1,
        'before_sha256': sha(source / name), 'after_sha256': sha(ROOT / name),
        'python_version': platform.python_version(),
        'ast_including_source_locations_identical': True, 'compiled_bytecode_identical': True,
        'ast_sha256': hashlib.sha256(before_ast.encode()).hexdigest(),
        'bytecode_sha256': hashlib.sha256(before_code).hexdigest(),
        'metadata_recovery': {'prior_terminal_exit_code_observed': 1,
            'reason': 'The v2 writer passed a string to a Path-only hash helper after verifying and writing the EOF correction and source maps. No numerical trial failed or was rerun.',
            'preserved_artifacts_sha256': {relative: sha(E / relative) for relative in [
                'admit_main_whitespace_v2.py', 'main-admission-driver-v2.stdout', 'main-admission-driver-v2.stderr',
                'main-admission-v2/source-hashes-before.json.gz', 'main-admission-v2/source-hashes-after.json.gz']}},
        'executed_frozen_source_copies_unchanged': True,
        'remaining_18_tooling_files_byte_identical_to_qualified_source': True,
        'production_arithmetic_geometry_public_apis_and_dependency_graph_changed': False,
        'pending_native_files_preserved': 35, 'non_current_work_ledger_unchanged': True,
        'source_mapping_before_sha256': sha(DEST / 'source-hashes-before.json.gz'),
        'source_mapping_after_sha256': sha(DEST / 'source-hashes-after.json.gz'),
        'admission_script_sha256': sha(__file__), 'limits': original['limits'],
        'P38_complete': False, 'plan_complete': False,
    }
    dump(DEST / 'admission.json', report)
    print(json.dumps({'admitted': True, 'files': len(delta), 'bytecode_identical': True,
                      'admission_sha256': sha(DEST / 'admission.json')}))


if __name__ == '__main__':
    main()
