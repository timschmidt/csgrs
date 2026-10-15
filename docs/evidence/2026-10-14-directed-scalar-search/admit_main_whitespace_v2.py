"""Remove one EOF blank line while preserving qualified Python AST and bytecode."""
import ast
import gzip
import hashlib
import json
import marshal
import platform
import subprocess

from close_v1 import E, ROOT, mapping, read, sha

DEST = E / 'main-admission-v2'


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not DEST.exists()
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only', '-z'], cwd=ROOT)
    before_map = mapping(E / 'main-admission-v1/source-hashes-after.json.gz')
    assert len(before_map) == 10170 and all(sha(ROOT / name) == digest for name, digest in before_map.items())
    name = 'crates/hyperreal/fuzz/directed/lexicase.py'
    path = ROOT / name
    before = path.read_text()
    after = before.rstrip('\n') + '\n'
    assert before == after + '\n'
    before_ast = ast.dump(ast.parse(before), include_attributes=True)
    after_ast = ast.dump(ast.parse(after), include_attributes=True)
    assert before_ast == after_ast
    before_code = marshal.dumps(compile(before, name, 'exec'))
    after_code = marshal.dumps(compile(after, name, 'exec'))
    assert before_code == after_code
    # Keep each previously executed source copy intact; a closed campaign is
    # replayed with its frozen source version rather than a rewritten archive.
    for version in range(1, 5):
        table = mapping(E / f'source-hashes-v{version}.json.gz')
        base = ROOT / f'target/directed-scalar-search-v{version}/source'
        assert all(sha(base / relative) == digest for relative, digest in table.items())
    source = ROOT / 'target/directed-scalar-bench-v2/source'
    assert all(sha(source / relative) == digest for relative, digest in mapping(E / 'competitive-source-v2.json.gz').items())
    original = read(E / 'main-admission-v1/admission.json')
    assert original['admitted'] and sha(path) == original['admitted_files_sha256'][name]
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {k: v for k, v in baseline.items() if k != 'current_work'}
    DEST.mkdir()
    (DEST / 'source-hashes-before.json.gz').write_bytes((E / 'main-admission-v1/source-hashes-after.json.gz').read_bytes())
    path.write_text(after)
    prefix = str(E.relative_to(ROOT)) + '/'
    entry = ledger['current_work']['directed_scalar_search']
    entry['admission'] = prefix + 'main-admission-v2/admission.json'
    entry['source_mapping'] = prefix + 'main-admission-v2/source-hashes-after.json.gz'
    entry['formatting_followup'] = {
        'file': name, 'change': 'One extra EOF blank line removed',
        'qualified_ast_and_bytecode_identical': True,
        'frozen_campaign_sources_preserved': True,
    }
    ledger['current_work']['scalar_adversarial_selection']['standalone_directed_scalar_search'] = entry['admission']
    dump(ledger_path, ledger)
    after_map = before_map | {name: sha(path), 'docs/properties-status.json': sha(ledger_path)}
    assert all(sha(ROOT / relative) == digest for relative, digest in after_map.items())
    (DEST / 'source-hashes-after.json.gz').write_bytes(gzip.compress((json.dumps(after_map, sort_keys=True) + '\n').encode(), mtime=0))
    delta = original['admitted_files_sha256'] | {name: sha(path)}
    report = {
        'complete': True, 'admitted': True, 'admitted_files_sha256': delta,
        'qualified_admission': prefix + 'main-admission-v1/admission.json',
        'qualified_admission_sha256': sha(E / 'main-admission-v1/admission.json'),
        'format_only_file': name, 'bytes_removed': 1,
        'before_sha256': before_map[name], 'after_sha256': sha(path),
        'python_version': platform.python_version(),
        'ast_including_source_locations_identical': True, 'compiled_bytecode_identical': True,
        'ast_sha256': hashlib.sha256(before_ast.encode()).hexdigest(),
        'bytecode_sha256': hashlib.sha256(before_code).hexdigest(),
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
