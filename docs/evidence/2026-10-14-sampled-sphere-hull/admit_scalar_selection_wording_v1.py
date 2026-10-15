"""Verify and admit only the qualified benchmark-description and README correction."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'scalar-selection-wording-v1'
DEST = E / 'scalar-selection-wording-admission-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not DEST.exists() and not (OUT / 'failure.json').exists()
    result = read(OUT / 'result.json')
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'scalar-selection-wording-scope-v1.json')
    base = mapping(E / 'structural-fuzz-inventory-admission-v5/source-hashes-after.json.gz')
    source = Path(protocol['source'])
    sources = mapping(OUT / 'source-hashes.json.gz')
    delta = scope['expected_delta_sha256']
    assert result['complete'] and result['commands_all_zero'] == 2
    assert sources == base | delta and len(delta) == 2
    for directory, table in [(ROOT, base), (source, sources), (OUT, result['artifacts_sha256']), (E, protocol['inputs_sha256'])]:
        assert all(sha(directory / name) == digest for name, digest in table.items())
    assert all(sha(row['path']) == row['sha256'] for row in protocol['preserved_binaries'])
    commands = read(OUT / 'commands.json')
    assert [(row['name'], row['exit_code']) for row in commands] == [('format', 0), ('benchmark-clippy', 0)]
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    name = 'crates/hyperreal/benches/adversarial_library.rs'
    before, after = (ROOT / name).read_text(), (source / name).read_text()
    old, new = scope['literal_replacements'][name][0]
    assert before.count(old) == after.count(new) == 1 and after.replace(new, old) == before
    assert set(delta) == {name, 'crates/hyperreal/fuzz/README.md'}
    pending = read(ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json')['pending_native_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    original = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {k: v for k, v in original.items() if k != 'current_work'}
    assert 'scalar_adversarial_selection' not in ledger['current_work']
    DEST.mkdir()
    for name in delta:
        (ROOT / name).write_bytes((source / name).read_bytes())
    prefix = str(E.relative_to(ROOT)) + '/'
    ledger['current_work']['scalar_adversarial_selection'] = {
        'status': 'current-timing-promotion-described-accurately-directed-selection-open',
        'review': prefix + 'scalar-selection-review-v1.json',
        'qualification': prefix + 'scalar-selection-wording-v1/result.json',
        'admission': prefix + 'scalar-selection-wording-admission-v1/admission.json',
        'source_mapping': prefix + 'scalar-selection-wording-admission-v1/source-hashes-after.json.gz',
        'description_correction_admitted': True, 'selection_algorithm_changed': False,
        'independent_representation_populations_implemented': False,
        'P38_complete': False, 'plan_complete': False,
    }
    dump(ledger_path, ledger)
    after = base | delta | {'docs/properties-status.json': sha(ledger_path)}
    assert len(after) == 10153 and all(sha(ROOT / name) == digest for name, digest in after.items())
    (DEST / 'source-hashes-after.json.gz').write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    dump(DEST / 'admission.json', {
        'complete': True, 'admitted': True, 'admitted_files_sha256': delta,
        'benchmark_code_changed_only_in_one_description_literal': True,
        'historical_reports_and_closed_numeric_evidence_unchanged': True,
        'qualification_result_sha256': sha(OUT / 'result.json'),
        'source_mapping_after_sha256': sha(DEST / 'source-hashes-after.json.gz'),
        'admission_script_sha256': sha(__file__), 'limits': scope['limits'],
        'P38_complete': False, 'plan_complete': False,
    })
    print(json.dumps({'admitted': True, 'changed_files': len(delta), 'admission_sha256': sha(DEST / 'admission.json')}))


if __name__ == '__main__':
    main()
