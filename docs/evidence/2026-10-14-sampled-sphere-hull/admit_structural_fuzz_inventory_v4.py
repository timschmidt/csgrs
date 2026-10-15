"""Independently check the closed fuzz qualification and admit its test-only delta."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'structural-fuzz-inventory-v4'
DEST = E / 'structural-fuzz-inventory-admission-v4'


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
    assert not DEST.exists() and not (OUT / 'failure.json').exists()
    result = read(OUT / 'result.json')
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'structural-fuzz-inventory-scope-v4.json')
    assert result['complete'] and result['commands_all_zero'] == 10
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    commands = read(OUT / 'commands.json')
    expected_names = ['format', 'fuzz-format', 'default-clippy', 'all-features-clippy',
                      'fuzz-clippy', 'default-tests', 'all-features-tests', 'fuzz-build',
                      'seed-replay', 'bounded-mutations']
    assert [row['name'] for row in commands] == expected_names
    assert [row['exit_code'] for row in commands] == [0] * 10
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    driver = [json.loads(line) for line in (E / 'structural-fuzz-inventory-driver-v4.stdout').read_text().splitlines()]
    assert [row for row in driver if 'exit_code' in row] == commands
    assert driver[-1] == {'complete': True, 'test_counts': result['test_counts'], 'result_sha256': sha(OUT / 'result.json')}
    assert (E / 'structural-fuzz-inventory-driver-v4.stderr').read_bytes() == b''
    sources = mapping(OUT / 'source-hashes.json.gz')
    roots = mapping(OUT / 'root-source-hashes.json.gz')
    source = Path(protocol['source'])
    parent = ROOT / scope['parent_source']
    parent_table = mapping(E / 'mpfr-test-cleanup-main-v1/source-hashes.json.gz')
    delta = scope['expected_delta_sha256']
    assert len(delta) == 27 and len(sources) == 10153 and len(roots) == 10130
    assert sources == parent_table | delta
    assert inventory(source) == sources
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    assert all(sha(parent / name) == digest for name, digest in parent_table.items())
    assert all(roots[name] == digest for name, digest in parent_table.items()
               if name != 'docs/properties-status.json')
    binaries = read(OUT / 'binaries.json')
    for row in [*protocol['preserved_binaries'], *binaries.values(), protocol['cargo_fuzz']]:
        assert sha(row['path']) == row['sha256']

    seed_dir = Path('crates/hyperreal/fuzz/corpus/structural_representations')
    seeds = inventory(source / seed_dir)
    assert len(seeds) == 43 and seeds == protocol['seed_sha256']
    assert set(seeds) == set(scope['original_seed_sha256']) | {f'stride-{i:02}' for i in range(22)}
    assert all(seeds[name] == digest for name, digest in scope['original_seed_sha256'].items())
    assert all((source / name).read_bytes() == bytes.fromhex(value)
               for name, value in scope['new_seed_hex'].items())
    pairs = {(left, (left + (source / seed_dir / f'stride-{stride:02}').read_bytes()[0]) % 22)
             for left in range(22) for stride in range(22)}
    assert pairs == {(left, right) for left in range(22) for right in range(22)}
    test = 'crates/hyperreal/tests/real_representations.rs'
    helper = 'crates/hyperreal/tests/support/representations.rs'
    original = (parent / test).read_text()
    start, end = original.index('struct RepresentationCase {'), original.index('fn structural_kind_index(')
    recipe = (source / helper).read_text().split('pub(crate) struct RepresentationCase {', 1)[1]
    recipe = 'struct RepresentationCase {' + recipe.replace('pub(crate) ', '')
    assert recipe.rstrip() == original[start:end].rstrip()
    imports = ('#[path = "support/representations.rs"]\nmod representations;\n'
               'use representations::{fraction, optimized_certificate_representatives};\n\n')
    assert (source / test).read_text() == original[:start] + imports + original[end:]
    assert set(delta) == {test, helper, 'crates/hyperreal/fuzz/README.md',
                          'crates/hyperreal/fuzz/fuzz_targets/structural_representations.rs',
                          'crates/hyperreal/fuzz/fuzz_targets/real_elementary.rs'} | set(scope['new_seed_hex'])
    formatted = 'crates/hyperreal/fuzz/fuzz_targets/real_elementary.rs'
    assert ''.join((source / formatted).read_text().split()) == ''.join((parent / formatted).read_text().split())
    for name, count in [('default-tests', 4), ('all-features-tests', 6)]:
        rows = re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;',
                          (OUT / (name + '.stdout')).read_text(), re.MULTILINE)
        assert rows == [(str(count), '0', '0', '0', '0')]
        assert result['test_counts'][name] == count
    build_log = (OUT / 'fuzz-build.stderr').read_text()
    for crate in ['std', 'hyperreal', 'structural_representations']:
        assert any('--crate-name ' + crate + ' ' in line and 'sanitizer=address' in line
                   and 'sanitizer-coverage-inline-8bit-counters' in line for line in build_log.splitlines())
    replay = (OUT / 'seed-replay.stderr').read_text()
    assert 'Running 43 inputs 1 time(s) each.' in replay
    assert re.findall(r'^Executed (.+) in \d+ ms$', replay, re.MULTILINE) == [str(source / seed_dir / name) for name in sorted(seeds)]
    mutation = (OUT / 'bounded-mutations.stderr').read_text()
    assert re.search(r'^Done 256 runs in \d+ second\(s\)$', mutation, re.MULTILINE)
    assert 'inline 8-bit counters' in mutation and 'cov:' in mutation
    for log in [replay, mutation]:
        assert not any(marker in log for marker in ['ERROR: AddressSanitizer:', 'ERROR: LeakSanitizer:', 'ERROR: libFuzzer:'])
    corpus = read(OUT / 'mutation-corpus.json')['files_sha256']
    assert inventory(OUT / 'retained-mutation-corpus') == corpus
    assert all(corpus[name] == digest for name, digest in seeds.items())
    pending = read(ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json')['pending_native_sha256']
    assert len(pending) == 35 and not set(pending).intersection(delta)
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {
        k: v for k, v in base.items() if k != 'current_work'}
    assert 'structural_fuzz_inventory' not in ledger['current_work']
    for name in delta:
        if name in roots:
            assert sha(ROOT / name) == parent_table[name]
        else:
            assert not (ROOT / name).exists()
    DEST.mkdir()
    (DEST / 'source-hashes-before.json.gz').write_bytes((OUT / 'root-source-hashes.json.gz').read_bytes())
    for name in delta:
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((source / name).read_bytes())
    prefix = str(E.relative_to(ROOT)) + '/'
    ledger['current_work']['structural_fuzz_inventory'] = {
        'status': 'shared-22-form-inventory-and-monotone-cache-checks-admitted',
        'scope': prefix + 'structural-fuzz-inventory-scope-v4.json',
        'result': prefix + 'structural-fuzz-inventory-v4/result.json',
        'admission': prefix + 'structural-fuzz-inventory-admission-v4/admission.json',
        'source_mapping': prefix + 'structural-fuzz-inventory-admission-v4/source-hashes-after.json.gz',
        'test_only_change': True, 'admitted': True, 'representatives': 22,
        'fixed_replayed_seeds': 43, 'retained_stride_schedule_ordered_pairs': 484,
        'bounded_mutation_runs': 256, 'test_counts': result['test_counts'],
        'limits': scope['limits'], 'directed_lexicase_campaign_complete': False, 'plan_complete': False,
    }
    dump(ledger_path, ledger)
    after = roots | delta | {'docs/properties-status.json': sha(ledger_path)}
    assert len(after) == 10153 and all(sha(ROOT / name) == digest for name, digest in after.items())
    (DEST / 'source-hashes-after.json.gz').write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    dump(DEST / 'admission.json', {
        'complete': True, 'qualification_trial_closed': True, 'admitted': True,
        'commands_all_zero': 10, 'admitted_files_sha256': delta,
        'production_arithmetic_geometry_and_public_api_changed': False,
        'original_representation_recipes_and_original_seed_bytes_unchanged': True,
        'pending_native_files_preserved': 35, 'test_counts': result['test_counts'],
        'fixed_replayed_seeds': 43, 'retained_stride_schedule_ordered_pairs': 484,
        'bounded_mutation_runs': 256, 'retained_corpus_files': len(corpus),
        'qualification_result_sha256': sha(OUT / 'result.json'),
        'source_mapping_before_sha256': sha(DEST / 'source-hashes-before.json.gz'),
        'source_mapping_after_sha256': sha(DEST / 'source-hashes-after.json.gz'),
        'admission_script_sha256': sha(__file__),
        'driver_stdout_sha256': sha(E / 'structural-fuzz-inventory-driver-v4.stdout'),
        'driver_stderr_sha256': sha(E / 'structural-fuzz-inventory-driver-v4.stderr'),
        'limits': scope['limits'], 'directed_lexicase_campaign_complete': False, 'plan_complete': False,
    })
    print(json.dumps({'admitted': True, 'admitted_files': len(delta), 'retained_corpus_files': len(corpus),
                      'admission_sha256': sha(DEST / 'admission.json')}))


if __name__ == '__main__':
    main()
