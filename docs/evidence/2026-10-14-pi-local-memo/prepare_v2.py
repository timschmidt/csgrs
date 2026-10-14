"""Freeze the reviewed Pi memo change on the unchanged borrowed runtime parent."""
import difflib
import gzip
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

from pi_memo_common_v2 import E, ROOT, BASE, DIAGNOSTIC, WORK, PREP, ENV, INPUTS, NEW_TESTS, sha, dump, compressed, verify_parent, verify


def main():
    parent = verify_parent(full=True)
    baseline = parent['variants']['baseline']
    parent_source = Path(baseline['source'])
    original = json.loads(gzip.decompress((BASE / 'preparation-v1/baseline-source-hashes.json.gz').read_bytes()))
    assert len(original) == 10135
    assert parent['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert parent['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    edits = json.loads((E / 'runtime-edits-v2.json').read_text())
    assert edits['parent_algebra_sha256'] == sha(parent_source / 'crates/hyperreal/src/computable/node/algebra.rs')
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    source = WORK / 'source'
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
    for row in edits['edits']:
        path = source / row['path']
        old = path.read_text()
        assert old.count(row['before']) == 1
        path.write_text(old.replace(row['before'], row['after']))
    added = edits['new_test_module']
    assert not (source / added['target']).exists()
    shutil.copyfile(E / added['source'], source / added['target'])
    with (PREP / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                       env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    assert len(mapping) == 10136 and set(delta) == {r['path'] for r in edits['edits']} | {added['target']}
    assert len(delta) == 2
    compressed(PREP / 'source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (parent_source / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (PREP / 'source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(PREP / 'source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    # This is test enumeration only, from the preserved prior default binary.
    # Its feature selection and test source are identical to the selected parent.
    binaries = json.loads((BASE / 'qualification-v1/binaries.json').read_text())
    binary = binaries['candidate-default']
    assert sha(binary['path']) == binary['sha256'] and binary['features'] == ['default']
    command = [binary['path'], '--list']
    listed = subprocess.run(command, cwd=ROOT, env=os.environ | ENV, capture_output=True, check=True)
    inventory = {'binary': binary, 'command': command, 'exit_code': listed.returncode}
    for stream in ['stdout', 'stderr']:
        path = PREP / ('parent-default-list.' + stream + '.gz')
        path.write_bytes(gzip.compress(getattr(listed, stream), mtime=0))
        inventory[stream + '_sha256'] = sha(path)
    default_names = {n.decode() for n in re.findall(rb'^(.+): test$', listed.stdout, re.MULTILINE)}
    raw = gzip.decompress((BASE / 'qualification-v1/candidate-scalar-all.stdout.gz').read_bytes())
    all_names = {n.decode() for n in re.findall(rb'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE)}
    assert len(all_names) == 879 and default_names < all_names
    assert not (all_names & NEW_TESTS)
    inventory['test_names'] = sorted(default_names)
    parent_lib = json.loads((BASE / 'qualification-v1/library.json').read_text())
    variant = {'source': str(source), 'files': len(mapping), 'delta_sha256': delta,
        'target_all': str(WORK / 'target-all'), 'target_default': str(WORK / 'target-default'),
        'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'),
        'source_patch_sha256': sha(PREP / 'source.patch.gz'),
        'source_delta_archive_sha256': sha(PREP / 'source-delta.tar.gz')}
    prior_paths = [DIAGNOSTIC / p for p in ['pi-memo-source-audit-v1.json', 'closure-v1.json',
        'interpretation-v1.json', 'assessment-v2.json', 'failure-v1.json']]
    prior_paths += [BASE / p for p in ['preparation-v1/protocol.json',
        'preparation-v1/baseline-source-hashes.json.gz', 'qualification-v1/candidate-scalar-all.stdout.gz',
        'qualification-v1/binaries.json', 'qualification-v1/library.json', 'qualification-assessment-v1.json']]
    prior_paths.append(E / 'preparation-failure-v1.json')
    prep = {'scope': __doc__, 'parent_source': str(parent_source), 'variants': {'candidate': variant},
        'inputs_sha256': {p: sha(E / p) for p in INPUTS},
        'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in prior_paths},
        'parent_default_inventory': inventory, 'parent_all_test_names': sorted(all_names),
        'expected_all_tests': 882, 'expected_default_tests': len(default_names) + 3,
        'expected_all_features': parent_lib['features'],
        'main_source_sha256': parent['main_source_sha256'], 'pending_native_sha256': parent['pending_native_sha256'],
        'compiler': parent['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'source_files': 10136, 'changed_or_added_paths': 2,
        'expected_all_tests': 882, 'expected_default_tests': prep['expected_default_tests'],
        'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
