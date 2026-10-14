"""Prepare a separate full-cache candidate with one narrow integer-leaf repair."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from common_v1 import E, ROOT, OLD, PARENT, WORK, PREP, ENV, sha, dump, compressed, verify_prior, verify


def main():
    verify_prior(full=True)
    parent_prep = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    parent = parent_prep['variants']['candidate']
    parent_source = Path(parent['source'])
    hashes_path = PARENT / 'preparation-v1/candidate-source-hashes.json.gz'
    assert sha(hashes_path) == parent['source_hashes_sha256']
    original = json.loads(gzip.decompress(hashes_path.read_bytes()))
    assert len(original) == 10133
    assert all(sha(parent_source / p) == h for p, h in original.items())
    old_protocol = json.loads((OLD / 'run-v1/protocol.json').read_text())
    assert old_protocol['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert old_protocol['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    source = WORK / 'source'
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    for row in edits['edits']:
        path = source / row['path']
        before = path.read_text()
        assert before.count(row['before']) == 1, row['path']
        path.write_text(before.replace(row['before'], row['after']))
    added = edits['new_test_module']
    assert not (source / added['target']).exists()
    shutil.copyfile(E / added['source'], source / added['target'])
    with (PREP / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                       env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    assert set(delta) == {r['path'] for r in edits['edits']} | {added['target']}
    assert len(delta) == 4 and len(mapping) == 10134
    compressed(PREP / 'source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (parent_source / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (PREP / 'source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(PREP / 'source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    inputs = ['common_v1.py', 'prepare_v1.py', 'qualify_v1.py', 'scope-v1.json',
              'runtime-edits-v1.json', 'borrowed_integer_tests_v1.rs']
    prior_inputs = [PARENT / 'preparation-v1/protocol.json', hashes_path,
                    PARENT / 'library-costs-v1/protocol.json', PARENT / 'library-costs-v1/libraries.json',
                    PARENT / 'library-costs-v1/binaries.json', PARENT / 'library-costs-v1/result.json',
                    PARENT / 'probes-v1/integer_probe.rs',
                    ROOT / 'docs/evidence/2026-10-13-construction-cache-coverage/diagnostic-v1/library-tests.stdout.gz',
                    OLD / 'assessment-v1.json', OLD / 'run-v1/protocol.json', OLD / 'run-v1/result.json']
    prep = {'scope': __doc__, 'source': str(source), 'target': str(WORK / 'target'),
            'parent_source': str(parent_source), 'files': len(mapping), 'delta_sha256': delta,
            'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'),
            'source_patch_sha256': sha(PREP / 'source.patch.gz'),
            'source_delta_archive_sha256': sha(PREP / 'source-delta.tar.gz'),
            'inputs_sha256': {p: sha(E / p) for p in inputs},
            'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in prior_inputs},
            'main_source_sha256': parent_prep['main_source_sha256'],
            'pending_native_sha256': parent_prep['pending_native_sha256'],
            'compiler': old_protocol['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
            'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'files': len(mapping), 'changed_or_added_paths': 4,
                      'original_and_main_sources_unchanged': True, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
