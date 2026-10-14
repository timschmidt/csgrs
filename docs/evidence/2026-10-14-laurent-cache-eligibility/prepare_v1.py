"""Freeze a control and candidate with identical new cache-contract tests."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from eligibility_common_v1 import E, ROOT, PARENT, WORK, PREP, ENV, INPUTS, sha, dump, compressed, verify_prior, verify


def main():
    base = verify_prior(full=True)
    parent_source = Path(base['source'])
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    assert len(original) == 10134
    assert base['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert base['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    variants = {}
    for group in ['baseline', 'candidate']:
        source = WORK / group / 'source'
        source.parent.mkdir(exist_ok=False)
        subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
        changes = edits['common_edits'] + (edits['candidate_edits'] if group == 'candidate' else [])
        for row in changes:
            path = source / row['path']
            before = path.read_text()
            assert before.count(row['before']) == 1, row['path']
            path.write_text(before.replace(row['before'], row['after']))
        added = edits['new_test_module']
        assert not (source / added['target']).exists()
        shutil.copyfile(E / added['source'], source / added['target'])
        with (PREP / (group + '-format.log')).open('w') as log:
            subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                           env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
        mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
        delta = {p: h for p, h in mapping.items() if original.get(p) != h}
        assert len(mapping) == 10135 and set(delta) == {r['path'] for r in changes} | {added['target']}
        assert len(delta) == 2
        compressed(PREP / (group + '-source-hashes.json.gz'), mapping)
        patch = ''.join(''.join(difflib.unified_diff(
            (parent_source / p).read_text().splitlines(True) if p in original else [],
            (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
        (PREP / (group + '-source.patch.gz')).write_bytes(gzip.compress(patch.encode(), mtime=0))
        with tarfile.open(PREP / (group + '-source-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(delta):
                archive.add(source / p, arcname=p)
        variants[group] = {'source': str(source), 'files': len(mapping), 'delta_sha256': delta,
            'target_all': str(WORK / group / 'target-all'),
            'target_default': str(WORK / group / 'target-default'),
            'source_hashes_sha256': sha(PREP / (group + '-source-hashes.json.gz')),
            'source_patch_sha256': sha(PREP / (group + '-source.patch.gz')),
            'source_delta_archive_sha256': sha(PREP / (group + '-source-delta.tar.gz'))}
    prior_inputs = [PARENT / p for p in [
        'preparation-v1/protocol.json', 'preparation-v1/source-hashes.json.gz',
        'qualification-v1/result.json', 'qualification-v1/library.json', 'qualification-v1/scalar-library.stdout.gz',
        'library-costs-v1/result.json', 'library-interpretation-v1.json',
        'rational-trace-assessment-v3.json', 'rational-trace-closure-v3.json',
        'rational-cache-eligibility-next-v1.json']]
    prep = {'scope': __doc__, 'parent_source': str(parent_source), 'variants': variants,
        'inputs_sha256': {p: sha(E / p) for p in INPUTS},
        'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in prior_inputs},
        'main_source_sha256': base['main_source_sha256'], 'pending_native_sha256': base['pending_native_sha256'],
        'compiler': base['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'variants': 2, 'files_each': 10135,
                      'changed_or_added_paths_each': 2, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
