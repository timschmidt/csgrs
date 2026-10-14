"""Freeze one memo-count overlay on the already qualified classifier source."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
from leaf_memo_common_v2 import E, ROOT, PARENT, WORK, PREP, ENV, INPUTS, sha, dump, compressed, verify_parent, verify


def main():
    parent = verify_parent(full=True)
    original = json.loads(gzip.decompress((PARENT / 'preparation-v2/source-hashes.json.gz').read_bytes()))
    assert len(original) == 10136
    assert parent['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert parent['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    source = WORK / 'source'
    parent_source = Path(parent['source'])
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
    edits = json.loads((E / 'instrumentation-edits-v2.json').read_text())
    for row in edits['edits']:
        path = source / row['path']
        text = path.read_text()
        assert text.count(row['before']) == 1, row['path']
        path.write_text(text.replace(row['before'], row['after']))
    for target, input_name in edits['new_files'].items():
        assert not (source / target).exists()
        shutil.copyfile(E / input_name, source / target)
    control = source / edits['appended_control']['target']
    control.write_text(control.read_text() + '\n' + (E / edits['appended_control']['input']).read_text())
    with (PREP / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source, env=os.environ | ENV,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    expected = {r['path'] for r in edits['edits']} | set(edits['new_files']) | {edits['appended_control']['target']}
    assert len(mapping) == 10138 and len(delta) == 5 and set(delta) == expected
    compressed(PREP / 'source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (parent_source / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (PREP / 'source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(PREP / 'source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    probes = {}
    for suite, row in parent['probes'].items():
        parent_probe = PARENT / 'preparation-v2' / (suite + '_probe.rs')
        raw = parent_probe.read_bytes()
        suffix = raw[raw.index(row['marker'].encode()):]
        path = PREP / (suite + '_probe.rs')
        path.write_bytes((E / 'trace_header_v2.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix)
        probes[suite] = {'sha256': sha(path), 'parent_path': str(parent_probe.relative_to(ROOT)),
                         'parent_sha256': sha(parent_probe), 'marker': row['marker'], 'suffix_bytes': len(suffix)}
    libraries = json.loads((PARENT / 'diagnostic-v2/libraries.json').read_text())
    reused = {'borrowed': libraries['borrowed']}
    paths = [PARENT / p for p in ['preparation-v2/protocol.json', 'preparation-v2/source-hashes.json.gz',
        'source-review-v2.json', 'diagnostic-v2/scalar-library.stdout.gz', 'diagnostic-v2/result.json',
        'diagnostic-v2/libraries.json', 'diagnostic-v2/samples.json', 'assessment-v2.json', 'interpretation-v1.json',
        'closure-v1.json', 'normalizer-work-audit-v1.json', 'failure-v1.json']]
    paths.append(E / 'failure-v1.json')
    prep = {'scope': __doc__, 'source': str(source), 'target': str(WORK / 'target'),
        'parent_source': str(parent_source), 'files': len(mapping), 'delta_sha256': delta,
        'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'), 'source_patch_sha256': sha(PREP / 'source.patch.gz'),
        'source_delta_archive_sha256': sha(PREP / 'source-delta.tar.gz'), 'probes': probes,
        'reused_libraries': reused, 'inputs_sha256': {p: sha(E / p) for p in INPUTS},
        'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in paths},
        'main_source_sha256': parent['main_source_sha256'], 'pending_native_sha256': parent['pending_native_sha256'],
        'compiler': parent['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'source_files': 10138, 'changed_or_added_paths': 5,
                      'expected_tests': 884, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
