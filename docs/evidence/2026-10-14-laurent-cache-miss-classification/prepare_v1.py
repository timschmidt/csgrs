"""Freeze a classified borrowed-runtime diagnostic without changing prior candidates."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from miss_classification_common_v1 import E, ROOT, PARENT, COVERAGE, BORROWED, WORK, PREP, ENV, INPUTS, sha, dump, compressed, verify_prior, verify


def main():
    audit = verify_prior(full=True)
    parent_prep = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    variant = parent_prep['variants']['baseline']
    parent_source = Path(variant['source'])
    hashes_path = PARENT / 'preparation-v1/baseline-source-hashes.json.gz'
    assert sha(hashes_path) == audit['parent_source_hashes_sha256'] == variant['source_hashes_sha256']
    original = json.loads(gzip.decompress(hashes_path.read_bytes()))
    assert len(original) == 10135
    assert parent_prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert parent_prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    source = WORK / 'source'
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
    old_edits = json.loads((COVERAGE / 'instrumentation-edits-v1.json').read_text())
    edits = json.loads((E / 'instrumentation-edits-v1.json').read_text())
    for row in old_edits['edits'] + edits['edits']:
        path = source / row['path']
        text = path.read_text()
        assert text.count(row['before']) == 1, row['path']
        path.write_text(text.replace(row['before'], row['after']))
    added = old_edits['new_module']
    assert not (source / added['target']).exists()
    shutil.copyfile(E / 'cache_trace_v1.rs', source / added['target'])
    control = source / old_edits['appended_control']['target']
    control.write_text(control.read_text() + '\n' + (COVERAGE / 'cache_trace_control_v1.rs').read_text()
                       + '\n' + (E / 'classification_control_v1.rs').read_text())
    with (PREP / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
            env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    expected = {row['path'] for row in old_edits['edits'] + edits['edits']} | {added['target'], old_edits['appended_control']['target']}
    assert len(mapping) == 10136 and len(delta) == 5 and set(delta) == expected
    compressed(PREP / 'source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (parent_source / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (PREP / 'source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(PREP / 'source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    probes = {}
    for suite, row in audit['probe_review'].items():
        raw = (ROOT / row['path']).read_bytes()
        suffix = raw[raw.index(row['suffix_marker'].encode()):]
        if suite == 'rational':
            assert suffix.count(b".chain([b'\\n'])") == 1
            suffix = suffix.replace(b".chain([b'\\n'])", b'.chain(*b"\\n")')
        path = PREP / (suite + '_probe.rs')
        path.write_bytes((E / 'trace_header_v1.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix)
        probes[suite] = {'sha256': sha(path), 'original_path': row['path'], 'original_sha256': row['sha256'],
                         'marker': row['suffix_marker'], 'suffix_bytes': len(suffix)}
    libraries = json.loads((PARENT / 'scalar-costs-v1/libraries.json').read_text())
    reused = {group: {name: libraries[old][name] for name in ['hyperreal', 'serde_json']}
              for group, old in [('main', 'main'), ('borrowed', 'borrowed'), ('eligibility', 'candidate')]}
    paths = [PARENT / p for p in ['preparation-v1/protocol.json', 'preparation-v1/baseline-source-hashes.json.gz',
        'qualification-v1/candidate-scalar-all.stdout.gz', 'scalar-costs-v1/libraries.json',
        'scalar-costs-v1/result.json', 'scalar-cost-assessment-v1.json', 'scalar-cost-interpretation-v1.json',
        'cache-miss-classification-audit-v1.json', 'closure-v1.json']]
    paths += [COVERAGE / p for p in ['instrumentation-edits-v1.json', 'cache_trace_v1.rs', 'cache_trace_control_v1.rs',
        'diagnostic-v1/samples.json', 'diagnostic-v1/result.json']]
    paths += [BORROWED / p for p in ['rational-trace-v2/samples.json', 'rational-trace-failure-v2.json', 'rational-trace-assessment-v3.json']]
    prep = {'scope': __doc__, 'source': str(source), 'target': str(WORK / 'target'),
        'parent_source': str(parent_source), 'files': len(mapping), 'delta_sha256': delta,
        'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'), 'source_patch_sha256': sha(PREP / 'source.patch.gz'),
        'source_delta_archive_sha256': sha(PREP / 'source-delta.tar.gz'), 'probes': probes, 'reused_libraries': reused,
        'lookup_kind_names': audit['node_kinds'] + [audit['extra_kind']], 'lookup_outcome_names': audit['outcomes'],
        'eligible_kinds': audit['eligible_kinds'], 'inputs_sha256': {p: sha(E / p) for p in INPUTS},
        'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in paths},
        'main_source_sha256': parent_prep['main_source_sha256'], 'pending_native_sha256': parent_prep['pending_native_sha256'],
        'compiler': parent_prep['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'source_files': 10136, 'changed_or_added_paths': 5,
                      'kind_buckets': 61, 'outcomes': 3, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
