"""Copy the full-cache parent and freeze diagnostic-only counter insertions."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from common_v1 import E, ROOT, OLD, PARENT, WORK, PREP, ENV, sha, dump, compressed, prior, verify


def main():
    old_prep = json.loads((OLD / 'preparation-v1/protocol.json').read_text())
    prior.verify(old_prep, full=True)
    parent_prep = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    parent = parent_prep['variants']['candidate']
    parent_source = Path(parent['source'])
    mapping_path = PARENT / 'preparation-v1/candidate-source-hashes.json.gz'
    assert sha(mapping_path) == parent['source_hashes_sha256']
    original = json.loads(gzip.decompress(mapping_path.read_bytes()))
    assert len(original) == 10133
    assert all(sha(parent_source / p) == h for p, h in original.items())
    trace = OLD / 'construction-trace-v1'
    trace_result = json.loads((trace / 'result.json').read_text())
    assert trace_result['complete'] and trace_result['counts_repeat_exactly_within_each_variant']
    assert all(sha(trace / p) == h for p, h in trace_result['artifact_sha256'].items())
    trace_protocol = json.loads((trace / 'protocol.json').read_text())
    assert trace_protocol['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert trace_protocol['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    source = WORK / 'source'
    subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
    edits = json.loads((E / 'instrumentation-edits-v1.json').read_text())
    for row in edits['edits']:
        path = source / row['path']
        text = path.read_text()
        assert text.count(row['before']) == 1, row['path']
        path.write_text(text.replace(row['before'], row['after']))
    added = edits['new_module']
    assert not (source / added['target']).exists()
    shutil.copyfile(E / added['source'], source / added['target'])
    control = edits['appended_control']
    path = source / control['target']
    path.write_text(path.read_text() + '\n' + (E / control['source']).read_text())
    with (PREP / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                       env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    expected = {r['path'] for r in edits['edits']} | {added['target'], control['target']}
    assert len(mapping) == 10134 and set(delta) == expected and len(delta) == 5
    compressed(PREP / 'source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (parent_source / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (PREP / 'source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(PREP / 'source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    old_probe = (trace / 'probe.rs').read_text()
    needle = '    let result = dispatch_trace::with_recording(body);'
    assert old_probe.count(needle) == 1
    probe = old_probe.replace(needle, '''    let (result, coverage) = hyperreal::construction_cache_trace::with_recording(|| {
        dispatch_trace::with_recording(body)
    });''')
    needle = '        "trace_phase": name, "iterations": count, "dispatch": dispatch,'
    assert probe.count(needle) == 1
    probe = probe.replace(needle, needle + '''
        "cache_events": hyperreal::construction_cache_trace::EVENT_NAMES.into_iter()
            .zip(coverage).map(|(name, count)| (name.to_owned(), serde_json::json!(count)))
            .collect::<serde_json::Map<String, serde_json::Value>>(),''')
    marker = 'fn inputs(index: usize)'
    assert probe[probe.index(marker):] == old_probe[old_probe.index(marker):]
    (PREP / 'probe.rs').write_text(probe)
    prior_libraries = trace_protocol['libraries']['full-cache']
    prior_binaries = json.loads((trace / 'binaries.json').read_text())
    cost_bins = json.loads((PARENT / 'library-costs-v1/binaries.json').read_text())
    prior_binaries |= {'cost-' + k: v for k, v in cost_bins.items()}
    inputs = ['common_v1.py', 'prepare_v1.py', 'measure_v1.py', 'assess_v1.py',
              'scope-v1.json', 'instrumentation-edits-v1.json', 'cache_trace_v1.rs', 'cache_trace_control_v1.rs']
    old_paths = [PARENT / 'preparation-v1/protocol.json', mapping_path,
                 OLD / 'next-cache-coverage-v1.json', OLD / 'construction-trace-assessment-v1.json',
                 OLD / 'construction-trace-interpretation-v1.json', OLD / 'preparation-v1/protocol.json',
                 OLD / 'library-costs-v1/candidate-scalar-library.stdout.gz',
                 OLD / 'library-costs-v1/commands.json', OLD / 'library-costs-v1/result.json',
                 PARENT / 'library-costs-v1/binaries.json', trace / 'result.json',
                 *[trace / p for p in trace_result['artifact_sha256']]]
    prep = {'scope': __doc__, 'source': str(source), 'target': str(WORK / 'target'),
            'parent_source': str(parent_source), 'files': len(mapping), 'delta_sha256': delta,
            'source_hashes_sha256': sha(PREP / 'source-hashes.json.gz'),
            'source_patch_sha256': sha(PREP / 'source.patch.gz'),
            'source_delta_archive_sha256': sha(PREP / 'source-delta.tar.gz'),
            'probe_sha256': sha(PREP / 'probe.rs'),
            'main_source_sha256': old_prep['main_source_sha256'],
            'pending_native_sha256': old_prep['pending_native_sha256'],
            'inputs_sha256': {p: sha(E / p) for p in inputs},
            'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in old_paths},
            'prior_libraries': prior_libraries, 'prior_binaries': prior_binaries,
            'compiler': trace_protocol['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
            'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'files': len(mapping), 'changed_or_added_paths': len(delta),
                      'parent_sources_and_binaries_unchanged': True, 'runtime_candidate': False}), flush=True)


if __name__ == '__main__':
    main()
