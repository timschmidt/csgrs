"""Freeze current main and the reviewed candidate without changing old trees."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from common_v1 import E, ROOT, WORK, PREP, ENV, sha, dump, compressed, sources, verify


def main():
    old = ROOT / 'docs/evidence/2026-10-13-drc-process-allocator/preparation-v2'
    parent = json.loads((old / 'protocol.json').read_text())
    original = Path(parent['variants']['candidate']['source'])
    expected = json.loads(gzip.decompress((old / 'candidate-source-hashes.json.gz').read_bytes()))
    assert len(expected) == 10133
    assert all(sha(original / p) == h for p, h in expected.items())
    main_sources = sources()
    assert main_sources == parent['main_source_sha256'] and len(main_sources) == 1134
    assert len(parent['pending_native_sha256']) == 35
    reviewed = json.loads((E / 'reviewed-source-inputs-v1.json').read_text())['changes']
    assert len(reviewed) == 17
    for row in reviewed:
        assert sha(original / row['path']) == row['candidate_sha256']
        assert (sha(ROOT / row['path']) if (ROOT / row['path']).exists() else None) == row['main_sha256']
    paths = set(filter(None, subprocess.check_output(
        ['git', 'ls-files', '-c', '-o', '--exclude-standard', '-z'], cwd=ROOT).decode().split('\0')))
    paths = {p for p in paths if not p.startswith(('docs/evidence/', 'crates/comparisons/', 'target/'))}
    assert paths == set(expected) - {r['path'] for r in reviewed if r['main_sha256'] is None}
    current = {p: sha(ROOT / p) for p in sorted(paths)}
    assert {p for p in paths if current[p] != expected[p]} == {
        r['path'] for r in reviewed if r['main_sha256'] is not None} | {'docs/properties-status.json'}
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    probes = E / 'probes-v1'
    probes.mkdir(exist_ok=False)
    history = {}
    scalar = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates/cost-v1'
    arc = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse/controls-v1/probes'
    inputs = {p.name: p for p in arc.glob('*.rs')}
    inputs |= {name + '_probe.rs': scalar / (name + '_probe.rs') for name in ['rational', 'quadratic', 'integer']}
    inputs |= {name: ROOT / 'docs/evidence/2026-10-12-region-metadata-first' / name
               for name in ['metadata_order_tests.rs', 'cost_probe.rs']}
    for name, path in sorted(inputs.items()):
        shutil.copyfile(path, probes / name)
        history[name] = {'path': str(path.relative_to(ROOT)), 'sha256': sha(path)}
    variants = {}
    for label in ['baseline', 'candidate', 'baseline-metadata', 'candidate-metadata']:
        source = WORK / label / 'source'
        source.parent.mkdir()
        subprocess.run(['cp', '-a', '--reflink=auto', str(original), str(source)], check=True)
        baseline = label.startswith('baseline')
        if baseline:
            for row in reviewed:
                path = source / row['path']
                if row['main_sha256'] is None:
                    path.unlink()
                else:
                    shutil.copyfile(ROOT / row['path'], path)
        shutil.copyfile(ROOT / 'docs/properties-status.json', source / 'docs/properties-status.json')
        original_map = current if baseline else expected | {'docs/properties-status.json': sha(ROOT / 'docs/properties-status.json')}
        if label.endswith('-metadata'):
            name = 'crates/hypercurve/src/bezier_region/combined_admission_metadata.rs'
            region = source / 'crates/hypercurve/src/bezier_region.rs'
            region.write_text(region.read_text() + '\n#[cfg(test)]\nmod combined_admission_metadata;\n')
            (source / name).write_text((probes / 'metadata_order_tests.rs').read_text() + '\n' + (probes / 'cost_probe.rs').read_text())
            subprocess.run(['rustfmt', '--edition', '2024', str(region), str(source / name)], check=True)
        mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
        changes = {p: h for p, h in mapping.items() if original_map.get(p) != h}
        assert set(changes) == ({'crates/hypercurve/src/bezier_region.rs', name} if label.endswith('-metadata') else set())
        assert set(mapping) == set(original_map) | (set(changes) if label.endswith('-metadata') else set())
        assert len(mapping) == (10128 if baseline else 10133) + int(label.endswith('-metadata'))
        with (PREP / (label + '-format.log')).open('w') as log:
            subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                           env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
        compressed(PREP / (label + '-source-hashes.json.gz'), mapping)
        delta = {p: h for p, h in mapping.items() if current.get(p) != h}
        with tarfile.open(PREP / (label + '-source-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(delta):
                archive.add(source / p, arcname=p)
        variants[label] = {'source': str(source), 'target': str(WORK / label / 'target'),
                           'files': len(mapping), 'source_hashes_sha256': sha(PREP / (label + '-source-hashes.json.gz')),
                           'delta_sha256': delta}
    compressed(PREP / 'main-workspace-hashes.json.gz', current)
    names = ['common_v1.py', 'measure_libraries_v1.py', 'assess_libraries_v1.py', 'scope-v1.json',
             'review-v1.json', 'review-correction-v1.json', 'reviewed-source-inputs-v1.json']
    names += ['probes-v1/' + name for name in history]
    result = {'scope': __doc__, 'runner_sha256': sha(__file__),
              'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'inputs_sha256': {name: sha(E / name) for name in names},
              'main_source_sha256': main_sources, 'pending_native_sha256': parent['pending_native_sha256'],
              'main_workspace_hashes_sha256': sha(PREP / 'main-workspace-hashes.json.gz'),
              'historical_probes': history, 'variants': variants,
              'parent': {'source': str(original), 'preparation': str((old / 'protocol.json').relative_to(ROOT)),
                         'preparation_sha256': sha(old / 'protocol.json'),
                         'hashes': str((old / 'candidate-source-hashes.json.gz').relative_to(ROOT)),
                         'hashes_sha256': sha(old / 'candidate-source-hashes.json.gz'),
                         'dependency_hashes': str((old / 'dependency-source-hashes.json.gz').relative_to(ROOT)),
                         'dependency_hashes_sha256': sha(old / 'dependency-source-hashes.json.gz')},
              'runtime_admitted': False, 'plan_complete': False}
    verify(result, full=True)
    dump(PREP / 'protocol.json', result)
    print(json.dumps({'prepared': True, 'variants': {k: v['files'] for k, v in variants.items()},
                      'historical_probes': len(history), 'main_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
