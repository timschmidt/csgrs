"""Freeze unchanged production inputs and an identical private metadata harness."""
import difflib
import gzip
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

from cost_common_v1 import E, ROOT, PARENT, PREP, ENV, COST_PREP, COST_WORK, sha, dump, compressed, verify_qualification, verify_costs


def main():
    base = verify_qualification(full=True)
    old = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    old_costs = PARENT / 'library-costs-v1'
    old_result = json.loads((old_costs / 'result.json').read_text())
    assert old_result['complete']
    assert all(sha(old_costs / p) == h for p, h in old_result['artifact_sha256'].items())
    old_libraries = json.loads((old_costs / 'libraries.json').read_text())
    old_binaries = json.loads((old_costs / 'binaries.json').read_text())
    assert base['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert base['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    COST_WORK.mkdir(exist_ok=False)
    COST_PREP.mkdir(exist_ok=False)
    variants, libraries, binaries = {}, {}, {}
    for label, old_label in [('main', 'baseline'), ('full-cache', 'candidate')]:
        libraries[label] = old_libraries[old_label]
        own_bins = {label + k.removeprefix(old_label): v for k, v in old_binaries.items() if k.startswith(old_label + '-')}
        assert len(own_bins) == 11 and len(libraries[label]) == 5
        binaries |= own_bins
        for suffix in ['', '-metadata']:
            name = label + suffix
            variant = old['variants'][old_label + suffix]
            shutil.copyfile(PARENT / 'preparation-v1' / (old_label + suffix + '-source-hashes.json.gz'),
                            COST_PREP / (name + '-source-hashes.json.gz'))
            variants[name] = variant | {'source_and_target_borrowed_unchanged': True}
    shutil.copyfile(PREP / 'source-hashes.json.gz', COST_PREP / 'candidate-source-hashes.json.gz')
    variants['candidate'] = {key: base[key] for key in ['source', 'files', 'source_hashes_sha256', 'delta_sha256']}
    variants['candidate'] |= {'target': str(COST_WORK / 'candidate-target'),
                              'source_borrowed_unchanged_from_qualification': True,
                              'target_is_new': True, 'delta_parent': 'full-cache'}
    source = COST_WORK / 'candidate-metadata' / 'source'
    source.parent.mkdir()
    subprocess.run(['cp', '-a', '--reflink=auto', base['source'], str(source)], check=True)
    region_path = 'crates/hypercurve/src/bezier_region.rs'
    module_path = 'crates/hypercurve/src/bezier_region/combined_admission_metadata.rs'
    region = source / region_path
    assert 'mod combined_admission_metadata;' not in region.read_text()
    region.write_text(region.read_text() + '\n#[cfg(test)]\nmod combined_admission_metadata;\n')
    module = Path(variants['full-cache-metadata']['source']) / module_path
    assert sha(module) == sha(Path(variants['main-metadata']['source']) / module_path)
    assert not (source / module_path).exists()
    shutil.copyfile(module, source / module_path)
    with (COST_PREP / 'metadata-format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                       env=os.environ | ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
    original = json.loads(gzip.decompress((PREP / 'source-hashes.json.gz').read_bytes()))
    mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    delta = {p: h for p, h in mapping.items() if original.get(p) != h}
    assert set(delta) == {region_path, module_path} and len(mapping) == 10135
    compressed(COST_PREP / 'candidate-metadata-source-hashes.json.gz', mapping)
    patch = ''.join(''.join(difflib.unified_diff(
        (Path(base['source']) / p).read_text().splitlines(True) if p in original else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
    (COST_PREP / 'metadata-source.patch.gz').write_bytes(gzip.compress(patch.encode(), mtime=0))
    with tarfile.open(COST_PREP / 'metadata-source-delta.tar.gz', 'w:gz') as archive:
        for p in sorted(delta):
            archive.add(source / p, arcname=p)
    variants['candidate-metadata'] = {'source': str(source), 'target': str(source.parent / 'target'),
        'files': len(mapping), 'source_hashes_sha256': sha(COST_PREP / 'candidate-metadata-source-hashes.json.gz'),
        'delta_sha256': base['delta_sha256'] | delta, 'metadata_only_delta_sha256': delta,
        'delta_parent': 'full-cache', 'metadata_delta_parent': 'candidate', 'target_is_new': True}
    probes = {p.name: {'path': str(p.relative_to(ROOT)), 'sha256': sha(p)}
              for p in sorted((PARENT / 'probes-v1').glob('*.rs'))}
    assert len(probes) == 18
    inputs = ['cost_common_v1.py', 'prepare_costs_v1.py', 'measure_costs_v1.py', 'assess_costs_v1.py',
              'cost_rows_v1.py', 'cost-scope-v1.json', 'next-cost-plan-v1.json', 'source-review-v1.json',
              'qualification-assessment-v1.json', 'preparation-v1/protocol.json', 'qualification-v1/result.json']
    prior_inputs = [PARENT / 'preparation-v1/protocol.json', PARENT / 'library-assessment-v1.json',
                    *[old_costs / name for name in ['protocol.json', 'libraries.json', 'binaries.json', 'result.json']],
                    ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json', module]
    prep = {'scope': __doc__, 'variants': variants, 'reused_libraries': libraries, 'reused_binaries': binaries,
            'probes': probes, 'compiler': base['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
            'inputs_sha256': {p: sha(E / p) for p in inputs},
            'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in prior_inputs},
            'metadata_patch_sha256': sha(COST_PREP / 'metadata-source.patch.gz'),
            'metadata_archive_sha256': sha(COST_PREP / 'metadata-source-delta.tar.gz'),
            'all_qualified_production_sources_and_artifacts_preserved': True,
            'runtime_admitted': False, 'plan_complete': False}
    verify_costs(prep, full=True)
    dump(COST_PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'production_files': 10134, 'metadata_files': 10135,
                      'reused_binaries': len(binaries), 'reused_library_groups': len(libraries),
                      'prior_sources_and_artifacts_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
