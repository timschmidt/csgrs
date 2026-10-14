"""Freeze a distinct combination; reuse immutable baseline artifacts only."""
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
    previous = ROOT / 'docs/evidence/2026-10-13-combined-native-admission'
    prior = json.loads((previous / 'preparation-v1/protocol.json').read_text())
    baseline = Path(prior['variants']['baseline']['source'])
    full_candidate = Path(prior['variants']['candidate']['source'])
    original = json.loads(gzip.decompress((previous / 'preparation-v1/baseline-source-hashes.json.gz').read_bytes()))
    candidate_original = json.loads(gzip.decompress((previous / 'preparation-v1/candidate-source-hashes.json.gz').read_bytes()))
    assert sources() == prior['main_source_sha256'] and len(original) == 10128
    assert all(sha(baseline / p) == h for p, h in original.items())
    assert all(sha(full_candidate / p) == h for p, h in candidate_original.items())
    project = {p: h for p, h in original.items() if p != 'docs/properties-status.json'}
    assert all(sha(ROOT / p) == h for p, h in project.items())
    paths = set(filter(None, subprocess.check_output(
        ['git', 'ls-files', '-c', '-o', '--exclude-standard', '-z'], cwd=ROOT).decode().split('\0')))
    paths = {p for p in paths if not p.startswith(('docs/evidence/', 'crates/comparisons/', 'target/'))}
    assert paths == set(original)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    assert len(edits['copied_paths']) == 11 and edits['leaf_path'] not in edits['copied_paths']
    prior_costs = previous / 'library-costs-v1'
    prior_result = json.loads((prior_costs / 'result.json').read_text())
    assert prior_result['complete']
    assert all(sha(prior_costs / p) == h for p, h in prior_result['artifact_sha256'].items())
    libraries = json.loads((prior_costs / 'libraries.json').read_text())['baseline']
    binaries = {k: v for k, v in json.loads((prior_costs / 'binaries.json').read_text()).items() if k.startswith('baseline-')}
    assert len(libraries) == 5 and len(binaries) == 11
    assert all(sha(r['path']) == r['sha256'] for r in [*libraries.values(), *binaries.values()])
    old_protocol = json.loads((prior_costs / 'protocol.json').read_text())
    assert subprocess.check_output(['rustc', '-vV'], text=True) == old_protocol['compiler']
    assert os.environ.get('RUSTFLAGS') == old_protocol['inherited_rustflags']
    WORK.mkdir(exist_ok=False)
    PREP.mkdir(exist_ok=False)
    variants = {}
    for label in ['baseline', 'baseline-metadata']:
        filename = label + '-source-hashes.json.gz'
        shutil.copyfile(previous / 'preparation-v1' / filename, PREP / filename)
        variants[label] = prior['variants'][label] | {'borrowed_unchanged': True}
    for label in ['candidate', 'candidate-metadata']:
        source = WORK / label / 'source'
        source.parent.mkdir()
        parent_source = baseline if label == 'candidate' else Path(variants['candidate']['source'])
        subprocess.run(['cp', '-a', '--reflink=auto', str(parent_source), str(source)], check=True)
        if label == 'candidate':
            for path in edits['copied_paths']:
                (source / path).parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(full_candidate / path, source / path)
            path = source / edits['leaf_path']
            before = path.read_text()
            assert before.count(edits['before']) == 1
            after = before.replace(edits['before'], edits['after'])
            path.write_text(after)
            assert after.replace(edits['after'], edits['before']) == before
        else:
            region = source / 'crates/hypercurve/src/bezier_region.rs'
            region.write_text(region.read_text() + '\n#[cfg(test)]\nmod combined_admission_metadata;\n')
            path = 'crates/hypercurve/src/bezier_region/combined_admission_metadata.rs'
            shutil.copyfile(Path(variants['baseline-metadata']['source']) / path, source / path)
            subprocess.run(['rustfmt', '--edition', '2024', str(region), str(source / path)], check=True)
        for path in edits['original_kernel_paths']:
            assert sha(source / path) == original[path]
        assert not (source / edits['absent_file']).exists()
        for path, digest in prior['pending_native_sha256'].items():
            if path not in edits['copied_paths']:
                assert sha(source / path) == digest
        mapping = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
        delta = {p: h for p, h in mapping.items() if original.get(p) != h}
        expected = set(edits['copied_paths']) | {edits['leaf_path']}
        if label.endswith('-metadata'):
            expected.add('crates/hypercurve/src/bezier_region/combined_admission_metadata.rs')
        assert set(delta) == expected
        assert len(mapping) == 10132 + int(label.endswith('-metadata'))
        with (PREP / (label + '-format.log')).open('w') as log:
            subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source, env=os.environ | ENV,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        compressed(PREP / (label + '-source-hashes.json.gz'), mapping)
        patch = ''.join(''.join(difflib.unified_diff(
            (baseline / p).read_text().splitlines(True) if p in original else [],
            (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(delta))
        (PREP / (label + '.patch.gz')).write_bytes(gzip.compress(patch.encode(), mtime=0))
        with tarfile.open(PREP / (label + '-source-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(delta):
                archive.add(source / p, arcname=p)
        variants[label] = {'source': str(source), 'target': str(WORK / label / 'target'),
                           'files': len(mapping), 'source_hashes_sha256': sha(PREP / (label + '-source-hashes.json.gz')),
                           'delta_sha256': delta, 'borrowed_unchanged': False}
    compressed(PREP / 'main-project-hashes.json.gz', project)
    historical_probes = {p.name: {'path': str(p.relative_to(ROOT)), 'sha256': sha(p)}
                         for p in sorted((previous / 'probes-v1').glob('*.rs'))}
    assert len(historical_probes) == 18
    prior_inputs = ['preparation-v1/protocol.json', 'preparation-v1/baseline-source-hashes.json.gz',
                    'preparation-v1/baseline-metadata-source-hashes.json.gz', 'preparation-v1/candidate-source-hashes.json.gz',
                    'library-costs-v1/protocol.json', 'library-costs-v1/libraries.json', 'library-costs-v1/binaries.json',
                    'library-costs-v1/result.json', 'library-costs-v1/commands.json', 'decision-v1.json', 'next-combination-v1.json']
    inputs = ['common_v1.py', 'measure_libraries_v1.py', 'assess_libraries_v1.py', 'runtime-edits-v1.json', 'scope-v1.json']
    result = {'scope': __doc__, 'runner_sha256': sha(__file__),
              'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'inputs_sha256': {p: sha(E / p) for p in inputs},
              'prior_inputs_sha256': {str((previous / p).relative_to(ROOT)): sha(previous / p) for p in prior_inputs},
              'main_source_sha256': prior['main_source_sha256'], 'pending_native_sha256': prior['pending_native_sha256'],
              'main_project_hashes_sha256': sha(PREP / 'main-project-hashes.json.gz'),
              'main_project_files_excluding_evidence_ledger': len(project),
              'historical_probes': historical_probes, 'variants': variants, 'parent': prior['parent'],
              'baseline_reuse': {'libraries': libraries, 'binaries': binaries,
                                 'source': 'Unchanged prior current-main production and instrumented metadata copies.',
                                 'measurement_reuse': False, 'compiler_and_rustflags_identical': True,
                                 'prior_protocol': str((prior_costs / 'protocol.json').relative_to(ROOT)),
                                 'prior_protocol_sha256': sha(prior_costs / 'protocol.json')},
              'runtime_admitted': False, 'plan_complete': False}
    verify(result, full=True)
    dump(PREP / 'protocol.json', result)
    print(json.dumps({'prepared': True, 'candidate_paths_changed': 12, 'baseline_binaries_reused': len(binaries),
                      'all_old_sources_and_binaries_unchanged': True, 'main_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
