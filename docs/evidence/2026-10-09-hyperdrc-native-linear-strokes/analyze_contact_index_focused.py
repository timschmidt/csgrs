"""Check the isolated contact-index repair and its independent interval oracle."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-outward-width'
SOURCE = ROOT / 'target/contact-index-outward-width/source'
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
sources = read(OUT / 'sources.json')
main = read(OUT / 'main-sources.json')
correction = read(OUT / 'main-sources-metadata-correction.json')
assert digest(OUT / 'main-sources-before-metadata-correction.json') == correction['before_sha256']
assert digest(OUT / 'main-sources.json') == correction['after_sha256']
assert read(OUT / 'main-sources-before-metadata-correction.json')['source_hashes'] == main['source_hashes']
audit = read(OUT / 'committed-source-audit.json')
assert all(digest(SOURCE / name) == value for name, value in sources['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items())
with tarfile.open(OUT / 'source-delta.tar.gz') as archive:
    for name in sources['only_modified_sources']:
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == sources['source_hashes'][name]
status = read(OUT / 'status.json')
assert len(status) == 2 and all(command['exit_code'] == 0 for command in status)
log = (OUT / 'contact-index-unit-tests.log').read_text()
assert re.search(r'test result: ok\. 3 passed; 0 failed;', log)
matrix = EVIDENCE / 'contact-index-range-matrix'
matrix_record = read(matrix / 'manifest.json')
assert [command['exit_code'] for command in matrix_record['commands']] == [0, 101, 0, 101, 0, 0]
assert 'range start is greater than range end' in (matrix / 'baseline-run.log').read_text()
partial = json.loads((matrix / 'width-only-run.jsonl').read_text())
complete = json.loads((matrix / 'candidate-run.jsonl').read_text())
assert partial == {'values':70, 'intervals':2486, 'overlap_comparisons':6354216, 'shared_omissions':1296, 'isolated_omissions':36, 'extras':0}
assert complete == partial | {'shared_omissions':0, 'isolated_omissions':0}
for name, expected in matrix_record['source_hashes'].items():
    assert digest(matrix / name) == expected
record = {
    'decision': 'focused exactness checks pass; full production suites and resource/output evidence still required before admission',
    'counterexample_record': 'diagnostics-47.json',
    'production_changes': sources['only_modified_sources'],
    'repair': [
        'Round each computed box width upward before taking the maximum.',
        'Treat both signed zeros as one mathematical endpoint in ordered keys.',
    ],
    'completeness_argument': [
        'For finite ordered endpoints a<=b, next_up(round_nearest(b-a)) is an upper bound on the exact real width, with positive infinity allowed as a conservative upper bound.',
        'For an overlapping query [c,d], c<=b and a<=d. The maximum stored width W satisfies W>=b-a, so c-W<=a.',
        'Nearest rounding is monotone and a is representable. Therefore round_nearest(c-W)<=a. If W is infinite, the computed lower bound is negative infinity.',
        'Keys preserve mathematical order, including identifying -0 and +0. The search range therefore includes the stored lower endpoint a.',
        'The unchanged four interval comparisons then reject only disjoint boxes. Unboxed contacts, inherited vertex boxes, incidence merging and ascending output order retain their existing behavior.',
    ],
    'range_matrix': {
        'endpoint_values':70, 'valid_intervals':2486, 'overlap_comparisons_per_completed_variant':6354216,
        'baseline': 'Panics on a valid signed-zero interval before producing its final summary; raw log preserved.',
        'width_only':partial, 'complete_repair':complete,
        'scope': 'Direct interval-overlap oracle with adapter-supplied valid conservative boxes. Signed-zero producer reachability and a full public Boolean failure are not established by this matrix.',
    },
    'production_tests': {
        'passed':3,
        'mathematical_key_pairs':72*72,
        'isolated_interval_point_queries':2486*70,
        'shared_interval_point_queries':70,
        'selected_point_pairs':24,
        'selected_point_candidate_queries':24*2*3,
        'selected_point_coverage': 'STRICT and APPROXIMATE_512; positive/negative powers 2^-1000, 1, 2^1000; both insertion directions; no explicit bounds warm-up versus explicit bounds warm-up; inherited vertex boxes, unboxed contact, merged incidences, repeated and cloned query points.',
    },
    'source_audit': {
        'committed_production_inputs':audit['checked_committed_inputs'],
        'all_baseline_inputs_match_git':audit['all_baseline_inputs_match_git'],
        'candidate_existing_differences':audit['candidate_existing_differences'],
        'candidate_new_sources':audit['candidate_new_sources'],
        'frozen_workspace_files':len(sources['source_hashes']),
        'frozen_main_inputs':len(main['source_hashes']),
    },
    'commands':status,
    'main_runtime_unchanged':True,
    'inventory_metadata_correction':correction,
    'native_stroke_prototype_admitted':False,
    'next_step':'Complete release curve/DRC tests, strict lints/docs and original board gate; compare preserved report bytes and proportionate candidate-index resource measurements.',
    'artifacts': {str(path.relative_to(EVIDENCE)):digest(path) for folder in [OUT,matrix] for path in sorted(folder.iterdir()) if path.is_file()},
}
(EVIDENCE / 'diagnostics-48.json').write_text(json.dumps(record, indent=2)+'\n')
print('Focused repair passed: 3 production tests; 6,354,216 independent interval comparisons; production change not admitted yet.')
