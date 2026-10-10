"""Record focused correctness for the equal/zero-endpoint width experiment."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'contact-index-exact-widths'
SOURCE = ROOT / 'target/contact-index-exact-widths/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
sources = read(OUT/'sources.json')
main = read(OUT/'main-sources.json')
assert all(digest(SOURCE/name)==value for name,value in sources['source_hashes'].items())
assert all(digest(ROOT/name)==value for name,value in main['source_hashes'].items())
status = read(OUT/'status.json')
assert len(status)==2 and all(row['exit_code']==0 for row in status)
assert re.search(r'test result: ok\. 4 passed; 0 failed;', (OUT/'contact-index-unit-tests.log').read_text())
with tarfile.open(OUT/'source-delta.tar.gz') as archive:
    assert set(archive.getnames())==set(sources['only_modified_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest()==sources['source_hashes'][name]
matrix = read(EVIDENCE/'contact-index-exact-widths-matrix/manifest.json')
assert all(row['exit_code']==0 for row in matrix['commands'])
assert matrix['results']=={'values':70,'intervals':2486,'overlap_comparisons':6354216,'shared_omissions':0,'isolated_omissions':0,'extras':0}
record = {
    'decision':'focused correctness passes for the equal/zero-endpoint exception experiment; resource review and broader gates remain required',
    'previous_candidate_record':'diagnostics-50.json',
    'only_modified_sources':sources['only_modified_sources'],
    'change':'Preserve the exact width for equal endpoints or a zero endpoint; otherwise keep the outward-rounded upper bound and mathematical-order zero keys.',
    'proof':'Subtraction of identical finite endpoints is zero. Subtraction with either endpoint zero is exact negation or identity. These widths are exact upper bounds; all other cases retain the previous conservative upward step. Therefore the completeness argument in diagnostics-48 still applies.',
    'production_unit_tests_passed':4,
    'new_regression':'Exact widths MAX for [0,MAX] and [-MAX,0] must remain finite; six degenerate extreme/zero boxes retain zero width.',
    'unchanged_regressions':'Mathematical keys, the 70-value/2486-interval oracle, and 24 selected-point pairs under both policies with explicit bounds warm-up and repeated/cloned queries.',
    'independent_interval_oracle':matrix['results'],
    'commands':status,
    'frozen_candidate_files':len(sources['source_hashes']),
    'frozen_main_inputs':len(main['source_hashes']),
    'main_runtime_unchanged':True,
    'full_production_validation_performed_for_this_candidate':False,
    'next_step':'Review the early resource measurements and the separately identified nonzero symmetric MAX-width case before starting full validation.',
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for folder in [OUT,EVIDENCE/'contact-index-exact-widths-matrix'] for path in sorted(folder.iterdir()) if path.is_file()},
}
(EVIDENCE/'diagnostics-51.json').write_text(json.dumps(record,indent=2)+'\n')
print('Refined focused checks pass: 4 production tests and 6,354,216 interval comparisons.')
