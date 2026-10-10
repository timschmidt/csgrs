"""Check the frozen residual implementation, production regressions and exact oracle."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile
import textwrap

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE/'contact-index-residual-width'
SOURCE = ROOT/'target/contact-index-residual-width/source'
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
text=(SOURCE/'crates/hypercurve/src/curve_region_boolean.rs').read_text()
begin=text.index('    /// Upper bound on the width of a finite, ordered binary64 interval.')
end=text.index('    /// Mathematical-order key',begin)
helper=textwrap.dedent(text[begin:end]).strip()+'\n'
assert helper==(EVIDENCE/'contact_index_width_upper_bound.rs').read_text()
oracle=read(EVIDENCE/'contact-index-width-oracle/manifest.json')
assert hashlib.sha256(helper.encode()).hexdigest()==oracle['helper_sha256']
compressed=EVIDENCE/'contact-index-width-oracle/run.jsonl.gz'
assert digest(compressed)==oracle['compressed_rows_sha256']
raw=gzip.decompress(compressed.read_bytes())
assert len(raw)==oracle['raw_rows_bytes'] and hashlib.sha256(raw).hexdigest()==oracle['raw_rows_sha256']
assert oracle['cases']==94442 and oracle['encoded_endpoint_exponents']==2047 and oracle['failures']==0
assert read(EVIDENCE/'contact-index-width-oracle/failures.json')==[]
matrix=read(EVIDENCE/'contact-index-residual-width-matrix/manifest.json')
assert all(row['exit_code']==0 for row in matrix['commands'])
assert matrix['results']=={'values':70,'intervals':2486,'overlap_comparisons':6354216,'shared_omissions':0,'isolated_omissions':0,'extras':0}
record={
    'decision':'focused exactness gates pass for the general subtraction-residual candidate; resource and full production admission gates remain pending',
    'previous_experiment':'diagnostics-52.json',
    'counterexample_record':'diagnostics-47.json',
    'only_modified_sources':sources['only_modified_sources'],
    'change':'Use a Two-Diff subtraction residual to keep exact/already-upward widths and round downward widths upward; reject nonfinite residuals as evidence for tighter bounds. Normalize signed zero in index keys.',
    'completeness_argument':'The resulting width is at least the exact rational endpoint difference. For overlapping [a,b] and [c,d], c-W<=a and a<=d; monotone rounding and mathematical-order keys retain the stored lower endpoint. Existing box-overlap comparisons and exact contact matching remain unchanged.',
    'finite_range_argument':oracle['range_argument'],
    'source_reference':oracle['primary_reference'],
    'same_helper_bytes_checked_by_independent_oracle':True,
    'independent_width_oracle':{key:oracle[key] for key in ['cases','buckets','encoded_endpoint_exponents','correct_infinite_upper_bounds','failures','oracle','limitations']},
    'independent_contact_oracle':matrix['results'],
    'production_unit_tests_passed':4,
    'production_regressions':'Mathematical key order; arbitrary conservative intervals including subnormals and overflow; identical selected points with different isolators under both policies, repeated/cloned queries and explicit bounds warm-up; finite exact MAX widths with zero or symmetric nonzero endpoints; degenerate zero widths.',
    'commands':status,
    'frozen_candidate_files':len(sources['source_hashes']),
    'frozen_main_inputs':len(main['source_hashes']),
    'main_runtime_unchanged':True,
    'next_step':'Review matched resources with both saturation controls, then run complete curve/DRC validation and full report replay before runtime admission.',
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for folder in [OUT,EVIDENCE/'contact-index-width-oracle',EVIDENCE/'contact-index-residual-width-matrix'] for path in sorted(folder.iterdir()) if path.is_file()},
}
(EVIDENCE/'diagnostics-53.json').write_text(json.dumps(record,indent=2)+'\n')
print('Residual focused gates: 4 production tests, 94,442 exact rational width checks, and 6,354,216 contact interval comparisons pass.')
