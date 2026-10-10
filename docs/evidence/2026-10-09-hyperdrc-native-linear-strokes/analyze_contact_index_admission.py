"""Combine the final candidate's correctness, resource and complete report gates."""
import gzip
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
read=lambda path:json.loads(path.read_text())
digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
focused=read(EVIDENCE/'diagnostics-55.json')
resources=read(EVIDENCE/'diagnostics-56.json')
validation=read(EVIDENCE/'diagnostics-57.json')
source=read(EVIDENCE/'contact-index-directed-width/sources.json')
main=read(Path(source['main_source_record']))
SOURCE=ROOT/'target/contact-index-directed-width/source'
assert all(digest(SOURCE/name)==value for name,value in source['source_hashes'].items())
assert all(digest(ROOT/name)==value for name,value in main['source_hashes'].items())
assert focused['production_unit_tests_passed']==4
assert focused['independent_width_oracle']['cases']==94442 and focused['independent_width_oracle']['failures']==0
assert focused['independent_contact_oracle']['overlap_comparisons']==6354216
assert resources['corrected_witness_rows']==12 and resources['unchanged_witness_rows']==28
assert resources['unchanged_allocation_rows']==28 and resources['allocation_samples_repeat_exactly']
assert validation['suites']['curve-tests']=={'test_binaries':49,'passed':2430,'failed':0,'ignored':28}
assert validation['suites']['drc-tests']=={'test_binaries':1,'passed':1559,'failed':0,'ignored':0}
assert validation['format_strict_clippy_and_docs_passed'] and validation['original_board_gate']['passed']
cost=EVIDENCE/'contact-index-directed-width-cost'
cost_record=read(cost/'manifest.json')
retained=read(cost/'retained-libraries.json')
for variant,record in retained['libraries'].items():
    assert record['sha256']==cost_record['libraries'][variant]['sha256']
    assert digest(Path(record['retained_path']))==record['sha256']
for name,record in cost_record['binaries'].items():
    assert digest(cost/name)==record['sha256']
replay=EVIDENCE/'contact-index-directed-replay'
reports=read(replay/'manifest.json')
assert len(reports['commands'])==20 and all(row['exit_code']==0 for row in reports['commands'])
assert reports['historical_row_deltas']==0 and len(read(replay/'rows.json'))==231
previous=read(EVIDENCE/'ray-filter-production-replay/manifest.json')
for board,record in reports['reports'].items():
    assert record['sha256']==previous['reports'][board]['sha256']
    assert record['complete_bytes_identical_across_both_variants_and_repetitions']
    for variant in ['baseline','candidate']:
        for repetition in [1,2]:
            raw=gzip.decompress((replay/f'{variant}-reports-{repetition}-{board}.json.gz').read_bytes())
            assert len(raw)==record['bytes'] and hashlib.sha256(raw).hexdigest()==record['sha256']
for variant in reports['variants'].values():
    for filename,record in variant['libraries'].items():assert digest(Path(filename))==record['sha256']
    for record in variant['binaries'].values():assert digest(Path(record['path']))==record['sha256']
record={
    'decision':'kept as an exactness repair after focused mathematical oracles, complete production suites, strict checks, original board gate, matched resources and unchanged complete reports',
    'counterexample_record':'diagnostics-47.json',
    'rejected_or_refined_experiments':['diagnostics-50.json','diagnostics-52.json','diagnostics-54.json'],
    'focused_record':'diagnostics-55.json',
    'resource_record':'diagnostics-56.json',
    'production_validation_record':'diagnostics-57.json',
    'only_modified_sources':source['only_modified_sources'],
    'production_suites':validation['suites'],
    'original_board_gate':validation['original_board_gate'],
    'width_oracle_cases':94442,
    'contact_interval_comparisons':6354216,
    'corrected_index_witness_rows':12,
    'unchanged_index_witness_rows':28,
    'historical_capability_rows_preserved':231,
    'complete_reports':reports['reports'],
    'resource_review':resources['resource_review'],
    'main_runtime_unchanged_when_validated_and_measured':True,
    'limitations':[
        'The confirmed wrong answer is a private candidate-index omission fed by actual public point producers and certified point equality; a complete public Boolean wrong-answer example has not been established.',
        'The width and interval oracles are deterministic corpora and an algebraic argument, not exhaustive binary64 enumeration or a machine-checked proof.',
        'Cost probes exercise the exact extracted index with actual production point producers and a public STRICT bounds wrapper. They are not complete Boolean/application performance measurements.',
        'The curve suite retains 28 existing ignored cases. Full workspace/pre-push validation and broader native-stroke admission remain outstanding.',
    ],
    'next_step':'Apply only the tested source delta, verify staged blobs against the tested archive and preserve every unrelated native-prototype edit, then commit the repair.',
    'artifacts':{str(path.relative_to(EVIDENCE)):digest(path) for path in [EVIDENCE/'diagnostics-55.json',EVIDENCE/'diagnostics-56.json',EVIDENCE/'diagnostics-57.json',replay/'manifest.json',cost/'manifest.json',cost/'retained-libraries.json']},
}
(EVIDENCE/'diagnostics-58.json').write_text(json.dumps(record,indent=2)+'\n')
print('Final candidate admitted by evidence: exact index recovery, 2430 curve + 1559 DRC passes, 231 preserved rows and unchanged complete report bytes.')
