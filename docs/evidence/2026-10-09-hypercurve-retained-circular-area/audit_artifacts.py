"""Replay hashes, frozen sources, old failures, exact rows and full report bytes."""
import gzip,hashlib,json,subprocess,tarfile
from pathlib import Path
record=Path(__file__).resolve().parent
previous=record.parent/'2026-10-09-hyperdrc-native-pad-primitives'
load=lambda p:json.loads(p.read_bytes())
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
manifest=load(record/'manifest.json');assert manifest['status']=='progress'
for path,witness in manifest['artifacts'].items(): assert info(record/path)==witness,path
for variant in ['rejected-global-projection','withdrawn-shared-witness-cache','direct-source-evaluation-failure']:
    status=load(record/variant/'status.json')
    assert status['runner_exit_code']==130 and not status['full_suite_complete']
    with tarfile.open(record/variant/'sources.tar.gz') as archive:
        for path,h in status['source_hashes'].items():
            assert hashlib.sha256(archive.extractfile(path).read()).hexdigest()==h
for variant in ['image-projection-variant','native-similarity-failure']:
    status=load(record/variant/'status.json')
    with tarfile.open(record/variant/'sources.tar.gz') as archive:
        for path,h in status['source_hashes'].items():
            assert hashlib.sha256(archive.extractfile(path).read()).hexdigest()==h
failure=load(record/'native-similarity-failure/status.json')
assert failure['runner_exit_code']==101 and failure['full_curve_suite_complete']
assert failure['curve_totals']==dict(passed=2435,failed=1,ignored=33)
base=manifest['base_revision'];baseline=load(record/'baseline.json')
assert baseline['exit_code']==101 and baseline['unchanged']
for path,h in baseline['source_hashes'].items():
    if path.endswith('bezier_region/retained_circular_tests.rs'):data=(record/'baseline-tests.rs').read_bytes()
    else:
        data=subprocess.check_output(['git','show',f'{base}:{path}'])
        if path.endswith('bezier_region.rs'):
            data=data.replace(b'mod point_location;\n',b'mod point_location;\n#[cfg(test)]\nmod retained_circular_tests;\n')
    assert hashlib.sha256(data).hexdigest()==h,path
focused=load(record/'focused/manifest.json')
assert focused['source_unchanged'] and all(s['exit_code']==0 for s in focused['steps'])
assert [s['name'] for s in focused['steps']]==['circular','root-views','similarity']
for path,h in focused['source_hashes'].items():assert info(Path(path))['sha256']==h,path
steps=load(record/'validation/status.json')
assert len(steps)==14
assert all(s['exit_code']==0 or (s['name']=='demo-tests' and s['exit_code']==101) for s in steps)
demo=load(record/'demo-baseline/manifest.json')
assert demo['status']=='matched-existing-failures'
assert demo['baseline_counts']==[1,2,0] and demo['candidate_counts']==[35,2,0]
assert info(record/'demo-baseline/baseline.log')==demo['baseline_log']
assert info(record/'demo-baseline/candidate.log')==demo['candidate_log']==info(record/'validation/demo-tests.log')
sources=load(record/'validation/sources.json');assert sources['unchanged']
for path,h in sources['source_hashes'].items():assert info(Path(path))['sha256']==h,path
assert (record/'cost/rows.json').read_bytes()==(previous/'cost/rows.json').read_bytes()
assert load(record/'cost/all-prior-row-deltas.json')==[]
fixture_audit=load(record/'cost/fixture-report-audit.json')
assert fixture_audit['ordered_check_counts_unchanged'] and fixture_audit['normalized_reports_unchanged']
capability=load(record/'capability-cost/manifest.json')
identities=lambda rows:[{k:v for k,v in r.items() if k!='elapsed_ns'} for r in rows]
for observation in capability['observations']:
    assert identities(observation['rows'])==load(record/'capability-cost/rows.json')
reports=load(record/'report-replay/manifest.json')
for label,result in reports['reports'].items():
    old=gzip.decompress((previous/f'report-replay/trial-1-{label}.json.gz').read_bytes())
    current=[gzip.decompress((record/f'report-replay/trial-{trial}-{label}.json.gz').read_bytes()) for trial in [1,2]]
    assert old.count(b'"check_version":18')==12
    assert current[0]==current[1]==old.replace(b'"check_version":18',b'"check_version":19')
    assert len(current[0])==result['bytes'] and hashlib.sha256(current[0]).hexdigest()==result['sha256']
for row in load(record/'snapshot-versions.json'):
    old=subprocess.check_output(['git','show',f"{base}:{row['path']}"])
    current=Path(row['path']).read_bytes()
    assert old.count(b'"check_version": 18')==row['changed_check_versions']==160
    assert current==old.replace(b'"check_version": 18',b'"check_version": 19')
pair=load(record/'original-pair/after.json')
assert pair['overlap_area_error'] is None and pair['overlap_native_view_available'] and pair['duplicate_findings']==[]
result=dict(status='passed',frozen_sources=len(sources['source_hashes']),baseline_sources=len(baseline['source_hashes']),validation_steps=14,passing_validation_steps=13,baseline_verified_failed_step='demo-tests',existing_demo_failures=2,prior_cost_rows=231,new_cost_rows=4,full_boards=2,complete_reports=4,snapshots=6,original_pair='Exact area and native contour available; strict positive area and upper-bound predicates replayed by original_pair_probe.rs.',scope='Scoped evidence only; the complete properties/future-work goal remains active.')
(record/'artifact-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
