"""Verify frozen source hashes, failed baselines, complete reports and cost rows."""
import ast
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess

record = Path(__file__).resolve().parent
load = lambda p: json.loads(p.read_bytes())
digest = lambda b: hashlib.sha256(b).hexdigest()
info = lambda p: dict(bytes=Path(p).stat().st_size,sha256=digest(Path(p).read_bytes()))
manifest = load(record/'manifest.json')
base = manifest['base_revision']
pattern = re.compile(r'^test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',re.M)
def results(path,passed,failed=0):
    rows = pattern.findall(path.read_text())
    assert rows and [sum(int(r[i]) for r in rows) for i in [1,2,3]] == [passed,failed,0],path
    return dict(passed=passed,failed=failed,ignored=0)

source_counts = {}
for folder,names in [
    ('validation',['fmt','clippy','test','docs','fixture-build']),
    ('consumer-validation',['fmt','consumer-clippy','consumer-drc','consumer-curves']),
    ('expanded',['fmt','clippy','pads','geometry','rectangles','kicad','duplicate-layer','bounds']),
]:
    steps = load(record/folder/'status.json')
    assert [s['name'] for s in steps] == names and all(s['exit_code'] == 0 for s in steps)
    sources = load(record/folder/'sources.json')
    assert sources['unchanged']
    for path,h in sources['source_hashes'].items(): assert info(path)['sha256'] == h,path
    source_counts[folder] = len(sources['source_hashes'])
assert source_counts == dict(validation=141,**{'consumer-validation':282,'expanded':141})
tests = {
    'drc':results(record/'validation/test.log',1561),
    'consumer-drc':results(record/'consumer-validation/consumer-drc.log',9),
    'consumer-curves':results(record/'consumer-validation/consumer-curves.log',5),
    'pads':results(record/'expanded/pads.log',28),
    'geometry':results(record/'expanded/geometry.log',59),
    'rectangles':results(record/'expanded/rectangles.log',20),
    'kicad':results(record/'expanded/kicad.log',45),
    'duplicates':results(record/'expanded/duplicate-layer.log',6),
    'bounds':results(record/'expanded/bounds.log',24),
    'baseline':results(record/'baseline.log',4,16),
    'pre-area-green':results(record/'pre-area-fix/validation/test.log',1559),
    'area-baseline':results(record/'pre-area-fix/duplicate_layer_islands_certify_cparti_rounded_pad_pair.log',0,1),
    'area-control':results(record/'pre-area-fix/duplicate_polygon_overlap_bound_preserves_tolerance_and_touching.log',1),
    'required-bound-failure':results(record/'required-bound-failure/validation/test.log',1555,1),
}
assert all(s['exit_code'] == 0 for s in load(record/'final-validation.json'))
assert all(s['exit_code'] == 0 for s in load(record/'checkpoint-validation.json'))
initial_checkpoint = load(record/'area-probe-assertion-failure/checkpoint-validation.json')
assert [s['exit_code'] for s in initial_checkpoint] == [0,0,0,0,1]
baseline = load(record/'baseline.json')
assert baseline['production_commit'] == base and baseline['exit_code'] == 101 and baseline['unchanged']
for path,h in baseline['source_hashes'].items():
    if path.endswith('/geometry/pad_regressions.rs'):
        data = (record/'baseline-regressions.rs').read_bytes()
    else:
        data = subprocess.check_output(['git','show',f'{base}:{path}'])
        if path.endswith('/geometry.rs'):
            anchor = baseline['injection_anchor'].encode(); assert data.count(anchor) == 1
            data = data.replace(anchor,anchor+baseline['test_module_registration'].encode())
    assert digest(data) == h,path
assert len(baseline['source_hashes']) == 138
for folder in ['candidate-initial','pre-area-fix/expanded','pre-area-fix/validation','pre-area-fix/consumer-validation']:
    assert load(record/folder/'sources.json')['unchanged']
    assert all(s['exit_code'] == 0 for s in load(record/folder/'status.json'))
area_baseline = load(record/'pre-area-fix/area-regression-baseline.json')
assert area_baseline['unchanged'] and [s['exit_code'] for s in area_baseline['commands']] == [101,0]
required_bound = load(record/'required-bound-failure/validation/sources.json')
assert required_bound['unchanged']
assert [s['exit_code'] for s in load(record/'required-bound-failure/validation/status.json')] == [0,0,101]
for path,h in required_bound['source_hashes'].items():
    data = gzip.decompress((record/'required-bound-failure/layer-before-optional.rs.gz').read_bytes()) if path.endswith('/checks/layer.rs') else Path(path).read_bytes()
    assert digest(data) == h,path
area_reconstruction = load(record/'pre-area-fix/area-baseline-reconstruction.json')
for stage,hashes in [
    ('green',load(record/'pre-area-fix/validation/sources.json')['source_hashes']),
    ('regression',area_baseline['source_hashes']),
]:
    for path,h in hashes.items():
        if path == area_reconstruction['path']:
            data = subprocess.check_output(['git','show',f'{base}:{path}'])
            if stage == 'regression':
                anchor = area_reconstruction['anchor'].encode(); assert data.count(anchor) == 1
                data = data.replace(anchor,(record/'pre-area-fix/area-baseline-tests.rs').read_bytes()+area_reconstruction['insert_suffix'].encode()+anchor)
        else:
            data = Path(path).read_bytes()
        assert digest(data) == h,(stage,path)

prior = record.parent/'2026-10-09-hyperdrc-native-rectangles'
for path,h in load(prior/'validation/sources.json')['source_hashes'].items():
    assert digest(subprocess.check_output(['git','show',f'{base}:{path}'])) == h,path
baseline_report = load(record/'baseline-reports/manifest.json')
assert info(record/'full_report_replay.rs') == baseline_report['sources']['full_report_replay']
for label,row in baseline_report['reports'].items():
    raw = gzip.decompress((record/f'baseline-reports/trial-1-{label}.json.gz').read_bytes())
    assert len(raw) == row['bytes'] and digest(raw) == row['sha256']
for row in load(record/'snapshot-versions.json'):
    old = subprocess.check_output(['git','show',f"{base}:{row['path']}"])
    assert old.count(b'"check_version": 17') == row['changed_check_versions'] == 160
    assert Path(row['path']).read_bytes() == old.replace(b'"check_version": 17',b'"check_version": 18')
    assert digest(old) == row['previous_sha256'] and info(row['path'])['sha256'] == row['current_sha256']

cost = load(record/'cost/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0
assert cost['source_hashes'] == load(record/'validation/sources.json')['source_hashes']
for key,stem in [('probe_source','operation'),('affine_probe_source','affine'),('native_drill_probe_source','native_drill'),('ipc_probe_source','ipc'),('layer_order_probe_source','layer_order'),('point_consumer_probe_source','point_consumer'),('authored_center_probe_source','authored_center'),('dense_cluster_probe_source','dense_cluster'),('fixture_access_probe_source','fixture_access'),('rectangle_probe_source','rectangle'),('pad_probe_source','pad')]:
    assert cost[key] == info(record/'cost'/f'{stem}_probe.rs')
rows = load(record/'cost/rows.json')
assert dict(count=len(rows),**info(record/'cost/rows.json')) == cost['rows'] == manifest['cost']['rows']
assert len(rows) == 231
observations = []
for trial in range(1,6):
    log = (record/f'cost/operation-{trial}.log').read_text(); parsed = []
    for line in log.splitlines():
        if line.startswith('operation_'):
            name,iterations,nanoseconds,witness = line.split('\t')
            parsed.append(dict(name=name,iterations=int(iterations),ns_per_query=int(nanoseconds)/int(iterations),witness=witness))
    assert [{k:v for k,v in row.items() if k != 'ns_per_query'} for row in parsed] == rows
    assert re.search(r'Exit status: 0\s*$',log)
    observations.append(dict(rows=parsed,peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',log)[1])))
assert observations == cost['observations']
assert {r['name']:statistics.median(o['rows'][i]['ns_per_query'] for o in observations) for i,r in enumerate(rows)} == cost['medians_ns']
subprocess.run(['python3',str(record/'audit_cost_rows.py'),str(record/'cost')],check=True,stdout=subprocess.PIPE)
fixture = (record/'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000
assert seconds == manifest['cost']['fixture_seconds'] < 120 and re.search(r'Exit status: 0\s*$',fixture)
fixture_audit = load(record/'cost/fixture-report-audit.json')
assert fixture_audit['ordered_check_counts_unchanged'] and fixture_audit['normalized_reports_unchanged']
assert fixture_audit['diff_lines'] == fixture_audit['finding_diff_lines'] == 0
assert fixture_audit['duplicate_marker_changes'] == []

area = load(record/'area-bound/manifest.json')
assert area['status'] == 'passed' and area['source'] == info(record/'area_bound_probe.rs')
for label,row in area['results'].items():
    assert row['build_exit_code'] == row['exit_code'] == 0 and row['output'] == info(record/f'area-bound/{label}.json')
old,new = [load(record/f'area-bound/{side}.json') for side in ['before','after']]
assert old['duplicate_findings'][0]['check'] == 'geometry-uncertainty' and new['duplicate_findings'] == []
assert {k:v for k,v in old.items() if k != 'duplicate_findings'} == {k:v for k,v in new.items() if k != 'duplicate_findings'}
assert old['overlap_area_error'] == 'region-area for KiCad pad could not be certified: area integral unavailable'
# Independent rational oracle: exact imports of the existing finite placement
# and size API, with pi > 3 giving a strict rounded-area lower bound. The upper
# tolerance uses rectangle area, which exceeds rounded area. No sampled overlap
# or approximate pi value participates in this proof.
fraction = lambda value: sum(map(Fraction,value.split()))
width,height,radius = map(Fraction.from_float,[0.3,1.15,0.075])
boxes = []
for x,y in [[141.995001,98.5],[141.995,97.5]]:
    x,y = map(Fraction.from_float,[x,y])
    boxes.append([x-width/2,y-height/2,x+width/2,y+height/2])
assert [[fraction(v) for v in b] for b in new['bounds']] == boxes
cap = (min(b[2] for b in boxes)-max(b[0] for b in boxes)) * (min(b[3] for b in boxes)-max(b[1] for b in boxes))
assert fraction(new['overlap_bound']) == cap
margin_lower = width*height-radius*radius-cap-max(width*height*Fraction('1e-9'),Fraction('1e-12'))
assert margin_lower > 0
pre = load(record/'pre-area-fix/reports/initial-comparison.json')
assert pre['status'] == 'failed' and pre['reports'] == {'cparti-fpga':False,'hvp109a':True}
report = load(record/'report-replay/manifest.json')
assert report['source_unchanged'] and report['source_hashes'] == cost['source_hashes']
assert all(t['exit_code'] == 0 for t in report['trials']) and len(report['trials']) == 2
assert report['source'] == info(record/'full_report_replay.rs')
for p,h in report['linked_libraries'].items():
    if p.endswith('/libhyperdrc.rlib'): assert h == cost['binaries'][p]
subprocess.run(['python3',str(record/'audit_report_deltas.py'),str(record/'report-replay')],check=True,stdout=subprocess.PIPE)

for path,row in load(record/'log-storage.json').items():
    archive = (record/row['raw_archive']).read_bytes(); raw = gzip.decompress(archive)
    assert digest(archive) == row['archive_sha256']
    assert len(raw) == row['raw_bytes'] and digest(raw) == row['raw_sha256']
    clean = b'\n'.join(line.rstrip(b' \t\r') for line in raw.split(b'\n'))
    assert clean == (record/path).read_bytes() and digest(clean) == row['display_sha256']
for path,h in load(record/'remaining-source-audit.json')['source_hashes'].items(): assert info(path)['sha256'] == h
for path,h in manifest['other_input_hashes'].items(): assert info(path)['sha256'] == h
status = load(Path('docs/properties-status.json'))
assert len(status['properties']) == 39 and len(status['targets']) == 14
assert all(p['status'] != 'complete' for p in status['properties']+status['targets'])
assert str(record.relative_to(Path.cwd())/'manifest.json') in Path('docs/properties-status.json').read_text()
for p in record.rglob('*.py'): ast.parse(p.read_text(),filename=str(p))
for path,expected in manifest['evidence_files'].items(): assert info(record/path) == expected,path
result = dict(status='passed',base_revision=base,source_counts=source_counts,tests=tests,
              baseline_sources_reconstructed=138,stable_rows=231,prior_rows_preserved=205,
              complete_report_version_only=True,fixture_seconds=seconds,
              original_pair_proved_without_changing_geometry=True,retained_circular_integral_still_open=True,
              independent_overlap_cap=str(cap),independent_margin_lower_bound=str(margin_lower),
              properties=39,targets=14,evidence_files=len(manifest['evidence_files']))
(record/'artifact-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
