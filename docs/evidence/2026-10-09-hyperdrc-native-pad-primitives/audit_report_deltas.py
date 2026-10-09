"""Require complete report bytes to change only their twelve version fields.

Usage: python3 audit_report_deltas.py REPORT_OUTPUT
The rejected pre-area-fix reports are retained separately and never accepted by
this comparison. No finding normalization or count-only acceptance is allowed.
"""
import gzip
import hashlib
import json
from pathlib import Path
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
manifest = json.loads((out/'manifest.json').read_bytes())
reports = {}
for label,count in [('cparti-fpga',2610),('hvp109a',658)]:
    before = gzip.decompress((record/f'baseline-reports/trial-1-{label}.json.gz').read_bytes())
    raw = [gzip.decompress((out/f'trial-{trial}-{label}.json.gz').read_bytes()) for trial in [1,2]]
    assert before.count(b'"check_version":17') == 12
    assert raw[0] == raw[1] == before.replace(b'"check_version":17', b'"check_version":18'), label
    old,new = map(json.loads,[before,raw[0]])
    assert old['violation_count'] == new['violation_count'] == count
    assert old['summary'] == new['summary']
    assert old['violations'] == new['violations']
    row = manifest['reports'][label]
    assert row['bytes'] == len(raw[0]) and row['sha256'] == hashlib.sha256(raw[0]).hexdigest()
    assert row['baseline_only_version_changed'] and row['identical_complete_bytes']
    reports[label] = dict(findings=count,version_fields=12,changed_findings=0,
                          complete_bytes=len(raw[0]),sha256=row['sha256'],all_other_bytes_unchanged=True)
result = dict(status='passed',scope='Both complete report serializations repeat exactly across two fresh processes on the fixed full-board paths and synthetic IPC sidecar. Only twelve version fields per board change from 17 to 18; every other byte, including findings, markers, coordinates, IDs and diagnostics, remains identical to the prior committed reports. No universal or path-independent determinism claim.',reports=reports)
(out/'baseline-delta-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print('Both complete reports: only twelve version fields each; all findings and other bytes unchanged.')
