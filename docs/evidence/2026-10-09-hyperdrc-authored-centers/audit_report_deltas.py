"""Accept only the six recorded connector decimal presentations and version fields.

This is an exact byte patch to the baseline, not general report normalization.
Any other message, marker, geometry, identity, receipt or byte change fails.
Usage: python3 audit_report_deltas.py REPORT_OUTPUT
"""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
manifest = json.loads((out / 'manifest.json').read_text())
result = {}
for label in ['cparti-fpga', 'hvp109a']:
    before = gzip.decompress((record / f'baseline-reports/{label}.json.gz').read_bytes())
    trials = [gzip.decompress((out / f'trial-{trial}-{label}.json.gz').read_bytes()) for trial in [1, 2]]
    assert trials[0] == trials[1]
    candidate = trials[0]
    expected = before.replace(b'"check_version":13', b'"check_version":14')
    assert before.count(b'"check_version":13') == 8
    baseline_json, candidate_json = json.loads(before), json.loads(candidate)
    changes = []
    if label == 'cparti-fpga':
        nets = ['/USB/USB_D+', '/USB/USB_D-', 'unconnected-(J1-SBU1-PadA8)',
                '/USB/USB_D+', '/USB/USB_D-', 'unconnected-(J1-SBU2-PadB8)']
        for index, net in enumerate(nets, 2):
            old = f'likely connector edge-rate net "{net}" is 0.000000e0 from board edge without parsed same-layer ground return copper within 0.900000'
            new = old.replace(' is 0.000000e0 ', ' is 0.000000 ')
            left, right = baseline_json['violations'][index], candidate_json['violations'][index]
            assert left['check'] == right['check'] == 'connector-return-path-readiness'
            assert left['message'] == old and right['message'] == new
            assert {k:v for k,v in left.items() if k != 'message'} == {k:v for k,v in right.items() if k != 'message'}
            changes.append(dict(path=['violations', index, 'message'], before=old, after=new))
        for (old, new), count in Counter((r['before'], r['after']) for r in changes).items():
            old_bytes, new_bytes = [json.dumps(text, ensure_ascii=False).encode() for text in [old, new]]
            assert expected.count(old_bytes) == count
            expected = expected.replace(old_bytes, new_bytes)
    assert candidate == expected, f'Unreviewed complete-report delta: {label}'
    row = manifest['reports'][label]
    assert len(candidate) == row['bytes'] and hashlib.sha256(candidate).hexdigest() == row['sha256']
    assert row['baseline_only_version_changed'] == (not changes)
    result[label] = dict(version_fields=8, diagnostic_changes=changes,
        candidate_bytes=len(candidate), candidate_sha256=hashlib.sha256(candidate).hexdigest(),
        all_other_bytes_unchanged=True, candidate_repetitions_identical=True)
audit = dict(status='passed', scope='Eight version fields per board and exactly six CPArti connector messages: scientific zero suffix e0 removed by the explicit six-place fixed-decimal view. Every other complete report byte is preserved. No general normalization or universal determinism claim.', reports=result)
(out / 'baseline-delta-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
print(json.dumps({label: {'version_fields': row['version_fields'], 'diagnostic_changes': len(row['diagnostic_changes']), 'all_other_bytes_unchanged': row['all_other_bytes_unchanged']} for label,row in result.items()}, indent=2))
