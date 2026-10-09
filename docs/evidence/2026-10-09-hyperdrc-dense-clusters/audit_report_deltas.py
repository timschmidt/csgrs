"""Permit only reviewed fixed-decimal messages and four version fields per board.

This patches explicit baseline messages, then compares every report byte.
It does not normalize geometry, identities, order, markers or unknown fields.
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
    assert before.count(b'"check_version":14') == 4
    expected = before.replace(b'"check_version":14', b'"check_version":15')
    baseline_json, candidate_json = json.loads(before), json.loads(candidate)
    changes = []
    indices = range(12) if label == 'cparti-fpga' else range(5, 8)
    for index in indices:
        left, right = baseline_json['violations'][index], candidate_json['violations'][index]
        old = left['message']
        assert left['check'] == right['check']
        assert left['check'] in ['local-fiducial-readiness', 'dense-pad-escape-readiness', 'dense-pad-mask-bridge-readiness']
        expected_zeros = 2 if label == 'cparti-fpga' and index >= 8 else 1
        assert old.count('0.000000e0') == expected_zeros
        new = old.replace('0.000000e0', '0.000000')
        assert right['message'] == new
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
    result[label] = dict(version_fields=4, diagnostic_changes=changes,
        candidate_bytes=len(candidate), candidate_sha256=hashlib.sha256(candidate).hexdigest(),
        all_other_bytes_unchanged=True, candidate_repetitions_identical=True)
audit = dict(status='passed', scope='Four version fields per board; exactly twelve CPArti and three HVP109A messages remove the scientific zero suffix e0 through the explicit six-place fixed-decimal view. Every other complete report byte is preserved. No general normalization or universal determinism claim.', reports=result)
(out / 'baseline-delta-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
print(json.dumps({label: {'version_fields': row['version_fields'], 'diagnostic_changes': len(row['diagnostic_changes']), 'all_other_bytes_unchanged': row['all_other_bytes_unchanged']} for label,row in result.items()}, indent=2))
