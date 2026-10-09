"""Review exact decimal-message deltas, then compare every complete report byte.

Usage: python3 audit_report_deltas.py REPORT_OUTPUT
Only the four version fields and enumerated numeric displays may change.
Message replacements are positional: the old symbolic irrational display names
several different gaps and cannot be replaced globally with one decimal.
"""
from collections import Counter
from decimal import Decimal, localcontext
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
manifest = json.loads((out / 'manifest.json').read_text())
message_pattern = re.compile(rb'"message":("(?:[^"\\]|\\.)*"|null)')

# Authored values from the retained seven-record fixture.ipc, in source order.
points = [('0', '0'), ('0.08', '0'), ('0.3', '0'), ('0.38', '0'),
          ('0', '0.1'), ('0.08', '0.1'), ('10', '10')]
diameters = ['0.05', '0.05', '0.02', None, '0.05', '0.05', '0.05']
ipc = (record / 'fixture.ipc').read_text().splitlines()
assert ipc[0] == 'P  UNITS CUST 1' and len(ipc) == 8
for line, point, diameter in zip(ipc[1:], points, diameters):
    match = re.search(r'X(\d+)Y(\d+)(?:D(\d+))?', line)
    assert [Decimal(v) / 10000 for v in match.group(1, 2)] == list(map(Decimal, point))
    assert (Decimal(match[3]) / 10000 if match[3] else None) == (Decimal(diameter) if diameter else None)
pairs = [(0, 1), (0, 2), (0, 4), (0, 5), (1, 2),
         (1, 4), (1, 5), (2, 4), (2, 5), (4, 5)]
old_gaps = ['3/100', '53/200', '1/20', '1 x Irrational', '37/200',
            '1 x Irrational', '1/20', '1 x Irrational', '1 x Irrational', '3/100']
fixed_gaps = ['0.030000', '0.265000', '0.050000', '0.078062', '0.185000',
              '0.078062', '0.050000', '0.281228', '0.206661', '0.030000']
with localcontext() as context:
    context.prec = 60
    for (i, j), fixed in zip(pairs, fixed_gaps):
        delta = [Decimal(points[i][k]) - Decimal(points[j][k]) for k in [0, 1]]
        gap = sum(d*d for d in delta).sqrt() - (Decimal(diameters[i]) + Decimal(diameters[j])) / 2
        assert format(gap, '.6f') == fixed

result = {}
for label, thermal_count, coverage_count in [('cparti-fpga', 2389, 164), ('hvp109a', 403, 13)]:
    before = gzip.decompress((record / f'baseline-reports/{label}.json.gz').read_bytes())
    trials = [gzip.decompress((out / f'trial-{trial}-{label}.json.gz').read_bytes()) for trial in [1, 2]]
    assert trials[0] == trials[1]
    candidate = trials[0]
    assert before.count(b'"check_version":15') == 4
    versioned = before.replace(b'"check_version":15', b'"check_version":16')
    baseline_json, candidate_json = json.loads(before), json.loads(candidate)
    violations = baseline_json['violations']
    assert Counter(v['check'] for v in violations) == {
        'thermal-copper-area-readiness': thermal_count,
        'testpoint-coverage-readiness': coverage_count,
        'testpoint-accessibility-readiness': 29,
    }
    assert len(violations) == len(candidate_json['violations'])
    replacements, changes = {}, []
    access_index = 0
    for index, (left, right) in enumerate(zip(violations, candidate_json['violations'])):
        old = new = left['message']
        if left['check'] == 'thermal-copper-area-readiness':
            fragment = ' copper zone within 1 1/5; review copper area '
            assert old.count(fragment) == 1
            new = old.replace(fragment, ' copper zone within 1.200000; review copper area ')
        elif left['check'] == 'testpoint-accessibility-readiness':
            slot = access_index
            access_index += 1
            if slot in [0, 2, 6, 13, 15, 17]:
                diameter, fixed = ('1/50', '0.020000') if slot == 6 else ('1/20', '0.050000')
                assert old == f'IPC-D-356 testpoint diameter {diameter} is below minimum probe diameter 3/20'
                new = f'IPC-D-356 testpoint diameter {fixed} is below minimum probe diameter 0.150000'
            elif slot in [1, 3, 7, 11, 14, 16, 18]:
                assert old == 'IPC-D-356 testpoint probe is below fixture edge clearance 1/2 at the board edge'
                new = 'IPC-D-356 testpoint probe is below fixture edge clearance 0.500000 at the board edge'
            elif slot >= 19:
                pair_index = slot - 19
                assert old == f'IPC-D-356 testpoint spacing {old_gaps[pair_index]} is below fixture probe spacing 3/5'
                new = f'IPC-D-356 testpoint spacing {fixed_gaps[pair_index]} is below fixture probe spacing 0.600000'
                assert left['locations'] == [[float(v) for v in points[i]] for i in pairs[pair_index]]
            else:
                assert slot in [4, 5, 8, 9, 10, 12]
        assert right['message'] == new, (label, index, old, right['message'])
        assert {k:v for k,v in left.items() if k != 'message'} == {k:v for k,v in right.items() if k != 'message'}
        if new != old:
            replacements[index] = new
            changes.append(dict(path=['violations', index, 'message'], before=old, after=new))
    assert access_index == 29 and len(changes) == thermal_count + 23
    matches = list(message_pattern.finditer(versioned))
    assert len(matches) == len(violations)
    chunks, start = [], 0
    for index, match in enumerate(matches):
        assert json.loads(match[1]) == violations[index]['message']
        if index in replacements:
            chunks.extend([versioned[start:match.start(1)], json.dumps(replacements[index], ensure_ascii=False).encode()])
            start = match.end(1)
    expected = b''.join(chunks) + versioned[start:]
    assert candidate == expected, f'Unreviewed complete-report delta: {label}'
    row = manifest['reports'][label]
    assert len(candidate) == row['bytes'] and hashlib.sha256(candidate).hexdigest() == row['sha256']
    assert not row['baseline_only_version_changed']
    result[label] = dict(version_fields=4, diagnostic_changes=changes,
        diagnostic_counts={'thermal-copper-area-readiness': thermal_count, 'testpoint-accessibility-readiness': 23},
        candidate_bytes=len(candidate), candidate_sha256=hashlib.sha256(candidate).hexdigest(),
        all_other_bytes_unchanged=True, candidate_repetitions_identical=True)
audit = dict(status='passed', scope='Four version fields per board; 2,412 CPArti and 426 HVP109A diagnostic messages use six-place fixed decimal instead of rational or symbolic displays. Each board has 23 accessibility message changes, including ten independently checked gaps from the retained synthetic IPC sidecar; all other changes display the unchanged thermal radius 1.2. Every other complete report byte is preserved. No general normalization or universal determinism claim.', reports=result)
(out / 'baseline-delta-audit.json').write_text(json.dumps(audit, indent=2) + '\n')
print(json.dumps({label: {'version_fields': row['version_fields'], 'diagnostic_changes': len(row['diagnostic_changes']), 'all_other_bytes_unchanged': row['all_other_bytes_unchanged']} for label,row in result.items()}, indent=2))
