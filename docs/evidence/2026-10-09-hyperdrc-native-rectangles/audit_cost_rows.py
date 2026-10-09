"""Audit all previous workloads and the five exact, independently replayed deltas.

Usage: python3 audit_cost_rows.py COST_OUTPUT
The auxiliary probes serialize complete findings against both retained libraries.
No tolerance, source-scale change or witness normalization is accepted.
"""
import hashlib
import json
from pathlib import Path
import re
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
prior = record.parent / '2026-10-09-hyperdrc-fixture-access/cost'
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
old, new = load(prior / 'rows.json'), load(out / 'rows.json')
assert len(old) == 193 and len(new) == 205
renames = load(record / 'cost/workload-changes.json')['renames']
expected_indices = [162, 163, 164, 165, 191]
deltas = []
for index, (before, after) in enumerate(zip(old, new)):
    assert after['name'] == renames.get(before['name'], before['name'])
    assert before['iterations'] == after['iterations']
    if before != after:
        deltas.append(dict(index=index, before=before, after=after))
assert [d['index'] for d in deltas] == expected_indices
assert deltas == load(out / 'previous-row-deltas.json')
assert deltas == load(out / 'manifest.json')['previous_row_deltas']

def witness(raw):
    value = 0xcbf29ce484222325
    for byte in raw:
        value = ((value ^ byte) * 0x100000001b3) & ((1 << 64) - 1)
    return f'{len(json.loads(raw))}:{len(raw)}:{value}'

auxiliary = load(out / 'auxiliary-probes/manifest.json')
findings = []
for delta in deltas:
    label = delta['after']['name']
    paths = [out / f'auxiliary-probes/{side}/{label}.json' for side in ['before', 'after']]
    before, after = [p.read_bytes() for p in paths]
    assert witness(before) == delta['before']['witness']
    assert witness(after) == delta['after']['witness']
    old_finding, new_finding = json.loads(before), json.loads(after)
    assert len(old_finding) == 1
    if label.endswith('optional_minimum'):
        assert before.count(b'; certified pad') == 1
        assert before.replace(b'; certified pad', b'; nearest pad') == after
        assert len(new_finding) == 1
        reason = 'All required shapes now certify, so the unchanged zero clearance is the nearest spacing rather than only a certified violating pair. Only certified -> nearest changes in the complete finding bytes.'
    else:
        assert after == b'[]' and old_finding[0]['check'] == 'geometry-uncertainty'
        assert old_finding[0]['message'].endswith('exact polygon contour construction failed: curve string has insufficient vertices')
        reason = 'The unchanged rectangle at center 1e20, width 16384 now has native area 268435456 and certified bounds. It is remote from the required probe/pad geometry, so the former construction-uncertainty finding disappears.'
    findings.append(dict(index=delta['index'], name=label, reason=reason,
                         before=old_finding, after=new_finding,
                         raw_artifacts={str(p.relative_to(out)): info(p) for p in paths}))
for label in ['underflow-after', 'row-findings-before', 'row-findings-after']:
    row = auxiliary[label]
    assert row['build_exit_code'] == row['exit_code'] == 0
    source = record / ('underflow-correction/probe.rs' if label == 'underflow-after' else 'cost/row_findings.rs')
    assert row['source'] == info(source)
    for path, expected in {**row['library'], **row['dependencies']}.items():
        if Path(path).exists():
            assert info(Path(path)) == expected

# Exact reconstruction restricts measured source edits to assertions/labels and
# the new rectangle module. Every old input, query and iteration is preserved.
source_proof = {}
for path in sorted(prior.glob('*.rs')):
    expected = path.read_text()
    if path.name == 'operation_probe.rs':
        expected = expected.replace('mod fixture_access_probe;\n', 'mod fixture_access_probe;\nmod rectangle_probe;\n')
        expected = expected.replace('    fixture_access_probe::run();\n', '    fixture_access_probe::run();\n    rectangle_probe::run();\n')
    elif path.name == 'dense_cluster_probe.rs':
        expected = expected.replace('failed_rectangle()', 'large_rectangle()')
        expected = expected.replace("    // Retain the staged regression's actual finite-source construction failure.\n    // Half-width rounds away at this coordinate; checked native queries fail.", '    // Retain the original hard source at the same scale; native corners now survive.')
        expected = expected.replace('assert!(region.try_bounding_box().is_err());', 'assert!(region.try_bounding_box().unwrap().is_some());\n    assert_eq!(region.try_area().unwrap(), Scalar::from(268_435_456));')
        expected = expected.replace('_required_shape_failure', '_required_large_shape')
        expected = expected.replace('measure(&label, 10, 1, "geometry-uncertainty", || rule.run(&board, &one, &one));', 'measure(&label, 10, 0, "unused", || rule.run(&board, &one, &one));')
        expected = expected.rstrip('\n') + '\n'
    elif path.name == 'fixture_access_probe.rs':
        expected = expected.replace('let failed = rectangle(1e20, 16_384.0); assert!(failed.try_bounding_box().is_err());', 'let large = rectangle(1e20, 16_384.0); assert!(large.try_bounding_box().unwrap().is_some());')
        expected = expected.replace('f.region = failed;', 'f.region = large;')
        expected = expected.replace('measure("fixture_copper_failed_shape", 100, 1, "geometry-uncertainty",', 'measure("fixture_copper_large_shape", 100, 0, "unused",')
    current = record / 'cost' / path.name
    assert current.read_text() == expected, path.name
    source_proof[path.name] = dict(previous=info(path), current=info(current), exact_reconstruction=True)
operation = (record / 'cost/operation_probe.rs').read_text()
driver = (record / 'cost/row_findings.rs').read_text()
for name in ['measure_value', 'findings_witness']:
    function = re.search(r'^fn ' + name + r'[\s\S]*?^}', operation, re.M)[0]
    assert function in driver

result = dict(status='passed', previous_rows=193, current_rows=205, identical_previous_rows=188,
              previous_inputs_and_iterations_preserved=True, changed_rows=deltas,
              complete_finding_deltas=findings, measured_source_reconstruction=source_proof,
              underflow_public_probe_passed=True,
              scope='The same five original workloads are replayed through both verified libraries. Every complete finding matches its measured FNV/length witness. Two messages change only certified -> nearest; three former construction failures become empty findings. The original large rectangle is neither moved nor shrunk. This is a correctness delta, not a matched speedup claim.')
(out / 'row-delta-audit.json').write_text(json.dumps(result, indent=2) + '\n')
print('193 prior inputs/iterations preserved; 188 rows identical; five exact finding deltas verified.')
