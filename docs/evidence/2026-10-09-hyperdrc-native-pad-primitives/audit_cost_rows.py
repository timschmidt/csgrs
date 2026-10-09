"""Require byte-stable prior witnesses and exact preservation of their sources.

Usage: python3 audit_cost_rows.py COST_OUTPUT
"""
import hashlib
import json
from pathlib import Path
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
prior = record.parent/'2026-10-09-hyperdrc-native-rectangles/cost'
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
old,new = load(prior/'rows.json'),load(out/'rows.json')
assert len(old) == 205 and len(new) == 231
assert old == new[:205]
assert load(out/'previous-row-deltas.json') == []
manifest = load(out/'manifest.json')
assert manifest['previous_rows'] == 205 and manifest['previous_row_deltas'] == []
assert manifest['previous_inputs_and_iterations_preserved']
sources = {}
for path in sorted(prior.glob('*_probe.rs')):
    expected = path.read_text()
    if path.name == 'operation_probe.rs':
        expected = expected.replace('mod rectangle_probe;\n','mod rectangle_probe;\nmod pad_probe;\n')
        expected = expected.replace('    rectangle_probe::run();\n','    rectangle_probe::run();\n    pad_probe::run();\n')
    current = record/'cost'/path.name
    assert current.read_text() == expected,path
    sources[path.name] = dict(previous=info(path),current=info(current),exact_reconstruction=True)
added = new[205:]
assert len(added) == 26 and all(r['name'].startswith('operation_pad_') for r in added)
assert [r['iterations'] for r in added] == [100]*20 + [1000]*6
assert len({r['name'] for r in new}) == 231
result = dict(status='passed',previous_rows=205,current_rows=231,
              all_previous_rows_and_iterations_identical=True,previous_sources_preserved=sources,
              added_rows=added,scope='All 205 previous complete correctness witnesses and iteration counts remain identical. Only the new pad module and call are added to the prior probe. Twenty new construction rows and six warm measurements preserve independent exact area/bounds checks and input/report-coordinate fingerprints.')
(out/'row-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print('All 205 previous witnesses, inputs and iterations preserved; 26 pad rows added.')
