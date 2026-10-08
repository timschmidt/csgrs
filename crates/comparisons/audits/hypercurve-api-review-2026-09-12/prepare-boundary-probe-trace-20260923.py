from pathlib import Path
import json
import shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-shared-source-box-2026-09-23')
root = Path('/tmp/hypercurve-boundary-probe-trace-2026-09-23')
root.mkdir()
for path in source.iterdir():
    if path.is_dir() and path.name not in ['hypercurve', 'hypersolve', 'dumps']:
        (root / path.name).symlink_to(path.resolve(), target_is_directory=True)
for name in ['hypercurve', 'hypersolve']:
    shutil.copytree(source / name, root / name)
(root / 'dumps').mkdir()
for name in ['hypercurve/src/bezier_offset.rs', 'hypersolve/src/algebraic_fiber.rs', 'hypersolve/src/root_sign.rs']:
    path = root / name
    path.write_text(path.read_text().replace(str(source), str(root)))

path = root / 'hypercurve/src/curve_region_boolean.rs'
text = path.read_text()
start = text.index('    fn regularized_algebraic_cusp_fragment_decision_by_probe(')
end = text.index('\n    fn ', start + 10)
body = text[start:end]
original_line = text[:start].count('\n') + 1
lines = body.splitlines(True)
mapping = {}
for index, line in enumerate(lines):
    if 'last_reason =' in line and 'let mut' not in line:
        label = original_line + index
        mapping[label] = ''.join(lines[max(0, index - 2):index + 3])
        indent = line[:len(line) - len(line.lstrip())]
        lines[index] = indent + f'eprintln!("BOUNDARY_PROBE reject source_line={label}");\n' + line
body = ''.join(lines)
body = body.replace('for outside in outside_points {', 'for (attempt, outside) in outside_points.into_iter().enumerate() {\n            eprintln!("BOUNDARY_PROBE begin target={carrier_index} attempt={attempt}");')
body = body.replace('            if !evidence.overlaps().is_empty() {', '            eprintln!("BOUNDARY_PROBE evidence contacts={} overlaps={} blockers={}", evidence.contacts().len(), evidence.overlaps().len(), evidence.blockers().len());\n            if !evidence.overlaps().is_empty() {')
body = body.replace('            for contact in evidence.contacts() {', '            for contact in evidence.contacts() {\n                eprintln!("BOUNDARY_PROBE contact carrier={} tangent={:?} transverse={}", contact.second().carrier_index(), contact.evidence.tangent_cross_is_positive(), contact.is_certified_transverse());')
body = body.replace('                if order == Ordering::Greater {', '                eprintln!("BOUNDARY_PROBE contact order={order:?}");\n                if order == Ordering::Greater {')
text = text[:start] + body + text[end:]
needle = '                    let intersections = match certified_chord_endpoint_incidence {'
assert text.count(needle) == 1
text = text.replace(needle, '                    eprintln!("CIRCLE_CHORD incidence={certified_chord_endpoint_incidence:?} circle={cusp_index} chord={chord_index}");\n' + needle)
path.write_text(text)
(audit / 'boundary-probe-trace-20260923-branches.json').write_text(json.dumps(mapping, indent=2) + '\n')

path = root / 'hypercurve/src/bezier_offset.rs'
text = path.read_text()
start = text.index('    fn recursive_projective_chord_intersections(')
insertion = text.index('        let parent_field = authority.field.clone();', start)
text = text[:insertion] + '        eprintln!("RECURSIVE_CHORD finite={clip_to_finite_chord} incidence={}", certified_endpoint_incidence.is_some());\n' + text[insertion:]
path.write_text(text)

runner = (audit / 'run-shared-source-box-trial-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('nonph-shared-source-box-20260923-', 'boundary-probe-trace-20260923-')
(audit / 'run-boundary-probe-trace-20260923.py').write_text(runner)
print('Prepared probe rejection diagnostics; no production source changed.')
