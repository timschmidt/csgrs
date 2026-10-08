from pathlib import Path
import shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-boundary-decision-trace-2026-09-23')
root = Path('/tmp/hypercurve-side-ray-trace-2026-09-23')
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
needle = '            let source_parameter = parameter\n                .map(|parameter| CurveParameter2::from(BezierParameter2::Exact(parameter)));'
assert text.count(needle) == 1
text = text.replace(needle, '            if carrier_index == 3 { eprintln!("SIDE_RAY representative parameter={:?} point=({:?},{:?}) tangent=({:?},{:?})", parameter.as_ref().and_then(Real::to_f64_lossy), representative.x().exact_rational_ref(), representative.y().exact_rational_ref(), tangent_x.to_f64_lossy(), tangent_y.to_f64_lossy()); }\n' + needle)
path.write_text(text)

path = root / 'hypercurve/src/bezier_region.rs'
text = path.read_text()
start = text.index('fn classify_point_with_retained_ray_skipping_origin(')
end = text.index('\nfn retained_parameters_equal(', start)
body = text[start:end]
body = body.replace('    let direction_x = &ray.direction_x;', '    let trace_source = skipped_origin.is_some_and(|origin| origin.fragment_index == Some(3));\n    if trace_source { eprintln!("SIDE_RAY begin direction=({:?},{:?})", ray.direction_x.to_f64_lossy(), ray.direction_y.to_f64_lossy()); }\n    let direction_x = &ray.direction_x;', 1)
body = body.replace('            && !is_source_fragment\n        {\n            continue;', '            && !is_source_fragment\n        {\n            if trace_source { eprintln!("SIDE_RAY fragment={fragment_index} pruned bounds"); }\n            continue;', 1)
body = body.replace('        {\n            continue;\n        }\n        let control_hull_order', '        {\n            if trace_source { eprintln!("SIDE_RAY fragment={fragment_index} pruned control hull"); }\n            continue;\n        }\n        let control_hull_order', 1)
body = body.replace('                Classification::Decided(delta) => winding += delta,', '                Classification::Decided(delta) => { if trace_source { eprintln!("SIDE_RAY fragment={fragment_index} procedural-delta={delta}"); } winding += delta },')
needle = '        let relation = match relation {\n            Classification::Decided(relation) => relation,'
assert body.count(needle) == 1
body = body.replace(needle, '        if trace_source { match &relation { Classification::Decided(BezierLineContactRelation::Contacts { contacts }) => eprintln!("SIDE_RAY fragment={fragment_index} contacts={} unit={unit_covers_range}", contacts.len()), Classification::Decided(_) => eprintln!("SIDE_RAY fragment={fragment_index} no finite contacts unit={unit_covers_range}"), Classification::Uncertain(reason) => eprintln!("SIDE_RAY fragment={fragment_index} uncertain={reason:?}") } }\n' + needle)
needle = '                    match retained {\n                        Classification::Decided(true) => {}'
assert body.count(needle) == 1
body = body.replace(needle, '                    if trace_source { eprintln!("SIDE_RAY fragment={fragment_index} contact scalar={:?} kind={:?} crossing={:?} ahead={ahead:?} retained={retained:?}", contact.parameter().scalar().and_then(Real::to_f64_lossy), contact.kind(), contact.crossing_direction()); }\n' + needle)
body = body.replace('                                source_origin_contact_was_skipped = true;\n                                continue;', '                                if trace_source { eprintln!("SIDE_RAY fragment={fragment_index} skipped origin"); }\n                                source_origin_contact_was_skipped = true;\n                                continue;')
body = body.replace('    Ok(Classification::Decided(RetainedRayWinding::Winding(', '    if trace_source { eprintln!("SIDE_RAY complete winding={winding}"); }\n    Ok(Classification::Decided(RetainedRayWinding::Winding(', 1)
text = text[:start] + body + text[end:]
path.write_text(text)

runner = (audit / 'run-boundary-decision-trace-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('boundary-decision-trace-20260923-', 'side-ray-trace-20260923-')
(audit / 'run-side-ray-trace-20260923.py').write_text(runner)
print('Prepared targeted rational-fragment side-ray diagnostics.')
