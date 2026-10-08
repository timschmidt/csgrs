from pathlib import Path
import shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-side-ray-trace-2026-09-23')
root = Path('/tmp/hypercurve-circle-ray-predicates-2026-09-23')
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

path = root / 'hypercurve/src/bezier_offset.rs'
text = path.read_text()
start = text.index('    fn retained_forward_ray_winding_delta(')
end = text.index('\n    fn forward_ray_winding_delta_with_origin_contact(', start)
body = text[start:end]
needle = '        let inside_circle = residual == RealSign::Negative;'
assert body.count(needle) == 1
body = body.replace(needle, '''        eprintln!("CIRCLE_RAY origin=({:?},{:?}) direction=({:?},{:?}) residual={residual:?} side={line_side:?} clockwise={clockwise} start_y={start_y:?} end_y={end_y:?}", origin.x().to_f64_lossy(), origin.y().to_f64_lossy(), direction_x.to_f64_lossy(), direction_y.to_f64_lossy());
        for (name, point) in [("start", &start), ("end", &end), ("center", &center)] {
            if let Classification::Decided(bounds) = algebraic_chord_endpoint_bounds_refined(point, 4, policy) {
                eprintln!("CIRCLE_RAY {name} bounds=({:?},{:?},{:?},{:?})", bounds.min_x().to_f64_lossy(), bounds.max_x().to_f64_lossy(), bounds.min_y().to_f64_lossy(), bounds.max_y().to_f64_lossy());
            }
        }
''' + needle)
body = body.replace('        let (lower, upper, delta) = match decision {', '        eprintln!("CIRCLE_RAY decision={decision:?}");\n        let (lower, upper, delta) = match decision {', 1)
body = body.replace('        if lower_x != std::cmp::Ordering::Less {', '        eprintln!("CIRCLE_RAY lower_x={lower_x:?}");\n        if lower_x != std::cmp::Ordering::Less {', 1)
body = body.replace('        Ok(Classification::Decided(\n            if upper_x', '        eprintln!("CIRCLE_RAY upper_x={upper_x:?}");\n        Ok(Classification::Decided(\n            if upper_x', 1)
text = text[:start] + body + text[end:]
path.write_text(text)

path = root / 'hypercurve/examples/nonph_contact_isolation_probe_20260923.rs'
text = path.read_text()
needle = '            eprintln!("selected candidate; begin point classification");'
assert text.count(needle) == 1
text = text.replace(needle, needle + '''
            for shift in [-1_i64, 1] {
                let query = Point2::new(
                    (Real::from(279) / Real::from(260)).unwrap() + (Real::from(shift) / Real::from(10400)).unwrap(),
                    (Real::from(5501) / Real::from(5200)).unwrap(),
                );
                eprintln!("LOCAL_QUERY shift={shift} result={:?}", filleted.classify_point(&query, &policy));
            }
            return;
''')
path.write_text(text)

runner = (audit / 'run-side-ray-trace-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('side-ray-trace-20260923-', 'circle-ray-predicates-20260923-')
(audit / 'run-circle-ray-predicates-20260923.py').write_text(runner)
print('Prepared public nearby-point queries and circle predicate diagnostics.')
