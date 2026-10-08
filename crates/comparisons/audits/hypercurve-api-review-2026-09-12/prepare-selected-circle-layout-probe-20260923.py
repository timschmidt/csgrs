from pathlib import Path
import hashlib, json, shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-overlap-callers-2026-09-23')
root = Path('/tmp/hypercurve-selected-circle-layout-2026-09-23')
prior = json.loads((audit / 'overlap-callers-20260923-focused1-terminal.json').read_text())
assert prior['all_processes_reaped'] and prior['all_sources_unchanged']
assert not root.exists()
root.mkdir()
shutil.copytree(source / 'hypercurve', root / 'hypercurve')
for path in source.iterdir():
    if path.name != 'hypercurve' and path.is_dir():
        (root / path.name).symlink_to(path.resolve(), target_is_directory=True)

path = root / 'hypercurve/src/bezier_region.rs'
text = path.read_text()
start = text.index('    fn nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel()')
end = text.index('\n    #[test]', start)
test = text[start:end]
needle = '                    for_each_corner_region(&outcome.value, |filleted| {\n'
assert test.count(needle) == 1
diagnostic = '''                        eprintln!("LAYOUT policy={policy:?} reversed={reversed} mode={mode:?} normalized={} loops={}", filleted.has_regularized_filled_left_topology(&policy), filleted.boundary_loops().len());
                        for (loop_index, boundary) in filleted.boundary_loops().iter().enumerate() {
                            for (index, fragment) in boundary.fragments().iter().enumerate() {
                                let kind = match fragment {
                                    BezierSplitFragment2::Materialized { .. } => "materialized",
                                    BezierSplitFragment2::RetainedBezier { .. } => "retained-bezier",
                                    BezierSplitFragment2::AnalyticParallel(_) => "parallel",
                                    BezierSplitFragment2::AlgebraicChord(_) => "chord",
                                    BezierSplitFragment2::AlgebraicCuspSemicircle(_) => "selected-circle",
                                    BezierSplitFragment2::SelectedFiber(_) => "selected-fiber",
                                };
                                eprintln!("LAYOUT loop={loop_index} index={index} kind={kind}");
                                if let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = fragment {
                                    let radial = circle.semicircle().radial_distance();
                                    eprintln!("LAYOUT fillet_radius={:?} source_radius={:?}",
                                        crate::classify::is_zero(&(radial * radial - &radius * &radius), &CurveContext::STRICT),
                                        crate::classify::is_zero(&(radial * radial - Real::one()), &CurveContext::STRICT));
                                }
                            }
                        }
'''
test = test.replace(needle, needle + diagnostic)
path.write_text(text[:start]+test+text[end:])
bindings = json.loads((audit/'overlap-callers-20260923-focused1-sources.json').read_text())
bindings['hypercurve/src/bezier_region.rs'] = hashlib.sha256(path.read_bytes()).hexdigest()
for name, sha in bindings.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
(audit/'selected-circle-layout-20260923-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner = (audit/'run-overlap-callers-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('overlap-callers-20260923', 'selected-circle-layout-20260923')
start = runner.index('selected=[')
end = runner.index('\n]\n', start)+3
runner = runner[:start]+'''selected=[
    ('nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel',90),
]
'''+runner[end:]
(audit/'run-selected-circle-layout-probe-20260923.py').write_text(runner)
print('Prepared diagnostic-only immutable sources:',root)
