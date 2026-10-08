from pathlib import Path
import shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-boundary-probe-trace-2026-09-23')
root = Path('/tmp/hypercurve-boundary-decision-trace-2026-09-23')
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
needle = '    fn blocked(&self, carrier_index: usize, reason: UncertaintyReason) -> ExactCurveError {\n'
assert text.count(needle) == 1
text = text.replace(needle, '    #[track_caller]\n' + needle + '        eprintln!("REGION_BLOCKED carrier={carrier_index} reason={reason:?} caller={}", std::panic::Location::caller());\n')
start = text.index('    fn build_regularized_region(')
end = text.index('\n    /// Resolves result-side actions', start)
body = text[start:end]
body = body.replace('        let simple_loop_filled_side', '        eprintln!("REGULARIZED split-counts={:?}", topology.split_fragments.iter().map(Vec::len).collect::<Vec<_>>());\n        let simple_loop_filled_side', 1)
body = body.replace('        let mut arrangement_fragments', '        eprintln!("REGULARIZED actions={:?} successors={:?}", fragment_selection.actions, fragment_selection.successor_edge_ids);\n        let mut arrangement_fragments', 1)
body = body.replace('        let traversal = match', '        eprintln!("REGULARIZED graph-edges={} certified-successors={:?}", arrangement_source_edge_ids.len(), certified_successors);\n        let traversal = match', 1)
body = body.replace('        let mut region = match', '        eprintln!("REGULARIZED traversal-chains={}", traversal.chains().len());\n        let mut region = match', 1)
text = text[:start] + body + text[end:]
text = text.replace('                        actions[carrier_index][split_index] = Some(decision.action);', '                        eprintln!("REGULARIZED geometric carrier={carrier_index} split={split_index} decision={decision:?}");\n                        actions[carrier_index][split_index] = Some(decision.action);')
path.write_text(text)

runner = (audit / 'run-boundary-probe-trace-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('boundary-probe-trace-20260923-', 'boundary-decision-trace-20260923-')
(audit / 'run-boundary-decision-trace-20260923.py').write_text(runner)
print('Prepared winding and graph diagnostics.')
