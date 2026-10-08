from pathlib import Path
import shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-circle-ray-predicates-2026-09-23')
root = Path('/tmp/hypercurve-fillet-companion-chart-2026-09-23')
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

path = root / 'hypercurve/src/curve_corner_chain.rs'
text = path.read_text()
needle = '            let canonical_anchor_tangent = if matches!('
assert text.count(needle) == 1
text = text.replace(needle, '''            // Canonicalization transports the cut parameter into its replacement
            // chart. Use that same chart for the companion point and tangent;
            // pairing the new parameter with the authored support changes the
            // point while leaving an apparently certified circle contact.
            let replacement_companion = other_cut.replacement.as_ref().map(|replacement| {
                match replacement {
                    CornerReplacement2::Curve(curve) => BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: curve.clone(),
                    },
                    CornerReplacement2::AnalyticParallel { fragment, .. } => {
                        BezierSplitFragment2::AnalyticParallel(fragment.clone())
                    }
                    CornerReplacement2::SelectedFiber { fragment, .. } => {
                        BezierSplitFragment2::SelectedFiber(fragment.as_ref().clone())
                    }
                }
            });
            let other_fragment = replacement_companion.as_ref().unwrap_or(other_fragment);
''' + needle)
start = text.index('            if !matches!(\n                other_fragment,')
end = text.index('            let fillet_clockwise =', start)
text = text[:start] + text[end:]
path.write_text(text)

runner = (audit / 'run-circle-ray-predicates-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace('circle-ray-predicates-20260923-', 'fillet-companion-chart-20260923-')
(audit / 'run-fillet-companion-chart-20260923.py').write_text(runner)
print('Prepared companion chart binding; baseline diagnostics remain archived.')
