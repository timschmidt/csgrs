"""Freeze exact native track ingestion, explicit migration, and acceptance tests."""
import gzip, hashlib, json, re, shutil, subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
BASE = 'a843bf79dfe329fb51ababaaf803752bb74a0f64'
SOURCE = W / 'candidate-source-v1'
O = W / 'candidate-preparation-v1'
ARCHIVE = ROOT / 'docs/evidence/2026-10-16-native-change-disposition/pending-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new, 1)

assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
reproduction = read(W / 'reproduction-v1/result.json')
assert reproduction['complete']
assert reproduction['counts']['archived-frontier']['counts'] == [['5', '21', '0']]
assert reproduction['counts']['current-source-failures']['counts'] == [['39', '0', '0']]
with gzip.open(W / 'reproduction-v1/source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
assert len(inputs) == 2835 and all(sha(ROOT / p) == sha(W / 'baseline-source-v1' / p) == h for p, h in inputs.items())
SOURCE.mkdir(exist_ok=False)
O.mkdir(exist_ok=False)
for name, h in inputs.items():
    dest = SOURCE / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, dest)
    assert sha(dest) == h
name = 'crates/hyperdrc/src/kicad/tracks.rs'
text = (SOURCE / name).read_text()
text = replace_once(text, '//! Checked source fields at the legacy finite straight-track boundary.', '//! Checked exact source fields and native round-capped straight tracks.')
text = replace_once(text, 'use hyperlimit::{PredicatePolicy, compare_reals};', '''use hypercurve::{
    CurveFamily2, CurveOperation2, CurvePath2, CurveRegion2, ExactCurveError, LineSeg2,
    OffsetCap, OffsetCornerStyle2, Point2,
};
use hyperlimit::{PredicatePolicy, compare_reals};''')
text = replace_once(text, 'use crate::geometry::{SourceScalar, SourceUnit, line_polygon, polygons_to_profile};', 'use crate::geometry::{SourceScalar, SourceUnit};')
text = replace_once(text, '    /// Valid source values cannot form nonempty geometry in the finite adapter.', '    /// Valid source values did not produce the required nonempty track geometry.')
text = replace_once(text, '    /// Native construction did not produce usable geometry.\n    Geometry(PcbGeometryUncertainty),', '''    /// The native curve constructor retained an invalid-input or exact-blocker diagnosis.
    Construction(ExactCurveError),
    /// Native construction or validation did not produce usable geometry.
    Geometry(PcbGeometryUncertainty),''')
text = replace_once(text, 'write!(f, "unsupported finite track geometry: {detail}")', 'write!(f, "unsupported track geometry: {detail}")')
text = replace_once(text, '            TrackLoadFailure::Geometry(error) => write!(f, "{error}"),', '''            TrackLoadFailure::Construction(error) => write!(f, "{error}"),
            TrackLoadFailure::Geometry(error) => write!(f, "{error}"),''')
text = replace_once(text, '            TrackLoadFailure::Geometry(error) => Some(error),', '''            TrackLoadFailure::Construction(error) => Some(error),
            TrackLoadFailure::Geometry(error) => Some(error),''')
start = text.index('fn finite_point(')
end = text.index('\nfn parse_segment(', start)
text = text[:start] + text[end + 1:]
start = text.index('        let start_finite = finite_point(&start)?;')
end = text.index('\n    };\n    let empty = region.try_is_empty();', start)
text = text[:start] + '''        // The format defines a round-capped stroke of the exact source line.
        // STRICT suspends any ambient approximate or preview policy; retain
        // its receipt and structured construction error before publishing.
        let native = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
            let line = LineSeg2::try_new(
                Point2::new(start[0].value.clone(), start[1].value.clone()),
                Point2::new(end[0].value.clone(), end[1].value.clone()),
            )
            .map_err(|cause| ExactCurveError::Invalid {
                operation: CurveOperation2::Construction,
                family: Some(CurveFamily2::Line),
                cause,
            })?;
            let path = CurvePath2::try_new(vec![line.into()])?;
            CurveRegion2::stroke_path(
                &path,
                crate::scalar::half(&width.value),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
            )
        });
        check_budget()?;
        let native = native.certified().ok_or_else(|| {
            TrackLoadFailure::Geometry(crate::operations::failure(
                "linear-stroke-construction",
                None,
                "missing STRICT receipt",
            ))
        })?;
        PcbRegion::new(native.map_err(TrackLoadFailure::Construction)?, metadata)''' + text[end:]
(SOURCE / name).write_text(text)

name = 'crates/hyperdrc/tests/kicad_track_failures.rs'
text = (SOURCE / name).read_text()
start = text.index('#[test]\nfn positive_source_that_the_finite_adapter_cannot_construct_is_an_error()')
end = text.index('\n#[test]\nfn second_segment_failure', start)
text = text[:start] + '''#[test]
fn exact_source_ranges_remain_certified_after_native_handoff() {
    // These are the original finite-adapter failure fixtures, now admitted
    // exactly. Endpoint components are nondecreasing in every listed case.
    for (fields, start, end, length, width) in [
        ("(start 0 0) (end 2 0) (width 1e-400)", ["0", "0"], ["2", "0"], "2", "1e-400"),
        ("(start 0 0) (end 1e-200 0) (width 2)", ["0", "0"], ["1e-200", "0"], "1e-200", "2"),
        ("(start 1e400 0) (end 1e400 2) (width 2)", ["1e400", "0"], ["1e400", "2"], "2", "2"),
        ("(start -1e308 0) (end 1e308 0) (width 2)", ["-1e308", "0"], ["1e308", "0"], "2e308", "2"),
        ("(start 0 0) (end 2 0) (width 1e400)", ["0", "0"], ["2", "0"], "2", "1e400"),
        ("(start 100000000000000000000 0) (end 100000000000000000001 0) (width 2)", ["100000000000000000000", "0"], ["100000000000000000001", "0"], "1", "2"),
    ] {
        let start = start.map(|v| v.parse::<Scalar>().unwrap());
        let end = end.map(|v| v.parse::<Scalar>().unwrap());
        let width = width.parse::<Scalar>().unwrap();
        let radius = (&width / Scalar::from(2)).unwrap();
        let expected_area = length.parse::<Scalar>().unwrap() * &width
            + Scalar::pi() * &radius * &radius;
        let midpoint = [
            ((&start[0] + &end[0]) / Scalar::from(2)).unwrap(),
            ((&start[1] + &end[1]) / Scalar::from(2)).unwrap(),
        ];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board(fields).unwrap();
                assert_eq!(model.copper.len(), 1);
                assert_eq!(model.copper[0].location, midpoint);
                let profile = model.all_copper().unwrap();
                assert_eq!(
                    profile.classify_xy(midpoint[0].clone(), midpoint[1].clone()),
                    Ok(RegionPointLocation::Inside)
                );
                assert_eq!(
                    profile.classify_xy(&start[0] - &radius, start[1].clone()),
                    Ok(RegionPointLocation::Boundary)
                );
                let native = profile.try_into_native().unwrap();
                let rebuilt = hypercurve::CurveRegion2::try_from_boundary_paths(
                    &native.boundary_paths().unwrap(),
                    hypercurve::FillRule::NonZero,
                ).unwrap();
                for region in [&native, &rebuilt] {
                    assert_eq!(hyperlimit::compare_reals(
                        &region.filled_area().unwrap().unwrap(), &expected_area,
                        hyperlimit::PredicatePolicy::STRICT,
                    ).value(), Some(std::cmp::Ordering::Equal), "{fields}");
                    let bounds = region.bounds().unwrap().unwrap();
                    for (actual, expected) in [
                        (bounds.min_x(), &start[0] - &radius),
                        (bounds.min_y(), &start[1] - &radius),
                        (bounds.max_x(), &end[0] + &radius),
                        (bounds.max_y(), &end[1] + &radius),
                    ] {
                        assert_eq!(hyperlimit::compare_reals(
                            actual, &expected, hyperlimit::PredicatePolicy::STRICT,
                        ).value(), Some(std::cmp::Ordering::Equal), "{fields}");
                    }
                }
            });
            assert!(outcome.certified().is_some());
        }
    }
}
''' + text[end:]
(SOURCE / name).write_text(text)

archive = (ARCHIVE / 'crates/hyperdrc/tests/native_linear_strokes.rs').read_text()
names = ['kicad_linear_tracks_include_both_round_caps', 'track_decimal_endpoints_and_width_are_never_rounded_first',
    'track_subrange_width_and_length_remain_nonzero', 'track_superrange_coordinates_stay_native',
    'exact_diagonal_track_area_includes_caps_without_rounding_the_width', 'irrational_diagonal_track_keeps_line_and_arc_geometry',
    'track_caps_drive_actual_short_readiness', 'tangent_track_caps_do_not_create_filled_overlap']
blocks = {m.group(1): m.group(0).rstrip() + '\n' for m in re.finditer(
    r'(?ms)^#\[test\]\nfn (\w+)\(\).*?(?=^#\[test\]|^fn deny_refinement|\Z)', archive)}
assert set(names) <= set(reproduction['counts']['archived-frontier']['failed_names'])
header = archive[:archive.index('#[test]')].replace('use hyperdrc::geometry::{line_polygon, polygons_to_profile};\n', '')
test = header + '\n'.join(blocks[n] for n in names) + '''
#[test]
fn source_strokes_retain_strict_receipts_inside_approximate_and_preview_scopes() {
    let check = || {
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board("(start 0 0) (end 1 1) (width 2)");
                let profile = model.all_copper().unwrap();
                same(&profile.try_area().unwrap(), &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap() + Scalar::pi()));
                assert_eq!(profile.classify_xy(scalar("-1"), Scalar::zero()), Ok(Boundary));
                assert_eq!(profile.classify_xy(scalar("-1.000000000001"), Scalar::zero()), Ok(Outside));
                let native = profile.try_into_native().unwrap();
                let rebuilt = hypercurve::CurveRegion2::try_from_boundary_paths(
                    &native.boundary_paths().unwrap(), hypercurve::FillRule::NonZero,
                ).unwrap();
                same(&rebuilt.filled_area().unwrap().unwrap(), &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap() + Scalar::pi()));
            });
            assert!(outcome.certified().is_some());
        }
    };
    check();
    hypercurve::CurvePreviewOptions::try_approximate_512(1000.0, 1000.0)
        .unwrap().evaluate(check);
}

#[test]
fn fresh_zero_budget_load_retains_native_rational_direction_certificates() {
    let outcome = hypercurve::evaluate_with_budget(
        hyperlimit::PredicatePolicy::STRICT, hypercurve::WorkBudget::ZERO,
        || board("(start 0 0) (end 3 4) (width 2)"),
    );
    assert_eq!(outcome.usage, hypercurve::WorkUsage::default());
    let model = outcome.result.unwrap().certified().unwrap();
    same(&model.copper[0].region.try_area().unwrap(), &(Scalar::from(10) + Scalar::pi()));
}
'''
added = 'crates/hyperdrc/tests/kicad_native_strokes.rs'
(SOURCE / added).write_text(test)

name = 'crates/hyperdrc/README.md'
text = (SOURCE / name).read_text()
text = replace_once(text, '### Straight-track load failures', '### Exact straight tracks and load failures')
text = replace_once(text, 'budget exhaustion, unsupported finite geometry, or native geometry failure.', 'budget exhaustion, unsupported geometry, a structured native `ExactCurveError`,\nor a native geometry validation failure. `TrackLoadFailure::Construction`\nretains the kernel diagnosis through `std::error::Error::source`.')
start = text.index('This is a failure-contract migration; readiness implementation version 21')
end = text.index('\n## Useful API', start)
text = text[:start] + '''Straight tracks now retain exact decimal/scientific endpoints and widths through
native round-cap construction. Coincident positive-width endpoints denote the
existing exact disk; distinct endpoints denote the segment's round stroke.
Endpoint caps, subrange widths and coordinates beyond binary64 range no longer
depend on the finite rectangular adapter. The selected geometry is certified
under STRICT even inside an ambient approximate or preview scope, and supports
checked area, bounds, point queries and native handoff without a report projection.

This is an explicit source-fidelity migration. Previously materialized rectangles
omitted the round caps and could round coordinates; their represented sets and
resulting findings can change. Readiness implementation version 22 invalidates
version-21 check evidence. The original finite-range failure fixtures now require
successful exact geometry, while malformed-source and budget failures retain
their typed contracts. Native boundary subdivisions are not source geometry.
The public finite `geometry::line_polygon` adapter and other KiCad object families
retain their existing contracts and open limitations. Shared refinement budgets
do not bound source parsing, total work, heap bytes, or cancellation.
''' + text[end:]
(SOURCE / name).write_text(text)
name = 'crates/hyperdrc/src/readiness.rs'
(SOURCE / name).write_text(replace_once((SOURCE / name).read_text(),
    'pub const CHECK_IMPLEMENTATION_VERSION: u32 = 21;', 'pub const CHECK_IMPLEMENTATION_VERSION: u32 = 22;'))
fmt = subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, capture_output=True)
(O / 'format.stdout').write_bytes(fmt.stdout)
(O / 'format.stderr').write_bytes(fmt.stderr)
assert fmt.returncode == 0
final = {p: sha(SOURCE / p) for p in inputs | {added: ''}}
changes = {p: h for p, h in final.items() if inputs.get(p) != h}
assert set(changes) == {'crates/hyperdrc/src/kicad/tracks.rs', 'crates/hyperdrc/src/readiness.rs',
    'crates/hyperdrc/tests/kicad_track_failures.rs', 'crates/hyperdrc/README.md', added}
assert all(sha(ROOT / p) == h for p, h in inputs.items())
assert all(blocks[n] in (SOURCE / added).read_text() for n in names)
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f:
    json.dump(final, f, sort_keys=True)
dump(O / 'result.json', {'complete': True, 'base_commit': BASE, 'source_inputs': len(final),
    'source_hashes_sha256': sha(O / 'source-hashes.json.gz'), 'changes': changes,
    'runner_sha256': sha(__file__), 'archived_track_tests_preserved_byte_identically': names,
    'readiness_version_migration': [21, 22], 'unsupported_range_expectation_explicitly_migrated': True,
    'main_sources_modified': False, 'native_runtime_admitted': False, 'full_plan_complete': False})
print(json.dumps({'complete': True, 'source_inputs': len(final), 'changed_paths': list(changes)}), flush=True)
