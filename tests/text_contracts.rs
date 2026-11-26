#![cfg(any(feature = "truetype-text", feature = "hershey-text"))]
//! Native text boundary witnesses for exact placement, typed failures and composition.
//!
//! Synthetic font data supplies independent square/arch/layout expectations;
//! retained real-font and Hershey workloads add coverage without replacing those
//! oracles. Optional STL checks compare the format's quantized position set, not
//! fidelity to the original exact font outlines.

/// Shared certified-zero control and independently unresolved sine-addition fixture.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryCertainty, GeometryContext, Real, curve,
    errors::{ValidationError, ValidationErrorKind},
};

/// Exact signed-integer test literal.
fn r(n: i32) -> Real {
    n.into()
}
/// Exact rational test literal; a zero denominator is a fixture-author error.
#[cfg(feature = "truetype-text")]
fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}
/// Requires a certified equality decision at the fixture's -1024 refinement limit.
/// An unresolved decision fails the witness rather than becoming tolerance equality.
fn exact(left: &Real, right: &Real) {
    assert_eq!(left.certified_eq_until(right, -1024).as_bool(), Some(true));
}
/// Verifies the old unit-circle identity, then obtains the separate unresolved zero control.
fn unresolved_zero() -> Real {
    let s = Real::e().sin();
    let c = Real::e().cos();
    uncertainty::terminal_zero(&s * &s + &c * &c - r(1))
}

/// Synthetic sfnt/glyph/table fixtures, including deliberate corruption helpers.
#[cfg(feature = "truetype-text")]
#[path = "support/text_font.rs"]
mod text_font;

/// Filled TrueType outline, exact scale, layout, failure and downstream witnesses.
#[cfg(feature = "truetype-text")]
mod truetype {
    use super::*;
    use hypercurve::{BooleanOp, CurveFamily2, CurveRegion2};

    /// Point size making one font unit one model unit for the 1000-unit/em fixture.
    fn unit_size() -> Real {
        q(360_000, 127)
    }
    /// Requires a native exact filled-area result for a fixture with area support.
    fn area(region: &CurveRegion2) -> Real {
        region.filled_area().unwrap().unwrap()
    }
    /// Checks strict classification at an integer witness, with `None` meaning boundary.
    fn inside(region: &CurveRegion2, x: i32, y: i32, expected: Option<bool>) {
        assert_eq!(curve::contains_xy(region, r(x), r(y)).unwrap(), expected);
    }

    /// Blank text/glyphs compose as empty while font, scale, glyph and table failures stay typed.
    /// Retains invalid-input companions to empty requests and checks native error sources;
    /// incomplete outline data must not be mistaken for a certified blank glyph.
    #[test]
    fn blank_text_is_distinct_from_font_glyph_and_scale_failures() {
        let font = text_font::font(false);
        for text in ["", " \n\t\r", "B"] {
            let region = curve::truetype_text(text, &font, unit_size()).unwrap();
            assert!(region.is_empty());
            let moved = curve::translated(&region, Real::pi(), r(2)).unwrap();
            assert!(
                moved
                    .boolean_region(&region, BooleanOp::Union)
                    .unwrap()
                    .is_empty()
            );
        }
        for text in ["", "R"] {
            let error = curve::truetype_text(text, b"not a font", r(1)).unwrap_err();
            assert!(matches!(error, ValidationError::FontFace(_)));
            assert!(std::error::Error::source(&error).is_some());
            for size in [r(0), r(-1)] {
                assert!(matches!(
                    curve::truetype_text(text, &font, size),
                    Err(ValidationError::NonPositiveParameter { .. })
                ));
            }
            assert_eq!(
                curve::truetype_text(text, &font, unresolved_zero())
                    .unwrap_err()
                    .kind(),
                ValidationErrorKind::Undecided
            );
        }
        assert!(matches!(
            curve::truetype_text("R?R", &font, r(1)),
            Err(ValidationError::MissingFontGlyph { character: '?' })
        ));
        assert!(matches!(
            curve::truetype_text("RXR", &font, r(1)),
            Err(ValidationError::InvalidFontOutline { glyph_id: 4 })
        ));
        for tag in [*b"cmap", *b"hmtx", *b"glyf"] {
            let mut broken = font.clone();
            text_font::truncate_table(&mut broken, tag, 1);
            // The glyph range check detects truncated glyf data even when its
            // lazy table object parses; required layout tables fail up front.
            let text = if tag == *b"glyf" { "R" } else { "" };
            assert_eq!(
                curve::truetype_text(text, &broken, r(1)).unwrap_err().kind(),
                ValidationErrorKind::InvalidInput
            );
        }
        let mut broken = font.clone();
        text_font::truncate_table(&mut broken, *b"loca", 4);
        assert!(matches!(
            curve::truetype_text("B", &broken, r(1)),
            Err(ValidationError::InvalidFontOutline { .. })
        ));
    }

    /// Synthetic square/quadratic outlines retain exact area, contacts and curve families.
    /// Both policies certify; translation and empty Boolean composition preserve
    /// the arch, while square extrusion has paired geometric edges and six-volume 120000.
    #[test]
    fn independent_square_and_quadratic_semantics_survive_composition() {
        let font = text_font::font(false);
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let square =
                curve::truetype_text_with_context("R", &font, unit_size(), context).unwrap();
            assert_eq!(square.certainty, GeometryCertainty::Certified);
            exact(&area(&square.value), &r(10_000));
            inside(&square.value, 50, 50, Some(true));
            inside(&square.value, 101, 50, Some(false));
            inside(&square.value, 0, 50, None);
            let arch =
                curve::truetype_text_with_context("Q", &font, unit_size(), context).unwrap();
            assert_eq!(arch.certainty, GeometryCertainty::Certified);
            // x(t)=100t, y(t)=200t(1-t), integral y dx = 10,000/3.
            exact(&area(&arch.value), &q(10_000, 3));
            inside(&arch.value, 50, 25, Some(true));
            inside(&arch.value, 50, 51, Some(false));
            inside(&arch.value, 50, 50, None);
            assert!(
                arch.value
                    .boundary_paths()
                    .unwrap()
                    .iter()
                    .flat_map(|p| p.curves())
                    .any(|c| c.family() == CurveFamily2::QuadraticBezier)
            );
            let moved = curve::translated(&arch.value, r(200), r(-2)).unwrap();
            exact(&area(&moved), &q(10_000, 3));
            let accumulated = CurveRegion2::empty()
                .boolean_region(&moved, BooleanOp::Union)
                .unwrap();
            exact(&area(&accumulated), &q(10_000, 3));
            let emptied = accumulated
                .boolean_region(&moved, BooleanOp::Difference)
                .unwrap();
            assert!(emptied.is_empty());
            let mesh = curve::try_extrude(&square.value, r(2), context).unwrap();
            assert_eq!(mesh.certainty, GeometryCertainty::Certified);
            assert!(
                mesh.value
                    .is_closed_manifold_geometry(&context.mesh_context())
                    .unwrap()
                    .value
            );
            let six_volume = mesh.value.triangles.iter().fold(r(0), |sum, triangle| {
                let [a, b, c] = triangle
                    .indices()
                    .map(|i| mesh.value.positions[i].to_vector());
                sum + a.dot(&b.cross(&c))
            });
            exact(&six_volume, &r(120_000));
        }
    }

    /// Point-to-millimeter scaling retains symbolic, sub-binary64 and enormous exact sizes.
    /// Compares square area with the independent quadratic scale law and distinguishes
    /// sizes separated by 2^-100; no finite-coordinate export participates.
    #[test]
    fn requested_scale_and_point_units_never_round_through_binary64() {
        let font = text_font::font(false);
        let epsilon = (r(1) / r(2).powi_i64(100).unwrap()).unwrap();
        let small = (r(1) / r(2).powi_i64(1100).unwrap()).unwrap();
        let scales = [
            r(1),
            r(1) + &epsilon,
            Real::pi(),
            small,
            r(2).powi_i64(1100).unwrap(),
        ];
        let mut areas = Vec::new();
        for size in scales {
            let region = curve::truetype_text("R", &font, size.clone()).unwrap();
            let unit = size * q(127, 360_000);
            let actual = area(&region);
            exact(&actual, &(r(10_000) * &unit * &unit));
            areas.push(actual);
        }
        assert_eq!(
            (&areas[1] - &areas[0]).certified_sign_until(-1024).sign(),
            Some(hyperreal::RealSign::Positive)
        );
    }

    /// Independent point probes distinguish covered pair adjustment and newline/carriage/tab layout.
    /// An uncovered Q pair must not inherit R's class-zero adjustment; a space still
    /// participates in pair history, unlike explicit layout controls.
    #[test]
    fn layout_advances_controls_and_class_coverage_have_independent_witnesses() {
        let font = text_font::font(true);
        let region = curve::truetype_text("RR", &font, unit_size()).unwrap();
        inside(&region, 850, 50, Some(true)); // covered R uses -200 pair adjustment
        inside(&region, 1050, 50, Some(false));
        let region = curve::truetype_text("QQ", &font, unit_size()).unwrap();
        inside(&region, 1050, 10, Some(true)); // Q is not in the coverage table
        inside(&region, 850, 10, Some(false));
        for (text, x, y) in [
            // R-space is covered too: 1000 - 200 + 1000 = 1800.
            ("R R", 1850, 50),
            ("R\tR", 5050, 50),
            ("R\nR", 50, -950),
            ("R\rR", 50, 50),
        ] {
            let region = curve::truetype_text(text, &font, unit_size()).unwrap();
            inside(&region, x, y, Some(true));
        }
    }

    /// Retained Asar text workloads certify and survive translation, empty union and extrusion.
    /// Checks boundary counts and geometric edge closure under both policies.
    /// Optional STL replay compares binary32 position sets after reimport and
    /// local validation, without claiming preservation of exact curved outlines.
    #[test]
    fn original_readme_and_benchmark_workloads_complete_under_both_policies() {
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            for (text, size, boundary_loops) in [("CSG", 72, 3), ("csgrs benchmark", 24, 19)] {
                let result = curve::truetype_text_with_context(
                    text,
                    include_bytes!("../asar.ttf"),
                    r(size),
                    context,
                )
                .unwrap();
                assert_eq!(result.certainty, GeometryCertainty::Certified);
                assert_eq!(result.value.len(), boundary_loops);
                let translated = curve::translated(&result.value, r(3), r(-2)).unwrap();
                let union = translated
                    .boolean_region(&CurveRegion2::empty(), BooleanOp::Union)
                    .unwrap();
                assert_eq!(union.len(), boundary_loops);
                assert!(!union.boundary_paths().unwrap().is_empty());
                // Curved extrusion uses the documented finite profile boundary.
                // Its local predicate certainty is not exact font-surface fidelity.
                let mesh = curve::try_extrude(&union, r(2), context).unwrap();
                assert_eq!(mesh.certainty, GeometryCertainty::Certified);
                assert!(
                    mesh.value
                        .is_closed_manifold_geometry(&context.mesh_context())
                        .unwrap()
                        .value
                );
                #[cfg(feature = "stl-io")]
                {
                    let bytes =
                        csgrs::io::stl::to_stl_binary(&mesh.value, "font outline").unwrap();
                    let imported = csgrs::io::stl::from_stl(&bytes).unwrap();
                    assert!(
                        imported
                            .is_closed_manifold_geometry(&context.mesh_context())
                            .unwrap()
                            .value
                    );
                    // This checks the format's represented f32 vertices, a
                    // different claim from preserving the original exact outline.
                    let quantized = |model: &csgrs::TriangleMesh| {
                        model
                            .positions
                            .iter()
                            .map(|p| {
                                [&p.x, &p.y, &p.z]
                                    .map(|v| (v.to_f64_lossy().unwrap() as f32).to_bits())
                            })
                            .collect::<std::collections::BTreeSet<_>>()
                    };
                    assert_eq!(quantized(&mesh.value), quantized(&imported));
                }
            }
        }
    }
}

/// Open Hershey stroke diagnostics, exact Cartesian placement and retained catalog witnesses.
#[cfg(feature = "hershey-text")]
mod hershey {
    use super::*;
    use hypercurve::{
        Point2,
        hershey::{Font, HersheyError, TextError, fonts},
    };

    /// Malformed records, missing glyphs, degenerate strokes and invalid sizes keep native causes.
    /// Empty requests still validate the font and size; unresolved size remains undecided.
    #[test]
    fn failures_retain_native_causes_through_public_wrappers() {
        let valid = Font::new(&["MWRMNV RRMVV"], 'A');
        let malformed = Font::new(&["MWRMNV RRMVV", "M"], 'A');
        let degenerate = Font::new(&["MWRRRR"], 'A');
        for (text, font, expected) in [
            ("", &malformed, ValidationErrorKind::InvalidInput),
            ("ABA", &malformed, ValidationErrorKind::InvalidInput),
            ("A", &degenerate, ValidationErrorKind::InvalidInput),
            ("AB", &valid, ValidationErrorKind::Unsupported),
        ] {
            let error = curve::hershey_strings(text, font, r(1)).unwrap_err();
            assert_eq!(error.kind(), expected);
            assert!(matches!(error, ValidationError::HersheyText(_)));
            assert!(std::error::Error::source(&error).is_some());
        }
        assert!(matches!(
            curve::hershey_strings("AB", &valid, r(1)),
            Err(ValidationError::HersheyText(TextError::Font(
                HersheyError::NoSuchGlyph('B')
            )))
        ));
        for text in ["", "A"] {
            assert!(matches!(
                curve::hershey_strings(text, &valid, r(0)),
                Err(ValidationError::HersheyText(TextError::ZeroSize))
            ));
            assert_eq!(
                curve::hershey_strings(text, &valid, unresolved_zero())
                    .unwrap_err()
                    .kind(),
                ValidationErrorKind::Undecided
            );
        }
    }

    /// Exact stroke endpoints and advances survive negative/symbolic size under both policies.
    /// Also requires certified empty whitespace output and retained catalog stroke/edge counts.
    #[test]
    fn exact_cartesian_strokes_spacing_negative_and_symbolic_sizes() {
        let font = Font::new(&["MWRMNV RRMVV"], 'A');
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            for size in [r(1), r(-1), Real::pi()] {
                let outcome =
                    curve::hershey_strings_with_context("AA", &font, size.clone(), context)
                        .unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                assert_eq!(outcome.value.len(), 4);
                for (stroke, (x0, y0, x1, y1)) in outcome.value.iter().zip([
                    (0, 5, -4, -4),
                    (0, 5, 4, -4),
                    (8, 5, 4, -4),
                    (8, 5, 12, -4),
                ]) {
                    assert_eq!(stroke.segments().len(), 1);
                    let edge = &stroke.segments()[0];
                    for (actual, expected) in [
                        (edge.start(), Point2::new(r(x0) * &size, r(y0) * &size)),
                        (edge.end(), Point2::new(r(x1) * &size, r(y1) * &size)),
                    ] {
                        exact(actual.x(), expected.x());
                        exact(actual.y(), expected.y());
                    }
                }
            }
            for text in ["", " \n\t"] {
                let empty =
                    curve::hershey_strings_with_context(text, &font, r(1), context).unwrap();
                assert!(empty.value.is_empty());
                assert_eq!(empty.certainty, GeometryCertainty::Certified);
            }
            for (text, font, size, count, edges) in [
                ("CSG", &fonts::ROWMANS, 1, 4, 55),
                ("CSGRS", &fonts::FUTURAL, 2, 8, 85),
            ] {
                let result =
                    curve::hershey_strings_with_context(text, font, r(size), context).unwrap();
                assert_eq!(result.certainty, GeometryCertainty::Certified);
                assert_eq!(result.value.len(), count);
                assert_eq!(
                    result.value.iter().map(|s| s.segments().len()).sum::<usize>(),
                    edges
                );
            }
        }
    }
}
