//! SVG document import and export through Hypercurve.
//!
//! CSGRS owns the interchange adapter only. Hypercurve owns SVG syntax,
//! styling, transforms, primitive construction, topology, finite projection,
//! and serialization.

use crate::io::IoError;
use hypercurve::{
    CurvePath2, CurveRegion2, CurveString2, SvgError, SvgGeometry2,
    export_svg_document_with_options, import_svg_document_with_options,
};

/// Native configuration for callers using Hypercurve's explicit-options entry points.
/// This module's convenience functions always use its default value.
pub use hypercurve::SvgOptions;

/// Maps native SVG diagnostics without erasing scalar, exact-curve or topology causes.
/// Text-only syntax/unsupported/geometry errors remain textual; size bounds keep
/// their limit label. This adds format context without reclassifying native failures.
fn map_svg_error(error: SvgError) -> IoError {
    match error {
        SvgError::MalformedInput(detail) => IoError::MalformedInput(detail),
        SvgError::Unsupported(detail) => IoError::Unsupported {
            format: "SVG",
            detail,
        },
        SvgError::Geometry(detail) => IoError::Geometry {
            format: "SVG",
            detail,
        },
        SvgError::Scalar(source) => IoError::Construction {
            format: "SVG",
            source: source.into(),
        },
        SvgError::Exact(source) => IoError::Construction {
            format: "SVG",
            source: source.into(),
        },
        SvgError::PolynomialTopology(source) => IoError::Construction {
            format: "SVG",
            source: crate::errors::ValidationError::PolynomialTopology(source),
        },
        SvgError::SizeOverflow { limit } => IoError::SizeOverflow {
            format: "SVG",
            limit,
        },
    }
}

/// Imports an SVG document into native Hypercurve carriers.
///
/// Returns the unified filled region, native line/arc stroke wires and remaining
/// stroke paths, in that order. Stroke carriers describe centerlines, not expanded
/// stroke-width regions. Uses [`SvgOptions::default`] and the native importer;
/// supported syntax, paint selection, transforms and admission remain Hypercurve's
/// responsibility. This is geometry import, not preservation of the SVG DOM or styling.
///
/// Versioned `data-hypercurve-path` definitions are decoded and validated before
/// use, including when hidden or unused. Ordinary hidden paths still require
/// valid command syntax. Native topology/replay uses strict decisions. Standard
/// circle/ellipse and rounded-rectangle primitives currently use sampled geometry;
/// their admission does not imply exact analytic-source replay. Native exact
/// extensions instead retain supported definitions and selections for later operations.
/// Malformed, unsupported, typed native and resource-limit failures are mapped to
/// [`IoError`]; this wrapper returns no separate aggregate certainty outcome.
pub fn import_svg(
    document: &str,
) -> Result<(CurveRegion2, Vec<CurveString2>, Vec<CurvePath2>), IoError> {
    import_svg_document_with_options(document, SvgOptions::default())
        .map(SvgGeometry2::into_parts)
        .map_err(map_svg_error)
}

/// Exports native Hypercurve carriers as a complete SVG document.
///
/// Clones the carriers into the native exporter with [`SvgOptions::default`].
/// The region is strictly revalidated, including qualified empty geometry.
/// Supported exact definitions are encoded in a versioned `data-hypercurve-path`
/// extension alongside standard SVG display paths; a reader that drops this
/// extension retains only the standard representation and its projection limits.
/// Unsupported retained definitions and exceeded native bounds fail explicitly.
///
/// Uses canonical black fill/strokes rather than restoring imported styling.
/// Empty geometry has a zero-root native graph and unit display viewport, which
/// adds no material. Default bounds constrain individual projections and definition
/// graphs, not total arithmetic work or memory. For explicit options, use
/// [`hypercurve::export_svg_document_with_options`].
pub fn export_svg(
    region: &CurveRegion2,
    wires: &[CurveString2],
    paths: &[CurvePath2],
) -> Result<String, IoError> {
    export_svg_document_with_options(
        &SvgGeometry2::new(region.clone(), wires.to_vec(), paths.to_vec()),
        SvgOptions::default(),
    )
    .map_err(map_svg_error)
}
