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

pub use hypercurve::SvgOptions;

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
        SvgError::SizeOverflow { limit } => IoError::SizeOverflow {
            format: "SVG",
            limit,
        },
    }
}

/// Imports an SVG document into native Hypercurve carriers.
pub fn import_svg(
    document: &str,
) -> Result<(CurveRegion2, Vec<CurveString2>, Vec<CurvePath2>), IoError> {
    import_svg_document_with_options(document, SvgOptions::default())
        .map(SvgGeometry2::into_parts)
        .map_err(map_svg_error)
}

/// Exports native Hypercurve carriers as a complete SVG document.
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
