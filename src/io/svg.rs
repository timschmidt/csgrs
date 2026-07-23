//! SVG document import and export through Hypercurve.
//!
//! CSGRS owns the [`Profile`] adapter only. Hypercurve owns SVG syntax,
//! styling, transforms, primitive construction, topology, finite projection,
//! and serialization.

use crate::io::IoError;
use crate::sketch::Profile;
use hypercurve::{
    SvgError, SvgGeometry2, export_svg_document_with_options, import_svg_document_with_options,
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

/// Imports complete SVG documents into CSGRS profiles.
pub trait FromSVG: Sized {
    fn from_svg(document: &str) -> Result<Self, IoError>;
    fn from_svg_with_options(document: &str, options: SvgOptions) -> Result<Self, IoError>;
}

impl FromSVG for Profile {
    fn from_svg(document: &str) -> Result<Self, IoError> {
        Self::from_svg_with_options(document, SvgOptions::default())
    }

    fn from_svg_with_options(document: &str, options: SvgOptions) -> Result<Self, IoError> {
        let geometry =
            import_svg_document_with_options(document, options).map_err(map_svg_error)?;
        let (region, wires, paths) = geometry.into_parts();
        Ok(Self::from_curve_topology(region, wires, paths))
    }
}

/// Exports CSGRS profiles as complete SVG documents.
pub trait ToSVG {
    fn to_svg(&self) -> Result<String, IoError>;
    fn to_svg_with_options(&self, options: SvgOptions) -> Result<String, IoError>;
}

impl ToSVG for Profile {
    fn to_svg(&self) -> Result<String, IoError> {
        self.to_svg_with_options(SvgOptions::default())
    }

    fn to_svg_with_options(&self, options: SvgOptions) -> Result<String, IoError> {
        export_svg_document_with_options(
            &SvgGeometry2::new(
                self.as_curve_region().clone(),
                self.wires().to_vec(),
                self.curve_paths().to_vec(),
            ),
            options,
        )
        .map_err(map_svg_error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::CSG;
    use hypercurve::{
        BezierSplitFragment2, BezierSubcurve2, Classification, Curve2, CurvePath2,
        CurvePolicy, CurveRegion2, Point2, Real,
    };

    #[test]
    fn imports_inherited_transform_and_svg_defaults() {
        let document = r#"<svg xmlns="http://www.w3.org/2000/svg"><g transform="translate(3 4)"><circle r="2"/></g></svg>"#;
        let profile = Profile::from_svg(document).unwrap();
        let bounds = profile.bounding_box();
        for (actual, expected) in [
            (&bounds.mins.x, 1.0),
            (&bounds.mins.y, 2.0),
            (&bounds.maxs.x, 5.0),
            (&bounds.maxs.y, 6.0),
        ] {
            assert!((actual.to_f64_lossy().unwrap() - expected).abs() < 0.001);
        }
    }

    #[test]
    fn imports_cubic_fill_and_stroke_without_demoting_topology() {
        let document = r#"<svg xmlns="http://www.w3.org/2000/svg"><path d="M 0 0 C 0 2 2 2 2 0 Z" stroke="black"/></svg>"#;
        let profile = Profile::from_svg(document).unwrap();

        assert_eq!(
            profile
                .as_curve_region()
                .loop_role_counts(&CurvePolicy::certified())
                .unwrap(),
            Classification::Decided((1, 0))
        );
        assert!(
            profile
                .as_curve_region()
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::Cubic(_),
                        ..
                    }
                ))
        );
        assert_eq!(profile.curve_paths().len(), 1);
        assert_eq!(profile.contains_xy(Real::one(), Real::one()), Some(true));
    }

    #[test]
    fn polyline_fill_closes_but_stroke_remains_open() {
        let document = r#"<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 2,0 1,1" stroke="black"/></svg>"#;
        let profile = Profile::from_svg(document).unwrap();
        assert_eq!(
            profile
                .as_curve_region()
                .loop_role_counts(&CurvePolicy::certified())
                .unwrap(),
            Classification::Decided((1, 0))
        );
        assert_eq!(profile.wires().len(), 1);
        assert_ne!(profile.wires()[0].start(), profile.wires()[0].end());
    }

    #[test]
    fn exported_svg_round_trips_through_hypercurve() {
        let square = Profile::square(Real::from(3_u8));
        let document = square.to_svg().unwrap();
        let reparsed = Profile::from_svg(&document).unwrap();
        let bounds = reparsed.bounding_box();
        assert_eq!(bounds.mins.x, Real::zero());
        assert_eq!(bounds.mins.y, Real::zero());
        assert_eq!(bounds.maxs.x, Real::from(3_u8));
        assert_eq!(bounds.maxs.y, Real::from(3_u8));
    }

    #[test]
    fn adapter_preserves_exact_hypercurve_nurbs_paths() {
        let curve = Curve2::try_nurbs(
            2,
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::from(2)),
                Point2::new(Real::from(3), Real::zero()),
            ],
            vec![
                Real::one(),
                (Real::one() / Real::from(3)).unwrap(),
                Real::one(),
            ],
            vec![0, 0, 0, 1, 1, 1].into_iter().map(Real::from).collect(),
        )
        .unwrap();
        let profile = Profile::from_curve_topology(
            CurveRegion2::empty(),
            Vec::new(),
            vec![CurvePath2::try_new(vec![curve.clone()]).unwrap()],
        );

        let document = profile.to_svg().unwrap();
        assert!(document.contains("data-hypercurve-path=\"1:"));
        let reparsed = Profile::from_svg(&document).unwrap();
        assert_eq!(reparsed.curve_paths()[0].curves(), &[curve]);
    }

    #[test]
    fn maps_hypercurve_svg_errors_to_io_categories() {
        for document in [
            r#"<svg xmlns="http://www.w3.org/2000/svg"><text>geometry</text></svg>"#,
            r#"<svg xmlns="http://www.w3.org/2000/svg"><line x2="1" stroke="black" stroke-dasharray="1 1"/></svg>"#,
            r#"<svg xmlns="http://www.w3.org/2000/svg"><svg x="1"><rect width="1" height="1"/></svg></svg>"#,
        ] {
            assert!(matches!(
                Profile::from_svg(document),
                Err(IoError::Unsupported { format: "SVG", .. })
            ));
        }
        assert!(matches!(
            Profile::from_svg(
                r#"<svg xmlns="http://www.w3.org/2000/svg"><ellipse rx="2" ry="-1"/></svg>"#
            ),
            Err(IoError::MalformedInput(_))
        ));
        assert!(Profile::empty().to_svg().is_err());
    }
}
