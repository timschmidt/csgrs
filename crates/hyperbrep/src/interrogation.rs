//! Exact differential interrogation of retained analytic surfaces.
//!
//! The first supported family is the full exact plane family. Reports retain
//! parameter tangents, both fundamental forms, oriented normal evidence, and
//! exact zero curvature. Unsupported families remain explicitly blocked.

use hyperlimit::{Point2, Point3};
use hyperreal::{Real, RealSign};

use crate::BrepSurfaceKind;
use crate::{BrepPlaneFrameAxis, BrepSurface, BrepSurfaceFrameReport, BrepSurfaceId};

/// Explicit blocker for exact surface differential interrogation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSurfaceInterrogationBlocker {
    /// The retained source is lossy or unknown.
    /// The surface family is not implemented by the exact interrogation core.
    UnsupportedSurface,
    /// No exact parameter frame was available.
    FrameNotReady,
    /// Exact tangent construction required an unsupported division.
    TangentDivisionFailed,
    /// The retained normal could not be normalized exactly/algebraically.
    NormalNormalizationFailed,
    /// Tangent-frame orientation relative to the retained normal was undecidable.
    UnknownOrientationAlignment,
}

/// Exact first fundamental form coefficients `E`, `F`, and `G`.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceFirstFundamentalForm {
    /// `S_u dot S_u`.
    pub e: Real,
    /// `S_u dot S_v`.
    pub f: Real,
    /// `S_v dot S_v`.
    pub g: Real,
    /// Gram determinant `E G - F^2`.
    pub determinant: Real,
}

/// Exact second fundamental form coefficients `L`, `M`, and `N`.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceSecondFundamentalForm {
    /// `S_uu dot n`.
    pub l: Real,
    /// `S_uv dot n`.
    pub m: Real,
    /// `S_vv dot n`.
    pub n: Real,
}

/// Differential and curvature evidence at one surface parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceDifferentialReport {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Source parameter.
    pub uv: Point2,
    /// Exact parameter frame report.
    pub frame: BrepSurfaceFrameReport,
    /// Evaluated model-space point.
    pub point: Option<Point3>,
    /// First partial derivative with respect to u.
    pub derivative_u: Option<Point3>,
    /// First partial derivative with respect to v.
    pub derivative_v: Option<Point3>,
    /// Cross product `S_u × S_v` before normalization.
    pub parameter_normal: Option<Point3>,
    /// Retained plane normal normalized without a primitive-float adapter.
    pub oriented_unit_normal: Option<Point3>,
    /// Sign of `(S_u × S_v) dot retained_normal`.
    pub orientation_alignment: Option<RealSign>,
    /// Exact first fundamental form.
    pub first_fundamental_form: Option<BrepSurfaceFirstFundamentalForm>,
    /// Exact second fundamental form.
    pub second_fundamental_form: Option<BrepSurfaceSecondFundamentalForm>,
    /// Exact Gaussian curvature when supported.
    pub gaussian_curvature: Option<Real>,
    /// Exact mean curvature when supported.
    pub mean_curvature: Option<Real>,
    /// Explicit blockers.
    pub blockers: Vec<BrepSurfaceInterrogationBlocker>,
    /// Whether point, tangents, normal, forms, and curvature are exact-ready.
    pub exact_differential_ready: bool,
}

impl BrepSurface {
    /// Interrogate this retained surface at one exact UV parameter.
    pub fn interrogate_uv(&self, uv: Point2) -> BrepSurfaceDifferentialReport {
        let evaluation = self.evaluate_frame_uv(uv.clone());
        let frame = evaluation.frame;
        let mut blockers = Vec::new();
        let plane = match &self.kind {
            BrepSurfaceKind::Plane(plane) => Some(plane.as_ref()),
            BrepSurfaceKind::Unsupported { .. } => {
                blockers.push(BrepSurfaceInterrogationBlocker::UnsupportedSurface);
                None
            }
        };
        if !frame.exact_frame_ready || evaluation.point.is_none() {
            blockers.push(BrepSurfaceInterrogationBlocker::FrameNotReady);
        }

        let tangents =
            plane.and_then(|plane| frame.axis.and_then(|axis| plane_tangents(plane, axis)));
        if plane.is_some() && frame.exact_frame_ready && tangents.is_none() {
            blockers.push(BrepSurfaceInterrogationBlocker::TangentDivisionFailed);
        }

        let (derivative_u, derivative_v, parameter_normal, first_fundamental_form) = match tangents
        {
            Some((du, dv)) => {
                let normal = cross(&du, &dv);
                let e = dot(&du, &du);
                let f = dot(&du, &dv);
                let g = dot(&dv, &dv);
                let determinant = e.clone() * g.clone() - f.clone() * f.clone();
                (
                    Some(du),
                    Some(dv),
                    Some(normal),
                    Some(BrepSurfaceFirstFundamentalForm {
                        e,
                        f,
                        g,
                        determinant,
                    }),
                )
            }
            None => (None, None, None, None),
        };

        let oriented_unit_normal = plane.and_then(|plane| normalize(&plane.normal));
        if plane.is_some() && oriented_unit_normal.is_none() {
            blockers.push(BrepSurfaceInterrogationBlocker::NormalNormalizationFailed);
        }
        let orientation_alignment =
            plane
                .zip(parameter_normal.as_ref())
                .and_then(|(plane, parameter_normal)| {
                    dot(parameter_normal, &plane.normal).refine_sign_until(-64)
                });
        if plane.is_some() && parameter_normal.is_some() && orientation_alignment.is_none() {
            blockers.push(BrepSurfaceInterrogationBlocker::UnknownOrientationAlignment);
        }

        let curvature_ready = plane.is_some()
            && derivative_u.is_some()
            && derivative_v.is_some()
            && oriented_unit_normal.is_some()
            && orientation_alignment.is_some();
        let zero = Real::zero();
        let second_fundamental_form = curvature_ready.then(|| BrepSurfaceSecondFundamentalForm {
            l: zero.clone(),
            m: zero.clone(),
            n: zero.clone(),
        });
        let gaussian_curvature = curvature_ready.then(|| zero.clone());
        let mean_curvature = curvature_ready.then_some(zero);
        let exact_differential_ready = blockers.is_empty()
            && evaluation.point.is_some()
            && first_fundamental_form.is_some()
            && second_fundamental_form.is_some();

        BrepSurfaceDifferentialReport {
            surface: self.id,
            uv,
            frame,
            point: evaluation.point,
            derivative_u,
            derivative_v,
            parameter_normal,
            oriented_unit_normal,
            orientation_alignment,
            first_fundamental_form,
            second_fundamental_form,
            gaussian_curvature,
            mean_curvature,
            blockers,
            exact_differential_ready,
        }
    }
}

fn plane_tangents(
    plane: &hyperlimit::Plane3,
    axis: BrepPlaneFrameAxis,
) -> Option<(Point3, Point3)> {
    let zero = Real::zero();
    let one = Real::one();
    Some(match axis {
        BrepPlaneFrameAxis::X => (
            Point3::new(
                divide_negative(&plane.normal.y, &plane.normal.x)?,
                one.clone(),
                zero.clone(),
            ),
            Point3::new(
                divide_negative(&plane.normal.z, &plane.normal.x)?,
                zero,
                one,
            ),
        ),
        BrepPlaneFrameAxis::Y => (
            Point3::new(
                zero.clone(),
                divide_negative(&plane.normal.z, &plane.normal.y)?,
                one.clone(),
            ),
            Point3::new(
                one,
                divide_negative(&plane.normal.x, &plane.normal.y)?,
                zero,
            ),
        ),
        BrepPlaneFrameAxis::Z => (
            Point3::new(
                one.clone(),
                zero.clone(),
                divide_negative(&plane.normal.x, &plane.normal.z)?,
            ),
            Point3::new(
                zero,
                one,
                divide_negative(&plane.normal.y, &plane.normal.z)?,
            ),
        ),
    })
}

fn divide_negative(numerator: &Real, denominator: &Real) -> Option<Real> {
    (-numerator.clone() / denominator).ok()
}

fn normalize(vector: &Point3) -> Option<Point3> {
    let length = dot(vector, vector).sqrt().ok()?;
    Some(Point3::new(
        (&vector.x / &length).ok()?,
        (&vector.y / &length).ok()?,
        (&vector.z / &length).ok()?,
    ))
}

fn dot(left: &Point3, right: &Point3) -> Real {
    left.x.clone() * right.x.clone()
        + left.y.clone() * right.y.clone()
        + left.z.clone() * right.z.clone()
}

fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        left.y.clone() * right.z.clone() - left.z.clone() * right.y.clone(),
        left.z.clone() * right.x.clone() - left.x.clone() * right.z.clone(),
        left.x.clone() * right.y.clone() - left.y.clone() * right.x.clone(),
    )
}

#[cfg(test)]
mod tests {
    use hyperlimit::Plane3;

    use super::*;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn diagonal_plane_has_exact_point_tangents_normal_and_zero_curvature() {
        let surface = BrepSurface::plane(BrepSurfaceId::new(1), Plane3::new(p(1, 1, 1), r(-6)));

        let report = surface.interrogate_uv(Point2::new(r(1), r(2)));

        assert!(report.exact_differential_ready);
        assert_eq!(report.frame.axis, Some(BrepPlaneFrameAxis::X));
        assert_eq!(report.point, Some(p(3, 1, 2)));
        assert_eq!(report.derivative_u, Some(p(-1, 1, 0)));
        assert_eq!(report.derivative_v, Some(p(-1, 0, 1)));
        assert_eq!(report.parameter_normal, Some(p(1, 1, 1)));
        assert_eq!(report.orientation_alignment, Some(RealSign::Positive));
        assert_eq!(report.gaussian_curvature, Some(Real::zero()));
        assert_eq!(report.mean_curvature, Some(Real::zero()));
        let first = report.first_fundamental_form.unwrap();
        assert_eq!(first.e, r(2));
        assert_eq!(first.f, r(1));
        assert_eq!(first.g, r(2));
        assert_eq!(first.determinant, r(3));
    }

    #[test]
    fn unsupported_surface_keeps_interrogation_blockers_visible() {
        let surface = BrepSurface::unsupported(BrepSurfaceId::new(2), "nurbs-surface");
        let report = surface.interrogate_uv(Point2::new(r(0), r(0)));

        assert!(!report.exact_differential_ready);
        assert!(report.point.is_none());
        assert!(
            report
                .blockers
                .contains(&BrepSurfaceInterrogationBlocker::UnsupportedSurface)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSurfaceInterrogationBlocker::FrameNotReady)
        );
    }
}
