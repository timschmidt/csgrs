//! Immutable spline inspection and exact extrusion point location.
use super::*;

/// Borrowed rational Bezier data on the curve's unit parameter domain.
/// Weights are strictly positive. This is inspection, not a new carrier or a
/// certificate that a topological trim has any particular image.
#[derive(Clone, Copy, Debug)]
pub struct RationalBezierCurveGeometry<'a> {
    /// Ordered Cartesian control points.
    pub control_points: &'a [Point3],
    /// One positive homogeneous weight per control point.
    pub weights: &'a [Real],
}

/// Borrowed finite clamped NURBS data in its authored parameter chart.
#[derive(Clone, Copy, Debug)]
pub struct NurbsCurveGeometry<'a> {
    /// Polynomial degree, strictly between zero and the control count.
    pub degree: usize,
    /// Ordered Cartesian control points.
    pub control_points: &'a [Point3],
    /// One positive homogeneous weight per control point.
    pub weights: &'a [Real],
    /// Nondecreasing full knot vector, including repeated knots.
    pub knots: &'a [Real],
}

impl Curve3 {
    /// Borrows authored rational Bezier coefficients without fitting or copying.
    pub fn rational_bezier_geometry(&self) -> Option<RationalBezierCurveGeometry<'_>> {
        let CurveGeometry3::RationalBezier(curve) = &self.data.geometry else {
            return None;
        };
        Some(RationalBezierCurveGeometry {
            control_points: &curve.control_points,
            weights: &curve.weights,
        })
    }

    /// Borrows authored NURBS coefficients without fitting or changing the chart.
    pub fn nurbs_geometry(&self) -> Option<NurbsCurveGeometry<'_>> {
        let CurveGeometry3::Nurbs(curve) = &self.data.geometry else {
            return None;
        };
        Some(NurbsCurveGeometry {
            degree: curve.degree,
            control_points: &curve.control_points,
            weights: &curve.weights,
            knots: &curve.knots,
        })
    }
}

impl Surface {
    /// Locates a point on an extrusion with a planar line, spline or retained
    /// planar-image generator
    /// transverse to its direction. Returns all generator parameters and the
    /// unique extrusion parameter `v` in `C(u) + v*direction`.
    ///
    /// A plane containing every generator control point is proved before
    /// projection. The generator's complete spatial point-location query then
    /// proves incidence; multiple preimages and selected roots are retained.
    /// A nonplanar generator or nontransverse plane is explicitly unsupported.
    /// This locates one point, not the complete image of an edge on the surface.
    pub fn extrusion_parameters_of(
        &self,
        point: &Point3,
    ) -> GeometryResult<(CurveParameterLocation, Real)> {
        crate::context::strict(|| {
            let SurfaceGeometry::Extrusion(extrusion) = &self.data.geometry else {
                return Err(GeometryError::UnsupportedIntersection);
            };
            let profile = &extrusion.profile;
            if let CurveGeometry3::PlanarImage(image) = &profile.data.geometry {
                // The injective frame certifies the entire source plane;
                // procedural curves need no control-hull approximation.
                return extrusion_point_location(
                    profile,
                    &extrusion.direction,
                    &image.origin,
                    &image.u.cross(&image.v),
                    point,
                );
            }
            let line_points;
            let controls = match &profile.data.geometry {
                CurveGeometry3::Line(line) => {
                    line_points = [line.start.clone(), line.end.clone()];
                    &line_points[..]
                }
                CurveGeometry3::RationalBezier(curve) => &curve.control_points,
                CurveGeometry3::Nurbs(curve) => &curve.control_points,
                _ => return Err(GeometryError::UnsupportedIntersection),
            };
            let origin = &controls[0];
            let direction = &extrusion.direction;
            let zero = |value: &Real| -> GeometryResult<bool> {
                Ok(decided_order(compare_reals(
                    value,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? == Ordering::Equal)
            };
            // Prefer the perpendicular plane, without requiring it. Oblique
            // extrusions use the same control-hull planarity proof below.
            let mut normal = direction.clone();
            let mut perpendicular = true;
            for control in controls {
                if !zero(&normal.dot(&(control - origin)))? {
                    perpendicular = false;
                    break;
                }
            }
            if !perpendicular {
                let mut first = None;
                let mut plane_normal = None;
                for point in &controls[1..] {
                    let delta = point - origin;
                    if zero(&delta.norm_squared())? {
                        continue;
                    }
                    if let Some(first) = &first {
                        let cross = delta.cross(first);
                        if !zero(&cross.norm_squared())? {
                            plane_normal = Some(cross);
                            break;
                        }
                    } else {
                        first = Some(delta);
                    }
                }
                normal = if let Some(normal) = plane_normal {
                    normal
                } else {
                    let first = first.ok_or(GeometryError::UnsupportedIntersection)?;
                    direction.clone()
                        - first.clone()
                            * ((direction.dot(&first) / first.norm_squared())
                                .map_err(GeometryError::Numeric)?)
                };
                for control in controls {
                    if !zero(&normal.dot(&(control - origin)))? {
                        return Err(GeometryError::UnsupportedIntersection);
                    }
                }
            }
            extrusion_point_location(profile, direction, origin, &normal, point)
        })
    }
}

fn extrusion_point_location(
    profile: &Curve3,
    direction: &Vector3,
    origin: &Point3,
    normal: &Vector3,
    point: &Point3,
) -> GeometryResult<(CurveParameterLocation, Real)> {
    let denominator = normal.dot(direction);
    if decided_order(compare_reals(
        &denominator,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? == Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let v = (normal.dot(&(point - origin)) / denominator).map_err(GeometryError::Numeric)?;
    let projected = point.clone() - direction.clone() * &v;
    Ok((profile.parameters_of(&projected)?, v))
}
