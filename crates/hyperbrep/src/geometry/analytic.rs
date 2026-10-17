//! Borrowed exact carrier data for inspection and exchange.
use super::{Curve3, CurveGeometry3, Point3, Real, Surface, SurfaceGeometry, Vector3};

/// A borrowed circular carrier and its exact angular parameterization.
///
/// The axes are orthonormal. For a curve parameter `t`, its angle is
/// `angle_at_start +/- (t - curve.domain().start())`, with the sign given by
/// `increasing_angle`. A topological edge may use a subinterval of that domain.
/// This is immutable inspection data, not a second curve or a construction proof.
#[derive(Clone, Copy, Debug)]
pub struct CircularArcGeometry<'a> {
    /// Circle center.
    pub center: &'a Point3,
    /// Unit direction at angle zero.
    pub x: &'a Vector3,
    /// Unit direction at angle pi/2.
    pub y: &'a Vector3,
    /// Strictly positive radius.
    pub radius: &'a Real,
    /// Angle corresponding to the spatial curve domain's start.
    pub angle_at_start: &'a Real,
    /// Whether angle increases with the spatial curve parameter.
    pub increasing_angle: bool,
}

/// A borrowed exact cylinder frame and radius.
///
/// The right-handed axes are orthonormal. Surface parameters `(u,v)` evaluate
/// as `origin + radius*(x*cos(u) + y*sin(u)) + axis*v`. The finite source chart
/// remains available through [`Surface::domain`].
#[derive(Clone, Copy, Debug)]
pub struct CylinderGeometry<'a> {
    /// Origin of the axial coordinate.
    pub origin: &'a Point3,
    /// Unit radial direction at angle zero.
    pub x: &'a Vector3,
    /// Unit radial direction at angle pi/2.
    pub y: &'a Vector3,
    /// Unit axial direction.
    pub axis: &'a Vector3,
    /// Strictly positive radius.
    pub radius: &'a Real,
}

/// A borrowed exact ring-torus frame and radii.
///
/// The right-handed axes are orthonormal. Both surface parameters are angles:
///
/// ```text
/// center + (major_radius + minor_radius*cos(v))*(x*cos(u) + y*sin(u))
///        + axis*minor_radius*sin(v)
/// ```
///
/// The major radius is strictly greater than the
/// positive minor radius. This view does not change the surface's finite chart.
#[derive(Clone, Copy, Debug)]
pub struct TorusGeometry<'a> {
    /// Center of the major circle.
    pub center: &'a Point3,
    /// Unit radial direction at longitude zero.
    pub x: &'a Vector3,
    /// Unit radial direction at longitude pi/2.
    pub y: &'a Vector3,
    /// Unit axial direction.
    pub axis: &'a Vector3,
    /// Radius of the circle traced by the minor circle's center.
    pub major_radius: &'a Real,
    /// Radius of the minor circle.
    pub minor_radius: &'a Real,
}

impl Curve3 {
    /// Borrows the exact carrier and angular map of a native circular arc.
    ///
    /// Other curve families return `None`; this does not fit, canonicalize or
    /// sample them. Reversal and subdivision retain their actual angular maps.
    pub fn circular_geometry(&self) -> Option<CircularArcGeometry<'_>> {
        let CurveGeometry3::CircleArc(arc) = &self.data.geometry else {
            return None;
        };
        Some(CircularArcGeometry {
            center: &arc.center,
            x: &arc.x,
            y: &arc.y,
            radius: &arc.x_radius,
            angle_at_start: &arc.angle_at_start,
            increasing_angle: arc.direction > 0,
        })
    }
}

impl Surface {
    /// Borrows the exact ring-torus carrier without changing its chart or frame.
    /// Other surface families return `None`.
    pub fn toroidal_geometry(&self) -> Option<TorusGeometry<'_>> {
        let SurfaceGeometry::Torus(torus) = &self.data.geometry else {
            return None;
        };
        Some(TorusGeometry {
            center: &torus.center,
            x: &torus.frame.x,
            y: &torus.frame.y,
            axis: &torus.frame.z,
            major_radius: &torus.major_radius,
            minor_radius: &torus.minor_radius,
        })
    }

    /// Borrows the exact cylinder carrier without changing its chart or frame.
    /// Other surface families return `None`.
    pub fn cylindrical_geometry(&self) -> Option<CylinderGeometry<'_>> {
        let SurfaceGeometry::Cylinder(cylinder) = &self.data.geometry else {
            return None;
        };
        Some(CylinderGeometry {
            origin: &cylinder.origin,
            x: &cylinder.frame.x,
            y: &cylinder.frame.y,
            axis: &cylinder.frame.z,
            radius: &cylinder.radius,
        })
    }
}
