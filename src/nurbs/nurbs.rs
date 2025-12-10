//! NURBS‑based CSG implementation leveraging Curvo’s boolean and transformation
//! algorithms. Mirrors the public functionality of `mesh.rs` but operates on
//! 2D/3D NURBS curves and surfaces (via the [`curvo`] crate’s `Region`,
//! `CompoundCurve`, and boolean infrastructure).
//!
//! The main exported type is [`Nurbs<S>`] which is a thin wrapper around a
//! [`Region<Real>`] (an exterior `CompoundCurve` with optional interior holes),
//! enriched with the same high‑level convenience API that `Mesh` exposes: union
//! / difference / intersection, affine transforms, lazy AABB calculation, etc.
//!
//! * Only **planar** NURBS are supported for now – i.e. curves that live in the
//!   Z = 0 plane. 3‑dimensional NURBS surfaces can be lifted into the 3D CSG
//!   world by extruding / sweeping before being wrapped in [`Nurbs`].
//! * All geometric ops delegate to Curvo’s boolean engine (`boolean` module).
//! * The module is self‑contained; no changes are required elsewhere in the
//!   code‑base – simply `use crate::nurbs::Nurbs;`.
//!
//! ## Example
//! ```rust
//! use crate::curve::{NurbsCurve2D, KnotStyle};
//! use crate::nurbs::Nurbs;
//!
//! let circle = Nurbs::circle(1.0, 64).translate(2.0, 0.0, 0.0);
//! let square = Nurbs::rectangle(2.0, 2.0, None);
//! let shape  = circle.union(&square).float();
//! ```

use std::sync::OnceLock;

use curvo::prelude::Boolean;
use curvo::prelude::operation::BooleanOperation;
//use curvo::boolean::{Boolean, BooleanOperation, Clip};
//use curvo::curve::{nurbs_curve::NurbsCurve2D, KnotStyle};
use curvo::prelude::nurbs_curve;
use curvo::prelude::NurbsCurve2D;
use curvo::prelude::KnotStyle;
use crate::float_types::{parry3d::bounding_volume::Aabb, Real};
use curvo::prelude::FloatingPoint;
use curvo::region::{CompoundCurve, Region};
use crate::csg::CSGOps;

use nalgebra::{Matrix4, Point3, Translation3, Vector3};

/// A CSG solid made of one or more planar NURBS curves (an exterior boundary
/// and zero or more interior holes).
#[derive(Clone, Debug)]
pub struct Nurbs<S: Clone + Send + Sync + std::fmt::Debug = ()> {
    /// The planar region that defines the boundary. All curves lie in Z = 0.
    region: Region<Real>,
    /// Lazily computed axis‑aligned bounding box in 3D (thickness‑less but with
    /// min.z = max.z = 0).
    bbox: OnceLock<Aabb>,
    /// Optional metadata carried along through boolean / transform operations.
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + std::fmt::Debug> Nurbs<S> {
    /* =========================================================================
     * Constructors – helpers to build primitive planar regions
     * ========================================================================= */

    /// Returns an *axis‑aligned* rectangle with the given *width* (X) and
    /// *height* (Y) centred at the origin. If `metadata` is `Some`, it will be
    /// stored in the created [`Nurbs`].
    pub fn rectangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let hw = width * 0.5;
        let hh = height * 0.5;
        let pts = [
            Point3::new(-hw, -hh, 0.0),
            Point3::new(hw, -hh, 0.0),
            Point3::new(hw, hh, 0.0),
            Point3::new(-hw, hh, 0.0),
            Point3::new(-hw, -hh, 0.0),
        ];
        let rect = NurbsCurve2D::polyline(&pts, true, /*normalise=*/);
        Self::from_exterior(rect, metadata)
    }

    /// Convenience: circle of given `radius` discretised with `segments`
    /// quadratic NURBS (centripetal parameterisation).
    pub fn circle(radius: Real, segments: usize) -> Self {
        use nalgebra::{Point2, Vector2};
        let center = Point2::origin();
        let x_axis = Vector2::x();
        let y_axis = Vector2::y();
        let circle =
            NurbsCurve2D::try_circle(&center, &x_axis, &y_axis, radius).expect("circle");
        Self::from_exterior(circle, None)
    }

    /// Wrap a single closed curve as exterior (no holes).
    pub fn from_exterior(exterior: NurbsCurve2D<Real>, metadata: Option<S>) -> Self {
        let region = Region::new(CompoundCurve::from(exterior), vec![]);
        Self {
            region,
            bbox: OnceLock::new(),
            metadata,
        }
    }

    /// Internal helper: build from a full [`Region<Real>`].
    fn from_region(region: Region<Real>, metadata: Option<S>) -> Self {
        Self {
            region,
            bbox: OnceLock::new(),
            metadata,
        }
    }

    /// Collect all control points (de‑homogenised) as 3‑D points.
    fn points_3d(&self) -> Vec<Point3<Real>> {
        self.region
            .exterior()
            .spans()
            .iter()
            .flat_map(|span| span.dehomogenized_control_points())
            .map(|p| Point3::new(p.x, p.y, 0.0))
            .collect()
    }
}

/* =============================================================================
 * Core boolean + transform behaviour – we simply delegate to Curvo
 * ============================================================================= */

impl<S: Clone + Send + Sync + std::fmt::Debug> CSGOps for Nurbs<S> {
    fn new() -> Self {
        // An *empty* region has no polygons; represent with a degenerate square
        // of zero area.
        Self::rectangle(0.0, 0.0, None)
    }

    fn union(&self, other: &Self) -> Self {
        let clip = self
            .region
            .boolean(BooleanOperation::Union, &other.region, None)
            .expect("boolean union failed");
        // `boolean` might return multiple disjoint regions; for now, keep only
        // the first (most common case). Extend if multi‑region support needed.
        let mut regions = clip.into_regions();
        let region = regions
            .pop()
            .unwrap_or_else(|| Region::new(CompoundCurve::new(vec![]), vec![]));
        Self::from_region(region, self.metadata.clone())
    }

    fn difference(&self, other: &Self) -> Self {
        let clip = self
            .region
            .boolean(BooleanOperation::Difference, &other.region, None)
            .expect("boolean difference failed");
        let mut regions = clip.into_regions();
        let region = regions
            .pop()
            .unwrap_or_else(|| Region::new(CompoundCurve::new(vec![]), vec![]));
        Self::from_region(region, self.metadata.clone())
    }

    fn intersection(&self, other: &Self) -> Self {
        let clip = self
            .region
            .boolean(BooleanOperation::Intersection, &other.region, None)
            .expect("boolean intersection failed");
        let mut regions = clip.into_regions();
        let region = regions
            .pop()
            .unwrap_or_else(|| Region::new(CompoundCurve::new(vec![]), vec![]));
        Self::from_region(region, self.metadata.clone())
    }

    fn xor(&self, other: &Self) -> Self {
        // XOR = (A \ B) ∪ (B \ A)
        let a_sub_b = self.difference(other);
        let b_sub_a = other.difference(self);
        a_sub_b.union(&b_sub_a)
    }

    fn transform(&self, mat: &Matrix4<Real>) -> Self {
        use curvo::prelude::Transformable;
        let mut region = self.region.clone();
        region.transform(mat);
        Self::from_region(region, self.metadata.clone())
    }

    fn bounding_box(&self) -> Aabb {
        *self.bbox.get_or_init(|| {
            // Project all 3‑D points (they all live at Z=0) and compute min/max.
            let points = self.points_3d();
            if points.is_empty() {
                return Aabb::new(Point3::origin(), Point3::origin());
            }
            let (mut min_x, mut min_y, mut max_x, mut max_y) =
                (Real::MAX, Real::MAX, -Real::MAX, -Real::MAX);
            for p in &points {
                if p.x < min_x {
                    min_x = p.x;
                }
                if p.y < min_y {
                    min_y = p.y;
                }
                if p.x > max_x {
                    max_x = p.x;
                }
                if p.y > max_y {
                    max_y = p.y;
                }
            }
            let mins = Point3::new(min_x, min_y, 0.0);
            let maxs = Point3::new(max_x, max_y, 0.0);
            Aabb::new(mins, maxs)
        })
    }

    fn invalidate_bounding_box(&mut self) {
        self.bbox = OnceLock::new();
    }

    fn inverse(&self) -> Self {
        use curvo::prelude::Invertible;
        let mut region = self.region.clone();
        region.invert();
        Self::from_region(region, self.metadata.clone())
    }
}
