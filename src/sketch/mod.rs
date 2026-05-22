//! `Profile` struct and implementations of the `CSGOps` trait for `Profile`

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{
    Real, hreal_cmp_f64, hreal_from_f64, hreal_mul, hreal_sign, hreal_sub, hreal_sum,
    hreal_to_f64, hrotation_between_vectors, hunit_vector3,
};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

use crate::csg::CSG;
use crate::vertex::Vertex;
use hypercurve::{
    Aabb2 as HyperAabb2, BooleanOp, Classification, Contour2, CurvePolicy, CurveResult,
    CurveString2, FillRule, FinitePolyline2, FiniteProjectionOptions, FiniteRegionProfile2,
    Point2, Region2, RegionPointLocation,
};
use nalgebra::{Matrix4, Point3, Vector3};
use std::cmp::Ordering;
use std::fmt::Debug;
use std::sync::OnceLock;

pub mod extrudes;
pub mod shapes;

#[cfg(feature = "hershey-text")]
pub mod hershey;

#[cfg(feature = "image-io")]
pub mod image;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "truetype-text")]
pub mod truetype;

pub mod triangulated;

/// Position transform plus rotation from the sketch's local XY plane into 3D.
///
/// The rotation is a finite homogeneous matrix assembled from hyperlattice
/// unit-vector/dot/cross predicates instead of a nalgebra unit quaternion.
/// This keeps sketch lifting aligned with Yap's exact-geometric-computation
/// boundary discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) type OriginTransform = (Vector3<Real>, Matrix4<Real>);

#[derive(Debug, Clone)]
pub struct GraphicLineString {
    pub points: Vec<[f32; 3]>,
}

impl GraphicLineString {
    pub fn line_count(&self) -> usize {
        self.points.len().saturating_sub(1)
    }
}

#[derive(Debug, Clone)]
pub struct GraphicLineStrings {
    pub line_strings: Vec<GraphicLineString>,
}

impl GraphicLineStrings {
    pub fn total_line_count(&self) -> usize {
        self.line_strings
            .iter()
            .map(GraphicLineString::line_count)
            .sum()
    }
}

#[derive(Clone, Debug)]
pub struct Profile<M> {
    /// Primary hypercurve region for 2D CAD topology.
    ///
    /// Filled Profile topology is owned by this region. Topology-sensitive
    /// operations should use this region and hypercurve predicates. This
    /// follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) region: Region2,

    /// Native open hypercurve wires for line/path sketches.
    ///
    /// Closed areas belong in [`Profile::region`]. Open paths are represented as
    /// `CurveString2`, so text strokes, SVG paths, Hilbert infill, and toolpath
    /// helpers compose with hypercurve directly.
    pub(crate) wires: Vec<CurveString2>,

    /// Lazily calculated AABB that spans the finite boundary projection.
    pub(crate) bounding_box: OnceLock<Aabb>,

    /// Whole-sketch metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub(crate) metadata: M,

    /// Origin of the sketch in 3D space.
    pub(crate) origin: Vertex,

    pub(crate) origin_transform: OriginTransform,
}

impl<M: Clone + Send + Sync + Debug> Profile<M> {
    /// Return a new empty sketch with explicit metadata.
    pub fn empty(metadata: M) -> Self {
        Profile {
            region: Region2::empty(),
            wires: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// True when the native hypercurve topology is empty.
    ///
    /// Using `Region2` and `CurveString2` as the authoritative emptiness test
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn is_empty(&self) -> bool {
        !self.has_native_topology()
    }

    /// Project the native hypercurve region into finite material profiles with
    /// their owned holes.
    ///
    /// This is the preferred API-boundary view for algorithms and file formats
    /// that consume polygon-with-holes records. The returned records are
    /// [`hypercurve::FiniteRegionProfile2`] values, so csgrs does not mirror
    /// hypercurve's CAD boundary type with its own wrapper. Hole ownership is
    /// decided before finite coordinates are emitted, following the
    /// point-in-polygon topology discussed by Hormann and Agathos, "The point
    /// in polygon problem for arbitrary polygons," *Computational Geometry*
    /// 20(3), 2001 (<https://doi.org/10.1016/S0925-7721(01)00012-8>).
    /// Keeping this projection separate from the source topology follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
    /// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn region_profiles(&self) -> Vec<FiniteRegionProfile2> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        }
    }

    /// Project the native hypercurve region to hypercurve-owned finite profiles.
    ///
    /// This is the preferred finite boundary API for Profile consumers that need
    /// polygon-with-holes records. The return type is
    /// [`hypercurve::FiniteRegionProfile2`], so csgrs does not define another
    /// CAD topology container at this layer. Projection remains a lossy
    /// `f64` boundary product with certificate data owned by hypercurve; exact
    /// predicates and topology stay in [`Region2`]. This follows Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
    /// discipline in Hobby, "Practical Segment Intersection with Finite
    /// Precision Output," *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn project_region_profiles(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<Classification<Vec<FiniteRegionProfile2>>> {
        self.region
            .project_to_finite_profiles(options, &CurvePolicy::certified())
    }

    /// Decided hypercurve-region containment for a promotable XY point.
    ///
    /// Returns `None` when the point lies on a certified boundary or when
    /// hypercurve cannot decide the classification under the certified policy.
    ///
    /// Coordinates are promoted through `hyperreal::Real` before constructing a
    /// native [`Point2`], keeping primitive numbers at the query boundary. This
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn contains_xy<X, Y>(&self, x: X, y: Y) -> Option<bool>
    where
        X: TryInto<hyperreal::Real>,
        Y: TryInto<hyperreal::Real>,
    {
        if self.region.is_empty() {
            return None;
        }
        let point = Point2::new(x.try_into().ok()?, y.try_into().ok()?);
        match self.region.classify_point(&point, &CurvePolicy::certified()) {
            Classification::Decided(RegionPointLocation::Inside) => Some(true),
            Classification::Decided(RegionPointLocation::Outside) => Some(false),
            Classification::Decided(RegionPointLocation::Boundary)
            | Classification::Uncertain(_) => None,
        }
    }

    /// Bounds of all native hypercurve topology projected to finite XY.
    ///
    /// This merges filled [`Region2`] contours and open [`CurveString2`] wires
    /// directly. Native hypercurve AABBs preserve line and circular-arc geometry without
    /// tessellating just to find extrema. Bounding boxes are still only
    /// broad-phase/API-edge data; exact topology stays in hyper geometry,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the certified
    /// broad-phase role follows Bentley and Ottmann, "Algorithms for Reporting
    /// and Counting Geometric Intersections," *IEEE Transactions on Computers*
    /// C-28(9), 1979.
    pub(crate) fn native_xy_bounds(&self) -> Option<(Real, Real, Real, Real)> {
        let policy = CurvePolicy::certified();
        let mut native_box = if self.region.is_empty() {
            None
        } else {
            match HyperAabb2::from_region(&self.region, &policy) {
                Ok(Classification::Decided(bbox)) => Some(bbox),
                Ok(Classification::Uncertain(_)) | Err(_) => {
                    return self.projected_native_xy_bounds();
                },
            }
        };

        for wire in &self.wires {
            let wire_box = match HyperAabb2::from_curve_string(wire, &policy) {
                Ok(Classification::Decided(bbox)) => bbox,
                Ok(Classification::Uncertain(_)) | Err(_) => {
                    return self.projected_native_xy_bounds();
                },
            };
            native_box = match native_box {
                Some(current) => match current.union(&wire_box, &policy) {
                    Classification::Decided(merged) => Some(merged),
                    Classification::Uncertain(_) => return self.projected_native_xy_bounds(),
                },
                None => Some(wire_box),
            };
        }

        native_box
            .as_ref()
            .and_then(hyper_aabb_to_finite_xy_bounds)
            .or_else(|| self.projected_native_xy_bounds())
    }

    fn projected_native_xy_bounds(&self) -> Option<(Real, Real, Real, Real)> {
        let options = FiniteProjectionOptions::try_new(1e-3).ok()?;
        let region_profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        let wire_polylines = self.project_wire_polylines(&options);
        let mut iter = region_profiles
            .iter()
            .flat_map(|profile| {
                std::iter::once(profile.material().points())
                    .chain(profile.holes().iter().map(|hole| hole.points()))
            })
            .flat_map(|ring| ring.iter())
            .chain(wire_polylines.iter().flat_map(|wire| wire.points()));
        let first = iter.next()?;
        hreal_from_f64(first[0]).ok()?;
        hreal_from_f64(first[1]).ok()?;
        let (mut min_x, mut min_y, mut max_x, mut max_y) =
            (first[0], first[1], first[0], first[1]);
        for point in iter {
            hreal_from_f64(point[0]).ok()?;
            hreal_from_f64(point[1]).ok()?;
            if matches!(hreal_cmp_f64(point[0], min_x), Ordering::Less) {
                min_x = point[0];
            }
            if matches!(hreal_cmp_f64(point[1], min_y), Ordering::Less) {
                min_y = point[1];
            }
            if matches!(hreal_cmp_f64(point[0], max_x), Ordering::Greater) {
                max_x = point[0];
            }
            if matches!(hreal_cmp_f64(point[1], max_y), Ordering::Greater) {
                max_y = point[1];
            }
        }
        Some((min_x, min_y, max_x, max_y))
    }

    /// Decide whether a native region has any material contour with nonzero
    /// area.
    ///
    /// `Region2::filled_area` keeps material-minus-hole area semantics and
    /// Green's-theorem contour accumulation inside hypercurve. Only exact zero
    /// area is rejected; finite projected rings are only a fallback for contours
    /// that cannot yet report exact area, preserving the exact-computation
    /// boundary described by Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) fn region_has_nonzero_area(region: &Region2) -> bool {
        match region.filled_area(&CurvePolicy::certified()) {
            Ok(Classification::Decided(Some(area))) => {
                if matches!(hreal_sign(&area), Some(hyperreal::RealSign::Zero)) {
                    return false;
                }
                return true;
            },
            Ok(Classification::Decided(None)) | Ok(Classification::Uncertain(_)) | Err(_) => {
            },
        }

        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance");
        match region.project_to_finite_profiles(&options, &CurvePolicy::certified()) {
            Ok(Classification::Decided(profiles)) => profiles.iter().any(|profile| {
                !matches!(
                    hreal_cmp_f64(profile.projected_filled_area(), 0.0),
                    Ordering::Equal
                )
            }),
            Ok(Classification::Uncertain(_)) | Err(_) => false,
        }
    }

    /// Create a Profile from a native hypercurve region.
    ///
    /// This is the preferred constructor for topology-producing code. Core CAD
    /// topology remains in [`Profile::region`]. Keeping exact topology internal
    /// and emitting finite
    /// coordinates only at API and file-format boundaries follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and Hobby,
    /// "Practical Segment Intersection with Finite Precision Output,"
    /// *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn from_region(region: Region2, metadata: M) -> Profile<M> {
        Self::from_region_and_wires(region, Vec::new(), metadata)
    }

    /// Create a Profile from one native hypercurve contour.
    ///
    /// This is the preferred constructor when callers already have exact
    /// hypercurve boundary topology. `csgrs` does not mirror the contour with
    /// a second polygon datatype; it wraps the contour in a `Region2` and keeps
    /// all topology in hypercurve, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_contour(contour: Contour2, metadata: M) -> Profile<M> {
        Self::from_region(Region2::from_material_contours(vec![contour]), metadata)
    }

    /// Create a Profile from a native hypercurve region and open curve strings.
    ///
    /// This is the preferred constructor for mixed area/path CAD. It composes
    /// Profile from hyper geometry directly: filled topology remains in
    /// `Region2`, and open paths remain in `CurveString2`. This follows
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>), and
    /// avoids making finite boundary products the source of CAD topology.
    pub fn from_region_and_wires(
        region: Region2,
        wires: Vec<CurveString2>,
        metadata: M,
    ) -> Profile<M> {
        Self::from_region_and_wires_with_origin(
            region,
            wires,
            metadata,
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        )
    }

    pub(crate) fn from_region_and_wires_with_origin(
        region: Region2,
        wires: Vec<CurveString2>,
        metadata: M,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Profile<M> {
        Profile {
            region,
            wires,
            bounding_box: OnceLock::new(),
            metadata,
            origin,
            origin_transform,
        }
    }

    /// Append native open wires and invalidate cached finite bounds.
    ///
    /// This keeps modules such as offsetting and importers composing with the
    /// `CurveString2` list directly. That ownership split follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[cfg(feature = "offset")]
    pub(crate) fn append_native_wires<I>(&mut self, wires: I)
    where
        I: IntoIterator<Item = CurveString2>,
    {
        self.wires.extend(wires);
        self.invalidate_bounding_box();
    }

    /// Create an open-path Profile from native hypercurve curve strings.
    ///
    /// This is the preferred constructor for path-producing code such as text
    /// strokes and infill curves. Ownership of the path topology remains in
    /// `CurveString2`. Keeping finite coordinates at API/export boundaries while internal
    /// predicates and topology use exact objects follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_wires(wires: Vec<CurveString2>, metadata: M) -> Profile<M> {
        Self::from_region_and_wires(Region2::empty(), wires, metadata)
    }

    /// Create a wire-only Profile from one native hypercurve curve string.
    ///
    /// This keeps open-path construction in `hypercurve::CurveString2`
    /// instead of forcing callers to go through finite line-string helpers.
    /// It follows Yap's exact-geometric-computation split between source
    /// topology and finite output boundaries
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_wire(wire: CurveString2, metadata: M) -> Profile<M> {
        Self::from_wires(vec![wire], metadata)
    }

    /// Borrow the native hypercurve region that now carries Profile topology.
    pub fn as_region(&self) -> &Region2 {
        &self.region
    }

    /// Borrow native open hypercurve wires carried by this sketch.
    pub fn wires(&self) -> &[CurveString2] {
        &self.wires
    }

    /// Consume this sketch into its native hypercurve CAD topology and metadata.
    ///
    /// This is the ownership counterpart to [`Profile::as_region`] and
    /// [`Profile::wires`]. Callers that need to pass Profile topology into other
    /// hyper crates can take the [`Region2`] and [`CurveString2`] values
    /// directly. Primitive `f32`/`f64` data remains restricted to
    /// API and file-format boundaries; internal topology stays in hypercurve
    /// objects backed by hyperreal predicates, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn into_region_and_wires(self) -> (Region2, Vec<CurveString2>, M) {
        (self.region, self.wires, self.metadata)
    }

    /// Project native open wires into finite XY polylines.
    ///
    /// The returned coordinates are for renderers, CAM post-processing, and
    /// other API edges. Internal path ownership remains in `CurveString2`.
    pub fn wire_polylines(&self) -> Vec<Vec<[Real; 2]>> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        self.project_wire_polylines(&options)
            .into_iter()
            .map(FinitePolyline2::into_points)
            .filter(|points| points.len() >= 2)
            .collect()
    }

    /// Project native open wires to hypercurve-owned finite polylines.
    ///
    /// This is the API-boundary view of open Profile paths. It keeps exact path
    /// ownership in [`CurveString2`] and delegates all sampling,
    /// finite export, and projection certificates to hypercurve. That mirrors
    /// Yap's exact-object boundary model (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>) while allowing renderers
    /// and file formats to consume primitive `f64` points explicitly.
    pub fn project_wire_polylines(
        &self,
        options: &FiniteProjectionOptions,
    ) -> Vec<FinitePolyline2> {
        self.wires
            .iter()
            .filter_map(|wire| wire.project_to_finite_polyline(options).ok())
            .filter(|polyline| polyline.points().len() >= 2)
            .collect()
    }

    /// Number of filled/material contours in the native hypercurve region.
    pub fn material_contour_count(&self) -> usize {
        self.region.material_contours().len()
    }

    /// Number of subtractive/hole contours in the native hypercurve region.
    pub fn hole_contour_count(&self) -> usize {
        self.region.hole_contours().len()
    }

    pub(crate) fn has_native_topology(&self) -> bool {
        !self.region.is_empty() || !self.wires.is_empty()
    }

    fn can_use_region_boolean_with(&self, other: &Self) -> bool {
        self.has_native_topology() || other.has_native_topology()
    }

    fn boolean_wires_with(&self, other: &Self, op: BooleanOp) -> Vec<CurveString2> {
        let mut wires = match op {
            BooleanOp::Union | BooleanOp::Intersection | BooleanOp::Xor => {
                let mut wires = self.wires.clone();
                wires.extend(other.wires.iter().cloned());
                wires
            },
            BooleanOp::Difference => self.wires.clone(),
        };
        wires.retain(|wire| curve_string_to_polyline(wire).is_some());
        wires
    }

    /// Resolve certified-disjoint region booleans without entering a finite
    /// polygon clipping backend.
    ///
    /// `Aabb2::overlaps` is inclusive, so tangent or shared-boundary cases still
    /// proceed to the full hypercurve boolean pipeline. Only decided misses use
    /// the algebraic identities here: disjoint union/xor are role-preserving
    /// region merges, difference is the left region, and intersection is empty.
    /// This broad-phase split follows Bentley and Ottmann, "Algorithms for
    /// Reporting and Counting Geometric Intersections," *IEEE Transactions on
    /// Computers* C-28(9), 1979, while keeping exact topology in `Region2`
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn disjoint_region_boolean_with(
        &self,
        other: &Self,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> Option<Region2> {
        if self.region.is_empty() || other.region.is_empty() {
            return None;
        }
        let lhs = match HyperAabb2::from_region(&self.region, policy).ok()? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(_) => return None,
        };
        let rhs = match HyperAabb2::from_region(&other.region, policy).ok()? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(_) => return None,
        };
        match lhs.overlaps(&rhs, policy) {
            Classification::Decided(true) | Classification::Uncertain(_) => None,
            Classification::Decided(false) => {
                let region = match op {
                    BooleanOp::Union | BooleanOp::Xor => {
                        let mut material = self.region.material_contours().to_vec();
                        material.extend(other.region.material_contours().iter().cloned());
                        let mut holes = self.region.hole_contours().to_vec();
                        holes.extend(other.region.hole_contours().iter().cloned());
                        Region2::new(material, holes)
                    },
                    BooleanOp::Difference => self.region.clone(),
                    BooleanOp::Intersection => Region2::empty(),
                };
                Some(region)
            },
        }
    }

    fn boolean_region_with(&self, other: &Self, op: BooleanOp) -> Option<Self> {
        if !self.can_use_region_boolean_with(other) {
            return None;
        }
        let policy = CurvePolicy::certified();
        let region =
            if let Some(region) = self.disjoint_region_boolean_with(other, op, &policy) {
                region
            } else {
                match self
                    .region
                    .boolean_region(&other.region, op, FillRule::NonZero, &policy)
                    .ok()?
                {
                    Classification::Decided(region) => region,
                    Classification::Uncertain(_) => return None,
                }
            };
        // Region booleans operate on filled topology only. Open `CurveString2`
        // wires are independent path topology, so preserve them as native
        // curve strings while keeping the filled result in hypercurve; this
        // matches Yap's exact-object
        // boundary by avoiding a finite detour solely to retain wires.
        let wires = self.boolean_wires_with(other, op);
        let mut sketch = Self::from_region_and_wires(region, wires, self.metadata.clone());
        sketch.origin = self.origin;
        sketch.origin_transform = self.origin_transform;
        Some(sketch)
    }

    fn transformed_region_with_matrix(&self, mat: &Matrix4<Real>) -> Option<Region2> {
        if self.region.is_empty() || !Self::region_has_nonzero_area(&self.region) {
            return None;
        }
        let determinant = hreal_mul(mat[(0, 0)], mat[(1, 1)]).and_then(|main| {
            hreal_mul(mat[(0, 1)], mat[(1, 0)]).and_then(|cross| hreal_sub(main, cross))
        })?;
        if !matches!(
            hreal_cmp_f64(determinant, 0.0),
            Ordering::Greater | Ordering::Less
        ) {
            return None;
        }

        let transform_point = |point: &[Real; 2]| -> Option<[Real; 2]> {
            Some([
                hreal_sum(&[
                    hreal_mul(mat[(0, 0)], point[0])?,
                    hreal_mul(mat[(0, 1)], point[1])?,
                    mat[(0, 3)],
                ])?,
                hreal_sum(&[
                    hreal_mul(mat[(1, 0)], point[0])?,
                    hreal_mul(mat[(1, 1)], point[1])?,
                    mat[(1, 3)],
                ])?,
            ])
        };

        let options = FiniteProjectionOptions::try_new(1e-3).ok()?;
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => return None,
        };
        let material = profiles
            .iter()
            .map(|profile| {
                profile
                    .material()
                    .points()
                    .iter()
                    .map(transform_point)
                    .collect::<Option<Vec<[Real; 2]>>>()
            })
            .filter_map(|ring| ring.and_then(|ring| Contour2::from_finite_ring(&ring).ok()))
            .collect::<Vec<_>>();
        let holes = profiles
            .iter()
            .flat_map(|profile| profile.holes().iter())
            .map(|ring| {
                ring.points()
                    .iter()
                    .map(transform_point)
                    .collect::<Option<Vec<[Real; 2]>>>()
            })
            .filter_map(|ring| ring.and_then(|ring| Contour2::from_finite_ring(&ring).ok()))
            .collect::<Vec<_>>();

        (!material.is_empty()).then(|| Region2::new(material, holes))
    }

    fn transformed_wires_with_matrix(&self, mat: &Matrix4<Real>) -> Vec<CurveString2> {
        let transform_point = |point: (Real, Real)| -> Option<[Real; 2]> {
            Some([
                hreal_sum(&[
                    hreal_mul(mat[(0, 0)], point.0)?,
                    hreal_mul(mat[(0, 1)], point.1)?,
                    mat[(0, 3)],
                ])?,
                hreal_sum(&[
                    hreal_mul(mat[(1, 0)], point.0)?,
                    hreal_mul(mat[(1, 1)], point.1)?,
                    mat[(1, 3)],
                ])?,
            ])
        };

        self.wires
            .iter()
            .filter_map(curve_string_to_polyline)
            .filter_map(|points| {
                let transformed = points
                    .into_iter()
                    .map(transform_point)
                    .collect::<Option<Vec<[Real; 2]>>>()?;
                CurveString2::from_finite_point_iter(transformed).ok()
            })
            .collect()
    }

    /// Return this sketch with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Profile<NewM> {
        Profile {
            region: self.region,
            wires: self.wires,
            bounding_box: OnceLock::new(),
            metadata,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Map this sketch's metadata while preserving its geometry and origin.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, f: F) -> Profile<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Profile {
            region: self.region,
            wires: self.wires,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Borrow this sketch's metadata without exposing topology internals.
    ///
    /// Profile geometry is owned by hypercurve [`Region2`] and [`CurveString2`]
    /// fields behind constructor/accessor APIs. Metadata remains the caller's
    /// domain payload, so this accessor mirrors [`crate::polygon::Polygon`]
    /// while allowing the Profile storage layout to keep moving toward
    /// hypercurve-owned CAD topology. Keeping payload data separate from exact
    /// geometric objects follows Yap's exact-geometric-computation split
    /// between combinatorial/topological structure and finite API data:
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub const fn metadata(&self) -> &M {
        &self.metadata
    }

    /// Mutably borrow this sketch's metadata.
    ///
    /// This changes only caller-owned payload data; hypercurve region and wire
    /// topology remain accessible through dedicated Profile APIs.
    pub const fn metadata_mut(&mut self) -> &mut M {
        &mut self.metadata
    }

    /// Replace this sketch's metadata in place.
    pub fn set_metadata(&mut self, metadata: M) {
        self.metadata = metadata;
    }

    /// Set the origin used when rendering or lifting this sketch into 3D.
    pub fn set_origin(&mut self, origin: Vertex) {
        self.origin_transform = Self::prepare_origin_transform(origin);
        self.origin = origin;
    }

    /// Return this sketch with a new 3D origin.
    pub fn origin(mut self, origin: Vertex) -> Self {
        self.set_origin(origin);
        self
    }

    pub(crate) fn prepare_origin_transform(origin: Vertex) -> OriginTransform {
        let default_origin = Vertex::default();
        let pos_transform = origin.position - default_origin.position;
        let default_normal = hunit_vector3(&default_origin.normal).unwrap_or_else(Vector3::z);
        let origin_normal = hunit_vector3(&origin.normal).unwrap_or(default_normal);
        let rotation = hrotation_between_vectors(&default_normal, &origin_normal)
            .unwrap_or_else(Matrix4::identity);

        (pos_transform, rotation)
    }

    pub(crate) fn apply_origin_transform_vertex(
        vertex: Vertex,
        origin_transform: OriginTransform,
    ) -> Vertex {
        let (pos_transform, rotation) = origin_transform;
        let rotated_position =
            Point3::from_homogeneous(rotation * vertex.position.to_homogeneous())
                .unwrap_or(vertex.position);
        let position = Point3::from(rotated_position.coords + pos_transform);
        let rotated_normal = rotation * vertex.normal.push(0.0);
        let normal = hunit_vector3(&Vector3::new(
            rotated_normal.x,
            rotated_normal.y,
            rotated_normal.z,
        ))
        .unwrap_or_else(Vector3::z);
        Vertex::new(position, normal)
    }

    fn apply_origin_transform_point(
        point: Point3<Real>,
        origin_transform: OriginTransform,
    ) -> Point3<Real> {
        let (pos_transform, rotation) = origin_transform;
        let rotated =
            Point3::from_homogeneous(rotation * point.to_homogeneous()).unwrap_or(point);
        Point3::from(rotated.coords + pos_transform)
    }

    /// Create render-ready line strings from this sketch's visible edges in 3D space.
    ///
    /// Region- and wire-backed sketches project visible boundaries directly
    /// from hypercurve contours and curve strings. Finite `f32` output is a
    /// renderer boundary, while the source topology remains `Region2` and
    /// `CurveString2`. If native topology is absent, display output is empty
    /// rather than reconstructed from stale finite boundary products. This
    /// follows Yap's exact-geometric-computation
    /// split (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>) and the finite-output
    /// discipline in Hobby, "Practical Segment Intersection with Finite
    /// Precision Output," Computational Geometry 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn build_graphic_line_strings(&self) -> GraphicLineStrings {
        let mut graphic_line_strings = Vec::new();

        if !self.region.is_empty() || !self.wires.is_empty() {
            let options = FiniteProjectionOptions::try_new(1e-3)
                .expect("positive finite projection tolerance is valid");
            let profiles = match self.project_region_profiles(&options) {
                Ok(Classification::Decided(profiles)) => profiles,
                Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
            };
            for profile in &profiles {
                for ring in std::iter::once(profile.material().points())
                    .chain(profile.holes().iter().map(|hole| hole.points()))
                {
                    graphic_line_strings.push(self.ring_to_graphic_line_string(ring));
                }
            }
            for wire in self.project_wire_polylines(&options) {
                graphic_line_strings.push(self.ring_to_graphic_line_string(wire.points()));
            }
            return GraphicLineStrings {
                line_strings: graphic_line_strings,
            };
        }

        GraphicLineStrings {
            line_strings: graphic_line_strings,
        }
    }

    fn ring_to_graphic_line_string(&self, ring: &[[Real; 2]]) -> GraphicLineString {
        let points = ring
            .iter()
            .map(|coord| {
                let point = Point3::new(coord[0], coord[1], 0.0);
                let point_transformed =
                    Self::apply_origin_transform_point(point, self.origin_transform);
                [
                    point_transformed.x as f32,
                    point_transformed.y as f32,
                    point_transformed.z as f32,
                ]
            })
            .collect();
        GraphicLineString { points }
    }

    /// Triangulate the native hypercurve region in this Profile.
    ///
    /// Filled topology is projected from [`Region2`] into hypercurve-owned finite profiles, then
    /// triangulated with hypertri using hyperreal predicates at the mesh boundary.
    /// This keeps exact topology authoritative as advocated by Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while the ear-clipping
    /// step follows Meisters, "Polygons Have Ears," *American Mathematical
    /// Monthly* 82(6), 1975 (<https://doi.org/10.2307/2319703>).
    ///
    /// # Returns
    ///
    /// A `Vec<[Point3<Real>; 3]>` containing all the triangles resulting from the triangulation.
    pub fn triangulate(&self) -> Vec<[Point3<Real>; 3]> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        let mut all_triangles = Vec::new();
        for profile in profiles {
            all_triangles.extend(profile.triangulate().unwrap_or_default().into_iter().map(
                |tri| {
                    [
                        Point3::new(tri[0][0], tri[0][1], 0.0),
                        Point3::new(tri[1][0], tri[1][1], 0.0),
                        Point3::new(tri[2][0], tri[2][1], 0.0),
                    ]
                },
            ));
        }

        all_triangles
    }

    /// Return a copy of this `Profile` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    ///
    /// Native hypercurve sketches already separate material and hole contours,
    /// and carry open paths as `CurveString2`, so renormalization preserves the
    /// native `Region2`/wire topology without rebuilding through winding-sensitive
    /// finite polygons. If no native topology exists, renormalization returns
    /// empty native topology rather than promoting boundary data into CAD topology.
    /// For polygon winding conventions and point-in-polygon background, see
    /// Hormann and Agathos, "The point in polygon problem for arbitrary
    /// polygons," Computational Geometry 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>). Keeping exact topology
    /// internal follows Yap, "Towards Exact Geometric Computation,"
    /// Computational Geometry 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn renormalize(&self) -> Profile<M> {
        if !self.wires.is_empty()
            || (!self.region.is_empty() && Self::region_has_nonzero_area(&self.region))
        {
            let mut sketch = Self::from_region_and_wires(
                self.region.clone(),
                self.wires.clone(),
                self.metadata.clone(),
            );
            sketch.origin = self.origin;
            sketch.origin_transform = self.origin_transform;
            return sketch;
        }

        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            Vec::new(),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }
}

impl Profile<()> {
    /// Return a new empty sketch with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

fn hyper_aabb_to_finite_xy_bounds(bbox: &HyperAabb2) -> Option<(Real, Real, Real, Real)> {
    Some((
        hreal_to_f64(bbox.min_x())?,
        hreal_to_f64(bbox.min_y())?,
        hreal_to_f64(bbox.max_x())?,
        hreal_to_f64(bbox.max_y())?,
    ))
}

fn curve_string_to_polyline(wire: &CurveString2) -> Option<Vec<(Real, Real)>> {
    let options = FiniteProjectionOptions::try_new(1e-3).ok()?;
    Some(
        wire.project_to_finite_polyline(&options)
            .ok()?
            .into_points()
            .into_iter()
            .map(|point| (point[0], point[1]))
            .collect(),
    )
}

impl<M: Clone + Send + Sync + Debug> CSG for Profile<M> {
    /// Return a new Profile representing union of the two Sketches.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Profile<M>) -> Profile<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Union) {
            return sketch;
        }
        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            self.boolean_wires_with(other, BooleanOp::Union),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Profile representing diffarence of the two Sketches.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Profile<M>) -> Profile<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Difference) {
            return sketch;
        }
        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            self.boolean_wires_with(other, BooleanOp::Difference),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Profile representing intersection of the two Sketches.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Profile<M>) -> Profile<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Intersection) {
            return sketch;
        }
        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            self.boolean_wires_with(other, BooleanOp::Intersection),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Profile representing space in this Profile excluding the space in the
    /// other Profile plus the space in the other Profile excluding the space in this Profile.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Profile<M>) -> Profile<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Xor) {
            return sketch;
        }
        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            self.boolean_wires_with(other, BooleanOp::Xor),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to this sketch.
    ///
    /// Closed area sketches with a non-singular XY affine part are transformed
    /// from the native hypercurve region: finite boundary rings are projected at
    /// the API edge, the XY affine part of the matrix is applied, and the result
    /// is promoted immediately back to `hypercurve::Region2`. This keeps
    /// topology in hyperreal-backed objects. Native open wires use the same finite affine
    /// boundary and are immediately rebuilt as `CurveString2`. Projection-style
    /// or singular transforms return empty native topology when they cannot be
    /// represented by the current hypercurve similarity path. The exact/topological boundary follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>); the affine projection
    /// is the ordinary homogeneous-coordinate convention described by Foley et
    /// al., *Computer Graphics: Principles and Practice*, 2nd ed., 1990.
    fn transform(&self, mat: &Matrix4<Real>) -> Profile<M> {
        if let Some(region) = self.transformed_region_with_matrix(mat) {
            // Open wires follow the same finite projection boundary as region
            // rings, but rebuild directly as `CurveString2` instead of routing
            // through another line-string datatype. Keeping the ownership on
            // hypercurve after the affine API boundary follows Yap, "Towards
            // Exact Geometric Computation," Computational Geometry 7(1-2),
            // 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
            let wires = self.transformed_wires_with_matrix(mat);
            return Self::from_region_and_wires_with_origin(
                region,
                wires,
                self.metadata.clone(),
                self.origin,
                self.origin_transform,
            );
        }

        if self.region.is_empty() && !self.wires.is_empty() {
            let wires = self.transformed_wires_with_matrix(mat);
            return Self::from_region_and_wires_with_origin(
                Region2::empty(),
                wires,
                self.metadata.clone(),
                self.origin,
                self.origin_transform,
            );
        }

        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            Vec::new(),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Returns an axis-aligned bounding box containing the effective finite
    /// 2D boundary projection, interpreted at z=0.
    ///
    /// Native hypercurve sketches compute bounds from `Region2`/`CurveString2`
    /// AABBs and only fall back to finite projection when hypercurve cannot
    /// decide a bound directly. The finite AABB is an acceleration/output boundary, while topological
    /// ownership remains in hyper geometry; this follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The finite projection
    /// fallback is a simple coordinate min/max envelope, the standard broad-phase box
    /// primitive described by Gottschalk, Lin, and Manocha, "OBBTree: A
    /// Hierarchical Structure for Rapid Interference Detection," SIGGRAPH 1996
    /// (<https://doi.org/10.1145/237170.237244>).
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            if let Some((min_x, min_y, max_x, max_y)) = self.native_xy_bounds() {
                return Aabb::new(
                    Point3::new(min_x, min_y, 0.0),
                    Point3::new(max_x, max_y, 0.0),
                );
            }

            Aabb::new(Point3::origin(), Point3::origin())
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Return the topology-preserving Profile inverse for native hypercurve data.
    ///
    /// Native Profile topology is owned by hypercurve [`Region2`] and
    /// [`CurveString2`], where material/hole semantics are explicit and not
    /// inferred from temporary ring winding. Inverse is therefore a
    /// topology-preserving operation for the hypercurve-backed Profile type.
    /// Keeping orientation out of the
    /// CAD ownership model follows the point-in-polygon topology treatment by
    /// Hormann and Agathos, "The point in polygon problem for arbitrary
    /// polygons," *Computational Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>), and keeps finite
    /// coordinates at API boundaries as advocated by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn inverse(&self) -> Profile<M> {
        if !self.wires.is_empty()
            || (!self.region.is_empty() && Self::region_has_nonzero_area(&self.region))
        {
            return Self::from_region_and_wires_with_origin(
                self.region.clone(),
                self.wires.clone(),
                self.metadata.clone(),
                self.origin,
                self.origin_transform,
            );
        }

        Self::from_region_and_wires_with_origin(
            Region2::empty(),
            Vec::new(),
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }
}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for Profile<M> {
    fn from(mesh: Mesh<M>) -> Self {
        mesh.flatten()
    }
}
