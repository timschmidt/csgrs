//! `Profile` struct and implementations of the `CSGOps` trait for `Profile`

use crate::hyper_math::{
    Aabb, Real, hreal_from_f64, hreal_mul, hreal_sign, hreal_sum, hrotation_between_vectors,
    hunit_vector3,
};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

use crate::csg::CSG;
use crate::errors::ProfileBooleanError;
use crate::vertex::Vertex;
use hypercurve::{
    Aabb2 as HyperAabb2, BezierSubcurve2, BooleanOp, Classification, Contour2, CubicBezier2,
    Curve2, CurveGeometry2, CurvePath2, CurvePolicy, CurveRegion2,
    CurveRegionNativeContourView2, CurveResult, CurveString2, FinitePolyline2,
    FiniteProjectionOptions, FiniteRegionProfile2, LineSeg2, NurbsCurve2, Point2,
    PolynomialSplineCurve2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    RegionPointLocation, Segment2, Similarity2,
};
use hyperlattice::{Matrix4, Point3, Vector3};
use hyperreal::RealSign;
use std::fmt::Debug;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};

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

static NEXT_PROFILE_ID: AtomicU64 = AtomicU64::new(1);

fn fresh_profile_id() -> u64 {
    NEXT_PROFILE_ID.fetch_add(1, Ordering::Relaxed)
}

/// Position transform plus rotation from the sketch's local XY plane into 3D.
///
/// The rotation is a finite homogeneous matrix assembled from hyperlattice
/// unit-vector/dot/cross predicates.
/// This keeps sketch lifting aligned with Yap's exact-geometric-computation
/// boundary discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) type OriginTransform = (Vector3, Matrix4);

#[derive(Debug, Clone)]
pub struct GraphicLineString {
    pub points: Vec<[Real; 3]>,
}

impl GraphicLineString {
    pub const fn line_count(&self) -> usize {
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
pub struct Profile {
    identity: u64,
    pub(crate) convex_tessellation: Option<Arc<Vec<[Real; 2]>>>,
    pub(crate) convex_edge_normals: Option<Arc<Vec<Vector3>>>,

    /// Authoritative unified hypercurve region for 2D CAD topology.
    ///
    /// Filled Profile topology is owned by this region. Topology-sensitive
    /// operations should use this region and hypercurve predicates. This
    /// follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) region: CurveRegion2,

    /// Native open hypercurve wires for line/path sketches.
    ///
    /// Closed areas belong in [`Profile::region`]. Open paths are represented as
    /// `CurveString2`, so text strokes, SVG paths, and Hilbert infill compose
    /// with hypercurve directly.
    pub(crate) wires: Vec<CurveString2>,

    /// Open higher-order paths that cannot be represented by [`CurveString2`].
    pub(crate) curve_wires: Vec<CurvePath2>,

    /// Lazily calculated AABB that spans the finite boundary projection.
    pub(crate) bounding_box: OnceLock<Aabb>,

    /// Origin of the sketch in 3D space.
    pub(crate) origin: Vertex,

    pub(crate) origin_transform: OriginTransform,
}

impl Profile {
    /// Return a new empty sketch.
    pub fn empty() -> Self {
        Profile {
            identity: fresh_profile_id(),
            convex_tessellation: None,
            convex_edge_normals: None,
            region: CurveRegion2::empty(),
            wires: Vec::new(),
            curve_wires: Vec::new(),
            bounding_box: OnceLock::new(),
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// True when the authoritative hypercurve topology is empty.
    ///
    /// Using `CurveRegion2`, `CurveString2`, and `CurvePath2` as the
    /// authoritative emptiness test
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn is_empty(&self) -> bool {
        !self.has_topology()
    }

    /// Project the hypercurve region into finite material profiles with
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

    /// Project the hypercurve region to hypercurve-owned finite profiles.
    ///
    /// This is the preferred finite boundary API for Profile consumers that need
    /// polygon-with-holes records. The return type is
    /// [`hypercurve::FiniteRegionProfile2`], so csgrs does not define another
    /// CAD topology container at this layer. Projection remains a lossy
    /// `f64` boundary product with certificate data owned by hypercurve; exact
    /// predicates and topology stay in [`CurveRegion2`]. This follows Yap, "Towards
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

    /// Decided hypercurve-region containment for a hyperreal XY point.
    ///
    /// Returns `None` when the point lies on a certified boundary or when
    /// hypercurve cannot decide the classification under the certified policy.
    ///
    /// Coordinates are accepted only as `Real`; primitive numbers
    /// must be promoted explicitly at API/IO boundaries before querying native
    /// [`Point2`] topology. This follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn contains_xy(&self, x: Real, y: Real) -> Option<bool> {
        if self.filled_is_empty() {
            return None;
        }
        let point = Point2::new(x, y);
        let classification = self
            .region
            .classify_point(&point, &CurvePolicy::certified())
            .ok()?;
        match classification {
            Classification::Decided(RegionPointLocation::Inside) => Some(true),
            Classification::Decided(RegionPointLocation::Outside) => Some(false),
            Classification::Decided(RegionPointLocation::Boundary)
            | Classification::Uncertain(_) => None,
        }
    }

    /// Certified bounds of all native hypercurve topology in XY.
    ///
    /// This merges the filled [`CurveRegion2`] and open [`CurveString2`] and
    /// [`CurvePath2`] wires directly. Hypercurve AABBs preserve exact curve
    /// geometry without tessellating just to find extrema. Bounding boxes are still only
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
            match self.region.bounds(&policy) {
                Ok(Classification::Decided(bbox)) => Some(bbox),
                Ok(Classification::Uncertain(_)) | Err(_) => return None,
            }
        };

        for wire in &self.wires {
            let wire_box = match HyperAabb2::from_curve_string(wire, &policy) {
                Ok(Classification::Decided(bbox)) => bbox,
                Ok(Classification::Uncertain(_)) | Err(_) => return None,
            };
            native_box = match native_box {
                Some(current) => match current.union(&wire_box, &policy) {
                    Classification::Decided(merged) => Some(merged),
                    Classification::Uncertain(_) => return None,
                },
                None => Some(wire_box),
            };
        }

        for path in &self.curve_wires {
            let path_box = path.bounds().ok()?.clone();
            native_box = match native_box {
                Some(current) => match current.union(&path_box, &policy) {
                    Classification::Decided(merged) => Some(merged),
                    Classification::Uncertain(_) => return None,
                },
                None => Some(path_box),
            };
        }

        native_box.as_ref().map(hyper_aabb_to_xy_bounds)
    }

    fn projected_xy_bounds(&self) -> Option<(Real, Real, Real, Real)> {
        let options = FiniteProjectionOptions::try_new(1e-3).ok()?;
        let profiles = match self.project_region_profiles(&options).ok()? {
            Classification::Decided(profiles) => profiles,
            Classification::Uncertain(_) => return None,
        };
        let wires = self.project_wire_polylines(&options);
        let mut bounds: Option<(f64, f64, f64, f64)> = None;
        let mut include = |[x, y]: [f64; 2]| {
            bounds = Some(match bounds {
                Some((min_x, min_y, max_x, max_y)) => {
                    (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
                },
                None => (x, y, x, y),
            });
        };
        for profile in &profiles {
            for point in profile.material().points() {
                include(*point);
            }
            for hole in profile.holes() {
                for point in hole.points() {
                    include(*point);
                }
            }
        }
        for wire in &wires {
            for point in wire.points() {
                include(*point);
            }
        }
        let (min_x, min_y, max_x, max_y) = bounds?;
        Some((
            hreal_from_f64(min_x).ok()?,
            hreal_from_f64(min_y).ok()?,
            hreal_from_f64(max_x).ok()?,
            hreal_from_f64(max_y).ok()?,
        ))
    }

    /// Decide whether a native region has any material contour with nonzero
    /// area.
    ///
    /// `CurveRegion2::filled_area` keeps material-minus-hole area semantics and
    /// Green's-theorem contour accumulation stays inside hypercurve. Only a
    /// certified nonzero area is accepted; uncertain classifications fail
    /// closed without a finite projection fallback, following Yap's
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn region_has_nonzero_area(region: &CurveRegion2) -> bool {
        match region.filled_area(&CurvePolicy::certified()) {
            Ok(Classification::Decided(Some(area))) => {
                if matches!(hreal_sign(&area), Some(RealSign::Zero)) {
                    return false;
                }
                true
            },
            Ok(Classification::Decided(None)) | Ok(Classification::Uncertain(_)) | Err(_) => {
                false
            },
        }
    }

    /// Create a Profile from one native hypercurve contour.
    ///
    /// This is the preferred constructor when callers already have exact
    /// hypercurve boundary topology. `csgrs` does not mirror the contour with
    /// a second polygon datatype; it promotes the contour into the unified
    /// `CurveRegion2` carrier and keeps all topology in hypercurve, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_contour(contour: Contour2) -> Profile {
        let region = CurveRegion2::try_from_native_material_contours(
            vec![contour],
            &CurvePolicy::certified(),
        )
        .unwrap_or_else(|_| CurveRegion2::empty());
        Self::from_curve_region(region)
    }

    fn from_topology_with_origin(
        region: CurveRegion2,
        wires: Vec<CurveString2>,
        curve_wires: Vec<CurvePath2>,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Profile {
        Profile {
            identity: fresh_profile_id(),
            convex_tessellation: None,
            convex_edge_normals: None,
            region,
            wires,
            curve_wires,
            bounding_box: OnceLock::new(),
            origin,
            origin_transform,
        }
    }

    /// Create a Profile from a full higher-order Hypercurve region.
    pub fn from_curve_region(region: CurveRegion2) -> Profile {
        Self::from_curve_region_and_paths(region, Vec::new())
    }

    /// Create a Profile from higher-order filled topology and native open wires.
    pub fn from_curve_region_and_wires(
        region: CurveRegion2,
        wires: Vec<CurveString2>,
    ) -> Profile {
        Self::from_curve_topology(region, wires, Vec::new())
    }

    /// Create a Profile from the complete unified Hypercurve topology set.
    pub fn from_curve_topology(
        region: CurveRegion2,
        wires: Vec<CurveString2>,
        paths: Vec<CurvePath2>,
    ) -> Profile {
        Self::from_topology_with_origin(
            region,
            wires,
            paths,
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        )
    }

    /// Create a Profile from higher-order filled topology and open paths.
    pub fn from_curve_region_and_paths(
        region: CurveRegion2,
        paths: Vec<CurvePath2>,
    ) -> Profile {
        Self::from_curve_topology_with_origin(
            region,
            paths,
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        )
    }

    pub(crate) fn from_curve_topology_with_origin(
        region: CurveRegion2,
        paths: Vec<CurvePath2>,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Profile {
        Self::from_topology_with_origin(region, Vec::new(), paths, origin, origin_transform)
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
    pub fn from_wires(wires: Vec<CurveString2>) -> Profile {
        Self::from_curve_region_and_wires(CurveRegion2::empty(), wires)
    }

    /// Create a wire-only Profile from one native hypercurve curve string.
    ///
    /// This keeps open-path construction in `hypercurve::CurveString2`
    /// instead of forcing callers to go through finite line-string helpers.
    /// It follows Yap's exact-geometric-computation split between source
    /// topology and finite output boundaries
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_wire(wire: CurveString2) -> Profile {
        Self::from_wires(vec![wire])
    }

    /// Create an open-path Profile from full higher-order Hypercurve paths.
    pub fn from_curve_paths(paths: Vec<CurvePath2>) -> Profile {
        Self::from_curve_region_and_paths(CurveRegion2::default(), paths)
    }

    /// Create an open-path Profile from one full higher-order path.
    pub fn from_curve_path(path: CurvePath2) -> Profile {
        Self::from_curve_paths(vec![path])
    }

    /// Borrow the native line/arc acceleration contours, when available.
    ///
    /// General curved profiles must use [`Self::as_curve_region`]. This view
    /// avoids transferring ownership to the legacy native region container.
    pub fn native_contours(&self) -> CurveRegionNativeContourView2<'_> {
        match self
            .region
            .native_contours_fast_path(&CurvePolicy::certified())
        {
            Ok(Classification::Decided(native)) => native,
            Ok(Classification::Uncertain(_)) | Err(_) => {
                panic!("higher-order Profile topology has no native contour fast path")
            },
        }
    }

    /// Borrow the authoritative filled topology without demoting curves.
    pub const fn region_geometry(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Borrow the authoritative unified region.
    pub const fn as_curve_region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Borrow native open hypercurve wires carried by this sketch.
    pub fn wires(&self) -> &[CurveString2] {
        &self.wires
    }

    /// Borrow open higher-order paths carried by this sketch.
    pub fn curve_paths(&self) -> &[CurvePath2] {
        &self.curve_wires
    }

    pub(crate) const fn storage_identity(&self) -> u64 {
        self.identity
    }

    pub(crate) fn retain_convex_tessellation(
        &mut self,
        points: Vec<[Real; 2]>,
        edge_normals: Vec<Vector3>,
    ) {
        debug_assert_eq!(points.len(), edge_normals.len());
        self.convex_tessellation = Some(Arc::new(points));
        self.convex_edge_normals = Some(Arc::new(edge_normals));
    }

    /// Consume this profile without losing its authoritative curve carrier.
    pub fn into_curve_topology(self) -> (CurveRegion2, Vec<CurveString2>, Vec<CurvePath2>) {
        (self.region, self.wires, self.curve_wires)
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
            .filter_map(|points| {
                points
                    .into_iter()
                    .map(|point| {
                        Some([
                            hreal_from_f64(point[0]).ok()?,
                            hreal_from_f64(point[1]).ok()?,
                        ])
                    })
                    .collect::<Option<Vec<_>>>()
            })
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
            .chain(
                self.curve_wires
                    .iter()
                    .filter_map(|path| path.project_to_finite_polyline(options).ok()),
            )
            .filter(|polyline| polyline.points().len() >= 2)
            .collect()
    }

    /// Number of filled/material loops in the hypercurve region.
    pub fn material_contour_count(&self) -> usize {
        match self.region.loop_roles(&CurvePolicy::certified()) {
            Ok(Classification::Decided(roles)) => roles
                .iter()
                .filter(|role| **role == hypercurve::CurveRegionLoopRole::Material)
                .count(),
            Ok(Classification::Uncertain(_)) | Err(_) => {
                projected_curve_region_counts(&self.region).0
            },
        }
    }

    /// Number of subtractive/hole loops in the hypercurve region.
    pub fn hole_contour_count(&self) -> usize {
        match self.region.loop_roles(&CurvePolicy::certified()) {
            Ok(Classification::Decided(roles)) => roles
                .iter()
                .filter(|role| **role == hypercurve::CurveRegionLoopRole::Hole)
                .count(),
            Ok(Classification::Uncertain(_)) | Err(_) => {
                projected_curve_region_counts(&self.region).1
            },
        }
    }

    pub(crate) fn has_topology(&self) -> bool {
        !self.filled_is_empty() || !self.wires.is_empty() || !self.curve_wires.is_empty()
    }

    pub(crate) fn filled_is_empty(&self) -> bool {
        self.region.is_empty()
    }

    fn can_use_region_boolean_with(&self, other: &Self) -> bool {
        self.has_topology() || other.has_topology()
    }

    fn boolean_wires_with(&self, other: &Self, op: BooleanOp) -> Vec<CurveString2> {
        match op {
            BooleanOp::Union | BooleanOp::Intersection | BooleanOp::Xor => {
                let mut wires = self.wires.clone();
                wires.extend(other.wires.iter().cloned());
                wires
            },
            BooleanOp::Difference => self.wires.clone(),
        }
    }

    fn boolean_curve_wires_with(&self, other: &Self, op: BooleanOp) -> Vec<CurvePath2> {
        match op {
            BooleanOp::Union | BooleanOp::Intersection | BooleanOp::Xor => {
                let mut wires = self.curve_wires.clone();
                wires.extend(other.curve_wires.iter().cloned());
                wires
            },
            BooleanOp::Difference => self.curve_wires.clone(),
        }
    }

    fn boolean_region_with(
        &self,
        other: &Self,
        op: BooleanOp,
    ) -> Result<Self, ProfileBooleanError> {
        if !self.can_use_region_boolean_with(other) {
            return Ok(Self::from_topology_with_origin(
                CurveRegion2::empty(),
                Vec::new(),
                Vec::new(),
                self.origin.clone(),
                self.origin_transform.clone(),
            ));
        }
        let policy = CurvePolicy::certified();
        let region = self.region.boolean_region(&other.region, op, &policy)?;
        // Region booleans operate on filled topology only. Open `CurveString2`
        // wires are independent path topology, so preserve them as native
        // curve strings while keeping the filled result in hypercurve; this
        // matches Yap's exact-object
        // boundary by avoiding a finite detour solely to retain wires.
        let wires = self.boolean_wires_with(other, op);
        let mut sketch = Self::from_topology_with_origin(
            region,
            wires,
            self.boolean_curve_wires_with(other, op),
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        );
        sketch.origin = self.origin.clone();
        sketch.origin_transform = self.origin_transform.clone();
        Ok(sketch)
    }

    fn transformed_wires_with_matrix(&self, mat: &Matrix4) -> Vec<CurveString2> {
        if let Some(wires) = self
            .wires
            .iter()
            .map(|wire| transform_line_curve_string(wire, mat))
            .collect::<Option<Vec<_>>>()
        {
            return wires;
        }
        if let Some(transform) = planar_similarity_from_matrix(mat) {
            return self
                .wires
                .iter()
                .filter_map(|wire| wire.transform_similarity(&transform).ok())
                .collect();
        }
        Vec::new()
    }

    fn transformed_curve_topology_with_matrix(
        &self,
        mat: &Matrix4,
    ) -> hypercurve::ExactCurveResult<(CurveRegion2, Vec<CurvePath2>)> {
        let region = transform_materialized_curve_region(&self.region, mat)?;
        let paths = self
            .curve_wires
            .iter()
            .map(|path| transform_curve_path_with_matrix(path, mat))
            .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?;
        Ok((region, paths))
    }

    /// Set the origin used when rendering or lifting this sketch into 3D.
    pub fn set_origin(&mut self, origin: Vertex) {
        self.origin_transform = Self::prepare_origin_transform(origin.clone());
        self.origin = origin;
        self.identity = fresh_profile_id();
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
        let origin_normal =
            hunit_vector3(&origin.normal).unwrap_or_else(|| default_normal.clone());
        let rotation = hrotation_between_vectors(&default_normal, &origin_normal)
            .unwrap_or_else(Matrix4::identity);

        (pos_transform, rotation)
    }

    pub(crate) fn apply_origin_transform_vertex(
        vertex: Vertex,
        origin_transform: OriginTransform,
    ) -> Vertex {
        let (pos_transform, rotation) = origin_transform;
        let rotated_position = rotation
            .transform_point3(&vertex.position)
            .unwrap_or_else(|_| vertex.position.clone());
        let position = rotated_position + pos_transform;
        let rotated_normal = rotation.transform_direction3(&vertex.normal);
        let normal = hunit_vector3(&rotated_normal).unwrap_or_else(Vector3::z);
        Vertex::new(position, normal)
    }

    fn apply_origin_transform_point(
        point: Point3,
        origin_transform: OriginTransform,
    ) -> Point3 {
        let (pos_transform, rotation) = origin_transform;
        let rotated = rotation
            .transform_point3(&point)
            .unwrap_or_else(|_| point.clone());
        rotated + pos_transform
    }

    /// Create render-ready line strings from this sketch's visible edges in 3D space.
    ///
    /// Region- and wire-backed sketches project visible boundaries directly
    /// from hypercurve regions and paths. The source topology remains
    /// `CurveRegion2`, `CurveString2`, and `CurvePath2`. If topology is absent, display output is empty
    /// rather than reconstructed from stale finite boundary products. This
    /// follows Yap's exact-geometric-computation
    /// split (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>) and the finite-output
    /// discipline in Hobby, "Practical Segment Intersection with Finite
    /// Precision Output," Computational Geometry 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn build_graphic_line_strings(&self) -> GraphicLineStrings {
        let mut graphic_line_strings = Vec::new();

        if self.has_topology() {
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

    fn ring_to_graphic_line_string(&self, ring: &[[f64; 2]]) -> GraphicLineString {
        let points = ring
            .iter()
            .filter_map(|coord| {
                let point = Point3::new(
                    hreal_from_f64(coord[0]).ok()?,
                    hreal_from_f64(coord[1]).ok()?,
                    Real::zero(),
                );
                let point_transformed =
                    Self::apply_origin_transform_point(point, self.origin_transform.clone());
                Some([
                    point_transformed.x,
                    point_transformed.y,
                    point_transformed.z,
                ])
            })
            .collect();
        GraphicLineString { points }
    }

    /// Triangulate the hypercurve region in this Profile.
    ///
    /// Filled topology is projected from [`CurveRegion2`] into hypercurve-owned finite profiles, then
    /// triangulated with hypertri using hyperreal predicates at the mesh boundary.
    /// This keeps exact topology authoritative as advocated by Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while the ear-clipping
    /// step follows Meisters, "Polygons Have Ears," *American Mathematical
    /// Monthly* 82(6), 1975 (<https://doi.org/10.2307/2319703>).
    ///
    /// # Returns
    ///
    /// A `Vec<[Point3; 3]>` containing all the triangles resulting from the triangulation.
    pub fn triangulate(&self) -> Vec<[Point3; 3]> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        let mut all_triangles = Vec::new();
        for profile in profiles {
            all_triangles.extend(
                profile
                    .triangulate()
                    .unwrap_or_default()
                    .into_iter()
                    .filter_map(|tri| {
                        Some([
                            Point3::new(
                                hreal_from_f64(tri[0][0]).ok()?,
                                hreal_from_f64(tri[0][1]).ok()?,
                                Real::zero(),
                            ),
                            Point3::new(
                                hreal_from_f64(tri[1][0]).ok()?,
                                hreal_from_f64(tri[1][1]).ok()?,
                                Real::zero(),
                            ),
                            Point3::new(
                                hreal_from_f64(tri[2][0]).ok()?,
                                hreal_from_f64(tri[2][1]).ok()?,
                                Real::zero(),
                            ),
                        ])
                    }),
            );
        }

        all_triangles
    }

    /// Return a copy of this `Profile` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    ///
    /// Hypercurve sketches already retain loop roles and carry open paths, so
    /// renormalization preserves the unified region/wire topology without
    /// rebuilding through winding-sensitive finite polygons. If no topology
    /// exists, renormalization returns empty topology.
    /// For polygon winding conventions and point-in-polygon background, see
    /// Hormann and Agathos, "The point in polygon problem for arbitrary
    /// polygons," Computational Geometry 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>). Keeping exact topology
    /// internal follows Yap, "Towards Exact Geometric Computation,"
    /// Computational Geometry 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn renormalize(&self) -> Profile {
        if self.has_topology() {
            return Self::from_topology_with_origin(
                self.region.clone(),
                self.wires.clone(),
                self.curve_wires.clone(),
                self.origin.clone(),
                self.origin_transform.clone(),
            );
        }

        Self::from_topology_with_origin(
            CurveRegion2::empty(),
            Vec::new(),
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        )
    }
}

fn projected_curve_region_counts(region: &CurveRegion2) -> (usize, usize) {
    let Ok(options) = FiniteProjectionOptions::try_new(1.0e-3) else {
        return (0, 0);
    };
    match region.project_to_finite_profiles(&options, &CurvePolicy::certified()) {
        Ok(Classification::Decided(profiles)) => {
            let holes = profiles.iter().map(|profile| profile.holes().len()).sum();
            (profiles.len(), holes)
        },
        Ok(Classification::Uncertain(_)) | Err(_) => (0, 0),
    }
}

fn transform_materialized_curve_region(
    region: &CurveRegion2,
    mat: &Matrix4,
) -> hypercurve::ExactCurveResult<CurveRegion2> {
    if region.is_empty() {
        return Ok(CurveRegion2::empty());
    }
    if let Some(transform) = planar_similarity_from_matrix(mat) {
        return region.transform_similarity(&transform, &CurvePolicy::certified());
    }
    region.transform_affine(
        &mat[0][0],
        &mat[0][1],
        &mat[1][0],
        &mat[1][1],
        &mat[0][3],
        &mat[1][3],
        &CurvePolicy::certified(),
    )
}

fn transform_curve_path_with_matrix(
    path: &CurvePath2,
    mat: &Matrix4,
) -> hypercurve::ExactCurveResult<CurvePath2> {
    let mut transformed = Vec::new();
    for curve in path.curves() {
        transformed.extend(transform_curve_with_matrix(curve, mat)?);
    }
    CurvePath2::try_new(transformed)
}

fn transform_curve_with_matrix(
    curve: &Curve2,
    mat: &Matrix4,
) -> hypercurve::ExactCurveResult<Vec<Curve2>> {
    let invalid = |cause| {
        hypercurve::ExactCurveError::invalid(
            hypercurve::CurveOperation2::Transformation,
            curve.family(),
            curve.source(),
            cause,
        )
    };
    let transformed = match curve.geometry() {
        CurveGeometry2::Line(line) => vec![Curve2::from(
            LineSeg2::try_new(
                transform_point2(line.start(), mat).ok_or_else(|| {
                    invalid(hypercurve::CurveError::InvalidSimilarityTransform)
                })?,
                transform_point2(line.end(), mat).ok_or_else(|| {
                    invalid(hypercurve::CurveError::InvalidSimilarityTransform)
                })?,
            )
            .map_err(invalid)?,
        )],
        CurveGeometry2::CircularArc(_) => {
            if let Some(transform) = planar_similarity_from_matrix(mat) {
                return curve
                    .transform_similarity(&transform)
                    .map(|curve| vec![curve]);
            }
            curve
                .native_bezier_fragments()?
                .iter()
                .map(|fragment| {
                    transform_bezier_subcurve_with_matrix(fragment.curve(), mat)
                        .map(Curve2::from)
                })
                .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?
        },
        CurveGeometry2::QuadraticBezier(curve) => vec![Curve2::from(QuadraticBezier2::new(
            transformed_point(curve.start(), mat, &invalid)?,
            transformed_point(curve.control(), mat, &invalid)?,
            transformed_point(curve.end(), mat, &invalid)?,
        ))],
        CurveGeometry2::CubicBezier(curve) => vec![Curve2::from(CubicBezier2::new(
            transformed_point(curve.start(), mat, &invalid)?,
            transformed_point(curve.control1(), mat, &invalid)?,
            transformed_point(curve.control2(), mat, &invalid)?,
            transformed_point(curve.end(), mat, &invalid)?,
        ))],
        CurveGeometry2::RationalQuadraticBezier(curve) => vec![Curve2::from(
            RationalQuadraticBezier2::try_new(
                transformed_point(curve.start(), mat, &invalid)?,
                transformed_point(curve.control(), mat, &invalid)?,
                transformed_point(curve.end(), mat, &invalid)?,
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )
            .map_err(invalid)?,
        )],
        CurveGeometry2::RationalBezier(curve) => vec![Curve2::from(
            RationalBezier2::try_new(
                curve
                    .control_points()
                    .iter()
                    .map(|point| transformed_point(point, mat, &invalid))
                    .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?,
                curve.weights().to_vec(),
            )
            .map_err(invalid)?,
        )],
        CurveGeometry2::PolynomialBSpline(curve) => {
            vec![Curve2::from(PolynomialSplineCurve2::try_new(
                curve.degree(),
                curve
                    .control_points()
                    .iter()
                    .map(|point| transformed_point(point, mat, &invalid))
                    .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?,
                curve.knots().to_vec(),
            )?)]
        },
        CurveGeometry2::Nurbs(curve) => vec![Curve2::from(NurbsCurve2::try_new(
            curve.degree(),
            curve
                .control_points()
                .iter()
                .map(|point| transformed_point(point, mat, &invalid))
                .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?,
            curve.weights().to_vec(),
            curve.knots().to_vec(),
        )?)],
    };
    Ok(transformed)
}

fn transform_bezier_subcurve_with_matrix(
    curve: &BezierSubcurve2,
    mat: &Matrix4,
) -> hypercurve::ExactCurveResult<BezierSubcurve2> {
    let family = Curve2::from(curve.clone()).family();
    let invalid = |cause| {
        hypercurve::ExactCurveError::invalid(
            hypercurve::CurveOperation2::Transformation,
            family,
            None,
            cause,
        )
    };
    match curve {
        BezierSubcurve2::Quadratic(curve) => {
            Ok(BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                transformed_point(curve.start(), mat, &invalid)?,
                transformed_point(curve.control(), mat, &invalid)?,
                transformed_point(curve.end(), mat, &invalid)?,
            )))
        },
        BezierSubcurve2::Cubic(curve) => Ok(BezierSubcurve2::Cubic(CubicBezier2::new(
            transformed_point(curve.start(), mat, &invalid)?,
            transformed_point(curve.control1(), mat, &invalid)?,
            transformed_point(curve.control2(), mat, &invalid)?,
            transformed_point(curve.end(), mat, &invalid)?,
        ))),
        BezierSubcurve2::RationalQuadratic(curve) => Ok(BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(
                transformed_point(curve.start(), mat, &invalid)?,
                transformed_point(curve.control(), mat, &invalid)?,
                transformed_point(curve.end(), mat, &invalid)?,
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )
            .map_err(invalid)?,
        )),
        BezierSubcurve2::Rational(curve) => Ok(BezierSubcurve2::Rational(
            RationalBezier2::try_new(
                curve
                    .control_points()
                    .iter()
                    .map(|point| transformed_point(point, mat, &invalid))
                    .collect::<hypercurve::ExactCurveResult<Vec<_>>>()?,
                curve.weights().to_vec(),
            )
            .map_err(invalid)?,
        )),
    }
}

fn transformed_point(
    point: &Point2,
    mat: &Matrix4,
    invalid: &impl Fn(hypercurve::CurveError) -> hypercurve::ExactCurveError,
) -> hypercurve::ExactCurveResult<Point2> {
    transform_point2(point, mat)
        .ok_or_else(|| invalid(hypercurve::CurveError::InvalidSimilarityTransform))
}

fn transform_line_curve_string(curve: &CurveString2, mat: &Matrix4) -> Option<CurveString2> {
    let segments = curve
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => LineSeg2::try_new(
                transform_point2(line.start(), mat)?,
                transform_point2(line.end(), mat)?,
            )
            .ok()
            .map(Segment2::Line),
            Segment2::Arc(_) => None,
        })
        .collect::<Option<Vec<_>>>()?;
    CurveString2::try_new(segments).ok()
}

fn transform_point2(point: &Point2, mat: &Matrix4) -> Option<Point2> {
    Some(Point2::new(
        hreal_sum(&[
            hreal_mul(&mat[0][0], point.x())?,
            hreal_mul(&mat[0][1], point.y())?,
            mat[0][3].clone(),
        ])?,
        hreal_sum(&[
            hreal_mul(&mat[1][0], point.x())?,
            hreal_mul(&mat[1][1], point.y())?,
            mat[1][3].clone(),
        ])?,
    ))
}

fn planar_similarity_from_matrix(mat: &Matrix4) -> Option<Similarity2> {
    Similarity2::try_from_real_affine(
        mat[0][0].clone(),
        mat[0][1].clone(),
        mat[1][0].clone(),
        mat[1][1].clone(),
        mat[0][3].clone(),
        mat[1][3].clone(),
    )
    .ok()
}

impl Profile {
    /// Compute a certified profile union.
    pub fn try_union(&self, other: &Self) -> Result<Self, ProfileBooleanError> {
        self.boolean_region_with(other, BooleanOp::Union)
    }

    /// Compute a certified profile difference.
    pub fn try_difference(&self, other: &Self) -> Result<Self, ProfileBooleanError> {
        self.boolean_region_with(other, BooleanOp::Difference)
    }

    /// Compute a certified profile intersection.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, ProfileBooleanError> {
        self.boolean_region_with(other, BooleanOp::Intersection)
    }

    /// Compute a certified profile symmetric difference.
    pub fn try_xor(&self, other: &Self) -> Result<Self, ProfileBooleanError> {
        self.boolean_region_with(other, BooleanOp::Xor)
    }
}

impl Profile {
    /// Return a new empty sketch.
    pub fn new() -> Self {
        Self::empty()
    }
}

fn hyper_aabb_to_xy_bounds(bbox: &HyperAabb2) -> (Real, Real, Real, Real) {
    (
        bbox.min_x().clone(),
        bbox.min_y().clone(),
        bbox.max_x().clone(),
        bbox.max_y().clone(),
    )
}

impl CSG for Profile {
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
    fn union(&self, other: &Profile) -> Profile {
        self.try_union(other)
            .unwrap_or_else(|error| panic!("profile union failed: {error}"))
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
    fn difference(&self, other: &Profile) -> Profile {
        self.try_difference(other)
            .unwrap_or_else(|error| panic!("profile difference failed: {error}"))
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
    fn intersection(&self, other: &Profile) -> Profile {
        self.try_intersection(other)
            .unwrap_or_else(|error| panic!("profile intersection failed: {error}"))
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
    fn xor(&self, other: &Profile) -> Profile {
        self.try_xor(other)
            .unwrap_or_else(|error| panic!("profile xor failed: {error}"))
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to this sketch.
    ///
    /// Closed area sketches with a non-singular XY affine part are transformed
    /// directly through the authoritative [`CurveRegion2`]. Native line/arc
    /// regions retain their optimized representation when possible, while
    /// higher-order curves remain higher-order under affine transforms.
    /// Projection-style or singular transforms fail closed. The exact/topological boundary follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>); the affine projection
    /// is the ordinary homogeneous-coordinate convention described by Foley et
    /// al., *Computer Graphics: Principles and Practice*, 2nd ed., 1990.
    fn transform(&self, mat: &Matrix4) -> Profile {
        let (region, curve_wires) = self
            .transformed_curve_topology_with_matrix(mat)
            .unwrap_or_else(|error| panic!("profile curve transform failed: {error}"));
        Self::from_topology_with_origin(
            region,
            self.transformed_wires_with_matrix(mat),
            curve_wires,
            self.origin.clone(),
            self.origin_transform.clone(),
        )
    }

    /// Returns an axis-aligned bounding box containing the effective finite
    /// 2D boundary projection, interpreted at z=0.
    ///
    /// Hypercurve sketches compute bounds from `CurveRegion2` and their paths'
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
        self.bounding_box
            .get_or_init(|| {
                if let Some((min_x, min_y, max_x, max_y)) = self.native_xy_bounds() {
                    return Aabb::new(
                        Point3::new(min_x, min_y, Real::zero()),
                        Point3::new(max_x, max_y, Real::zero()),
                    );
                }

                if let Some((min_x, min_y, max_x, max_y)) = self.projected_xy_bounds() {
                    return Aabb::new(
                        Point3::new(min_x, min_y, Real::zero()),
                        Point3::new(max_x, max_y, Real::zero()),
                    );
                }

                Aabb::origin()
            })
            .clone()
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
        self.convex_tessellation = None;
        self.convex_edge_normals = None;
        self.identity = fresh_profile_id();
    }

    /// Return the topology-preserving Profile inverse for native hypercurve data.
    ///
    /// Profile topology is owned by hypercurve [`CurveRegion2`],
    /// [`CurveString2`], and [`CurvePath2`], where material/hole semantics are explicit and not
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
    fn inverse(&self) -> Profile {
        if self.has_topology() {
            return Self::from_topology_with_origin(
                self.region.clone(),
                self.wires.clone(),
                self.curve_wires.clone(),
                self.origin.clone(),
                self.origin_transform.clone(),
            );
        }

        Self::from_topology_with_origin(
            CurveRegion2::empty(),
            Vec::new(),
            Vec::new(),
            self.origin.clone(),
            self.origin_transform.clone(),
        )
    }
}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for Profile {
    fn from(mesh: Mesh<M>) -> Self {
        mesh.flatten()
    }
}
