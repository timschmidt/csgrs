//! `Sketch` struct and implementations of the `CSGOps` trait for `Sketch`

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{
    PI, Real, hreal_from_f64, hreal_gt_f64, hreal_lt_f64, hreal_sign, hreal_to_f64, tolerance,
};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

use crate::csg::CSG;
use crate::vertex::Vertex;
use geo::algorithm::winding_order::Winding;
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, Geometry,
    GeometryCollection, Line, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, Rect,
    orient::Direction,
};
use hypercurve::{
    Aabb2 as HyperAabb2, BooleanOp, Classification, Contour2, CurvePolicy, CurveResult,
    CurveString2, FillRule, FinitePolyline2, FiniteProjectionOptions, FiniteRegionProfile2,
    Point2, Region2, RegionPointLocation, Similarity2, finite_ring_signed_area,
};
use nalgebra::{Matrix4, Point3, UnitQuaternion, Vector3, partial_max, partial_min};
use std::borrow::Cow;
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
pub(crate) type OriginTransform = (Vector3<Real>, UnitQuaternion<Real>);

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
pub struct Sketch<M> {
    /// Primary hypercurve region for 2D CAD topology.
    ///
    /// The legacy `geo` geometry collection is retained temporarily as an
    /// interop cache while constructors, IO, text, and toolpath modules migrate.
    /// New topology-sensitive operations should use this region and hypercurve
    /// predicates. This follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub region: Region2,

    /// Native open hypercurve wires for line/path sketches.
    ///
    /// Closed areas belong in [`Sketch::region`]. Open paths are represented as
    /// `CurveString2` while the crate migrates text, SVG paths, Hilbert infill,
    /// and toolpath helpers away from the temporary `geo` cache.
    pub wires: Vec<CurveString2>,

    /// Temporary finite `geo` interop cache.
    ///
    /// This is deliberately crate-private: the public Sketch topology is the
    /// hypercurve [`Region2`]. Consumers that still need finite `geo` output
    /// should call [`Sketch::geometry`], which derives a cache from the native
    /// region when needed.
    pub(crate) geometry: GeometryCollection<Real>,

    /// Lazily calculated AABB that spans the finite boundary projection.
    pub bounding_box: OnceLock<Aabb>,

    /// Whole-sketch metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub metadata: M,

    /// Origin of the sketch in 3D space.
    pub(crate) origin: Vertex,

    pub(crate) origin_transform: OriginTransform,
}

impl<M: Clone + Send + Sync + Debug> Sketch<M> {
    /// Return a new empty sketch with explicit metadata.
    pub fn empty(metadata: M) -> Self {
        Sketch {
            region: Region2::empty(),
            wires: Vec::new(),
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// Project this sketch into a finite [`geo::MultiPolygon`] compatibility view.
    ///
    /// Native hypercurve regions are projected directly to rings before falling
    /// back to the temporary compatibility cache. This keeps the `geo` type at
    /// the API edge rather than making it the source of sketch topology, in line
    /// with Yap's exact-geometric-computation boundary split (Computational
    /// Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        if let Ok(Classification::Decided(profiles)) = self.project_region_profiles(&options) {
            if !profiles.is_empty() {
                return MultiPolygon(polygons_from_finite_region_profiles(&profiles));
            }
        }

        let geometry = self.effective_geometry();

        let polygons = geometry
            .iter()
            .flat_map(|geom| match geom {
                Geometry::Polygon(poly) => vec![poly.clone()],
                Geometry::MultiPolygon(mp) => mp.0.clone(),
                _ => vec![],
            })
            .collect();

        MultiPolygon(polygons)
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
    /// This is the preferred finite boundary API for Sketch consumers that need
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

    /// Decided hypercurve-region containment for a finite XY point.
    ///
    /// Returns `None` when the point lies on a certified boundary or when
    /// hypercurve cannot decide the classification under the certified policy.
    pub fn contains_xy(&self, x: Real, y: Real) -> Option<bool> {
        if self.region.is_empty() {
            return None;
        }
        let point = Point2::new(hreal_from_f64(x).ok()?, hreal_from_f64(y).ok()?);
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
    /// before any temporary `geo` compatibility cache is consulted. Native
    /// hypercurve AABBs preserve line and circular-arc geometry without
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
        let (mut min_x, mut min_y, mut max_x, mut max_y) =
            (first[0], first[1], first[0], first[1]);
        for point in iter {
            min_x = min_x.min(point[0]);
            min_y = min_y.min(point[1]);
            max_x = max_x.max(point[0]);
            max_y = max_y.max(point[1]);
        }
        Some((min_x, min_y, max_x, max_y))
    }

    pub(crate) fn effective_geometry(&self) -> Cow<'_, GeometryCollection<Real>> {
        if !self.wires.is_empty()
            || !self.region.is_empty()
                && (self.geometry.0.is_empty() || Self::region_has_nonzero_area(&self.region))
        {
            Cow::Owned(Self::geometry_from_native(&self.region, &self.wires))
        } else {
            Cow::Borrowed(&self.geometry)
        }
    }

    /// Finite `geo` compatibility view of this sketch.
    ///
    /// The returned geometry is an API-boundary projection of the native
    /// hypercurve topology, not the authoritative CAD representation. This
    /// split follows the exact-geometric-computation boundary advocated by Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
    /// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>): predicates and
    /// topology stay in exact objects, while finite coordinates are emitted for
    /// interop and display.
    pub fn geometry(&self) -> Cow<'_, GeometryCollection<Real>> {
        self.effective_geometry()
    }

    /// Create a Sketch from a finite `geo::GeometryCollection` API boundary.
    ///
    /// The input collection is treated as interop data, not as Sketch's
    /// authoritative topology. Closed rings and open linework are promoted to
    /// native `Region2`/`CurveString2` when possible, and the finite geometry
    /// cache is then regenerated from those hyper types. This keeps `geo` at
    /// the boundary described by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: M) -> Sketch<M> {
        Self::from_compat_bridge_geometry_with_origin(
            geometry,
            metadata,
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        )
    }

    /// Build a Sketch from finite compatibility geometry while immediately
    /// deriving the authoritative hypercurve region.
    ///
    /// This is a migration bridge for algorithms that still emit `geo`
    /// primitives, such as offsetting and text import. New CAD topology should
    /// prefer [`Sketch::from_region`]. The conversion boundary follows Yap's
    /// exact-geometric-computation split: finite coordinates may be accepted at
    /// API and interop edges, but internal topology is represented by exact
    /// geometric objects; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) fn from_compat_geometry_with_origin(
        geometry: GeometryCollection<Real>,
        metadata: M,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Sketch<M> {
        let region = Self::region_from_geo(&geometry);
        let wires = Self::wires_from_geo(&geometry);
        Sketch {
            region,
            wires,
            geometry,
            bounding_box: OnceLock::new(),
            metadata,
            origin,
            origin_transform,
        }
    }

    /// Rebuild a Sketch from a finite compatibility result, then discard that
    /// result as the source of truth whenever it can be represented by native
    /// hypercurve topology.
    ///
    /// This is for transitional algorithms that still need `geo` to compute a
    /// finite answer, such as mixed boolean fallbacks. The returned `Sketch`
    /// owns `Region2`/`CurveString2` and regenerates the compatibility cache
    /// from those native types, matching the exact-computation boundary in Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) fn from_compat_bridge_geometry_with_origin(
        geometry: GeometryCollection<Real>,
        metadata: M,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Sketch<M> {
        let region = Self::region_from_geo(&geometry);
        let wires = Self::wires_from_geo(&geometry);
        if (region.is_empty() || !Self::region_has_nonzero_area(&region)) && wires.is_empty() {
            Self::from_compat_geometry_with_origin(
                geometry,
                metadata,
                origin,
                origin_transform,
            )
        } else {
            Self::from_region_and_wires_with_origin(
                region,
                wires,
                metadata,
                origin,
                origin_transform,
            )
        }
    }

    /// Decide whether a native region has any material contour with nonzero
    /// area.
    ///
    /// `Region2::filled_area` keeps material-minus-hole area semantics and
    /// Green's-theorem contour accumulation inside hypercurve. Finite sampled
    /// rings are only a fallback for contours that cannot yet report exact
    /// area, preserving the exact-computation boundary described by Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) fn region_has_nonzero_area(region: &Region2) -> bool {
        match region.filled_area(&CurvePolicy::certified()) {
            Ok(Classification::Decided(Some(area))) => {
                if hreal_gt_f64(&area, tolerance()) || hreal_lt_f64(&area, -tolerance()) {
                    return true;
                }
                if matches!(hreal_sign(&area), Some(hyperreal::RealSign::Zero)) {
                    return false;
                }
            },
            Ok(Classification::Decided(None)) | Ok(Classification::Uncertain(_)) | Err(_) => {
            },
        }

        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance");
        match region.project_to_finite_profiles(&options, &CurvePolicy::certified()) {
            Ok(Classification::Decided(profiles)) => profiles.iter().any(|profile| {
                finite_ring_signed_area(profile.material().points()).abs() > tolerance()
            }),
            Ok(Classification::Uncertain(_)) | Err(_) => false,
        }
    }

    /// Create a Sketch from a native hypercurve region.
    ///
    /// This is the preferred constructor for topology-producing code. The
    /// `geo` geometry collection is regenerated as a finite interop cache for
    /// IO and legacy rendering paths; core CAD topology remains in
    /// [`Sketch::region`]. Keeping exact topology internal and emitting finite
    /// coordinates only at compatibility boundaries follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and Hobby,
    /// "Practical Segment Intersection with Finite Precision Output,"
    /// *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn from_region(region: Region2, metadata: M) -> Sketch<M> {
        Self::from_region_and_wires(region, Vec::new(), metadata)
    }

    /// Create a Sketch from a native hypercurve region and open curve strings.
    ///
    /// This is the preferred constructor for mixed area/path CAD. It composes
    /// Sketch from hyper geometry directly: filled topology remains in
    /// `Region2`, open paths remain in `CurveString2`, and the temporary finite
    /// `geo` cache is regenerated only as an interop projection. This follows
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>), and
    /// avoids making the compatibility cache the source of CAD topology.
    pub fn from_region_and_wires(
        region: Region2,
        wires: Vec<CurveString2>,
        metadata: M,
    ) -> Sketch<M> {
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
    ) -> Sketch<M> {
        let geometry = Self::geometry_from_native(&region, &wires);
        Sketch {
            region,
            wires,
            geometry,
            bounding_box: OnceLock::new(),
            metadata,
            origin,
            origin_transform,
        }
    }

    /// Create an open-path Sketch from native hypercurve curve strings.
    ///
    /// This is the preferred constructor for path-producing code such as text
    /// strokes and infill curves. The finite `geo` cache is only an interop
    /// projection; ownership of the path topology remains in `CurveString2`.
    /// Keeping finite coordinates at API/export boundaries while internal
    /// predicates and topology use exact objects follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_wires(wires: Vec<CurveString2>, metadata: M) -> Sketch<M> {
        Self::from_region_and_wires(Region2::empty(), wires, metadata)
    }

    /// Build a native straight-segment contour from finite boundary
    /// coordinates supplied at the API edge.
    ///
    /// The `f64` coordinates are immediately promoted to `hyperreal::Real`; the
    /// finite points are used only as the public boundary representation. This
    /// keeps primitive Sketch constructors on the exact-predicate side of the
    /// API boundary, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub(crate) fn contour_from_points(points: &[[Real; 2]]) -> Option<Contour2> {
        Contour2::from_finite_ring(points).ok()
    }

    pub(crate) fn region_from_geo(geometry: &GeometryCollection<Real>) -> Region2 {
        fn contour_from_line_string(line_string: &LineString<Real>) -> Option<Contour2> {
            let mut points = finite_line_string_points(line_string);
            if points.len() < 3 {
                return None;
            }
            // Open polylines are native wires, not filled contours. Only closed
            // compatibility rings are promoted into Region2, matching the
            // boundary/topology separation in Yap, "Towards Exact Geometric
            // Computation," Computational Geometry 7(1-2), 1997
            // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
            let first = *points.first()?;
            let last = *points.last()?;
            let closed = (first[0] - last[0]).abs() <= tolerance()
                && (first[1] - last[1]).abs() <= tolerance();
            if !closed {
                return None;
            }
            points.pop();
            if points.len() < 3 {
                return None;
            }
            Contour2::from_finite_ring(&points).ok()
        }

        fn collect_geometry(
            geometry: &Geometry<Real>,
            material: &mut Vec<Contour2>,
            holes: &mut Vec<Contour2>,
        ) {
            match geometry {
                Geometry::Polygon(poly) => {
                    if let Some(exterior) = contour_from_line_string(poly.exterior()) {
                        material.push(exterior);
                    }
                    for interior in poly.interiors() {
                        if let Some(hole) = contour_from_line_string(interior) {
                            holes.push(hole);
                        }
                    }
                },
                Geometry::MultiPolygon(multi) => {
                    for poly in multi {
                        collect_geometry(&Geometry::Polygon(poly.clone()), material, holes);
                    }
                },
                Geometry::LineString(line_string) => {
                    if let Some(contour) = contour_from_line_string(line_string) {
                        material.push(contour);
                    }
                },
                Geometry::MultiLineString(lines) => {
                    for line_string in lines {
                        if let Some(contour) = contour_from_line_string(line_string) {
                            material.push(contour);
                        }
                    }
                },
                Geometry::Rect(rect) => {
                    collect_geometry(&Geometry::Polygon(rect.to_polygon()), material, holes);
                },
                Geometry::Triangle(triangle) => {
                    collect_geometry(
                        &Geometry::Polygon(triangle.to_polygon()),
                        material,
                        holes,
                    );
                },
                Geometry::GeometryCollection(collection) => {
                    for child in collection {
                        collect_geometry(child, material, holes);
                    }
                },
                _ => {},
            }
        }

        let mut material = Vec::new();
        let mut holes = Vec::new();
        for geometry in geometry {
            collect_geometry(geometry, &mut material, &mut holes);
        }
        Region2::new(material, holes)
    }

    pub(crate) fn geometry_from_region(region: &Region2) -> GeometryCollection<Real> {
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let polygons =
            match region.project_to_finite_profiles(&options, &CurvePolicy::certified()) {
                Ok(Classification::Decided(profiles)) => {
                    polygons_from_finite_region_profiles(&profiles)
                },
                Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
            };

        if polygons.is_empty() {
            GeometryCollection::default()
        } else {
            GeometryCollection(vec![Geometry::MultiPolygon(MultiPolygon(polygons))])
        }
    }

    fn geometry_from_native(
        region: &Region2,
        wires: &[CurveString2],
    ) -> GeometryCollection<Real> {
        let mut geometry = Self::geometry_from_region(region);
        geometry.0.extend(
            wires
                .iter()
                .filter_map(wire_to_line_string)
                .map(Geometry::LineString),
        );
        geometry
    }

    fn inverted_compat_geometry(
        geometry: &GeometryCollection<Real>,
    ) -> GeometryCollection<Real> {
        GeometryCollection(
            geometry
                .iter()
                .map(|geom| match geom {
                    Geometry::Polygon(p) => {
                        let flipped = if p.exterior().is_ccw() {
                            p.clone().orient(Direction::Reversed)
                        } else {
                            p.clone().orient(Direction::Default)
                        };
                        Geometry::Polygon(flipped)
                    },
                    Geometry::MultiPolygon(mp) => {
                        let flipped_polys: Vec<GeoPolygon<Real>> =
                            mp.0.iter()
                                .map(|p| {
                                    if p.exterior().is_ccw() {
                                        p.clone().orient(Direction::Reversed)
                                    } else {
                                        p.clone().orient(Direction::Default)
                                    }
                                })
                                .collect();

                        Geometry::MultiPolygon(MultiPolygon(flipped_polys))
                    },
                    _ => geom.clone(),
                })
                .collect(),
        )
    }

    fn wires_from_geo(geometry: &GeometryCollection<Real>) -> Vec<CurveString2> {
        fn collect_geometry(geometry: &Geometry<Real>, wires: &mut Vec<CurveString2>) {
            match geometry {
                Geometry::Line(line) => {
                    if let Some(wire) = wire_from_points([
                        [line.start.x, line.start.y],
                        [line.end.x, line.end.y],
                    ]) {
                        wires.push(wire);
                    }
                },
                Geometry::LineString(line_string) => {
                    if let Some(wire) = line_string_to_wire(line_string) {
                        wires.push(wire);
                    }
                },
                Geometry::MultiLineString(line_strings) => {
                    for line_string in line_strings {
                        if let Some(wire) = line_string_to_wire(line_string) {
                            wires.push(wire);
                        }
                    }
                },
                Geometry::GeometryCollection(collection) => {
                    for child in collection {
                        collect_geometry(child, wires);
                    }
                },
                _ => {},
            }
        }

        let mut wires = Vec::new();
        for geometry in geometry {
            collect_geometry(geometry, &mut wires);
        }
        wires
    }

    /// Borrow the native hypercurve region that now carries Sketch topology.
    pub fn as_region(&self) -> &Region2 {
        &self.region
    }

    /// Borrow native open hypercurve wires carried by this sketch.
    pub fn wires(&self) -> &[CurveString2] {
        &self.wires
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
    /// This is the non-`geo` API-boundary view of open Sketch paths. It keeps
    /// exact path ownership in [`CurveString2`] and delegates all sampling,
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

    fn has_native_topology(&self) -> bool {
        !self.region.is_empty() || !self.wires.is_empty()
    }

    fn can_use_region_boolean_with(&self, other: &Self) -> bool {
        self.has_native_topology() || other.has_native_topology()
    }

    /// Append non-area boolean sidecars with native hypercurve wires as the
    /// source of truth.
    ///
    /// Filled topology is handled by `Region2` booleans or, only when neither
    /// operand carries native topology, the finite area bridge; open path
    /// topology is independent CAD data and is therefore reattached from
    /// `CurveString2` before consulting legacy finite compatibility geometry.
    /// Empty filled regions are still valid boolean operands in the native path:
    /// preserving that identity/annihilator algebra keeps stale finite caches
    /// from becoming authoritative. Keeping exact-owned objects authoritative
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while the output
    /// projection boundary follows Hobby, "Practical Segment Intersection with
    /// Finite Precision Output," *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    fn append_boolean_sidecars(&self, out: &mut GeometryCollection<Real>) {
        if !self.wires.is_empty() {
            for wire in &self.wires {
                if let Some(line_string) = wire_to_line_string(wire) {
                    out.0.push(Geometry::LineString(line_string));
                }
            }
            return;
        }

        let geometry = self.effective_geometry();
        for geometry in &geometry.0 {
            match geometry {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {},
                _ => out.0.push(geometry.clone()),
            }
        }
    }

    #[cfg(feature = "offset")]
    pub(crate) fn compat_geometry_is_area_only(&self) -> bool {
        self.geometry.0.is_empty() || self.geometry.iter().all(is_area_geometry)
    }

    fn boolean_region_with(&self, other: &Self, op: BooleanOp) -> Option<Self> {
        if !self.can_use_region_boolean_with(other) {
            return None;
        }
        let policy = CurvePolicy::certified();
        let region = match self
            .region
            .boolean_region(&other.region, op, FillRule::NonZero, &policy)
            .ok()?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(_) => return None,
        };
        // Region booleans operate on filled topology only. Open `CurveString2`
        // wires are independent path topology, so preserve them with the same
        // sidecar semantics as the finite compatibility fallback while keeping
        // the filled result in hypercurve; this matches Yap's exact-object
        // boundary by avoiding a detour through `geo` solely to retain wires.
        let mut wires = match op {
            BooleanOp::Union | BooleanOp::Intersection | BooleanOp::Xor => {
                let mut wires = self.wires.clone();
                wires.extend(other.wires.iter().cloned());
                wires
            },
            BooleanOp::Difference => self.wires.clone(),
        };
        wires.retain(|wire| curve_string_to_polyline(wire).is_some());
        let mut sketch = Self::from_region_and_wires(region, wires, self.metadata.clone());
        sketch.origin = self.origin;
        sketch.origin_transform = self.origin_transform;
        Some(sketch)
    }

    fn transformed_region_with_matrix(&self, mat: &Matrix4<Real>) -> Option<Region2> {
        if self.region.is_empty() || !Self::region_has_nonzero_area(&self.region) {
            return None;
        }
        let determinant = mat[(0, 0)] * mat[(1, 1)] - mat[(0, 1)] * mat[(1, 0)];
        if determinant.abs() <= tolerance() {
            return None;
        }

        if let Some(similarity) = similarity_from_matrix(mat) {
            if let Ok(region) = self.region.transform_similarity(&similarity) {
                return Some(region);
            }
        }

        let transform_point = |point: &[Real; 2]| -> [Real; 2] {
            [
                mat[(0, 0)] * point[0] + mat[(0, 1)] * point[1] + mat[(0, 3)],
                mat[(1, 0)] * point[0] + mat[(1, 1)] * point[1] + mat[(1, 3)],
            ]
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
                    .collect::<Vec<[Real; 2]>>()
            })
            .filter_map(|ring| Self::contour_from_points(&ring))
            .collect::<Vec<_>>();
        let holes = profiles
            .iter()
            .flat_map(|profile| profile.holes().iter())
            .map(|ring| {
                ring.points()
                    .iter()
                    .map(transform_point)
                    .collect::<Vec<[Real; 2]>>()
            })
            .filter_map(|ring| Self::contour_from_points(&ring))
            .collect::<Vec<_>>();

        (!material.is_empty()).then(|| Region2::new(material, holes))
    }

    fn transformed_wires_with_matrix(&self, mat: &Matrix4<Real>) -> Vec<CurveString2> {
        if let Some(similarity) = similarity_from_matrix(mat) {
            let wires = self
                .wires
                .iter()
                .filter_map(|wire| wire.transform_similarity(&similarity).ok())
                .collect::<Vec<_>>();
            if wires.len() == self.wires.len() {
                return wires;
            }
        }

        let transform_point = |point: (Real, Real)| -> [Real; 2] {
            [
                mat[(0, 0)] * point.0 + mat[(0, 1)] * point.1 + mat[(0, 3)],
                mat[(1, 0)] * point.0 + mat[(1, 1)] * point.1 + mat[(1, 3)],
            ]
        };

        self.wires
            .iter()
            .filter_map(curve_string_to_polyline)
            .filter_map(|points| wire_from_points(points.into_iter().map(transform_point)))
            .collect()
    }

    /// Return this sketch with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Sketch<NewM> {
        Sketch {
            region: self.region,
            wires: self.wires,
            geometry: self.geometry,
            bounding_box: OnceLock::new(),
            metadata,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Map this sketch's metadata while preserving its geometry and origin.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, f: F) -> Sketch<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Sketch {
            region: self.region,
            wires: self.wires,
            geometry: self.geometry,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
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
        let default_normal = default_origin
            .normal
            .try_normalize(crate::float_types::tolerance())
            .unwrap_or_else(Vector3::z);
        let origin_normal = origin
            .normal
            .try_normalize(crate::float_types::tolerance())
            .unwrap_or(default_normal);
        let rotation_quat = UnitQuaternion::rotation_between(&default_normal, &origin_normal)
            .unwrap_or_else(|| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI));

        (pos_transform, rotation_quat)
    }

    pub(crate) fn apply_origin_transform_vertex(
        vertex: Vertex,
        origin_transform: OriginTransform,
    ) -> Vertex {
        let (pos_transform, rotation_quat) = origin_transform;
        let position = Point3::from(rotation_quat * vertex.position.coords + pos_transform);
        let normal = rotation_quat * vertex.normal;
        Vertex::new(position, normal)
    }

    fn apply_origin_transform_point(
        point: Point3<Real>,
        origin_transform: OriginTransform,
    ) -> Point3<Real> {
        let (pos_transform, rotation_quat) = origin_transform;
        Point3::from(rotation_quat * point.coords + pos_transform)
    }

    /// Create render-ready line strings from this sketch's visible edges in 3D space.
    ///
    /// Region- and wire-backed sketches project visible boundaries directly
    /// from hypercurve contours and curve strings. Finite `f32` output is a
    /// renderer boundary, while the source topology remains `Region2` and
    /// `CurveString2`, following Yap's exact-geometric-computation split
    /// (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>).
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

        self.collect_graphic_line_strings(
            &self.effective_geometry(),
            &mut graphic_line_strings,
        );
        GraphicLineStrings {
            line_strings: graphic_line_strings,
        }
    }

    fn collect_graphic_line_strings(
        &self,
        geometry: &GeometryCollection<Real>,
        out: &mut Vec<GraphicLineString>,
    ) {
        for geom in geometry {
            match geom {
                Geometry::Polygon(polygon) => {
                    out.push(self.line_string_to_graphic_line_string(polygon.exterior()));
                    for interior in polygon.interiors() {
                        out.push(self.line_string_to_graphic_line_string(interior));
                    }
                },
                Geometry::Line(line) => {
                    out.push(self.ring_to_graphic_line_string(&[
                        [line.start.x, line.start.y],
                        [line.end.x, line.end.y],
                    ]));
                },
                Geometry::LineString(line_string) => {
                    out.push(self.line_string_to_graphic_line_string(line_string));
                },
                Geometry::MultiLineString(line_strings) => {
                    for line_string in line_strings {
                        out.push(self.line_string_to_graphic_line_string(line_string));
                    }
                },
                Geometry::MultiPolygon(polygons) => {
                    for polygon in polygons {
                        out.push(self.line_string_to_graphic_line_string(polygon.exterior()));
                        for interior in polygon.interiors() {
                            out.push(self.line_string_to_graphic_line_string(interior));
                        }
                    }
                },
                Geometry::Rect(rect) => {
                    let line_string = Self::lines_to_line_string(&rect.to_lines());
                    out.push(self.line_string_to_graphic_line_string(&line_string));
                },
                Geometry::Triangle(triangle) => {
                    let line_string = Self::lines_to_line_string(&triangle.to_lines());
                    out.push(self.line_string_to_graphic_line_string(&line_string));
                },
                Geometry::GeometryCollection(collection) => {
                    self.collect_graphic_line_strings(collection, out);
                },
                _ => {},
            }
        }
    }

    fn lines_to_line_string(lines: &[Line<Real>]) -> LineString<Real> {
        let Some(first_line) = lines.first() else {
            return LineString::empty();
        };

        let mut coord_vec = Vec::with_capacity(lines.len() + 1);
        coord_vec.push(first_line.start);
        coord_vec.extend(lines.iter().map(|line| line.end));
        LineString::new(coord_vec)
    }

    fn line_string_to_graphic_line_string(
        &self,
        line_string: &LineString<Real>,
    ) -> GraphicLineString {
        let points = line_string
            .0
            .iter()
            .map(|coord| {
                let point = Point3::new(coord.x, coord.y, 0.0);
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

    /// Triangulate a polygon and holes with hypertri.
    ///
    /// The input rings are finite `f64` boundary coordinates. This method
    /// normalizes duplicate closing vertices, promotes coordinates to
    /// `hyperreal::Real`, and lets hypertri make the 2D topology decisions with
    /// exact predicates. The ear-clipping theorem is due to Meisters, "Polygons
    /// Have Ears" (American Mathematical Monthly 82(6), 1975,
    /// <https://doi.org/10.2307/2319703>); the exact-predicate boundary follows
    /// Yap, "Towards Exact Geometric Computation" (Computational Geometry
    /// 7(1-2), 1997, <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn triangulate_with_holes(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        fn push_ring(
            ring: &[[Real; 2]],
            vertices: &mut Vec<[Real; 2]>,
            exact: &mut Vec<hypertri::Point2>,
        ) -> Option<usize> {
            let start = vertices.len();
            let end = if ring.len() > 1
                && (ring[0][0] - ring[ring.len() - 1][0]).abs() <= tolerance()
                && (ring[0][1] - ring[ring.len() - 1][1]).abs() <= tolerance()
            {
                ring.len() - 1
            } else {
                ring.len()
            };
            if end < 3 {
                return None;
            }
            for &[x, y] in ring.iter().take(end) {
                if !x.is_finite() || !y.is_finite() {
                    return None;
                }
                vertices.push([x, y]);
                exact.push(hypertri::Point2::new(
                    hreal_from_f64(x).ok()?,
                    hreal_from_f64(y).ok()?,
                ));
            }
            Some(start)
        }

        let mut vertices = Vec::new();
        let mut exact = Vec::new();
        if push_ring(outer, &mut vertices, &mut exact).is_none() {
            return Vec::new();
        }

        let mut hole_indices = Vec::with_capacity(holes.len());
        for hole in holes {
            if let Some(start) = push_ring(hole, &mut vertices, &mut exact) {
                hole_indices.push(start);
            }
        }

        let indices = match hypertri::earcut(&exact, &hole_indices) {
            Ok(indices) => indices,
            Err(_) => return Vec::new(),
        };

        indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let a = *vertices.get(tri[0])?;
                let b = *vertices.get(tri[1])?;
                let c = *vertices.get(tri[2])?;
                Some([
                    Point3::new(a[0], a[1], 0.0),
                    Point3::new(b[0], b[1], 0.0),
                    Point3::new(c[0], c[1], 0.0),
                ])
            })
            .collect()
    }

    /// Triangulate all polygons in this Sketch.
    ///
    /// This function converts all polygons (including those from MultiPolygons) contained
    /// in the Sketch's geometry into a list of triangles. Each triangle is represented as
    /// a `[Point3<Real>; 3]`, where the Z-coordinate is 0.0.
    ///
    /// # Returns
    ///
    /// A `Vec<[Point3<Real>; 3]>` containing all the triangles resulting from the triangulation.
    pub fn triangulate(&self) -> Vec<[Point3<Real>; 3]> {
        let mut all_triangles = Vec::new();
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        if !profiles.is_empty() {
            for profile in profiles {
                let hole_refs = profile
                    .holes()
                    .iter()
                    .map(|hole| hole.points())
                    .collect::<Vec<_>>();
                all_triangles.extend(Self::triangulate_with_holes(
                    profile.material().points(),
                    &hole_refs,
                ));
            }
            return all_triangles;
        }

        let geometry = self.effective_geometry();

        for geom in geometry.iter() {
            match geom {
                geo::Geometry::Polygon(poly) => {
                    let outer = finite_line_string_points(poly.exterior());
                    let holes: Vec<Vec<[Real; 2]>> = poly
                        .interiors()
                        .iter()
                        .map(finite_line_string_points)
                        .collect();
                    let hole_refs: Vec<&[[Real; 2]]> = holes.iter().map(|v| &v[..]).collect();
                    let tris = Self::triangulate_with_holes(&outer, &hole_refs);
                    all_triangles.extend(tris);
                },
                geo::Geometry::MultiPolygon(mp) => {
                    for poly in &mp.0 {
                        let outer = finite_line_string_points(poly.exterior());
                        let holes: Vec<Vec<[Real; 2]>> = poly
                            .interiors()
                            .iter()
                            .map(finite_line_string_points)
                            .collect();
                        let hole_refs: Vec<&[[Real; 2]]> =
                            holes.iter().map(|v| &v[..]).collect();
                        let tris = Self::triangulate_with_holes(&outer, &hole_refs);
                        all_triangles.extend(tris);
                    }
                },
                // For other geometry types (LineString, Point, etc.), we might choose to ignore them
                // or handle them differently if needed. Currently, ignoring them.
                _ => {},
            }
        }

        all_triangles
    }

    /// Return a copy of this `Sketch` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    ///
    /// Native hypercurve sketches already separate material and hole contours,
    /// and carry open paths as `CurveString2`, so renormalization can preserve
    /// the native `Region2`/wire topology without rebuilding through
    /// winding-sensitive `geo` polygons. For polygon winding conventions and
    /// point-in-polygon background, see Hormann and Agathos, "The point in
    /// polygon problem for arbitrary polygons," Computational Geometry 20(3),
    /// 2001 (<https://doi.org/10.1016/S0925-7721(01)00012-8>). Keeping exact
    /// topology internal follows Yap, "Towards Exact Geometric Computation,"
    /// Computational Geometry 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn renormalize(&self) -> Sketch<M> {
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

        // Re-build the collection, orienting only what’s supported.
        let geometry = self.effective_geometry();
        let oriented_geoms: Vec<Geometry<Real>> = geometry
            .iter()
            .map(|geom| match geom {
                Geometry::Polygon(p) => {
                    Geometry::Polygon(p.clone().orient(Direction::Default))
                },
                Geometry::MultiPolygon(mp) => {
                    Geometry::MultiPolygon(mp.clone().orient(Direction::Default))
                },
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        let geometry = GeometryCollection(oriented_geoms);
        Self::from_compat_bridge_geometry_with_origin(
            geometry,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }
}

impl Sketch<()> {
    /// Return a new empty sketch with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

#[cfg(feature = "offset")]
fn is_area_geometry(geometry: &Geometry<Real>) -> bool {
    match geometry {
        Geometry::Polygon(_) | Geometry::MultiPolygon(_) => true,
        Geometry::GeometryCollection(collection) => collection.iter().all(is_area_geometry),
        _ => false,
    }
}

fn line_string_from_ring(ring: &[[Real; 2]]) -> Option<LineString<Real>> {
    (ring.len() >= 4).then(|| {
        LineString::from(
            ring.iter()
                .map(|point| (point[0], point[1]))
                .collect::<Vec<_>>(),
        )
    })
}

fn finite_line_string_points(line_string: &LineString<Real>) -> Vec<[Real; 2]> {
    line_string.0.iter().map(|coord| [coord.x, coord.y]).collect()
}

fn polygons_from_finite_region_profiles(
    profiles: &[FiniteRegionProfile2],
) -> Vec<GeoPolygon<Real>> {
    profiles
        .iter()
        .filter_map(|profile| {
            let exterior = line_string_from_ring(profile.material().points())?;
            let holes = profile
                .holes()
                .iter()
                .filter_map(|hole| line_string_from_ring(hole.points()))
                .collect();
            Some(GeoPolygon::new(exterior, holes))
        })
        .collect()
}

fn hyper_aabb_to_finite_xy_bounds(bbox: &HyperAabb2) -> Option<(Real, Real, Real, Real)> {
    Some((
        hreal_to_f64(bbox.min_x())?,
        hreal_to_f64(bbox.min_y())?,
        hreal_to_f64(bbox.max_x())?,
        hreal_to_f64(bbox.max_y())?,
    ))
}

/// Build a hypercurve similarity transform from Sketch's 4x4 matrix boundary.
///
/// `Similarity2` owns validation and native line/arc preserving transforms; the
/// Sketch layer only extracts the XY affine entries from nalgebra.
fn similarity_from_matrix(mat: &Matrix4<Real>) -> Option<Similarity2> {
    Similarity2::try_from_f64_affine(
        mat[(0, 0)],
        mat[(0, 1)],
        mat[(1, 0)],
        mat[(1, 1)],
        mat[(0, 3)],
        mat[(1, 3)],
        tolerance(),
    )
    .ok()
}

/// Group projected native material rings with the holes that they contain.
///
/// The grouping feeds both finite export projections and hypertri
/// triangulation. It deliberately stays at the API boundary: authoritative
/// ownership remains in `Region2`, while representative-point assignment is
/// only used to build finite grouped rings for algorithms whose inputs are
/// polygons-with-holes.
fn line_string_to_wire(line_string: &LineString<Real>) -> Option<CurveString2> {
    let points = finite_line_string_points(line_string);
    CurveString2::from_finite_line_string(&points).ok()
}

fn wire_to_line_string(wire: &CurveString2) -> Option<LineString<Real>> {
    curve_string_to_polyline(wire).map(LineString::from)
}

pub(crate) fn wire_from_points<I>(points: I) -> Option<CurveString2>
where
    I: IntoIterator<Item = [Real; 2]>,
{
    let points = points.into_iter().collect::<Vec<_>>();
    CurveString2::from_finite_line_string(&points).ok()
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

impl<M: Clone + Send + Sync + Debug> CSG for Sketch<M> {
    /// Return a new Sketch representing union of the two Sketches.
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
    fn union(&self, other: &Sketch<M>) -> Sketch<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Union) {
            return sketch;
        }

        // Extract multipolygon from geometry
        let polys1 = self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform union on those multipolygons
        let unioned = polys1.union(polys2); // This is valid if each is a MultiPolygon
        let oriented = unioned.orient(Direction::Default);

        // Wrap the unioned multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        self.append_boolean_sidecars(&mut final_gc);
        other.append_boolean_sidecars(&mut final_gc);

        Self::from_compat_bridge_geometry_with_origin(
            final_gc,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Sketch representing diffarence of the two Sketches.
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
    fn difference(&self, other: &Sketch<M>) -> Sketch<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Difference) {
            return sketch;
        }

        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform difference on those multipolygons
        let differenced = polys1.difference(polys2);
        let oriented = differenced.orient(Direction::Default);

        // Wrap the differenced multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        self.append_boolean_sidecars(&mut final_gc);

        Self::from_compat_bridge_geometry_with_origin(
            final_gc,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Sketch representing intersection of the two Sketches.
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
    fn intersection(&self, other: &Sketch<M>) -> Sketch<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Intersection) {
            return sketch;
        }

        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform intersection on those multipolygons
        let intersected = polys1.intersection(polys2);
        let oriented = intersected.orient(Direction::Default);

        // Wrap the intersected multipolygons + lines/points into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        self.append_boolean_sidecars(&mut final_gc);
        other.append_boolean_sidecars(&mut final_gc);

        Self::from_compat_bridge_geometry_with_origin(
            final_gc,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Return a new Sketch representing space in this Sketch excluding the space in the
    /// other Sketch plus the space in the other Sketch excluding the space in this Sketch.
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
    fn xor(&self, other: &Sketch<M>) -> Sketch<M> {
        if let Some(sketch) = self.boolean_region_with(other, BooleanOp::Xor) {
            return sketch;
        }

        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform symmetric difference (XOR)
        let xored = polys1.xor(polys2);
        let oriented = xored.orient(Direction::Default);

        // Wrap in a new GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        self.append_boolean_sidecars(&mut final_gc);
        other.append_boolean_sidecars(&mut final_gc);

        Self::from_compat_bridge_geometry_with_origin(
            final_gc,
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
    /// topology in hyperreal-backed objects rather than re-deriving it from the
    /// temporary `geo` cache. Native open wires use the same finite affine
    /// boundary and are immediately rebuilt as `CurveString2`. Projection-style
    /// or singular transforms and non-area compatibility geometry still fall
    /// back to finite `geo` transformation, then supported output is recomposed
    /// into hypercurve types. The exact/topological boundary follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>); the affine projection
    /// is the ordinary homogeneous-coordinate convention described by Foley et
    /// al., *Computer Graphics: Principles and Practice*, 2nd ed., 1990.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<M> {
        if let Some(region) = self.transformed_region_with_matrix(mat) {
            // Open wires follow the same finite projection boundary as region
            // rings, but rebuild directly as `CurveString2` instead of routing
            // through a temporary `geo` line string. Keeping the ownership on
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

        // Convert the top-left 2×2 submatrix + translation of a 4×4 into a geo::AffineTransform
        // The 4x4 looks like:
        //  [ m11  m12  m13  m14 ]
        //  [ m21  m22  m23  m24 ]
        //  [ m31  m32  m33  m34 ]
        //  [ m41  m42  m43  m44 ]
        //
        // For 2D, we use the sub-block:
        //   a = m11,  b = m12,
        //   d = m21,  e = m22,
        //   xoff = m14,
        //   yoff = m24,
        // ignoring anything in z.
        //
        // So the final affine transform in 2D has matrix:
        //   [a   b   xoff]
        //   [d   e   yoff]
        //   [0   0    1  ]
        let a = mat[(0, 0)];
        let b = mat[(0, 1)];
        let xoff = mat[(0, 3)];
        let d = mat[(1, 0)];
        let e = mat[(1, 1)];
        let yoff = mat[(1, 3)];

        let affine2 = AffineTransform::new(a, b, xoff, d, e, yoff);

        // Transform the finite compatibility projection in 2D, then promote
        // supported linework/area topology back into native hyper geometry.
        // The affine operation is still a finite bridge for unsupported
        // geometry, but it is no longer the authoritative Sketch state when a
        // `Region2` or `CurveString2` can represent the result.
        let geometry = self.effective_geometry().affine_transform(&affine2);
        Self::from_compat_bridge_geometry_with_origin(
            geometry,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }

    /// Returns an axis-aligned bounding box containing the effective finite
    /// 2D boundary projection, interpreted at z=0.
    ///
    /// Native hypercurve sketches compute bounds from projected `Region2`
    /// contours before consulting the temporary `geo` cache. The finite AABB is
    /// an acceleration/output boundary, while topological ownership remains in
    /// hyper geometry; this follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            if let Some((min_x, min_y, max_x, max_y)) = self.native_xy_bounds() {
                return Aabb::new(
                    Point3::new(min_x, min_y, 0.0),
                    Point3::new(max_x, max_y, 0.0),
                );
            }

            // Track overall min/max in x, y, z among the finite 2D projection's bounding rect.
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // Gather from the finite 2D compatibility projection using `geo::BoundingRect`.
            // This gives us (min_x, min_y) / (max_x, max_y)
            // Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
            let geometry = self.effective_geometry();
            let maybe_rect: Option<Rect<Real>> = geometry.bounding_rect();

            if let Some(rect) = maybe_rect {
                let min_pt = rect.min();
                let max_pt = rect.max();

                // Merge the 2D bounds into our existing min/max, forcing z=0 for 2D geometry.
                min_x = *partial_min(&min_x, &min_pt.x).unwrap();
                min_y = *partial_min(&min_y, &min_pt.y).unwrap();
                min_z = *partial_min(&min_z, &0.0).unwrap();

                max_x = *partial_max(&max_x, &max_pt.x).unwrap();
                max_y = *partial_max(&max_y, &max_pt.y).unwrap();
                max_z = *partial_max(&max_z, &0.0).unwrap();
            }

            // If still uninitialized (e.g., no geometry), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Sketch's finite boundary orientation.
    ///
    /// This preserves the historical Sketch behavior of flipping polygon
    /// winding rather than constructing an unbounded planar complement. Native
    /// hypercurve topology keeps its explicit material/hole bins, while only
    /// the finite compatibility projection is wound the old way. This avoids
    /// making ring orientation the source of CAD topology, matching Hormann and
    /// Agathos, "The point in polygon problem for arbitrary polygons,"
    /// *Computational Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>), and keeps finite
    /// coordinates at API boundaries as advocated by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn inverse(&self) -> Sketch<M> {
        if !self.wires.is_empty()
            || (!self.region.is_empty() && Self::region_has_nonzero_area(&self.region))
        {
            return Sketch {
                region: self.region.clone(),
                wires: self.wires.clone(),
                geometry: Self::inverted_compat_geometry(&Self::geometry_from_native(
                    &self.region,
                    &self.wires,
                )),
                bounding_box: OnceLock::new(),
                metadata: self.metadata.clone(),
                origin: self.origin,
                origin_transform: self.origin_transform,
            };
        }

        let geometry = self.effective_geometry();
        let geometry = Self::inverted_compat_geometry(&geometry);
        Self::from_compat_bridge_geometry_with_origin(
            geometry,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        )
    }
}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for Sketch<M> {
    fn from(mesh: Mesh<M>) -> Self {
        mesh.flatten()
    }
}
