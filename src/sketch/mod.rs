//! `Sketch` struct and implementations of the `CSGOps` trait for `Sketch`

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{PI, Real, hreal_from_f64, hreal_to_f64, tolerance};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

use crate::csg::CSG;
use crate::vertex::Vertex;
use geo::algorithm::winding_order::Winding;
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, CoordsIter,
    Geometry, GeometryCollection, Line, LineString, MultiPolygon, Orient,
    Polygon as GeoPolygon, Rect, orient::Direction,
};
use hypercurve::{
    BooleanOp, BulgeVertex2, CircularArc2, Classification, Contour2, CurvePolicy,
    CurveString2, FillRule, LineSeg2, Point2, Region2, RegionPointLocation, Segment2,
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

/// Finite boundary rings projected from Sketch's native hypercurve region.
///
/// This is an API-edge representation: each ring has ordinary `Real`
/// coordinates for triangulation, toolpaths, and export, while the authoritative
/// topology remains in `hypercurve::Region2`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SketchRegionRings {
    /// Filled/material boundary rings.
    pub material: Vec<Vec<[Real; 2]>>,
    /// Subtractive/hole boundary rings.
    pub holes: Vec<Vec<[Real; 2]>>,
}

impl SketchRegionRings {
    /// True when no finite region rings were projected.
    pub fn is_empty(&self) -> bool {
        self.material.is_empty() && self.holes.is_empty()
    }

    /// Iterate material rings followed by hole rings.
    pub fn iter_all(&self) -> impl Iterator<Item = &[[Real; 2]]> {
        self.material
            .iter()
            .chain(self.holes.iter())
            .map(Vec::as_slice)
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
        let region_rings = self.region_rings();
        if !region_rings.material.is_empty() {
            return Self::region_rings_to_multipolygon(region_rings);
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

    fn region_rings_to_multipolygon(rings: SketchRegionRings) -> MultiPolygon<Real> {
        let mut holes = rings
            .holes
            .into_iter()
            .filter_map(|ring| line_string_from_ring(&ring))
            .collect::<Vec<_>>();

        let polygons = rings
            .material
            .into_iter()
            .filter_map(|ring| {
                let exterior = line_string_from_ring(&ring)?;
                let polygon_holes = if holes.is_empty() {
                    Vec::new()
                } else {
                    std::mem::take(&mut holes)
                };
                Some(GeoPolygon::new(exterior, polygon_holes))
            })
            .collect();

        MultiPolygon(polygons)
    }

    /// Project the native hypercurve region into finite material and hole
    /// boundary rings.
    ///
    /// The returned coordinates are suitable for API boundaries such as
    /// triangulation and toolpath generation. Topology-sensitive decisions
    /// should continue to use [`Sketch::as_region`] and hypercurve predicates.
    /// This split follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). Point containment
    /// against these regions follows the boundary-first structure of Hormann
    /// and Agathos, "The point in polygon problem for arbitrary polygons,"
    /// *Computational Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>), as implemented by
    /// `hypercurve`.
    pub fn region_rings(&self) -> SketchRegionRings {
        SketchRegionRings {
            material: self
                .region
                .material_contours()
                .iter()
                .filter_map(|contour| contour_to_ring(contour, 1e-3))
                .collect(),
            holes: self
                .region
                .hole_contours()
                .iter()
                .filter_map(|contour| contour_to_ring(contour, 1e-3))
                .collect(),
        }
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

    pub(crate) fn region_xy_bounds(&self) -> Option<(Real, Real, Real, Real)> {
        let rings = self.region_rings();
        let mut iter = rings.iter_all().flat_map(|ring| ring.iter());
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
        if self.geometry.0.is_empty() && (!self.region.is_empty() || !self.wires.is_empty()) {
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

    /// Create a Sketch from a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: M) -> Sketch<M> {
        Self::from_compat_geometry_with_origin(
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

    /// Build a Sketch when a native hypercurve region is already available but
    /// a finite compatibility cache must be retained for open wires or legacy
    /// rendering.
    pub(crate) fn from_region_and_compat_geometry_with_origin(
        region: Region2,
        geometry: GeometryCollection<Real>,
        metadata: M,
        origin: Vertex,
        origin_transform: OriginTransform,
    ) -> Sketch<M> {
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
        if points.len() < 3 {
            return None;
        }

        let mut end = points.len();
        if points.len() > 1
            && same_xy(
                (points[0][0], points[0][1]),
                (points[points.len() - 1][0], points[points.len() - 1][1]),
            )
        {
            end -= 1;
        }
        if end < 3 {
            return None;
        }

        let vertices = points
            .iter()
            .take(end)
            .map(|point| {
                Some(BulgeVertex2::new(
                    Point2::new(
                        hreal_from_f64(point[0]).ok()?,
                        hreal_from_f64(point[1]).ok()?,
                    ),
                    hreal_from_f64(0.0).ok()?,
                ))
            })
            .collect::<Option<Vec<_>>>()?;
        Contour2::from_bulge_vertices(&vertices).ok()
    }

    pub(crate) fn region_from_geo(geometry: &GeometryCollection<Real>) -> Region2 {
        fn coord_same(a: geo::Coord<Real>, b: geo::Coord<Real>) -> bool {
            (a.x - b.x).abs() <= tolerance() && (a.y - b.y).abs() <= tolerance()
        }

        fn contour_from_line_string(line_string: &LineString<Real>) -> Option<Contour2> {
            let coords = line_string.coords_iter().collect::<Vec<_>>();
            if coords.len() < 3 {
                return None;
            }
            // Open polylines are native wires, not filled contours. Only closed
            // compatibility rings are promoted into Region2, matching the
            // boundary/topology separation in Yap, "Towards Exact Geometric
            // Computation," Computational Geometry 7(1-2), 1997
            // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
            if !coord_same(coords[0], coords[coords.len() - 1]) {
                return None;
            }
            let end = coords.len() - 1;
            if end < 3 {
                return None;
            }
            let vertices = coords
                .into_iter()
                .take(end)
                .map(|coord| {
                    Some(BulgeVertex2::new(
                        Point2::new(
                            hreal_from_f64(coord.x).ok()?,
                            hreal_from_f64(coord.y).ok()?,
                        ),
                        hreal_from_f64(0.0).ok()?,
                    ))
                })
                .collect::<Option<Vec<_>>>()?;
            Contour2::from_bulge_vertices(&vertices).ok()
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
        let mut polygons = Vec::new();
        let mut holes = region
            .hole_contours()
            .iter()
            .filter_map(|hole| contour_to_line_string(hole, 1e-3))
            .collect::<Vec<_>>();

        for material in region.material_contours() {
            let Some(exterior) = contour_to_line_string(material, 1e-3) else {
                continue;
            };
            let polygon_holes = if polygons.is_empty() {
                std::mem::take(&mut holes)
            } else {
                Vec::new()
            };
            polygons.push(GeoPolygon::new(exterior, polygon_holes));
        }

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

    fn wires_from_geo(geometry: &GeometryCollection<Real>) -> Vec<CurveString2> {
        fn collect_geometry(geometry: &Geometry<Real>, wires: &mut Vec<CurveString2>) {
            match geometry {
                Geometry::Line(line) => {
                    let line_string = LineString::from(vec![line.start, line.end]);
                    if let Some(wire) = line_string_to_wire(&line_string) {
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
        self.wires
            .iter()
            .filter_map(curve_string_to_polyline)
            .map(|points| points.into_iter().map(|(x, y)| [x, y]).collect::<Vec<_>>())
            .filter(|points| points.len() >= 2)
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

    fn can_use_region_boolean_with(&self, other: &Self) -> bool {
        !self.region.is_empty()
            && !other.region.is_empty()
            && self.compat_geometry_is_area_only()
            && other.compat_geometry_is_area_only()
    }

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
        let mut sketch = Self::from_region(region, self.metadata.clone());
        sketch.origin = self.origin;
        sketch.origin_transform = self.origin_transform;
        Some(sketch)
    }

    fn transformed_region_with_matrix(&self, mat: &Matrix4<Real>) -> Option<Region2> {
        if self.region.is_empty() || !self.compat_geometry_is_area_only() {
            return None;
        }
        let determinant = mat[(0, 0)] * mat[(1, 1)] - mat[(0, 1)] * mat[(1, 0)];
        if determinant.abs() <= tolerance() {
            return None;
        }

        let transform_point = |point: &[Real; 2]| -> [Real; 2] {
            [
                mat[(0, 0)] * point[0] + mat[(0, 1)] * point[1] + mat[(0, 3)],
                mat[(1, 0)] * point[0] + mat[(1, 1)] * point[1] + mat[(1, 3)],
            ]
        };

        let rings = self.region_rings();
        let material = rings
            .material
            .iter()
            .map(|ring| ring.iter().map(transform_point).collect::<Vec<[Real; 2]>>())
            .filter_map(|ring| Self::contour_from_points(&ring))
            .collect::<Vec<_>>();
        let holes = rings
            .holes
            .iter()
            .map(|ring| ring.iter().map(transform_point).collect::<Vec<[Real; 2]>>())
            .filter_map(|ring| Self::contour_from_points(&ring))
            .collect::<Vec<_>>();

        (!material.is_empty()).then(|| Region2::new(material, holes))
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
    /// Region-backed sketches project visible boundaries directly from
    /// hypercurve contours. Finite `f32` output is a renderer boundary, while
    /// the source topology remains `Region2`, following Yap's
    /// exact-geometric-computation split (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn build_graphic_line_strings(&self) -> GraphicLineStrings {
        let mut graphic_line_strings = Vec::new();
        if !self.region.is_empty() && self.compat_geometry_is_area_only() {
            for ring in self.region_rings().iter_all() {
                graphic_line_strings.push(self.ring_to_graphic_line_string(ring));
            }
            return GraphicLineStrings {
                line_strings: graphic_line_strings,
            };
        }
        if self.geometry.0.is_empty() && !self.wires.is_empty() {
            for wire in &self.wires {
                if let Some(line_string) = wire_to_line_string(wire) {
                    graphic_line_strings
                        .push(self.line_string_to_graphic_line_string(&line_string));
                }
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
                    out.push(self.line_string_to_graphic_line_string(&LineString::from(
                        vec![line.start, line.end],
                    )));
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
            .coords()
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
        let rings = self.region_rings();
        if !rings.material.is_empty() {
            for (index, outer) in rings.material.iter().enumerate() {
                let hole_refs = if index == 0 {
                    rings.holes.iter().map(|ring| ring.as_slice()).collect()
                } else {
                    Vec::new()
                };
                all_triangles.extend(Self::triangulate_with_holes(outer, &hole_refs));
            }
            return all_triangles;
        }

        let geometry = self.effective_geometry();

        for geom in geometry.iter() {
            match geom {
                geo::Geometry::Polygon(poly) => {
                    let outer: Vec<[Real; 2]> =
                        poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                    let holes: Vec<Vec<[Real; 2]>> = poly
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();
                    let hole_refs: Vec<&[[Real; 2]]> = holes.iter().map(|v| &v[..]).collect();
                    let tris = Self::triangulate_with_holes(&outer, &hole_refs);
                    all_triangles.extend(tris);
                },
                geo::Geometry::MultiPolygon(mp) => {
                    for poly in &mp.0 {
                        let outer: Vec<[Real; 2]> =
                            poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                        let holes: Vec<Vec<[Real; 2]>> = poly
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
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
    /// so renormalization can preserve the `Region2` topology without rebuilding
    /// through winding-sensitive `geo` polygons. For polygon winding conventions
    /// and point-in-polygon background, see Hormann and Agathos, "The point in
    /// polygon problem for arbitrary polygons," Computational Geometry 20(3),
    /// 2001 (<https://doi.org/10.1016/S0925-7721(01)00012-8>).
    pub fn renormalize(&self) -> Sketch<M> {
        if !self.region.is_empty() && self.compat_geometry_is_area_only() {
            let mut sketch = Self::from_region(self.region.clone(), self.metadata.clone());
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
        Self::from_compat_geometry_with_origin(
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

fn is_area_geometry(geometry: &Geometry<Real>) -> bool {
    match geometry {
        Geometry::Polygon(_) | Geometry::MultiPolygon(_) => true,
        Geometry::GeometryCollection(collection) => collection.iter().all(is_area_geometry),
        _ => false,
    }
}

fn contour_to_line_string(
    contour: &Contour2,
    error_distance: Real,
) -> Option<LineString<Real>> {
    contour_to_ring(contour, error_distance).map(|ring| {
        LineString::from(
            ring.into_iter()
                .map(|point| (point[0], point[1]))
                .collect::<Vec<_>>(),
        )
    })
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

fn line_string_to_wire(line_string: &LineString<Real>) -> Option<CurveString2> {
    wire_from_points(line_string.coords().map(|coord| [coord.x, coord.y]))
}

fn wire_to_line_string(wire: &CurveString2) -> Option<LineString<Real>> {
    curve_string_to_polyline(wire).map(LineString::from)
}

pub(crate) fn wire_from_points<I>(points: I) -> Option<CurveString2>
where
    I: IntoIterator<Item = [Real; 2]>,
{
    let points = points.into_iter().collect::<Vec<_>>();
    if points.len() < 2 {
        return None;
    }
    let mut segments = Vec::new();
    for edge in points.windows(2) {
        let start = Point2::new(
            hreal_from_f64(edge[0][0]).ok()?,
            hreal_from_f64(edge[0][1]).ok()?,
        );
        let end = Point2::new(
            hreal_from_f64(edge[1][0]).ok()?,
            hreal_from_f64(edge[1][1]).ok()?,
        );
        if let Ok(line) = LineSeg2::try_new(start, end) {
            segments.push(Segment2::Line(line));
        }
    }
    CurveString2::try_new(segments).ok()
}

fn curve_string_to_polyline(wire: &CurveString2) -> Option<Vec<(Real, Real)>> {
    let first = wire.segments().first()?;
    let mut points = Vec::with_capacity(wire.len() + 1);
    points.push(finite_point2_tuple(first.start())?);
    for segment in wire.segments() {
        match segment {
            Segment2::Line(line) => points.push(finite_point2_tuple(line.end())?),
            Segment2::Arc(arc) => append_arc_samples(&mut points, arc, 1e-3),
        }
    }
    Some(points)
}

fn finite_point2_tuple(point: &Point2) -> Option<(Real, Real)> {
    Some((hreal_to_f64(point.x())?, hreal_to_f64(point.y())?))
}

fn contour_to_ring(contour: &Contour2, error_distance: Real) -> Option<Vec<[Real; 2]>> {
    let mut points = approximate_contour(contour, error_distance);
    close_region_ring(&mut points);
    (points.len() >= 4).then(|| points.into_iter().map(|(x, y)| [x, y]).collect::<Vec<_>>())
}

fn approximate_contour(contour: &Contour2, error_distance: Real) -> Vec<(Real, Real)> {
    let mut points = Vec::new();
    for segment in contour.segments() {
        if let Some(start) = f64_point(segment_start(segment)) {
            if points
                .last()
                .is_none_or(|last: &(Real, Real)| !same_xy(*last, start))
            {
                points.push(start);
            }
        }
        match segment {
            Segment2::Line(_) => {},
            Segment2::Arc(arc) => append_arc_samples(&mut points, arc, error_distance),
        }
    }
    points
}

fn append_arc_samples(
    points: &mut Vec<(Real, Real)>,
    arc: &CircularArc2,
    error_distance: Real,
) {
    let (Some(start), Some(end), Some(center)) = (
        f64_point(arc.start()),
        f64_point(arc.end()),
        f64_point(arc.center()),
    ) else {
        return;
    };
    let radius = ((start.0 - center.0).powi(2) + (start.1 - center.1).powi(2)).sqrt();
    if radius <= tolerance() {
        return;
    }
    let a0 = (start.1 - center.1).atan2(start.0 - center.0);
    let a1 = (end.1 - center.1).atan2(end.0 - center.0);
    let mut sweep = a1 - a0;
    if arc.is_clockwise() {
        if sweep > 0.0 {
            sweep -= 2.0 * PI;
        }
    } else if sweep < 0.0 {
        sweep += 2.0 * PI;
    }
    let error = error_distance.abs().max(tolerance());
    let max_angle = (1.0 - (error / radius).min(1.0)).acos().max(1e-3) * 2.0;
    let steps = ((sweep.abs() / max_angle).ceil() as usize).max(1);
    for step in 1..=steps {
        let t = step as Real / steps as Real;
        let angle = a0 + sweep * t;
        points.push((
            center.0 + radius * angle.cos(),
            center.1 + radius * angle.sin(),
        ));
    }
}

fn close_region_ring(points: &mut Vec<(Real, Real)>) {
    if points.len() >= 2
        && points
            .first()
            .zip(points.last())
            .is_some_and(|(a, b)| !same_xy(*a, *b))
    {
        let first = points[0];
        points.push(first);
    }
}

fn segment_start(segment: &Segment2) -> &Point2 {
    match segment {
        Segment2::Line(line) => line.start(),
        Segment2::Arc(arc) => arc.start(),
    }
}

fn f64_point(point: &Point2) -> Option<(Real, Real)> {
    Some((hreal_to_f64(point.x())?, hreal_to_f64(point.y())?))
}

fn same_xy(a: (Real, Real), b: (Real, Real)) -> bool {
    (a.0 - b.0).abs() <= tolerance() && (a.1 - b.1).abs() <= tolerance()
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

        // re-insert lines & points from both sets:
        let self_geometry = self.effective_geometry();
        let other_geometry = other.effective_geometry();
        for g in &self_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }

        Self::from_compat_geometry_with_origin(
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

        // Re-insert lines & points from self only
        // (If you need to exclude lines/points that lie inside other, you'd need more checks here.)
        let self_geometry = self.effective_geometry();
        for g in &self_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Self::from_compat_geometry_with_origin(
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

        // For lines and points: keep them only if they intersect in both sets
        // todo: detect intersection of non-polygons
        let self_geometry = self.effective_geometry();
        let other_geometry = other.effective_geometry();
        for g in &self_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Self::from_compat_geometry_with_origin(
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

        // Re-insert lines & points from both sets
        let self_geometry = self.effective_geometry();
        let other_geometry = other.effective_geometry();
        for g in &self_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other_geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Self::from_compat_geometry_with_origin(
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
    /// temporary `geo` cache. Projection-style or singular transforms, open
    /// wires, and non-area compatibility geometry still fall back to finite
    /// `geo` transformation; native wires are then regenerated from the
    /// transformed compatibility projection. The exact/topological boundary
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>); the affine projection
    /// is the ordinary homogeneous-coordinate convention described by Foley et
    /// al., *Computer Graphics: Principles and Practice*, 2nd ed., 1990.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<M> {
        if let Some(region) = self.transformed_region_with_matrix(mat) {
            let mut wires = Vec::new();
            if !self.wires.is_empty() {
                let a = mat[(0, 0)];
                let b = mat[(0, 1)];
                let xoff = mat[(0, 3)];
                let d = mat[(1, 0)];
                let e = mat[(1, 1)];
                let yoff = mat[(1, 3)];
                let affine2 = AffineTransform::new(a, b, xoff, d, e, yoff);
                let wire_geometry = Self::geometry_from_native(&Region2::empty(), &self.wires)
                    .affine_transform(&affine2);
                wires = Self::wires_from_geo(&wire_geometry);
            }
            return Self::from_region_and_wires_with_origin(
                region,
                wires,
                self.metadata.clone(),
                self.origin,
                self.origin_transform,
            );
        }

        let mut sketch = self.clone();

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

        // Transform the finite compatibility projection in 2D.
        sketch.geometry = self.effective_geometry().affine_transform(&affine2);
        sketch.region = Self::region_from_geo(&sketch.geometry);
        sketch.wires = Self::wires_from_geo(&sketch.geometry);

        // invalidate the old cached bounding box
        sketch.bounding_box = OnceLock::new();

        sketch
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
            if let Some((min_x, min_y, max_x, max_y)) = self.region_xy_bounds() {
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

    /// Invert this Sketch (flip inside vs. outside)
    fn inverse(&self) -> Sketch<M> {
        // Re-build the collection, orienting only what’s supported.
        let geometry = self.effective_geometry();
        let oriented_geoms: Vec<Geometry<Real>> = geometry
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
                    // Loop over every polygon inside and apply the same rule.
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
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        let geometry = GeometryCollection(oriented_geoms);
        Self::from_compat_geometry_with_origin(
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
        // If mesh is empty, return empty Sketch
        if mesh.polygons.is_empty() {
            return Sketch::empty(mesh.metadata);
        }

        // Convert mesh into a collection of 2D polygons
        let mut flattened_3d = Vec::new(); // will store geo::Polygon<Real>

        for poly in &mesh.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let ring = vec![
                    (tri[0].position.x, tri[0].position.y),
                    (tri[1].position.x, tri[1].position.y),
                    (tri[2].position.x, tri[2].position.y),
                    (tri[0].position.x, tri[0].position.y), // close ring explicitly
                ];
                let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
                flattened_3d.push(polygon_2d);
            }
        }

        // Union all these polygons together into one MultiPolygon
        // (We could chain them in a fold-based union.)
        let unioned_from_3d = if flattened_3d.is_empty() {
            MultiPolygon::new(Vec::new())
        } else {
            // Start with the first polygon as a MultiPolygon
            let mut mp_acc = MultiPolygon(vec![flattened_3d[0].clone()]);
            // Union in the rest
            for p in flattened_3d.iter().skip(1) {
                mp_acc = mp_acc.union(&MultiPolygon(vec![p.clone()]));
            }
            mp_acc
        };

        // Ensure consistent orientation (CCW for exteriors):
        let oriented = unioned_from_3d.orient(Direction::Default);

        // Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));

        Self::from_compat_geometry_with_origin(
            new_gc,
            mesh.metadata,
            Vertex::default(),
            Self::prepare_origin_transform(Vertex::default()),
        )
    }
}
