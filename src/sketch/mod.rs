//! `Sketch` struct and implementations of the `CSGOps` trait for `Sketch`

use crate::float_types::Real;
use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::mesh::Mesh;
use crate::traits::CSG;
use geo::algorithm::winding_order::Winding;
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, Coord, Geometry,
    GeometryCollection, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, Rect,
    orient::Direction,
};
use nalgebra::{Matrix4, Point3, partial_max, partial_min};
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

#[derive(Clone, Debug)]
pub struct Sketch<S> {
    /// 2D points, lines, polylines, polygons, and multipolygons
    pub geometry: GeometryCollection<Real>,

    /// Lazily calculated AABB that spans `geometry`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Sketch<S> {
    /// Take the [`geo::Polygon`]'s from the `CSG`'s geometry collection
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        let polygons = self
            .geometry
            .iter()
            .flat_map(|geom| match geom {
                Geometry::Polygon(poly) => vec![poly.clone()],
                Geometry::MultiPolygon(mp) => mp.0.clone(),
                _ => vec![],
            })
            .collect();

        MultiPolygon(polygons)
    }

    /// Create a Sketch from a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: Option<S>) -> Sketch<S> {
        let mut new_sketch = Sketch::new();
        new_sketch.geometry = geometry;
        new_sketch.metadata = metadata;
        new_sketch
    }

    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    pub fn triangulate_2d(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> =
            outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> =
                    hole.iter().map(|&[x, y]| Coord { x, y }).collect();
                LineString::new(coords)
            })
            .collect();

        // Ear-cut triangulation on the polygon (outer + holes)
        let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);

        #[cfg(feature = "earcut")]
        {
            use geo::TriangulateEarcut;
            let triangulation = polygon.earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert the 2D result (x,y) into 3D triangles with z=0
            let mut result = Vec::with_capacity(triangle_indices.len() / 3);
            for tri in triangle_indices.chunks_exact(3) {
                let pts = [
                    Point3::new(vertices[2 * tri[0]], vertices[2 * tri[0] + 1], 0.0),
                    Point3::new(vertices[2 * tri[1]], vertices[2 * tri[1] + 1], 0.0),
                    Point3::new(vertices[2 * tri[2]], vertices[2 * tri[2] + 1], 0.0),
                ];
                result.push(pts);
            }
            result
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            // We want polygons with holes => constrained triangulation.
            // For safety, handle the Result the trait returns:
            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                // If a triangulation error is a possibility,
                // pick the error-handling you want here:
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                // Each `triangle` is a geo_types::Triangle whose `.0, .1, .2`
                // are the 2D coordinates. We'll embed them at z=0.
                let [a, b, c] = [triangle.0, triangle.1, triangle.2];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }
    }

    /// Return a copy of this `Sketch` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    pub fn renormalize(&self) -> Sketch<S> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
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

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for Sketch<S> {
    /// Returns a new empty Sketch
    fn new() -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

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
    fn union(&self, other: &Sketch<S>) -> Sketch<S> {
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
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
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
    fn difference(&self, other: &Sketch<S>) -> Sketch<S> {
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
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
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
    fn intersection(&self, other: &Sketch<S>) -> Sketch<S> {
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
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
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
    fn xor(&self, other: &Sketch<S>) -> Sketch<S> {
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform symmetric difference (XOR)
        let xored = polys1.xor(polys2);
        let oriented = xored.orient(Direction::Default);

        // Wrap in a new GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // Re-insert lines & points from both sets
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<S> {
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

        // Transform sketch.geometry (the GeometryCollection) in 2D
        sketch.geometry = sketch.geometry.affine_transform(&affine2);

        // invalidate the old cached bounding box
        sketch.bounding_box = OnceLock::new();

        sketch
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] containing:
    /// The 2D bounding rectangle of `self.geometry`, interpreted at z=0.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons and the 2D geometry’s bounding_rect.
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // Gather from the 2D geometry using `geo::BoundingRect`
            // This gives us (min_x, min_y) / (max_x, max_y)
            // Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
            let maybe_rect: Option<Rect<Real>> = self.geometry.bounding_rect();

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
    fn inverse(&self) -> Sketch<S> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
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

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<Mesh<S>> for Sketch<S> {
    fn from(mesh: Mesh<S>) -> Self {
        // If mesh is empty, return empty Sketch
        if mesh.polygons.is_empty() {
            return Sketch::new();
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
                    (tri[0].pos.x, tri[0].pos.y),
                    (tri[1].pos.x, tri[1].pos.y),
                    (tri[2].pos.x, tri[2].pos.y),
                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
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

        Sketch {
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}
