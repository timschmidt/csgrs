use crate::float_types::Real;
use crate::traits::{BooleanOps, TransformOps};
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, Coord, CoordsIter, Geometry,
    GeometryCollection, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, Rect,
    orient::Direction,
};
use nalgebra::{Matrix4, Vector3};
use std::convert::TryInto;
use std::fmt::Debug;
use std::sync::OnceLock;
use crate::float_types::parry3d::bounding_volume::Aabb;

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
        // allocate vec to fit all polygons
        let mut polygons = Vec::with_capacity(self.geometry.0.iter().fold(0, |len, geom| {
            len + match geom {
                Geometry::Polygon(_) => len + 1,
                Geometry::MultiPolygon(mp) => len + mp.0.len(),
                // ignore lines, points, etc.
                _ => len,
            }
        }));

        for geom in &self.geometry.0 {
            match geom {
                Geometry::Polygon(poly) => polygons.push(poly.clone()),
                Geometry::MultiPolygon(mp) => polygons.extend(mp.0.clone()),
                // ignore lines, points, etc.
                _ => {}
            }
        }

        MultiPolygon(polygons)
    }
}

impl<S: Clone + Send + Sync + Debug> BooleanOps for Sketch<S> {
    type Output = Self;
    
	/// Return a new Sketch representing union of the two Sketches.
    ///
    /// ```no_run
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
                }
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                }
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
    /// ```no_run
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
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
    /// ```no_run
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
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
    /// ```no_run
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
}

impl<S: Clone + Send + Sync + Debug> TransformOps for Sketch<S> {
    fn new() -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<S> {
        let mat_inv_transpose = mat
            .try_inverse()
            .expect("Matrix not invertible?")
            .transpose(); // todo catch error
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
}

impl<S: Clone + Send + Sync + Debug> From<crate::mesh::mesh::Mesh<S>> for Sketch<S> {
    fn from(mesh: crate::mesh::mesh::Mesh<S>) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}
