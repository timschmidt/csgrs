use crate::traits::{BooleanOps, TransformOps};
use crate::float_types::Real;
use geo::{GeometryCollection, BooleanOps as GeoBool, AffineOps};
use nalgebra::{Matrix4, Vector3, Point3, partial_min, partial_max};
use std::convert::TryInto;
use std::sync::OnceLock;
use crate::mesh::polygon::Polygon;
use crate::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use crate::mesh::bsp::Node;
use std::fmt::Debug;

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone + Send + Sync + Debug> {
	/// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,
    
    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
	/// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
	fn partition_polys(
		polys: &[Polygon<S>],
		other_bb: &Aabb,
	) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
		let mut maybe = Vec::new();
		let mut never = Vec::new();
		for p in polys {
			if p.bounding_box().intersects(other_bb) {
				maybe.push(p.clone());
			} else {
				never.push(p.clone());
			}
		}
		(maybe, never)
	}
	
	/// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    pub fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;
    
            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                    min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                    min_z = *partial_min(&min_z, &v.pos.z).unwrap();
    
                    max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                    max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                    max_z = *partial_max(&max_z, &v.pos.z).unwrap();
                }
            }
    
            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }
    
            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }
}

impl<S: Clone + Send + Sync + Debug> BooleanOps for Mesh<S> {
    type Output = Self;

	/// Return a new Mesh representing union of the two Meshes.
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
    fn union(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip,  a_passthru) = Self::partition_polys(&self.polygons,  &other.bounding_box());
        let (b_clip,  b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        
        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        final_polys.extend(b_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
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
    fn difference(&self, other: &Mesh<S>) -> Mesh<S> {
		// avoid splitting obvious non‑intersecting faces
        let (a_clip,  a_passthru) = Self::partition_polys(&self.polygons,  &other.bounding_box());
        let (b_clip,  _b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();
        
        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        
        Mesh { 
			polygons: final_polys,
			bounding_box: OnceLock::new(),
			metadata: None,
		}
    }
    
    /// Return a new CSG representing intersection of the two CSG's.
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
    fn intersection(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip,  _a_passthru) = Self::partition_polys(&self.polygons,  &other.bounding_box());
        let (b_clip,  _b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        Mesh {
            polygons: a.all_polygons(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
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
    fn xor(&self, other: &Mesh<S>) -> Mesh<S> {
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }
}

impl<S: Clone + Send + Sync + Debug> TransformOps for Mesh<S> {
	fn new() -> Self {
		Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
	}
	
    fn transform(&self, m:&Matrix4<Real>)->Self {

        Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
    }
}

impl<S: Clone + Send + Sync + Debug> From<crate::sketch::sketch::Sketch<S>> for Mesh<S> {
	fn from(sketch: crate::sketch::sketch::Sketch<S>) -> Self {
	
		Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
	}
}
