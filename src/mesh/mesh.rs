use crate::traits::{BooleanOps, TransformOps};
use crate::float_types::Real;
use geo::{GeometryCollection, BooleanOps as GeoBool, AffineOps};
use nalgebra::{Matrix4, Vector3};
use std::convert::TryInto;
use std::sync::OnceLock;
use crate::mesh::polygon::Polygon;
use crate::float_types::parry3d::bounding_volume::Aabb;

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone> {
	/// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,
    
    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone> BooleanOps for Mesh<S> {
    type Output = Self;

    fn union(&self, other: &Self)->Self {
    
        Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
    }
    
    fn difference(&self, other: &Self)->Self {
    
        Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
    }
    
    fn intersection(&self, other: &Self)->Self {
    
        Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
    }
}

impl<S: Clone> TransformOps for Mesh<S> {
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

impl<S: Clone> From<crate::sketch::sketch::Sketch<S>> for Mesh<S> {
	fn from(sketch: crate::sketch::sketch::Sketch<S>) -> Self {
	
		Mesh { 
			polygons: Vec::new(),
			bounding_box: OnceLock::new(),
			metadata: None,
		}
	}
}
