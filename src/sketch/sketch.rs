use crate::traits::{BooleanOps, TransformOps};
use crate::float_types::Real;
use geo::{GeometryCollection, BooleanOps as GeoBool, AffineOps};
use nalgebra::{Matrix4, Vector3};
use std::convert::TryInto;

#[derive(Clone, Debug)]
pub struct Sketch<S> {
	/// 2D points, lines, polylines, polygons, and multipolygons
    pub geometry: GeometryCollection<Real>,
    
    /// Metadata
    pub metadata: Option<S>,
}

impl<S> BooleanOps for Sketch<S> {
    type Output = Self;
    
    fn union(&self, other: &Self)->Self {
    
        Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
    }
    
    fn difference(&self, other: &Self)->Self {
    
        Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
    }
    
    fn intersection(&self, other: &Self)->Self {
    
        Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
    }
}

impl<S> TransformOps for Sketch<S> {
	fn new() -> Self {
		Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
	}
	
    fn transform(&self, m:&Matrix4<Real>)->Self {
        // ignore Z after affine, keep XY
        //let a = geo::AffineTransform::from(m.fixed_view::<3,3>(0,0).clone_owned());
        Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
    }
}

impl<S: Clone> From<crate::mesh::mesh::Mesh<S>> for Sketch<S> {
	fn from(mesh: crate::mesh::mesh::Mesh<S>) -> Self {
	
		Sketch {
			geometry: GeometryCollection::default(),
			metadata: None,
		}
	}
}
