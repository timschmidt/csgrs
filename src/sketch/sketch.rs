use crate::traits::{BooleanOps, TransformOps, Convert};
use crate::float_types::Real;
use geo::{GeometryCollection, BooleanOps as GeoBool, AffineOps};
use nalgebra::{Matrix4, Vector3};
use std::convert::TryInto;

#[derive(Clone, Debug)]
pub struct Sketch {
    pub geom: GeometryCollection<Real>,
}

impl BooleanOps for Sketch {
    type Output = Self;
    
    fn union(&self, other: &Self)->Self {
    
        Self { geom: GeometryCollection::default() }
    }
    fn difference(&self, other: &Self)->Self {
    
        Self { geom: GeometryCollection::default() }
    }
    fn intersection(&self, other: &Self)->Self {
    
        Self { geom: GeometryCollection::default() }
    }
}

impl TransformOps for Sketch {
	fn new() -> Self {
		Self { geom: GeometryCollection::default() }
	}
    fn transform(&self, m:&Matrix4<Real>)->Self {
        // ignore Z after affine, keep XY
        //let a = geo::AffineTransform::from(m.fixed_view::<3,3>(0,0).clone_owned());
        Self{ geom: GeometryCollection::default() }
    }
}

impl Convert<crate::mesh::mesh::Mesh> for Sketch {
	fn to(&self) -> Target {
		Self{ geom: GeometryCollection::default() }
	}
	
	fn from(source: &Target) -> Self {
		Self{ geom: GeometryCollection::default() }
	}
}
