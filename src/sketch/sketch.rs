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
    
        Self { geom:self.geom[0].try_into().union(&other.geom[0].into_multi_polygon().into()) }
    }
    fn difference(&self, other: &Self)->Self {
    
        Self { geom:self.geom[0].try_into().difference(&other.geom[0].into_multi_polygon().into()) }
    }
    fn intersection(&self, other: &Self)->Self {
    
        Self { geom:self.geom[0].try_into().intersection(&other.geom[0].into_multi_polygon().into()) }
    }
}

impl TransformOps for Sketch {
    fn transform(&self, m:&Matrix4<Real>)->Self {
        // ignore Z after affine, keep XY
        let a = geo::AffineTransform::from_4x4(m.fixed_view::<3,3>(0,0).clone_owned());
        Self{ geom: self.geom.affine_transform(&a) }
    }
}

//impl Convert<crate::mesh::Solid> for Sketch { /* extrude / revolve helpers */ }
//impl Convert<crate::voxels::VoxelGrid> for Sketch { /* slice-and-fill */ }

