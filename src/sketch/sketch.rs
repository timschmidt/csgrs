use crate::kernel::{BooleanOps, TransformOps, Convert};
use crate::float_types::Real;
use geo::{GeometryCollection, BooleanOps as GeoBool, AffineOps};
use nalgebra::{Matrix4, Vector3};

#[derive(Clone, Debug)]
pub struct Sketch {
    pub geom: GeometryCollection<Real>,
}

impl BooleanOps for Sketch {
    type Output = Self;
    fn union(&self, rhs:&Self)->Self     { Self{ geom:self.geom.union(&rhs.geom) } }
    fn difference(&self, rhs:&Self)->Self{ Self{ geom:self.geom.difference(&rhs.geom)}}
    fn intersection(&self, rhs:&Self)->Self{ Self{ geom:self.geom.intersection(&rhs.geom)}}
}

impl TransformOps for Sketch {
    fn transform(&self, m:&Matrix4<Real>)->Self {
        // ignore Z after affine, keep XY
        let a = geo::AffineTransform::from_4x4(m.fixed_slice::<3,3>(0,0).clone_owned()
                                               .append_column(&m.fixed_slice::<3,1>(0,3)));
        Self{ geom: self.geom.affine_transform(&a) }
    }
}

//impl Convert<crate::mesh::Solid> for Sketch { /* extrude / revolve helpers */ }
//impl Convert<crate::voxels::VoxelGrid> for Sketch { /* slice-and-fill */ }

