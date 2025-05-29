use crate::float_types::Real;
use crate::traits::{BooleanOps, TransformOps};
use geo::{AffineOps, BooleanOps as GeoBool, GeometryCollection};
use nalgebra::{Matrix4, Vector3};
use std::convert::TryInto;
use std::fmt::Debug;

#[derive(Clone, Debug)]
pub struct Sketch<S> {
    /// 2D points, lines, polylines, polygons, and multipolygons
    pub geometry: GeometryCollection<Real>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> BooleanOps for Sketch<S> {
    type Output = Self;

    fn union(&self, other: &Self) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }

    fn difference(&self, other: &Self) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }

    fn intersection(&self, other: &Self) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }

    fn xor(&self, other: &Self) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }
}

impl<S: Clone + Send + Sync + Debug> TransformOps for Sketch<S> {
    fn new() -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }

    fn transform(&self, m: &Matrix4<Real>) -> Self {
        // ignore Z after affine, keep XY
        //let a = geo::AffineTransform::from(m.fixed_view::<3,3>(0,0).clone_owned());
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    pub fn transform(&self, mat: &Matrix4<Real>) -> Sketch<S> {
        let mat_inv_transpose = mat
            .try_inverse()
            .expect("Matrix not invertible?")
            .transpose(); // todo catch error
        let mut csg = self.clone();

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

        // Transform csg.geometry (the GeometryCollection) in 2D
        // Using geo’s map-coords approach or the built-in AffineOps trait.
        // Below we use the `AffineOps` trait if you have `use geo::AffineOps;`
        csg.geometry = csg.geometry.affine_transform(&affine2);

        // invalidate the old cached bounding box
        csg.bounding_box = OnceLock::new();

        csg
    }
}

impl<S: Clone + Send + Sync + Debug> From<crate::mesh::mesh::Mesh<S>> for Sketch<S> {
    fn from(mesh: crate::mesh::mesh::Mesh<S>) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            metadata: None,
        }
    }
}
