#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

//! Scalar adapter facade for the hyperreal-backed `csgrs` core.
//!
//! This crate keeps primitive scalar types at the API boundary. Geometry stored
//! in the wrapped `Mesh` and `Profile` values remains the raw `hyperreal::Real`
//! representation owned by `csgrs`, `hyperlattice`, `hypercurve`, and
//! `hypermesh`.

pub mod mesh;
#[cfg(feature = "sketch")]
pub mod profile;
pub mod scalar;

pub use csgrs as core;
pub use hyperreal::Real;
pub use mesh::{GraphicsMesh, Mesh, MeshF32, MeshF64, MeshI128, MeshVertex, RawMesh};
#[cfg(feature = "sketch")]
pub use profile::{Profile, ProfileF32, ProfileF64, ProfileI128, RawProfile};
pub use scalar::{AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter};

/// Adapter-space axis-aligned bounding box.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb3<S> {
    pub mins: [S; 3],
    pub maxs: [S; 3],
}

#[cfg(test)]
mod tests {
    use crate::{F64, I128, Mesh, Profile, RawReal, ScalarAdapter};
    use hyperreal::Real;

    #[test]
    fn f64_mesh_adapter_converts_at_edges() {
        let cube = Mesh::<F64, ()>::cube(2.0, ()).unwrap();
        let moved = cube.translate(1.0, 0.0, 0.0).unwrap();
        let bounds = moved.bounding_box().unwrap();

        assert_eq!(bounds.mins, [1.0, 0.0, 0.0]);
        assert_eq!(bounds.maxs, [3.0, 2.0, 2.0]);
    }

    #[test]
    fn i128_adapter_rejects_fractional_output() {
        let one_half = (Real::one() / Real::from(2_u8)).unwrap();
        assert!(I128::from_real(&one_half).is_err());
        assert_eq!(I128::from_real(&Real::from(12_i128)).unwrap(), 12);
    }

    #[test]
    fn raw_real_profile_stays_exact_at_boundary() {
        let width = Real::from(3_i128);
        let profile = Profile::<RawReal>::square(width.clone()).unwrap();
        let bounds = profile.bounding_box().unwrap();

        assert_eq!(bounds.mins, [Real::zero(), Real::zero(), Real::zero()]);
        assert_eq!(bounds.maxs, [width.clone(), width.clone(), Real::zero()]);
    }
}
