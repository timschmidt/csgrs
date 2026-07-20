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
pub use mesh::{
    GraphicsMesh, IndexedMeshBuffers, Mesh, MeshF32, MeshF64, MeshI128, MeshVertex, RawMesh,
};
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
    use crate::{AdapterError, F32, F64, I128, Mesh, Profile, RawReal, ScalarAdapter};
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

    #[test]
    fn primitive_graphics_adapters_match_core_gpu_buffers() {
        let raw = Mesh::<RawReal, ()>::cube(Real::from(2), ())
            .unwrap()
            .into_raw();

        let expected_f32 = raw.try_to_gpu_mesh_f32().unwrap();
        let actual_f32 = Mesh::<F32, ()>::from_raw(raw.clone())
            .graphics_mesh()
            .unwrap();
        assert_eq!(
            actual_f32.vertices,
            expected_f32
                .positions
                .into_iter()
                .zip(expected_f32.normals)
                .collect::<Vec<_>>()
        );
        assert_eq!(actual_f32.indices, expected_f32.indices);

        let expected_f64 = raw.try_to_gpu_mesh_f64().unwrap();
        let actual_f64 = Mesh::<F64, ()>::from_raw(raw).graphics_mesh().unwrap();
        assert_eq!(
            actual_f64.vertices,
            expected_f64
                .positions
                .into_iter()
                .zip(expected_f64.normals)
                .collect::<Vec<_>>()
        );
        assert_eq!(actual_f64.indices, expected_f64.indices);
    }

    #[test]
    fn exact_and_integer_graphics_adapters_keep_generic_conversion() {
        let raw = Mesh::<RawReal, ()>::cube(Real::from(2), ())
            .unwrap()
            .into_raw();
        let expected = raw.build_graphics_mesh();
        let actual = Mesh::<RawReal, ()>::from_raw(raw).graphics_mesh().unwrap();
        assert_eq!(actual.vertices, expected.vertices.to_vec());
        assert_eq!(actual.indices, expected.indices.to_vec());

        let integer = Mesh::<I128, ()>::cube(2, ())
            .unwrap()
            .graphics_mesh()
            .unwrap();
        assert_eq!(integer.vertices.len(), 36);
        assert_eq!(integer.indices, (0..36).collect::<Vec<_>>());
    }

    #[test]
    fn f32_graphics_adapter_preserves_strict_overflow_error() {
        let mut huge = Real::from(2);
        for _ in 0..8 {
            huge = huge.clone() * huge;
        }
        let raw = Mesh::<RawReal, ()>::cube(huge, ()).unwrap().into_raw();

        assert_eq!(
            Mesh::<F32, ()>::from_raw(raw).graphics_mesh(),
            Err(AdapterError::NotFiniteApproximation)
        );
    }
}
