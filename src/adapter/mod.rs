//! Primitive-scalar facade for the Hyperreal-backed CSGRS core.
//!
//! This module keeps primitive scalar types at the API boundary. Geometry stored
//! in the wrapped native mesh and curve values remains the raw `hyperreal::Real`
//! representation owned by `csgrs`, `hyperlattice`, `hypercurve`, and
//! `hypermesh`.

#[cfg(feature = "curve")]
pub mod curve;
pub mod mesh;
pub mod scalar;

#[cfg(feature = "curve")]
pub use curve::{
    CurveRegionF32, CurveRegionF64, CurveRegionI128, RawCurveRegion, ScalarCurve,
};
pub use hyperreal::Real;
pub use mesh::{
    GraphicsMesh, IndexedMeshBuffers, MeshVertex, RawTriangleMesh, ScalarMesh,
    TriangleMeshF32, TriangleMeshF64, TriangleMeshI128,
};
pub use scalar::{AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter};

/// Axis-aligned bounds expressed in the adapter's host scalar type.
///
/// Native minimum and maximum coordinates are converted independently. With a
/// lossy float adapter these are rounded views, not certified outward bounds;
/// they need not enclose the original exact geometry. Public fields impose no
/// validation when callers construct this type directly.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb3<S> {
    /// Converted lower XYZ coordinates.
    pub mins: [S; 3],
    /// Converted upper XYZ coordinates.
    pub maxs: [S; 3],
}

/// Convert native extrema in minimum-then-maximum XYZ order.
///
/// Returns the first scalar export error. No outward rounding, topology proof,
/// or certainty receipt is added to the selected adapter's conversion contract.
fn convert_bounds<A: ScalarAdapter>(
    bounds: hyperlattice::Aabb,
) -> AdapterResult<Aabb3<A::Scalar>> {
    Ok(Aabb3 {
        mins: [
            A::from_real(&bounds.mins.x)?,
            A::from_real(&bounds.mins.y)?,
            A::from_real(&bounds.mins.z)?,
        ],
        maxs: [
            A::from_real(&bounds.maxs.x)?,
            A::from_real(&bounds.maxs.y)?,
            A::from_real(&bounds.maxs.z)?,
        ],
    })
}

/// Boundary regressions for primitive and exact scalar adapter strategies.
#[cfg(test)]
mod tests {
    #[cfg(feature = "curve")]
    use crate::adapter::ScalarCurve;
    use crate::adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarMesh};
    use hyperreal::Real;

    /// Checks translated cube bounds through the binary64 facade.
    #[test]
    fn f64_mesh_adapter_converts_at_edges() {
        let cube = ScalarMesh::<F64>::cube(2.0).unwrap();
        let moved = cube.translate(1.0, 0.0, 0.0).unwrap();
        let bounds = moved.bounding_box().unwrap().unwrap();

        assert_eq!(bounds.mins, [1.0, 0.0, 0.0]);
        assert_eq!(bounds.maxs, [3.0, 2.0, 2.0]);
    }

    /// Checks rejection of a known fraction and exact export of an integer.
    #[test]
    fn i128_adapter_rejects_fractional_output() {
        let one_half = (Real::one() / Real::from(2_u8)).unwrap();
        assert!(I128::from_real(&one_half).is_err());
        assert_eq!(I128::from_real(&Real::from(12_i128)).unwrap(), 12);
    }

    /// Checks exact planar bounds through the native-real curve facade.
    #[test]
    #[cfg(feature = "curve")]
    fn raw_real_profile_stays_exact_at_boundary() {
        let width = Real::from(3_i128);
        let profile = ScalarCurve::<RawReal>::square(width.clone()).unwrap();
        let bounds = profile.bounding_box().unwrap().unwrap();

        assert_eq!(bounds.mins, [Real::zero(), Real::zero(), Real::zero()]);
        assert_eq!(bounds.maxs, [width.clone(), width.clone(), Real::zero()]);
    }

    /// Checks matching vertex counts and index topology for binary32/64 exports.
    #[test]
    fn primitive_graphics_adapters_match_core_gpu_buffers() {
        let raw = ScalarMesh::<RawReal>::cube(Real::from(2))
            .unwrap()
            .into_native();

        let actual_f32 = ScalarMesh::<F32>::from_native(raw.clone())
            .graphics_mesh()
            .unwrap();
        let actual_f64 = ScalarMesh::<F64>::from_native(raw).graphics_mesh().unwrap();
        assert_eq!(actual_f32.vertices.len(), actual_f64.vertices.len());
        assert_eq!(actual_f32.indices, actual_f64.indices);
    }

    /// Checks cube vertex counts and indices through native-real and integer exports.
    #[test]
    fn exact_and_integer_graphics_adapters_keep_generic_conversion() {
        let raw = ScalarMesh::<RawReal>::cube(Real::from(2))
            .unwrap()
            .into_native();
        let actual = ScalarMesh::<RawReal>::from_native(raw)
            .graphics_mesh()
            .unwrap();
        assert_eq!(actual.vertices.len(), 36);
        assert_eq!(actual.indices.as_ref(), (0..36).collect::<Vec<_>>());

        let integer = ScalarMesh::<I128>::cube(2).unwrap().graphics_mesh().unwrap();
        assert_eq!(integer.vertices.len(), 36);
        assert_eq!(integer.indices.as_ref(), (0..36).collect::<Vec<_>>());
    }

    /// Checks that an oversized exact cube fails binary32 graphics export.
    #[test]
    fn f32_graphics_adapter_preserves_strict_overflow_error() {
        let mut huge = Real::from(2);
        for _ in 0..8 {
            huge = huge.clone() * huge;
        }
        let raw = ScalarMesh::<RawReal>::cube(huge).unwrap().into_native();

        assert_eq!(
            ScalarMesh::<F32>::from_native(raw).graphics_mesh(),
            Err(AdapterError::NotFiniteApproximation)
        );
    }
}
