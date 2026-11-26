#![allow(unsafe_code)]
#![allow(private_bounds, private_interfaces)]
#![allow(
    clippy::missing_safety_doc,
    reason = "the generated C ABI shares one crate-level pointer and ownership contract"
)]
#![deny(unused)]

//! C ABI for the scalar-adapted CSGRS surface.
//!
//! This workspace crate is the native binding boundary above `csgrs`. Four
//! scalar families choose ingress/egress representation; all handles retain
//! native Hyper geometry internally. Finite floats enter as their exact binary
//! values (`0.1_f64` is not one tenth). Float exports are lossy projections;
//! integer exports require a recognized in-range integer. Opaque real exports
//! retain native scalars, except that region profiles first pass through a
//! sampled binary64 display representation even in the real family.
//!
//! Geometry algorithms and validation remain in the adapters and owning kernels.
//! The ABI exposes no policy selector or separate operation-local certainty
//! receipt. Most decision-bearing paths select strict predicates; region
//! Booleans use Hypercurve's active scope (strict outside a preview/provisional
//! scope). Mesh construction history is queryable through
//! [`csgrs_triangle_mesh_construction_evidence`]; it is not a topology or
//! analytic-source fidelity certificate. Extrusion, revolution and profile
//! extraction explicitly discard their separate outcome receipts.
//!
//! Status-returning calls translate adapter errors and catch unwinding panics.
//! This does not catch aborts, allocation aborts, invalid-pointer access or
//! panics in the error-reporting code itself. Free functions have no unwind
//! wrapper. There is no ABI-wide work, memory or cancellation budget.
//!
//! # Safety
//!
//! Every exported function follows the declarations in `include/csgrs.h`.
//! Non-null pointers must be aligned, live and valid for their documented reads
//! or writes, without conflicting access during the call. Null pointers are
//! diagnosed where checked; they are not a substitute for validating non-null
//! addresses, lengths or lifetimes. Input arrays permit null at length zero;
//! nonempty arrays must satisfy Rust slice allocation/size requirements. Raw
//! real components embedded in vectors/matrices borrow live scalar handles.
//! Opaque handles must originate from this library and match the selected
//! family. Immutable input handles may be shared, including equal Boolean
//! operands; freeing them requires exclusive ownership and no outstanding use.
//!
//! Output slots must be writable without aliasing borrowed inputs. Successful
//! calls overwrite slots without freeing any prior ownership. Handle and buffer
//! outputs transfer ownership only on `Ok`; ordinary error paths leave their
//! slots untouched. Bounds return `NoBounds` without writing an empty record.
//! Release each successful allocation once with its matching free function,
//! preserving buffer pointers, lengths and nested real handles. Copying a
//! `Copy` ABI record does not duplicate ownership. Callers must copy borrowed
//! error text before the next ABI call on that thread or thread termination.

use csgrs::adapter::{
    Aabb3, AdapterError, F32, F64, I128, RawReal, Real, ScalarAdapter, ScalarCurve, ScalarMesh,
};
use hyperlattice::Matrix4;
use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::fmt;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;
use std::slice;

/// Native-backed mesh adapter selected by an ABI family; the scalar type is a boundary choice.
type TriangleMeshOf<F> = ScalarMesh<<F as Family>::Adapter>;
/// Native filled-region adapter selected by an ABI family, sharing Hypercurve authority.
type CurveRegionOf<F> = ScalarCurve<<F as Family>::Adapter>;
/// Host scalar used by the selected adapter before import or after export.
type AdapterScalar<F> = <<F as Family>::Adapter as ScalarAdapter>::Scalar;

/// Stable numeric result categories for C callers.
///
/// `NoBounds` is a completed empty query; only `Ok` writes an owned result.
/// The last-error string supplies diagnostic text, not a serialized native error tree.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CsgrsStatus {
    /// The operation completed and wrote any declared output.
    Ok = 0,
    /// A required pointer was null; non-null address validity is the caller's obligation.
    NullPointer = 1,
    /// A live geometry handle has another boundary-family tag.
    WrongScalarFamily = 2,
    /// Scalar import/export failed outside native construction validation.
    ConversionFailed = 3,
    /// An input or ABI encoding was rejected as invalid.
    ValidationFailed = 4,
    /// A required native decision could not be certified by the selected policy.
    PredicateUndecided = 5,
    /// A native operation reported exhaustion of a configured allowance.
    ResourceExhausted = 6,
    /// The native path does not support the requested representation or operation.
    Unsupported = 7,
    /// Other native construction failure not mapped to a more specific category.
    GeometryFailed = 8,
    /// The requested image/result is unbounded and cannot be returned as finite geometry.
    Unbounded = 9,
    /// The bounds query completed on empty geometry; no AABB was written.
    NoBounds = 10,
    /// Storage was unavailable without reaching a configured work limit.
    ResourceUnavailable = 11,
    /// The action unwound and was caught; aborts and memory unsafety are not caught.
    Panic = 255,
}

/// Boundary scalar strategy stored on a geometry handle; internal coordinates remain native.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CsgrsScalarFamily {
    /// Binary32 ingress and lossy binary32 egress.
    F32 = 1,
    /// Binary64 ingress and lossy binary64 egress.
    F64 = 2,
    /// Exact signed 128-bit integer ingress and restricted integer egress.
    I128 = 3,
    /// Borrowed/owned opaque native scalar handles at the boundary.
    Real = 4,
}

/// Construction summary. Each field is 0 or 1; unknown history cannot be
/// certified by interpreting `approximate_512_consumed == 0` alone.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsConstructionEvidence {
    /// Some source history is unavailable; zero in other flags cannot recover it.
    pub source_history_missing: u8,
    /// Retained construction history records use of the approximate terminal.
    pub approximate_512_consumed: u8,
    /// Retained history records a coordinate projection boundary.
    pub coordinates_projected: u8,
    /// Retained history records sampling of an intended source.
    pub source_sampled: u8,
    /// Native buffers changed in a way recorded by construction history.
    pub buffers_changed: u8,
}

/// Read retained mesh construction-history flags for any scalar family.
///
/// A false approximate flag cannot certify unknown history, topology or source
/// fidelity. Failure leaves the output record untouched.
///
/// # Safety
/// Borrow a live mesh and provide a nonaliasing writable output record, following
/// the crate pointer contract. No ownership is transferred by this query.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_triangle_mesh_construction_evidence(
    mesh: *const CsgrsTriangleMesh,
    out: *mut CsgrsConstructionEvidence,
) -> CsgrsStatus {
    ffi_status(|| {
        let mesh = unsafe { ptr_ref(mesh) }?;
        let evidence = match &mesh.inner {
            TriangleMeshKind::F32(mesh) => mesh.native().construction_evidence(),
            TriangleMeshKind::F64(mesh) => mesh.native().construction_evidence(),
            TriangleMeshKind::I128(mesh) => mesh.native().construction_evidence(),
            TriangleMeshKind::Real(mesh) => mesh.native().construction_evidence(),
        };
        *unsafe { out_ptr(out) }? = CsgrsConstructionEvidence {
            source_history_missing: u8::from(evidence.source_history_missing()),
            approximate_512_consumed: u8::from(evidence.approximate_512_consumed()),
            coordinates_projected: u8::from(evidence.coordinates_projected()),
            source_sampled: u8::from(evidence.source_sampled()),
            buffers_changed: u8::from(evidence.buffers_changed()),
        };
        Ok(())
    })
}

/// Portable two's-complement signed 128-bit value, represented as high and low words.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsI128 {
    /// Signed high 64 bits; sign extension preserves negative values.
    pub hi: i64,
    /// Unsigned low 64 bits.
    pub lo: u64,
}

/// 2-component binary32 coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2F32 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
}

/// 3-component binary32 coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3F32 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

/// 2-component binary64 coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2F64 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
}

/// 3-component binary64 coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3F64 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

/// 2-component split-word signed integer coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsVec2I128 {
    /// X component.
    pub x: CsgrsI128,
    /// Y component.
    pub y: CsgrsI128,
}

/// 3-component split-word signed integer coordinate record.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsVec3I128 {
    /// X component.
    pub x: CsgrsI128,
    /// Y component.
    pub y: CsgrsI128,
    /// Z component.
    pub z: CsgrsI128,
}

/// 2-component opaque native scalar coordinate record.
///
/// Inputs borrow each component; successful output records own separate handles.
/// Copying the record does not clone those handles.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2Real {
    /// X component.
    pub x: *mut CsgrsReal,
    /// Y component.
    pub y: *mut CsgrsReal,
}

/// 3-component opaque native scalar coordinate record.
///
/// Inputs borrow each component; successful output records own separate handles.
/// Copying the record does not clone those handles.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3Real {
    /// X component.
    pub x: *mut CsgrsReal,
    /// Y component.
    pub y: *mut CsgrsReal,
    /// Z component.
    pub z: *mut CsgrsReal,
}

/// Ordered triangle indices into the corresponding exported vertex array.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsTriangleU32 {
    /// First vertex index.
    pub a: u32,
    /// Second vertex index.
    pub b: u32,
    /// Third vertex index; order retains triangle winding.
    pub c: u32,
}

/// Axis-aligned bounds exported as binary32 endpoints.
///
/// Float endpoints are projections, not an outward-enclosure certificate.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3F32 {
    /// Minimum XYZ coordinates in the exported representation.
    pub mins: CsgrsVec3F32,
    /// Maximum XYZ coordinates in the exported representation.
    pub maxs: CsgrsVec3F32,
}

/// Axis-aligned bounds exported as binary64 endpoints.
///
/// Float endpoints are projections, not an outward-enclosure certificate.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3F64 {
    /// Minimum XYZ coordinates in the exported representation.
    pub mins: CsgrsVec3F64,
    /// Maximum XYZ coordinates in the exported representation.
    pub maxs: CsgrsVec3F64,
}

/// Axis-aligned bounds exported as split-word signed integer endpoints.
///
/// Export fails unless every endpoint is a recognized in-range integer.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsAabb3I128 {
    /// Minimum XYZ coordinates in the exported representation.
    pub mins: CsgrsVec3I128,
    /// Maximum XYZ coordinates in the exported representation.
    pub maxs: CsgrsVec3I128,
}

/// Axis-aligned bounds exported as opaque native scalar endpoints.
///
/// Owns six handles; release once with [`csgrs_aabb3_real_free_values`].
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3Real {
    /// Minimum XYZ coordinates in the exported representation.
    pub mins: CsgrsVec3Real,
    /// Maximum XYZ coordinates in the exported representation.
    pub maxs: CsgrsVec3Real,
}

/// Row-major 4×4 homogeneous transform with binary32 coefficients.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4F32 {
    /// Sixteen coefficients in row-major order.
    pub values: [f32; 16],
}

/// Row-major 4×4 homogeneous transform with binary64 coefficients.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4F64 {
    /// Sixteen coefficients in row-major order.
    pub values: [f64; 16],
}

/// Row-major 4×4 homogeneous transform with split-word signed integer coefficients.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsMatrix4I128 {
    /// Sixteen coefficients in row-major order.
    pub values: [CsgrsI128; 16],
}

/// Row-major 4×4 homogeneous transform with opaque native scalar coefficients.
///
/// All sixteen scalar handles are borrowed for the duration of the call.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4Real {
    /// Sixteen coefficients in row-major order.
    pub values: [*const CsgrsReal; 16],
}

/// Exported position and native-certified unit normal in the binary32 family.
///
/// Scalar export can round or fail; this record is not a separate fidelity receipt.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexF32 {
    /// Exported vertex position.
    pub position: CsgrsVec3F32,
    /// Exported unit-normal coordinates; projection can change their norm.
    pub normal: CsgrsVec3F32,
}

/// Exported position and native-certified unit normal in the binary64 family.
///
/// Scalar export can round or fail; this record is not a separate fidelity receipt.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexF64 {
    /// Exported vertex position.
    pub position: CsgrsVec3F64,
    /// Exported unit-normal coordinates; projection can change their norm.
    pub normal: CsgrsVec3F64,
}

/// Exported position and native-certified unit normal in the split-word signed integer family.
///
/// Scalar export can round or fail; this record is not a separate fidelity receipt.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsGraphicsVertexI128 {
    /// Exported vertex position.
    pub position: CsgrsVec3I128,
    /// Exported unit-normal coordinates; projection can change their norm.
    pub normal: CsgrsVec3I128,
}

/// Exported position and native-certified unit normal in the opaque native scalar family.
///
/// Scalar export can round or fail; this record is not a separate fidelity receipt.
/// Owns six scalar handles when returned inside a graphics mesh.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexReal {
    /// Exported vertex position.
    pub position: CsgrsVec3Real,
    /// Exported unit-normal coordinates; projection can change their norm.
    pub normal: CsgrsVec3Real,
}

/// Declare the family-specific owning buffer layouts shared with the C header.
///
/// Raw pointers carry no automatic destructor; generated free functions reclaim
/// the boxed arrays and, for the real family, their nested scalar handles.
macro_rules! define_buffers {
    ($mesh:ident, $graphics:ident, $region:ident, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        /// Owned indexed triangle buffers; `index_len` counts triangle records, not scalar indices.
        #[repr(C)]
        #[derive(Debug)]
        pub struct $mesh {
            /// Owned array of vertices; release with this family's matching buffer free.
            pub vertices: *mut $vec3,
            /// Number of vertex records, including zero for empty output.
            pub vertex_len: usize,
            /// Owned index array; its element type determines the count unit.
            pub indices: *mut CsgrsTriangleU32,
            /// Number of index-array elements, not bytes.
            pub index_len: usize,
        }

        /// Owned graphics vertices and flat index stream; `index_len` counts individual indices.
        #[repr(C)]
        #[derive(Debug)]
        pub struct $graphics {
            /// Owned array of vertices; release with this family's matching buffer free.
            pub vertices: *mut $gvertex,
            /// Number of vertex records, including zero for empty output.
            pub vertex_len: usize,
            /// Owned index array; its element type determines the count unit.
            pub indices: *mut u32,
            /// Number of index-array elements, not bytes.
            pub index_len: usize,
        }

        /// Owned display-profile rows with two levels of sentinel offsets.
        ///
        /// `ring_offsets` indexes points and `region_offsets` indexes rings. Each
        /// region lists its exterior first, then holes. Even the real family stores
        /// sampled/projected profiles rather than exact source curves.
        #[repr(C)]
        #[derive(Debug)]
        pub struct $region {
            /// Owned flattened XY point array.
            pub points: *mut $vec2,
            /// Number of points.
            pub point_len: usize,
            /// Owned point offsets, starting at zero and ending at `point_len`.
            pub ring_offsets: *mut usize,
            /// Number of ring offsets: one more than the ring count.
            pub ring_offset_len: usize,
            /// Owned ring offsets, starting at zero and ending at the ring count.
            pub region_offsets: *mut usize,
            /// Number of region offsets: one more than the region count.
            pub region_offset_len: usize,
        }
    };
}

define_buffers!(
    CsgrsTriangleMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);
define_buffers!(
    CsgrsTriangleMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);
define_buffers!(
    CsgrsTriangleMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);
define_buffers!(
    CsgrsTriangleMeshBuffersReal,
    CsgrsGraphicsMeshReal,
    CsgrsRegionProfilesReal,
    CsgrsVec2Real,
    CsgrsVec3Real,
    CsgrsGraphicsVertexReal
);

/// Opaque owned native scalar handle; created and released only by this library.
pub struct CsgrsReal {
    /// Native payload whose ownership is transferred by the opaque handle.
    inner: Real,
}

/// Opaque owned mesh plus boundary-family tag; operations borrow it and return new handles.
pub struct CsgrsTriangleMesh {
    /// Native payload whose ownership is transferred by the opaque handle.
    inner: TriangleMeshKind,
}

/// Opaque owned filled region plus boundary-family tag; never inspect its layout from C.
pub struct CsgrsCurveRegion {
    /// Native payload whose ownership is transferred by the opaque handle.
    inner: CurveRegionKind,
}

/// Runtime family discriminator for native-backed mesh storage.
enum TriangleMeshKind {
    /// Geometry exposed through the `F32` scalar family.
    F32(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarMesh<F32>,
    ),
    /// Geometry exposed through the `F64` scalar family.
    F64(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarMesh<F64>,
    ),
    /// Geometry exposed through the `I128` scalar family.
    I128(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarMesh<I128>,
    ),
    /// Geometry exposed through the `Real` scalar family.
    Real(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarMesh<RawReal>,
    ),
}

/// Runtime family discriminator for native-backed region storage.
enum CurveRegionKind {
    /// Geometry exposed through the `F32` scalar family.
    F32(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarCurve<F32>,
    ),
    /// Geometry exposed through the `F64` scalar family.
    F64(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarCurve<F64>,
    ),
    /// Geometry exposed through the `I128` scalar family.
    I128(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarCurve<I128>,
    ),
    /// Geometry exposed through the `Real` scalar family.
    Real(
        /// Authoritative native geometry wrapped by the selected boundary adapter.
        ScalarCurve<RawReal>,
    ),
}

/// Private error bridge preserving native construction categories until status mapping.
#[derive(Debug)]
enum FfiError {
    /// A required pointer is null.
    NullPointer,
    /// A geometry handle belongs to another ABI scalar family.
    WrongScalarFamily,
    /// The adapter failure, including its typed native construction cause.
    Adapter(
        /// Error retained until diagnostic formatting and status selection.
        AdapterError,
    ),
    /// A valid bounds query completed without an AABB.
    NoBounds,
}

impl fmt::Display for FfiError {
    /// Format pointer/family/absence errors or delegate the native adapter diagnostic.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NullPointer => f.write_str("null pointer"),
            Self::WrongScalarFamily => f.write_str("handle belongs to another scalar family"),
            Self::Adapter(error) => write!(f, "{error}"),
            Self::NoBounds => f.write_str("empty geometry has no bounds"),
        }
    }
}

impl From<AdapterError> for FfiError {
    /// Preserve the adapter error for typed status mapping and diagnostic display.
    fn from(value: AdapterError) -> Self {
        Self::Adapter(value)
    }
}

/// Internal adapter/pointer outcome before conversion to a C status and diagnostic.
type FfiResult<T> = Result<T, FfiError>;

thread_local! {
    /// Thread-local owned diagnostic; replacing it invalidates previously returned C pointers.
    static LAST_ERROR: RefCell<CString> =
        RefCell::new(CString::new("").expect("empty string has no interior nul"));
}

/// Compile-time bridge from one C layout to its native-backed scalar adapter.
///
/// Implementations borrow raw real pointers under the exported caller contract.
/// These safe private methods rely on that boundary; they do not validate non-null
/// addresses. Export constructors transfer arrays without RAII for nested handles.
trait Family {
    /// Native scalar conversion strategy.
    type Adapter: ScalarAdapter;
    /// C scalar input representation; real handles are borrowed.
    type CScalar: Copy;
    /// XY coordinate layout.
    type CVec2: Copy;
    /// XYZ coordinate layout.
    type CVec3: Copy;
    /// Row-major homogeneous matrix layout.
    type CMatrix4: Copy;
    /// Exported bound layout.
    type CAabb: Copy;
    /// Exported position/normal layout.
    type CGraphicsVertex: Copy;
    /// Owning indexed-mesh array descriptor.
    type CMeshBuffers;
    /// Owning graphics array descriptor.
    type CGraphicsMesh;
    /// Owning sampled-profile array descriptor.
    type CRegionProfiles;

    /// Convert the C scalar representation to the adapter scalar.
    ///
    /// Primitive floats are only copied here; finite validation occurs at native
    /// import. The real family clones the borrowed handle under the caller contract.
    fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<AdapterScalar<Self>>;
    /// Import X then Y through this family, stopping at the first conversion error.
    fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[AdapterScalar<Self>; 2]>;
    /// Import X, Y then Z through this family, stopping at the first conversion error.
    fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[AdapterScalar<Self>; 3]>;
    /// Pack two already-exported adapter scalars; the real family allocates two owned handles.
    fn vec2_from_adapter(value: [AdapterScalar<Self>; 2]) -> FfiResult<Self::CVec2>;
    /// Pack three already-exported adapter scalars; the real family allocates three owned handles.
    fn vec3_from_adapter(value: [AdapterScalar<Self>; 3]) -> FfiResult<Self::CVec3>;
    /// Import all sixteen row-major coefficients into a native homogeneous matrix.
    ///
    /// This conversion does not establish invertibility or geometric applicability.
    fn matrix4_to_core(value: Self::CMatrix4) -> FfiResult<Matrix4>;
    /// Pack already-exported extrema; the real family allocates six owned handles.
    fn aabb_from_adapter(value: Aabb3<AdapterScalar<Self>>) -> FfiResult<Self::CAabb>;
    /// Pack already-exported position/normal rows, allocating handles in the real family.
    fn graphics_vertex_from_adapter(
        position: [AdapterScalar<Self>; 3],
        normal: [AdapterScalar<Self>; 3],
    ) -> FfiResult<Self::CGraphicsVertex>;
    /// Tag a native-backed mesh with this boundary family without converting its coordinates.
    fn triangle_mesh_kind(mesh: TriangleMeshOf<Self>) -> TriangleMeshKind;
    /// Tag a native-backed region with this boundary family without converting its coordinates.
    fn curve_region_kind(region: CurveRegionOf<Self>) -> CurveRegionKind;
    /// Borrow the mesh payload only when its stored family matches this implementation.
    fn triangle_mesh_ref(mesh: &CsgrsTriangleMesh) -> FfiResult<&TriangleMeshOf<Self>>;
    /// Borrow the region payload only when its stored family matches this implementation.
    fn curve_region_ref(region: &CsgrsCurveRegion) -> FfiResult<&CurveRegionOf<Self>>;
    /// Transfer vertex and triangle vectors into owned boxed-array ABI descriptors.
    fn mesh_buffers(
        vertices: Vec<Self::CVec3>,
        indices: Vec<CsgrsTriangleU32>,
    ) -> Self::CMeshBuffers;
    /// Transfer graphics vertices and flat indices into owned boxed-array ABI descriptors.
    fn graphics_mesh(
        vertices: Vec<Self::CGraphicsVertex>,
        indices: Vec<u32>,
    ) -> Self::CGraphicsMesh;
    /// Transfer flattened points and both sentinel-offset vectors into ABI ownership.
    fn region_profiles(
        points: Vec<Self::CVec2>,
        ring_offsets: Vec<usize>,
        region_offsets: Vec<usize>,
    ) -> Self::CRegionProfiles;
}

/// Zero-sized dispatch marker for the `F32` ABI family.
struct F32Family;
/// Zero-sized dispatch marker for the `F64` ABI family.
struct F64Family;
/// Zero-sized dispatch marker for the `I128` ABI family.
struct I128Family;
/// Zero-sized dispatch marker for the `Real` ABI family.
struct RealFamily;

/// Reassemble signed high and unsigned low words without losing two's-complement bits.
fn i128_from_parts(value: CsgrsI128) -> i128 {
    (i128::from(value.hi) << 64) | i128::from(value.lo)
}

/// Split an integer into signed high and unsigned low words without numeric conversion.
fn i128_to_parts(value: i128) -> CsgrsI128 {
    CsgrsI128 {
        hi: (value >> 64) as i64,
        lo: value as u64,
    }
}

/// Allocate one opaque scalar owner; the caller must later free the returned handle.
fn real_handle(value: Real) -> *mut CsgrsReal {
    Box::into_raw(Box::new(CsgrsReal { inner: value }))
}

/// Clone a borrowed native scalar, diagnosing null without taking ownership.
///
/// # Safety
/// A non-null pointer must reference a live scalar readable for the call.
unsafe fn real_from_ptr(ptr: *const CsgrsReal) -> FfiResult<Real> {
    if ptr.is_null() {
        return Err(FfiError::NullPointer);
    }
    Ok(unsafe { &*ptr }.inner.clone())
}

/// Require a directly recognized rational that converts to an in-range integer.
///
/// Does not refine an unrecognized expression to decide integrality.
fn real_to_i128(value: &Real) -> FfiResult<i128> {
    let rational = value.exact_rational().ok_or(AdapterError::NotExactInteger)?;
    rational
        .try_into()
        .map_err(AdapterError::from)
        .map_err(Into::into)
}

/// Implement value-layout conversions and family checks for binary floats and split integers.
///
/// Scalar validation belongs to native ingress. The accepted `$family_tag` and
/// `$scalar_from_real` arguments are not used by this expansion.
macro_rules! impl_primitive_family {
    (
        $family:ty,
        $adapter:ty,
        $scalar:ty,
        $vec2:ty,
        $vec3:ty,
        $matrix:ty,
        $aabb:ty,
        $gvertex:ty,
        $mesh_buffers:ty,
        $graphics_mesh:ty,
        $region_profiles:ty,
        $mesh_variant:ident,
        $region_variant:ident,
        $family_tag:expr,
        $scalar_to_adapter:expr,
        $scalar_from_real:expr,
        $vec2_to_array:expr,
        $vec3_to_array:expr,
        $vec2_from_array:expr,
        $vec3_from_array:expr,
        $matrix_values:expr,
        $aabb_from_parts:expr,
        $gvertex_from_parts:expr
    ) => {
        impl Family for $family {
            /// Native scalar conversion strategy.
            type Adapter = $adapter;
            /// C scalar input representation; real handles are borrowed.
            type CScalar = $scalar;
            /// XY coordinate layout.
            type CVec2 = $vec2;
            /// XYZ coordinate layout.
            type CVec3 = $vec3;
            /// Row-major homogeneous matrix layout.
            type CMatrix4 = $matrix;
            /// Exported bound layout.
            type CAabb = $aabb;
            /// Exported position/normal layout.
            type CGraphicsVertex = $gvertex;
            /// Owning indexed-mesh array descriptor.
            type CMeshBuffers = $mesh_buffers;
            /// Owning graphics array descriptor.
            type CGraphicsMesh = $graphics_mesh;
            /// Owning sampled-profile array descriptor.
            type CRegionProfiles = $region_profiles;

            /// Convert the C scalar representation to the adapter scalar.
            ///
            /// Primitive floats are only copied here; finite validation occurs at native
            /// import. The real family clones the borrowed handle under the caller contract.
            fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<AdapterScalar<Self>> {
                ($scalar_to_adapter)(value)
            }

            /// Import X then Y through this family, stopping at the first conversion error.
            fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[AdapterScalar<Self>; 2]> {
                let [x, y] = ($vec2_to_array)(value);
                Ok([Self::scalar_to_adapter(x)?, Self::scalar_to_adapter(y)?])
            }

            /// Import X, Y then Z through this family, stopping at the first conversion error.
            fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[AdapterScalar<Self>; 3]> {
                let [x, y, z] = ($vec3_to_array)(value);
                Ok([
                    Self::scalar_to_adapter(x)?,
                    Self::scalar_to_adapter(y)?,
                    Self::scalar_to_adapter(z)?,
                ])
            }

            /// Pack two already-exported adapter scalars; the real family allocates two owned handles.
            fn vec2_from_adapter(value: [AdapterScalar<Self>; 2]) -> FfiResult<Self::CVec2> {
                let [x, y] = value;
                ($vec2_from_array)([x, y])
            }

            /// Pack three already-exported adapter scalars; the real family allocates three owned handles.
            fn vec3_from_adapter(value: [AdapterScalar<Self>; 3]) -> FfiResult<Self::CVec3> {
                let [x, y, z] = value;
                ($vec3_from_array)([x, y, z])
            }

            /// Import all sixteen row-major coefficients into a native homogeneous matrix.
            ///
            /// This conversion does not establish invertibility or geometric applicability.
            fn matrix4_to_core(value: Self::CMatrix4) -> FfiResult<Matrix4> {
                let values = ($matrix_values)(value);
                let values = values
                    .into_iter()
                    .map(Self::scalar_to_adapter)
                    .collect::<FfiResult<Vec<_>>>()?;
                let values: [AdapterScalar<Self>; 16] = values.try_into().map_err(|_| {
                    AdapterError::Validation("matrix must have 16 values".into())
                })?;
                let values = values
                    .map(|value| <$adapter as ScalarAdapter>::into_real(value))
                    .into_iter()
                    .collect::<Result<Vec<_>, _>>()?;
                let values: [Real; 16] = values.try_into().map_err(|_| {
                    AdapterError::Validation("matrix must have 16 values".into())
                })?;
                Ok(Matrix4::from_row_major(values))
            }

            /// Pack already-exported extrema; the real family allocates six owned handles.
            fn aabb_from_adapter(value: Aabb3<AdapterScalar<Self>>) -> FfiResult<Self::CAabb> {
                ($aabb_from_parts)(value)
            }

            /// Pack already-exported position/normal rows, allocating handles in the real family.
            fn graphics_vertex_from_adapter(
                position: [AdapterScalar<Self>; 3],
                normal: [AdapterScalar<Self>; 3],
            ) -> FfiResult<Self::CGraphicsVertex> {
                ($gvertex_from_parts)(position, normal)
            }

            /// Tag a native-backed mesh with this boundary family without converting its coordinates.
            fn triangle_mesh_kind(mesh: TriangleMeshOf<Self>) -> TriangleMeshKind {
                TriangleMeshKind::$mesh_variant(mesh)
            }

            /// Tag a native-backed region with this boundary family without converting its coordinates.
            fn curve_region_kind(region: CurveRegionOf<Self>) -> CurveRegionKind {
                CurveRegionKind::$region_variant(region)
            }

            /// Borrow the mesh payload only when its stored family matches this implementation.
            fn triangle_mesh_ref(
                mesh: &CsgrsTriangleMesh,
            ) -> FfiResult<&TriangleMeshOf<Self>> {
                match &mesh.inner {
                    TriangleMeshKind::$mesh_variant(mesh) => Ok(mesh),
                    _ => Err(FfiError::WrongScalarFamily),
                }
            }

            /// Borrow the region payload only when its stored family matches this implementation.
            fn curve_region_ref(region: &CsgrsCurveRegion) -> FfiResult<&CurveRegionOf<Self>> {
                match &region.inner {
                    CurveRegionKind::$region_variant(region) => Ok(region),
                    _ => Err(FfiError::WrongScalarFamily),
                }
            }

            /// Transfer vertex and triangle vectors into owned boxed-array ABI descriptors.
            fn mesh_buffers(
                vertices: Vec<Self::CVec3>,
                indices: Vec<CsgrsTriangleU32>,
            ) -> Self::CMeshBuffers {
                let (vertices, vertex_len) = boxed_slice_parts(vertices);
                let (indices, index_len) = boxed_slice_parts(indices);
                <$mesh_buffers>::new(vertices, vertex_len, indices, index_len)
            }

            /// Transfer graphics vertices and flat indices into owned boxed-array ABI descriptors.
            fn graphics_mesh(
                vertices: Vec<Self::CGraphicsVertex>,
                indices: Vec<u32>,
            ) -> Self::CGraphicsMesh {
                let (vertices, vertex_len) = boxed_slice_parts(vertices);
                let (indices, index_len) = boxed_slice_parts(indices);
                <$graphics_mesh>::new(vertices, vertex_len, indices, index_len)
            }

            /// Transfer flattened points and both sentinel-offset vectors into ABI ownership.
            fn region_profiles(
                points: Vec<Self::CVec2>,
                ring_offsets: Vec<usize>,
                region_offsets: Vec<usize>,
            ) -> Self::CRegionProfiles {
                let (points, point_len) = boxed_slice_parts(points);
                let (ring_offsets, ring_offset_len) = boxed_slice_parts(ring_offsets);
                let (region_offsets, region_offset_len) = boxed_slice_parts(region_offsets);
                <$region_profiles>::new(
                    points,
                    point_len,
                    ring_offsets,
                    ring_offset_len,
                    region_offsets,
                    region_offset_len,
                )
            }
        }
    };
}

/// Provide private descriptor constructors; no allocation, pointer validation or destructor is added.
macro_rules! impl_buffer_constructors {
    ($mesh:ty, $graphics:ty, $region:ty, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        impl $mesh {
            /// Assemble a descriptor from arrays whose ownership was already transferred.
            ///
            /// Lengths count the corresponding element type; this constructor performs no validation.
            fn new(
                vertices: *mut $vec3,
                vertex_len: usize,
                indices: *mut CsgrsTriangleU32,
                index_len: usize,
            ) -> Self {
                Self {
                    vertices,
                    vertex_len,
                    indices,
                    index_len,
                }
            }
        }

        impl $graphics {
            /// Assemble a descriptor from arrays whose ownership was already transferred.
            ///
            /// Lengths count the corresponding element type; this constructor performs no validation.
            fn new(
                vertices: *mut $gvertex,
                vertex_len: usize,
                indices: *mut u32,
                index_len: usize,
            ) -> Self {
                Self {
                    vertices,
                    vertex_len,
                    indices,
                    index_len,
                }
            }
        }

        impl $region {
            /// Assemble a descriptor from arrays whose ownership was already transferred.
            ///
            /// Lengths count the corresponding element type; this constructor performs no validation.
            fn new(
                points: *mut $vec2,
                point_len: usize,
                ring_offsets: *mut usize,
                ring_offset_len: usize,
                region_offsets: *mut usize,
                region_offset_len: usize,
            ) -> Self {
                Self {
                    points,
                    point_len,
                    ring_offsets,
                    ring_offset_len,
                    region_offsets,
                    region_offset_len,
                }
            }
        }
    };
}

impl_buffer_constructors!(
    CsgrsTriangleMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);
impl_buffer_constructors!(
    CsgrsTriangleMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);
impl_buffer_constructors!(
    CsgrsTriangleMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);
impl_buffer_constructors!(
    CsgrsTriangleMeshBuffersReal,
    CsgrsGraphicsMeshReal,
    CsgrsRegionProfilesReal,
    CsgrsVec2Real,
    CsgrsVec3Real,
    CsgrsGraphicsVertexReal
);

impl_primitive_family!(
    F32Family,
    F32,
    f32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsMatrix4F32,
    CsgrsAabb3F32,
    CsgrsGraphicsVertexF32,
    CsgrsTriangleMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    F32,
    F32,
    CsgrsScalarFamily::F32,
    |value: f32| Ok(value),
    |value: &Real| F32::from_real(value).map_err(Into::into),
    |value: CsgrsVec2F32| [value.x, value.y],
    |value: CsgrsVec3F32| [value.x, value.y, value.z],
    |value: [f32; 2]| Ok(CsgrsVec2F32 {
        x: value[0],
        y: value[1]
    }),
    |value: [f32; 3]| Ok(CsgrsVec3F32 {
        x: value[0],
        y: value[1],
        z: value[2]
    }),
    |value: CsgrsMatrix4F32| value.values,
    |value: Aabb3<f32>| Ok(CsgrsAabb3F32 {
        mins: CsgrsVec3F32 {
            x: value.mins[0],
            y: value.mins[1],
            z: value.mins[2]
        },
        maxs: CsgrsVec3F32 {
            x: value.maxs[0],
            y: value.maxs[1],
            z: value.maxs[2]
        }
    }),
    |position: [f32; 3], normal: [f32; 3]| Ok(CsgrsGraphicsVertexF32 {
        position: CsgrsVec3F32 {
            x: position[0],
            y: position[1],
            z: position[2]
        },
        normal: CsgrsVec3F32 {
            x: normal[0],
            y: normal[1],
            z: normal[2]
        }
    })
);

impl_primitive_family!(
    F64Family,
    F64,
    f64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsMatrix4F64,
    CsgrsAabb3F64,
    CsgrsGraphicsVertexF64,
    CsgrsTriangleMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    F64,
    F64,
    CsgrsScalarFamily::F64,
    |value: f64| Ok(value),
    |value: &Real| F64::from_real(value).map_err(Into::into),
    |value: CsgrsVec2F64| [value.x, value.y],
    |value: CsgrsVec3F64| [value.x, value.y, value.z],
    |value: [f64; 2]| Ok(CsgrsVec2F64 {
        x: value[0],
        y: value[1]
    }),
    |value: [f64; 3]| Ok(CsgrsVec3F64 {
        x: value[0],
        y: value[1],
        z: value[2]
    }),
    |value: CsgrsMatrix4F64| value.values,
    |value: Aabb3<f64>| Ok(CsgrsAabb3F64 {
        mins: CsgrsVec3F64 {
            x: value.mins[0],
            y: value.mins[1],
            z: value.mins[2]
        },
        maxs: CsgrsVec3F64 {
            x: value.maxs[0],
            y: value.maxs[1],
            z: value.maxs[2]
        }
    }),
    |position: [f64; 3], normal: [f64; 3]| Ok(CsgrsGraphicsVertexF64 {
        position: CsgrsVec3F64 {
            x: position[0],
            y: position[1],
            z: position[2]
        },
        normal: CsgrsVec3F64 {
            x: normal[0],
            y: normal[1],
            z: normal[2]
        }
    })
);

impl_primitive_family!(
    I128Family,
    I128,
    CsgrsI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsMatrix4I128,
    CsgrsAabb3I128,
    CsgrsGraphicsVertexI128,
    CsgrsTriangleMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    I128,
    I128,
    CsgrsScalarFamily::I128,
    |value: CsgrsI128| Ok(i128_from_parts(value)),
    |value: &Real| real_to_i128(value).map(i128_to_parts),
    |value: CsgrsVec2I128| [value.x, value.y],
    |value: CsgrsVec3I128| [value.x, value.y, value.z],
    |value: [i128; 2]| Ok(CsgrsVec2I128 {
        x: i128_to_parts(value[0]),
        y: i128_to_parts(value[1])
    }),
    |value: [i128; 3]| Ok(CsgrsVec3I128 {
        x: i128_to_parts(value[0]),
        y: i128_to_parts(value[1]),
        z: i128_to_parts(value[2])
    }),
    |value: CsgrsMatrix4I128| value.values,
    |value: Aabb3<i128>| Ok(CsgrsAabb3I128 {
        mins: CsgrsVec3I128 {
            x: i128_to_parts(value.mins[0]),
            y: i128_to_parts(value.mins[1]),
            z: i128_to_parts(value.mins[2])
        },
        maxs: CsgrsVec3I128 {
            x: i128_to_parts(value.maxs[0]),
            y: i128_to_parts(value.maxs[1]),
            z: i128_to_parts(value.maxs[2])
        }
    }),
    |position: [i128; 3], normal: [i128; 3]| Ok(CsgrsGraphicsVertexI128 {
        position: CsgrsVec3I128 {
            x: i128_to_parts(position[0]),
            y: i128_to_parts(position[1]),
            z: i128_to_parts(position[2])
        },
        normal: CsgrsVec3I128 {
            x: i128_to_parts(normal[0]),
            y: i128_to_parts(normal[1]),
            z: i128_to_parts(normal[2])
        }
    })
);

impl Family for RealFamily {
    /// Native scalar conversion strategy.
    type Adapter = RawReal;
    /// C scalar input representation; real handles are borrowed.
    type CScalar = *const CsgrsReal;
    /// XY coordinate layout.
    type CVec2 = CsgrsVec2Real;
    /// XYZ coordinate layout.
    type CVec3 = CsgrsVec3Real;
    /// Row-major homogeneous matrix layout.
    type CMatrix4 = CsgrsMatrix4Real;
    /// Exported bound layout.
    type CAabb = CsgrsAabb3Real;
    /// Exported position/normal layout.
    type CGraphicsVertex = CsgrsGraphicsVertexReal;
    /// Owning indexed-mesh array descriptor.
    type CMeshBuffers = CsgrsTriangleMeshBuffersReal;
    /// Owning graphics array descriptor.
    type CGraphicsMesh = CsgrsGraphicsMeshReal;
    /// Owning sampled-profile array descriptor.
    type CRegionProfiles = CsgrsRegionProfilesReal;

    /// Convert the C scalar representation to the adapter scalar.
    ///
    /// Primitive floats are only copied here; finite validation occurs at native
    /// import. The real family clones the borrowed handle under the caller contract.
    fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<Real> {
        unsafe { real_from_ptr(value) }
    }

    /// Import X then Y through this family, stopping at the first conversion error.
    fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[Real; 2]> {
        Ok([
            Self::scalar_to_adapter(value.x)?,
            Self::scalar_to_adapter(value.y)?,
        ])
    }

    /// Import X, Y then Z through this family, stopping at the first conversion error.
    fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[Real; 3]> {
        Ok([
            Self::scalar_to_adapter(value.x)?,
            Self::scalar_to_adapter(value.y)?,
            Self::scalar_to_adapter(value.z)?,
        ])
    }

    /// Pack two already-exported adapter scalars; the real family allocates two owned handles.
    fn vec2_from_adapter(value: [Real; 2]) -> FfiResult<Self::CVec2> {
        let [x, y] = value;
        Ok(CsgrsVec2Real {
            x: real_handle(x),
            y: real_handle(y),
        })
    }

    /// Pack three already-exported adapter scalars; the real family allocates three owned handles.
    fn vec3_from_adapter(value: [Real; 3]) -> FfiResult<Self::CVec3> {
        let [x, y, z] = value;
        Ok(CsgrsVec3Real {
            x: real_handle(x),
            y: real_handle(y),
            z: real_handle(z),
        })
    }

    /// Import all sixteen row-major coefficients into a native homogeneous matrix.
    ///
    /// This conversion does not establish invertibility or geometric applicability.
    fn matrix4_to_core(value: Self::CMatrix4) -> FfiResult<Matrix4> {
        let values = value
            .values
            .into_iter()
            .map(Self::scalar_to_adapter)
            .collect::<FfiResult<Vec<_>>>()?;
        let values: [Real; 16] = values
            .try_into()
            .map_err(|_| AdapterError::Validation("matrix must have 16 values".into()))?;
        Ok(Matrix4::from_row_major(values))
    }

    /// Pack already-exported extrema; the real family allocates six owned handles.
    fn aabb_from_adapter(value: Aabb3<Real>) -> FfiResult<Self::CAabb> {
        let [min_x, min_y, min_z] = value.mins;
        let [max_x, max_y, max_z] = value.maxs;
        Ok(CsgrsAabb3Real {
            mins: CsgrsVec3Real {
                x: real_handle(min_x),
                y: real_handle(min_y),
                z: real_handle(min_z),
            },
            maxs: CsgrsVec3Real {
                x: real_handle(max_x),
                y: real_handle(max_y),
                z: real_handle(max_z),
            },
        })
    }

    /// Pack already-exported position/normal rows, allocating handles in the real family.
    fn graphics_vertex_from_adapter(
        position: [Real; 3],
        normal: [Real; 3],
    ) -> FfiResult<Self::CGraphicsVertex> {
        Ok(CsgrsGraphicsVertexReal {
            position: Self::vec3_from_adapter(position)?,
            normal: Self::vec3_from_adapter(normal)?,
        })
    }

    /// Tag a native-backed mesh with this boundary family without converting its coordinates.
    fn triangle_mesh_kind(mesh: TriangleMeshOf<Self>) -> TriangleMeshKind {
        TriangleMeshKind::Real(mesh)
    }

    /// Tag a native-backed region with this boundary family without converting its coordinates.
    fn curve_region_kind(region: CurveRegionOf<Self>) -> CurveRegionKind {
        CurveRegionKind::Real(region)
    }

    /// Borrow the mesh payload only when its stored family matches this implementation.
    fn triangle_mesh_ref(mesh: &CsgrsTriangleMesh) -> FfiResult<&TriangleMeshOf<Self>> {
        match &mesh.inner {
            TriangleMeshKind::Real(mesh) => Ok(mesh),
            _ => Err(FfiError::WrongScalarFamily),
        }
    }

    /// Borrow the region payload only when its stored family matches this implementation.
    fn curve_region_ref(region: &CsgrsCurveRegion) -> FfiResult<&CurveRegionOf<Self>> {
        match &region.inner {
            CurveRegionKind::Real(region) => Ok(region),
            _ => Err(FfiError::WrongScalarFamily),
        }
    }

    /// Transfer vertex and triangle vectors into owned boxed-array ABI descriptors.
    fn mesh_buffers(
        vertices: Vec<Self::CVec3>,
        indices: Vec<CsgrsTriangleU32>,
    ) -> Self::CMeshBuffers {
        let (vertices, vertex_len) = boxed_slice_parts(vertices);
        let (indices, index_len) = boxed_slice_parts(indices);
        CsgrsTriangleMeshBuffersReal::new(vertices, vertex_len, indices, index_len)
    }

    /// Transfer graphics vertices and flat indices into owned boxed-array ABI descriptors.
    fn graphics_mesh(
        vertices: Vec<Self::CGraphicsVertex>,
        indices: Vec<u32>,
    ) -> Self::CGraphicsMesh {
        let (vertices, vertex_len) = boxed_slice_parts(vertices);
        let (indices, index_len) = boxed_slice_parts(indices);
        CsgrsGraphicsMeshReal::new(vertices, vertex_len, indices, index_len)
    }

    /// Transfer flattened points and both sentinel-offset vectors into ABI ownership.
    fn region_profiles(
        points: Vec<Self::CVec2>,
        ring_offsets: Vec<usize>,
        region_offsets: Vec<usize>,
    ) -> Self::CRegionProfiles {
        let (points, point_len) = boxed_slice_parts(points);
        let (ring_offsets, ring_offset_len) = boxed_slice_parts(ring_offsets);
        let (region_offsets, region_offset_len) = boxed_slice_parts(region_offsets);
        CsgrsRegionProfilesReal::new(
            points,
            point_len,
            ring_offsets,
            ring_offset_len,
            region_offsets,
            region_offset_len,
        )
    }
}

/// Transfer a vector into a boxed array and return its thin pointer and element count.
///
/// An empty array may have a non-null dangling pointer valid only for zero length.
fn boxed_slice_parts<T>(values: Vec<T>) -> (*mut T, usize) {
    let boxed = values.into_boxed_slice();
    let len = boxed.len();
    (Box::into_raw(boxed) as *mut T, len)
}

/// Reclaim an array allocated by `boxed_slice_parts`; null is a no-op.
///
/// # Safety
/// A non-null pointer/length must describe the original boxed slice, still owned
/// exactly once and with no live references. Length is in elements, not bytes.
unsafe fn free_boxed_slice<T>(ptr: *mut T, len: usize) {
    if !ptr.is_null() {
        let slice = ptr::slice_from_raw_parts_mut(ptr, len);
        unsafe {
            drop(Box::from_raw(slice));
        }
    }
}

/// Replace this thread's diagnostic, escaping interior NULs as printable `\0`.
///
/// Allocation/formatting here is outside the action's panic catcher.
fn set_last_error(message: impl fmt::Display) {
    let sanitized = message.to_string().replace('\0', "\\0");
    let c_string = CString::new(sanitized).unwrap_or_else(|_| {
        CString::new("failed to format error").expect("literal has no interior nul")
    });
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = c_string;
    });
}

/// Map pointer, family and bounds outcomes directly; preserve typed construction failures.
///
/// Other adapter failures collapse to `ConversionFailed`; details remain in text.
fn status_for_error(error: &FfiError) -> CsgrsStatus {
    match error {
        FfiError::NullPointer => CsgrsStatus::NullPointer,
        FfiError::WrongScalarFamily => CsgrsStatus::WrongScalarFamily,
        FfiError::NoBounds => CsgrsStatus::NoBounds,
        FfiError::Adapter(AdapterError::Validation(_)) => CsgrsStatus::ValidationFailed,
        FfiError::Adapter(AdapterError::Construction(error)) => {
            status_for_validation_kind(error.kind())
        },
        FfiError::Adapter(_) => CsgrsStatus::ConversionFailed,
    }
}

/// Map native validation categories to their stable ABI statuses, with a generic fallback.
fn status_for_validation_kind(kind: csgrs::errors::ValidationErrorKind) -> CsgrsStatus {
    use csgrs::errors::ValidationErrorKind;
    match kind {
        ValidationErrorKind::InvalidInput => CsgrsStatus::ValidationFailed,
        ValidationErrorKind::Undecided => CsgrsStatus::PredicateUndecided,
        ValidationErrorKind::Unbounded => CsgrsStatus::Unbounded,
        ValidationErrorKind::ResourceExhausted => CsgrsStatus::ResourceExhausted,
        ValidationErrorKind::ResourceUnavailable => CsgrsStatus::ResourceUnavailable,
        ValidationErrorKind::Unsupported => CsgrsStatus::Unsupported,
        _ => CsgrsStatus::GeometryFailed,
    }
}

/// Run an action under an unwind catcher, then publish status and thread-local text.
///
/// Success clears prior text; errors format it, and caught panics get generic text.
/// Does not undo side effects or reclaim raw allocations leaked by a failing action.
fn ffi_status(action: impl FnOnce() -> FfiResult<()>) -> CsgrsStatus {
    match catch_unwind(AssertUnwindSafe(action)) {
        Ok(Ok(())) => {
            set_last_error("");
            CsgrsStatus::Ok
        },
        Ok(Err(error)) => {
            set_last_error(&error);
            status_for_error(&error)
        },
        Err(_) => {
            set_last_error("panic crossing csgrs ffi boundary");
            CsgrsStatus::Panic
        },
    }
}

/// Check null and borrow the output slot; no non-null pointer validity is checked.
///
/// # Safety
/// The pointer must permit an exclusive aligned `T` reference for the chosen
/// lifetime. The caller must not create an alias or outlive its storage.
unsafe fn out_ptr<'a, T>(out: *mut T) -> FfiResult<&'a mut T> {
    if out.is_null() {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { &mut *out })
    }
}

/// Allocate and publish an opaque owner after checking the output slot for null.
///
/// # Safety
/// The slot must satisfy `out_ptr` and hold no ownership that would be lost on overwrite.
unsafe fn out_handle<T>(out: *mut *mut T, value: T) -> FfiResult<()> {
    let out = unsafe { out_ptr(out) }?;
    *out = Box::into_raw(Box::new(value));
    Ok(())
}

/// Check null and borrow a handle without validating its provenance.
///
/// # Safety
/// A non-null pointer must reference a live aligned `T` for the chosen lifetime,
/// with no conflicting mutation or destruction.
unsafe fn ptr_ref<'a, T>(ptr: *const T) -> FfiResult<&'a T> {
    if ptr.is_null() {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { &*ptr })
    }
}

/// Borrow an input array; zero length returns an empty slice without inspecting the pointer.
///
/// # Safety
/// For nonzero length, the non-null pointer must describe initialized aligned elements
/// in one live allocation, with total size at most `isize::MAX`, readable for the
/// chosen lifetime and without conflicting mutation.
unsafe fn c_slice<'a, T>(ptr: *const T, len: usize) -> FfiResult<&'a [T]> {
    if len == 0 {
        Ok(&[])
    } else if ptr.is_null() {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { slice::from_raw_parts(ptr, len) })
    }
}

/// Copy a NUL-terminated UTF-8 string; null and invalid UTF-8 become errors.
///
/// # Safety
/// A non-null pointer must satisfy `CStr::from_ptr` for the entire read.
unsafe fn c_str(ptr: *const c_char) -> FfiResult<String> {
    if ptr.is_null() {
        return Err(FfiError::NullPointer);
    }
    let text = unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .map_err(|err| AdapterError::Validation(err.to_string()))?;
    Ok(text.to_owned())
}

/// Parse an entire signed decimal integer string using Rust's `i128` parser.
///
/// Whitespace, fractional syntax and overflow fail validation; no float parsing occurs.
fn parse_i128(text: &str) -> FfiResult<i128> {
    text.parse::<i128>()
        .map_err(|err| AdapterError::Validation(err.to_string()).into())
}

/// Wrap a mesh in its ABI family discriminator before heap ownership is published.
fn triangle_mesh_handle<F: Family>(mesh: TriangleMeshOf<F>) -> CsgrsTriangleMesh {
    CsgrsTriangleMesh {
        inner: F::triangle_mesh_kind(mesh),
    }
}

/// Wrap a region in its ABI family discriminator before heap ownership is published.
fn curve_region_handle<F: Family>(region: CurveRegionOf<F>) -> CsgrsCurveRegion {
    CsgrsCurveRegion {
        inner: F::curve_region_kind(region),
    }
}

/// Partition a flat index buffer into borrowed face ranges after checking offset coverage.
///
/// Requires at least two offsets, first zero, last index count and monotone in-bounds
/// ranges. Equal offsets are allowed here; native construction validates face contents.
fn build_faces<'a>(offsets: &'a [usize], indices: &'a [usize]) -> FfiResult<Vec<&'a [usize]>> {
    if offsets.len() < 2 {
        return Err(AdapterError::Validation(
            "face_offsets must contain at least two values".into(),
        )
        .into());
    }
    if offsets.first() != Some(&0) || offsets.last() != Some(&indices.len()) {
        return Err(AdapterError::Validation(
            "face_offsets must cover the complete face_indices buffer".into(),
        )
        .into());
    }
    let mut faces = Vec::with_capacity(offsets.len() - 1);
    for window in offsets.windows(2) {
        let start = window[0];
        let end = window[1];
        if start > end || end > indices.len() {
            return Err(
                AdapterError::Validation("face offset is out of bounds".into()).into(),
            );
        }
        faces.push(&indices[start..end]);
    }
    Ok(faces)
}

/// Export vertices and ordered triangles through the family and transfer their arrays.
///
/// Float coordinates are projected; integer export may fail; real coordinates remain native.
/// The descriptor has no destructor if a later output-pointer check fails.
fn create_mesh_buffers<F: Family>(mesh: &TriangleMeshOf<F>) -> FfiResult<F::CMeshBuffers> {
    let (vertices, indices) = mesh.vertices_and_indices()?;
    let vertices = vertices
        .into_iter()
        .map(F::vec3_from_adapter)
        .collect::<FfiResult<Vec<_>>>()?;
    let indices = indices
        .into_iter()
        .map(|[a, b, c]| CsgrsTriangleU32 { a, b, c })
        .collect();
    Ok(F::mesh_buffers(vertices, indices))
}

/// Request native normals and adapter projections, then allocate graphics ABI arrays.
///
/// Normal certification/export can fail. No separate certainty receipt is returned,
/// and raw arrays have no destructor if a later output-pointer check fails.
fn create_graphics_mesh<F: Family>(mesh: &TriangleMeshOf<F>) -> FfiResult<F::CGraphicsMesh> {
    let graphics = mesh.graphics_mesh()?;
    let vertices = graphics
        .vertices
        .iter()
        .cloned()
        .map(|(position, normal)| F::graphics_vertex_from_adapter(position, normal))
        .collect::<FfiResult<Vec<_>>>()?;
    Ok(F::graphics_mesh(vertices, graphics.indices.to_vec()))
}

/// Flatten strict sampled profiles as exterior-first rings with two sentinel-offset arrays.
///
/// The adapter projects through binary64 even for real handles. The separate receipt
/// is discarded. Empty output has no points and one zero in each offset array; these
/// raw arrays have no destructor if a later output-pointer check fails.
fn create_region_profiles<F: Family>(
    region: &CurveRegionOf<F>,
) -> FfiResult<F::CRegionProfiles> {
    let profiles = region
        .region_profiles(csgrs::GeometryContext::STRICT)?
        .into_value();
    let mut points = Vec::new();
    let mut ring_offsets = vec![0];
    let mut region_offsets = vec![0];
    let mut ring_count = 0;

    for region in profiles {
        for point in region.exterior {
            points.push(F::vec2_from_adapter(point)?);
        }
        ring_count += 1;
        ring_offsets.push(points.len());

        for hole in region.holes {
            for point in hole {
                points.push(F::vec2_from_adapter(point)?);
            }
            ring_count += 1;
            ring_offsets.push(points.len());
        }
        region_offsets.push(ring_count);
    }

    Ok(F::region_profiles(points, ring_offsets, region_offsets))
}

/// Generate one family's C entry points while delegating geometry to the shared adapters.
///
/// Inputs are borrowed, new handles are published on success, and adapter errors
/// flow through `ffi_status`. The expansion defines every generated function
/// with its operation and pointer contract.
macro_rules! export_family {
    (
        $family:ty,
        mesh: {
            cube: $mesh_cube:ident,
            cuboid: $mesh_cuboid:ident,
            sphere: $mesh_sphere:ident,
            cylinder: $mesh_cylinder:ident,
            polyhedron: $mesh_polyhedron:ident,
            union: $mesh_union:ident,
            difference: $mesh_difference:ident,
            intersection: $mesh_intersection:ident,
            xor: $mesh_xor:ident,
            transform: $mesh_transform:ident,
            translate: $mesh_translate:ident,
            scale: $mesh_scale:ident,
            rotate: $mesh_rotate:ident,
            inverse: $mesh_inverse:ident,
            center: $mesh_center:ident,
            float: $mesh_float:ident,
            bounding_box: $mesh_bounding_box:ident,
            vertices_and_indices: $mesh_vertices_and_indices:ident,
            graphics_mesh: $mesh_graphics_mesh:ident
        },
        region: {
            square: $region_square:ident,
            rectangle: $region_rectangle:ident,
            circle: $region_circle:ident,
            polygon: $region_polygon:ident,
            union: $region_union:ident,
            difference: $region_difference:ident,
            intersection: $region_intersection:ident,
            xor: $region_xor:ident,
            transform: $region_transform:ident,
            translate: $region_translate:ident,
            scale: $region_scale:ident,
            rotate: $region_rotate:ident,
            bounding_box: $region_bounding_box:ident,
            extrude: $region_extrude:ident,
            extrude_vector: $region_extrude_vector:ident,
            revolve: $region_revolve:ident,
            revolve_solid: $region_revolve_solid:ident,
            region_profiles: $region_region_profiles:ident
        }
    ) => {
        /// Construct `[0,width]^3` with strict nonnegative dimension validation.
        ///
        /// Zero width succeeds as canonical empty geometry; negative or undecided width fails.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cube(
            width: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = TriangleMeshOf::<$family>::cube(
                    <$family as Family>::scalar_to_adapter(width)?,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Construct an origin-anchored box with XYZ dimensions `width`, `length`, `height`.
        ///
        /// All dimensions must be nonnegative under strict predicates. A zero dimension
        /// gives canonical empty geometry only after validation of all dimensions.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cuboid(
            width: <$family as Family>::CScalar,
            length: <$family as Family>::CScalar,
            height: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = TriangleMeshOf::<$family>::cuboid(
                    <$family as Family>::scalar_to_adapter(width)?,
                    <$family as Family>::scalar_to_adapter(length)?,
                    <$family as Family>::scalar_to_adapter(height)?,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Construct the sampled triangle sphere centered at the origin, with Y-axis poles.
        ///
        /// Requires nonnegative radius, at least three segments and two stacks, even
        /// for zero radius. Construction history records sampling; success is not an
        /// analytic-sphere fidelity certificate.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_sphere(
            radius: <$family as Family>::CScalar,
            segments: usize,
            stacks: usize,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = TriangleMeshOf::<$family>::sphere(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    segments,
                    stacks,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Construct a sampled capped cylinder from Z zero to `height`.
        ///
        /// Requires nonnegative radius and at least three segments; signed height
        /// may be negative. Parameters are validated before zero-size empty success.
        /// The boundary is polygonal.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cylinder(
            radius: <$family as Family>::CScalar,
            height: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = TriangleMeshOf::<$family>::cylinder(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    <$family as Family>::scalar_to_adapter(height)?,
                    segments,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Triangulate indexed planar polygon faces with authored winding under strict predicates.
        ///
        /// Offsets must partition the complete index buffer from zero to its length.
        /// Native validation checks faces/indices; success need not be a closed solid.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        /// Input arrays must cover their declared element counts (null only at zero
        /// length); real point components borrow live scalar handles.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_polyhedron(
            points: *const <$family as Family>::CVec3,
            point_len: usize,
            face_indices: *const usize,
            face_index_len: usize,
            face_offsets: *const usize,
            face_offset_len: usize,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let points = unsafe { c_slice(points, point_len) }?
                    .iter()
                    .copied()
                    .map(<$family as Family>::vec3_to_adapter)
                    .collect::<FfiResult<Vec<_>>>()?;
                let face_indices = unsafe { c_slice(face_indices, face_index_len) }?;
                let face_offsets = unsafe { c_slice(face_offsets, face_offset_len) }?;
                let faces = build_faces(face_offsets, face_indices)?;
                let mesh = TriangleMeshOf::<$family>::polyhedron(&points, &faces)?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Return the strict regularized union of two same-family meshes.
        ///
        /// Native input validation and failure categories are preserved. No separate
        /// operation receipt is returned; retained mesh history remains queryable.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_union(
            lhs: *const CsgrsTriangleMesh,
            rhs: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(lhs.union(rhs)?)) }
            })
        }

        /// Return the strict regularized material of `lhs` outside `rhs`.
        ///
        /// Both operands use the same scalar family; native validation/failures remain
        /// authoritative and no separate operation receipt is returned.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_difference(
            lhs: *const CsgrsTriangleMesh,
            rhs: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe {
                    out_handle(out, triangle_mesh_handle::<$family>(lhs.difference(rhs)?))
                }
            })
        }

        /// Return the strict regularized common material of two same-family meshes.
        ///
        /// Empty success remains a mesh handle; it is distinct from invalid or undecided
        /// input. No separate operation receipt is returned.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_intersection(
            lhs: *const CsgrsTriangleMesh,
            rhs: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe {
                    out_handle(out, triangle_mesh_handle::<$family>(lhs.intersection(rhs)?))
                }
            })
        }

        /// Return the strict regularized material belonging to exactly one mesh operand.
        ///
        /// Native validation/failure categories are preserved without a separate
        /// operation receipt at this boundary.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_xor(
            lhs: *const CsgrsTriangleMesh,
            rhs: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(lhs.xor(rhs)?)) }
            })
        }

        /// Apply an imported row-major homogeneous matrix through the checked mesh transform.
        ///
        /// Strict validation rejects singular/undecided transforms and images reaching
        /// infinity. Winding correction preserves oriented material where admitted.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_transform(
            mesh: *const CsgrsTriangleMesh,
            matrix: <$family as Family>::CMatrix4,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let matrix = <$family as Family>::matrix4_to_core(matrix)?;
                let result = mesh.transform(&matrix)?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Translate native mesh coordinates by the imported XYZ displacement.
        ///
        /// Uses the native adapter translation without a separate operation receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_translate(
            mesh: *const CsgrsTriangleMesh,
            x: <$family as Family>::CScalar,
            y: <$family as Family>::CScalar,
            z: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.translate(
                    <$family as Family>::scalar_to_adapter(x)?,
                    <$family as Family>::scalar_to_adapter(y)?,
                    <$family as Family>::scalar_to_adapter(z)?,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Scale XYZ independently under strict predicates and correct winding for reflections.
        ///
        /// Zero axes are invalid and unresolved signs fail; input scalar conversion
        /// precedes native transform validation.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_scale(
            mesh: *const CsgrsTriangleMesh,
            sx: <$family as Family>::CScalar,
            sy: <$family as Family>::CScalar,
            sz: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.scale(
                    <$family as Family>::scalar_to_adapter(sx)?,
                    <$family as Family>::scalar_to_adapter(sy)?,
                    <$family as Family>::scalar_to_adapter(sz)?,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Rotate about X, then Y, then Z using imported angles in degrees.
        ///
        /// Uses strict checked transforms without a separate operation receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_rotate(
            mesh: *const CsgrsTriangleMesh,
            x_degrees: <$family as Family>::CScalar,
            y_degrees: <$family as Family>::CScalar,
            z_degrees: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.rotate(
                    <$family as Family>::scalar_to_adapter(x_degrees)?,
                    <$family as Family>::scalar_to_adapter(y_degrees)?,
                    <$family as Family>::scalar_to_adapter(z_degrees)?,
                )?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Return a mesh with every triangle winding reversed and positions preserved.
        ///
        /// This operation does not construct the unbounded set complement.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_inverse(
            mesh: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh.inverse())) }
            })
        }

        /// Move the native bounds center to the origin under strict predicates.
        ///
        /// Canonical empty geometry is preserved; bounds errors propagate. Placement
        /// uses native coordinates without first exporting them to the scalar family.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_center(
            mesh: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.center()?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Move the native minimum Z bound to zero under strict predicates.
        ///
        /// This is geometric placement, not scalar conversion. Empty geometry is
        /// preserved and unresolved bounds fail without a new handle.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_float(
            mesh: *const CsgrsTriangleMesh,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.float()?;
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(result)) }
            })
        }

        /// Query strict native extrema, then export them through this scalar family.
        ///
        /// `NoBounds` leaves a valid output record untouched and allocates no real
        /// endpoints. Float output is not an outward-enclosure certificate; integer
        /// export can fail. Only `Ok` writes a record.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. The input handle must be live and
        /// match this function's family; `out` must be a writable record slot. Preserve
        /// all returned pointers/lengths and use the matching free function exactly once
        /// for owned buffer outputs or real-valued bounds.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_bounding_box(
            mesh: *const CsgrsTriangleMesh,
            out: *mut <$family as Family>::CAabb,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let out = unsafe { out_ptr(out) }?;
                let bounds = mesh.bounding_box()?.ok_or(FfiError::NoBounds)?;
                *out = <$family as Family>::aabb_from_adapter(bounds)?;
                Ok(())
            })
        }

        /// Export indexed positions and ordered triangle rows into owned arrays.
        ///
        /// Float coordinates are lossy; integer conversion can fail. The real family
        /// clones native scalars. Counts describe rows, and zero rows are valid success.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. The input handle must be live and
        /// match this function's family; `out` must be a writable record slot. Preserve
        /// all returned pointers/lengths and use the matching free function exactly once
        /// for owned buffer outputs or real-valued bounds.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_vertices_and_indices(
            mesh: *const CsgrsTriangleMesh,
            out: *mut <$family as Family>::CMeshBuffers,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                *unsafe { out_ptr(out) }? = create_mesh_buffers::<$family>(mesh)?;
                Ok(())
            })
        }

        /// Export positions, certified native normals and flat indices into owned graphics arrays.
        ///
        /// Normal certification and scalar export can fail. Projection may change normal
        /// lengths; the result has no separate fidelity or certainty receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. The input handle must be live and
        /// match this function's family; `out` must be a writable record slot. Preserve
        /// all returned pointers/lengths and use the matching free function exactly once
        /// for owned buffer outputs or real-valued bounds.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_graphics_mesh(
            mesh: *const CsgrsTriangleMesh,
            out: *mut <$family as Family>::CGraphicsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::triangle_mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                *unsafe { out_ptr(out) }? = create_graphics_mesh::<$family>(mesh)?;
                Ok(())
            })
        }

        /// Construct an origin-anchored XY square with strict nonnegative width validation.
        ///
        /// Zero width returns canonical empty geometry; negative or undecided width fails.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_square(
            width: <$family as Family>::CScalar,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region = CurveRegionOf::<$family>::square(
                    <$family as Family>::scalar_to_adapter(width)?,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(region)) }
            })
        }

        /// Construct an origin-anchored XY rectangle with nonnegative width and length.
        ///
        /// Both dimensions are validated under strict predicates before zero-size empty success.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_rectangle(
            width: <$family as Family>::CScalar,
            length: <$family as Family>::CScalar,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region = CurveRegionOf::<$family>::rectangle(
                    <$family as Family>::scalar_to_adapter(width)?,
                    <$family as Family>::scalar_to_adapter(length)?,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(region)) }
            })
        }

        /// Construct a regular sampled XY polygon centered at the origin.
        ///
        /// Requires nonnegative radius and at least three segments, including zero-radius
        /// input. This is not an analytic circular boundary or a fidelity certificate.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_circle(
            radius: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region = CurveRegionOf::<$family>::circle(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    segments,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(region)) }
            })
        }

        /// Construct a filled XY polygon from a valid ring of at least three vertices.
        ///
        /// Native strict validation rejects empty/malformed rings. The input is copied
        /// through the selected scalar adapter before region construction.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Real-family scalar components
        /// must borrow live handles. `out` must be a writable handle slot; on success
        /// its new owner must be freed once with the matching geometry free function.
        /// The point array must cover `point_len` readable elements (null only at zero
        /// length); real components must borrow live scalar handles.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_polygon(
            points: *const <$family as Family>::CVec2,
            point_len: usize,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let points = unsafe { c_slice(points, point_len) }?
                    .iter()
                    .copied()
                    .map(<$family as Family>::vec2_to_adapter)
                    .collect::<FfiResult<Vec<_>>>()?;
                let region = CurveRegionOf::<$family>::polygon(&points)?;
                unsafe { out_handle(out, curve_region_handle::<$family>(region)) }
            })
        }

        /// Return the regularized union of same-family filled regions.
        ///
        /// Delegates to Hypercurve's active policy scope, strict outside a preview or
        /// provisional scope, and returns no separate operation receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_union(
            lhs: *const CsgrsCurveRegion,
            rhs: *const CsgrsCurveRegion,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, curve_region_handle::<$family>(lhs.union(rhs)?)) }
            })
        }

        /// Return the regularized part of `lhs` outside `rhs`.
        ///
        /// Uses Hypercurve's active policy scope (normally strict), with no separate receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_difference(
            lhs: *const CsgrsCurveRegion,
            rhs: *const CsgrsCurveRegion,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe {
                    out_handle(out, curve_region_handle::<$family>(lhs.difference(rhs)?))
                }
            })
        }

        /// Return the regularized common material of two same-family filled regions.
        ///
        /// Uses Hypercurve's active policy scope (normally strict), with no separate receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_intersection(
            lhs: *const CsgrsCurveRegion,
            rhs: *const CsgrsCurveRegion,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe {
                    out_handle(out, curve_region_handle::<$family>(lhs.intersection(rhs)?))
                }
            })
        }

        /// Return regularized material belonging to exactly one filled-region operand.
        ///
        /// Uses Hypercurve's active policy scope (normally strict), with no separate receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_xor(
            lhs: *const CsgrsCurveRegion,
            rhs: *const CsgrsCurveRegion,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::curve_region_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, curve_region_handle::<$family>(lhs.xor(rhs)?)) }
            })
        }

        /// Apply a row-major homogeneous transform with a nonsingular affine XY restriction.
        ///
        /// Strict planar admission rejects perspective/out-of-plane maps and diagnoses
        /// invalid or unbounded weight cases. The separate outcome receipt is discarded.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_transform(
            region: *const CsgrsCurveRegion,
            matrix: <$family as Family>::CMatrix4,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let matrix = <$family as Family>::matrix4_to_core(matrix)?;
                let result = region.transform(&matrix)?;
                unsafe { out_handle(out, curve_region_handle::<$family>(result)) }
            })
        }

        /// Translate the planar region under strict predicates.
        ///
        /// Nonzero Z displacement cannot remain in the XY carrier. The adapter discards
        /// the separate local receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_translate(
            region: *const CsgrsCurveRegion,
            x: <$family as Family>::CScalar,
            y: <$family as Family>::CScalar,
            z: <$family as Family>::CScalar,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let result = region.translate(
                    <$family as Family>::scalar_to_adapter(x)?,
                    <$family as Family>::scalar_to_adapter(y)?,
                    <$family as Family>::scalar_to_adapter(z)?,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(result)) }
            })
        }

        /// Scale the planar XY image under strict predicates.
        ///
        /// Z scale is imported but has no planar effect; a singular XY image is rejected.
        /// The adapter discards the separate local receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_scale(
            region: *const CsgrsCurveRegion,
            sx: <$family as Family>::CScalar,
            sy: <$family as Family>::CScalar,
            sz: <$family as Family>::CScalar,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let result = region.scale(
                    <$family as Family>::scalar_to_adapter(sx)?,
                    <$family as Family>::scalar_to_adapter(sy)?,
                    <$family as Family>::scalar_to_adapter(sz)?,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(result)) }
            })
        }

        /// Apply X, then Y, then Z rotations in degrees when the image remains in XY.
        ///
        /// Strict planar transform admission may reject the map; the adapter discards
        /// the separate local receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_rotate(
            region: *const CsgrsCurveRegion,
            x_degrees: <$family as Family>::CScalar,
            y_degrees: <$family as Family>::CScalar,
            z_degrees: <$family as Family>::CScalar,
            out: *mut *mut CsgrsCurveRegion,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let result = region.rotate(
                    <$family as Family>::scalar_to_adapter(x_degrees)?,
                    <$family as Family>::scalar_to_adapter(y_degrees)?,
                    <$family as Family>::scalar_to_adapter(z_degrees)?,
                )?;
                unsafe { out_handle(out, curve_region_handle::<$family>(result)) }
            })
        }

        /// Query strict native region bounds and export the endpoints through this family.
        ///
        /// Trimmed fragments may use a looser certified source envelope. Float endpoints
        /// are not an outward-enclosure certificate. `NoBounds` leaves a valid output
        /// record untouched and allocates no real handles.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. The input handle must be live and
        /// match this function's family; `out` must be a writable record slot. Preserve
        /// all returned pointers/lengths and use the matching free function exactly once
        /// for owned buffer outputs or real-valued bounds.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_bounding_box(
            region: *const CsgrsCurveRegion,
            out: *mut <$family as Family>::CAabb,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let out = unsafe { out_ptr(out) }?;
                let bounds = region.bounding_box()?.ok_or(FfiError::NoBounds)?;
                *out = <$family as Family>::aabb_from_adapter(bounds)?;
                Ok(())
            })
        }

        /// Extrude the region along Z by a nonzero signed height under strict predicates.
        ///
        /// Nonlinear profile interiors may be sampled with mesh construction history.
        /// The separate outcome receipt is discarded; exact source fidelity is not implied.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_extrude(
            region: *const CsgrsCurveRegion,
            height: <$family as Family>::CScalar,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let mesh = region
                    .extrude(
                        <$family as Family>::scalar_to_adapter(height)?,
                        csgrs::GeometryContext::STRICT,
                    )?
                    .into_value();
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Extrude along the imported vector, requiring a nonzero Z component.
        ///
        /// Uses strict predicates and the native profile-lowering fidelity contract;
        /// the separate outcome receipt is discarded.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_extrude_vector(
            region: *const CsgrsCurveRegion,
            direction: <$family as Family>::CVec3,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let mesh = region
                    .extrude_vector(
                        <$family as Family>::vec3_to_adapter(direction)?,
                        csgrs::GeometryContext::STRICT,
                    )?
                    .into_value();
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Build a segmented swept boundary about Y, with profile X as signed radius.
        ///
        /// Angles are in degrees; at least two segments (three for a full turn) are
        /// required. Zero and oversized noninteger-turn angles are invalid. Partial
        /// sweeps reject axis-crossing loops and may retain overlapping winding.
        /// Strict decisions do not certify analytic-surface fidelity; the separate
        /// outcome receipt is discarded.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_revolve(
            region: *const CsgrsCurveRegion,
            angle_degrees: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let mesh = region
                    .revolve(
                        <$family as Family>::scalar_to_adapter(angle_degrees)?,
                        segments,
                        csgrs::GeometryContext::STRICT,
                    )?
                    .into_value();
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Regularize the union of segmented revolution pieces about Y.
        ///
        /// Uses the revolution angle/resolution domain, admits axis-crossing partial
        /// profiles and counts overlapping material once. Closed point/edge contacts
        /// may remain nonmanifold. Uses strict predicates and discards the separate receipt.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. Input geometry must be live and
        /// match this function's scalar family; real scalar components are borrowed.
        /// `out` must be a writable handle slot; success transfers one new owner.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_revolve_solid(
            region: *const CsgrsCurveRegion,
            angle_degrees: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsTriangleMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                let mesh = region
                    .revolve_solid(
                        <$family as Family>::scalar_to_adapter(angle_degrees)?,
                        segments,
                        csgrs::GeometryContext::STRICT,
                    )?
                    .into_value();
                unsafe { out_handle(out, triangle_mesh_handle::<$family>(mesh)) }
            })
        }

        /// Export sampled exterior/hole rings with sentinel offsets under strict predicates.
        ///
        /// All families, including real, pass through binary64 display profiles. This
        /// is not exact native replay. The separate receipt is discarded; empty output
        /// has zero points and one zero in each offset array.
        ///
        /// # Safety
        /// Follow the crate pointer/aliasing contract. The input handle must be live and
        /// match this function's family; `out` must be a writable record slot. Preserve
        /// all returned pointers/lengths and use the matching free function exactly once
        /// for owned buffer outputs or real-valued bounds.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_region_profiles(
            region: *const CsgrsCurveRegion,
            out: *mut <$family as Family>::CRegionProfiles,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let region =
                    <$family as Family>::curve_region_ref(unsafe { ptr_ref(region) }?)?;
                *unsafe { out_ptr(out) }? = create_region_profiles::<$family>(region)?;
                Ok(())
            })
        }
    };
}

export_family!(
    F32Family,
    mesh: {
        cube: csgrs_triangle_mesh_f32_cube,
        cuboid: csgrs_triangle_mesh_f32_cuboid,
        sphere: csgrs_triangle_mesh_f32_sphere,
        cylinder: csgrs_triangle_mesh_f32_cylinder,
        polyhedron: csgrs_triangle_mesh_f32_polyhedron,
        union: csgrs_triangle_mesh_f32_union,
        difference: csgrs_triangle_mesh_f32_difference,
        intersection: csgrs_triangle_mesh_f32_intersection,
        xor: csgrs_triangle_mesh_f32_xor,
        transform: csgrs_triangle_mesh_f32_transform,
        translate: csgrs_triangle_mesh_f32_translate,
        scale: csgrs_triangle_mesh_f32_scale,
        rotate: csgrs_triangle_mesh_f32_rotate,
        inverse: csgrs_triangle_mesh_f32_inverse,
        center: csgrs_triangle_mesh_f32_center,
        float: csgrs_triangle_mesh_f32_float,
        bounding_box: csgrs_triangle_mesh_f32_bounding_box,
        vertices_and_indices: csgrs_triangle_mesh_f32_vertices_and_indices,
        graphics_mesh: csgrs_triangle_mesh_f32_graphics_mesh
    },
    region: {
        square: csgrs_curve_region_f32_square,
        rectangle: csgrs_curve_region_f32_rectangle,
        circle: csgrs_curve_region_f32_circle,
        polygon: csgrs_curve_region_f32_polygon,
        union: csgrs_curve_region_f32_union,
        difference: csgrs_curve_region_f32_difference,
        intersection: csgrs_curve_region_f32_intersection,
        xor: csgrs_curve_region_f32_xor,
        transform: csgrs_curve_region_f32_transform,
        translate: csgrs_curve_region_f32_translate,
        scale: csgrs_curve_region_f32_scale,
        rotate: csgrs_curve_region_f32_rotate,
        bounding_box: csgrs_curve_region_f32_bounding_box,
        extrude: csgrs_curve_region_f32_extrude,
        extrude_vector: csgrs_curve_region_f32_extrude_vector,
        revolve: csgrs_curve_region_f32_revolve,
        revolve_solid: csgrs_curve_region_f32_revolve_solid,
        region_profiles: csgrs_curve_region_f32_region_profiles
    }
);

export_family!(
    F64Family,
    mesh: {
        cube: csgrs_triangle_mesh_f64_cube,
        cuboid: csgrs_triangle_mesh_f64_cuboid,
        sphere: csgrs_triangle_mesh_f64_sphere,
        cylinder: csgrs_triangle_mesh_f64_cylinder,
        polyhedron: csgrs_triangle_mesh_f64_polyhedron,
        union: csgrs_triangle_mesh_f64_union,
        difference: csgrs_triangle_mesh_f64_difference,
        intersection: csgrs_triangle_mesh_f64_intersection,
        xor: csgrs_triangle_mesh_f64_xor,
        transform: csgrs_triangle_mesh_f64_transform,
        translate: csgrs_triangle_mesh_f64_translate,
        scale: csgrs_triangle_mesh_f64_scale,
        rotate: csgrs_triangle_mesh_f64_rotate,
        inverse: csgrs_triangle_mesh_f64_inverse,
        center: csgrs_triangle_mesh_f64_center,
        float: csgrs_triangle_mesh_f64_float,
        bounding_box: csgrs_triangle_mesh_f64_bounding_box,
        vertices_and_indices: csgrs_triangle_mesh_f64_vertices_and_indices,
        graphics_mesh: csgrs_triangle_mesh_f64_graphics_mesh
    },
    region: {
        square: csgrs_curve_region_f64_square,
        rectangle: csgrs_curve_region_f64_rectangle,
        circle: csgrs_curve_region_f64_circle,
        polygon: csgrs_curve_region_f64_polygon,
        union: csgrs_curve_region_f64_union,
        difference: csgrs_curve_region_f64_difference,
        intersection: csgrs_curve_region_f64_intersection,
        xor: csgrs_curve_region_f64_xor,
        transform: csgrs_curve_region_f64_transform,
        translate: csgrs_curve_region_f64_translate,
        scale: csgrs_curve_region_f64_scale,
        rotate: csgrs_curve_region_f64_rotate,
        bounding_box: csgrs_curve_region_f64_bounding_box,
        extrude: csgrs_curve_region_f64_extrude,
        extrude_vector: csgrs_curve_region_f64_extrude_vector,
        revolve: csgrs_curve_region_f64_revolve,
        revolve_solid: csgrs_curve_region_f64_revolve_solid,
        region_profiles: csgrs_curve_region_f64_region_profiles
    }
);

export_family!(
    I128Family,
    mesh: {
        cube: csgrs_triangle_mesh_i128_cube,
        cuboid: csgrs_triangle_mesh_i128_cuboid,
        sphere: csgrs_triangle_mesh_i128_sphere,
        cylinder: csgrs_triangle_mesh_i128_cylinder,
        polyhedron: csgrs_triangle_mesh_i128_polyhedron,
        union: csgrs_triangle_mesh_i128_union,
        difference: csgrs_triangle_mesh_i128_difference,
        intersection: csgrs_triangle_mesh_i128_intersection,
        xor: csgrs_triangle_mesh_i128_xor,
        transform: csgrs_triangle_mesh_i128_transform,
        translate: csgrs_triangle_mesh_i128_translate,
        scale: csgrs_triangle_mesh_i128_scale,
        rotate: csgrs_triangle_mesh_i128_rotate,
        inverse: csgrs_triangle_mesh_i128_inverse,
        center: csgrs_triangle_mesh_i128_center,
        float: csgrs_triangle_mesh_i128_float,
        bounding_box: csgrs_triangle_mesh_i128_bounding_box,
        vertices_and_indices: csgrs_triangle_mesh_i128_vertices_and_indices,
        graphics_mesh: csgrs_triangle_mesh_i128_graphics_mesh
    },
    region: {
        square: csgrs_curve_region_i128_square,
        rectangle: csgrs_curve_region_i128_rectangle,
        circle: csgrs_curve_region_i128_circle,
        polygon: csgrs_curve_region_i128_polygon,
        union: csgrs_curve_region_i128_union,
        difference: csgrs_curve_region_i128_difference,
        intersection: csgrs_curve_region_i128_intersection,
        xor: csgrs_curve_region_i128_xor,
        transform: csgrs_curve_region_i128_transform,
        translate: csgrs_curve_region_i128_translate,
        scale: csgrs_curve_region_i128_scale,
        rotate: csgrs_curve_region_i128_rotate,
        bounding_box: csgrs_curve_region_i128_bounding_box,
        extrude: csgrs_curve_region_i128_extrude,
        extrude_vector: csgrs_curve_region_i128_extrude_vector,
        revolve: csgrs_curve_region_i128_revolve,
        revolve_solid: csgrs_curve_region_i128_revolve_solid,
        region_profiles: csgrs_curve_region_i128_region_profiles
    }
);

export_family!(
    RealFamily,
    mesh: {
        cube: csgrs_triangle_mesh_real_cube,
        cuboid: csgrs_triangle_mesh_real_cuboid,
        sphere: csgrs_triangle_mesh_real_sphere,
        cylinder: csgrs_triangle_mesh_real_cylinder,
        polyhedron: csgrs_triangle_mesh_real_polyhedron,
        union: csgrs_triangle_mesh_real_union,
        difference: csgrs_triangle_mesh_real_difference,
        intersection: csgrs_triangle_mesh_real_intersection,
        xor: csgrs_triangle_mesh_real_xor,
        transform: csgrs_triangle_mesh_real_transform,
        translate: csgrs_triangle_mesh_real_translate,
        scale: csgrs_triangle_mesh_real_scale,
        rotate: csgrs_triangle_mesh_real_rotate,
        inverse: csgrs_triangle_mesh_real_inverse,
        center: csgrs_triangle_mesh_real_center,
        float: csgrs_triangle_mesh_real_float,
        bounding_box: csgrs_triangle_mesh_real_bounding_box,
        vertices_and_indices: csgrs_triangle_mesh_real_vertices_and_indices,
        graphics_mesh: csgrs_triangle_mesh_real_graphics_mesh
    },
    region: {
        square: csgrs_curve_region_real_square,
        rectangle: csgrs_curve_region_real_rectangle,
        circle: csgrs_curve_region_real_circle,
        polygon: csgrs_curve_region_real_polygon,
        union: csgrs_curve_region_real_union,
        difference: csgrs_curve_region_real_difference,
        intersection: csgrs_curve_region_real_intersection,
        xor: csgrs_curve_region_real_xor,
        transform: csgrs_curve_region_real_transform,
        translate: csgrs_curve_region_real_translate,
        scale: csgrs_curve_region_real_scale,
        rotate: csgrs_curve_region_real_rotate,
        bounding_box: csgrs_curve_region_real_bounding_box,
        extrude: csgrs_curve_region_real_extrude,
        extrude_vector: csgrs_curve_region_real_extrude_vector,
        revolve: csgrs_curve_region_real_revolve,
        revolve_solid: csgrs_curve_region_real_revolve_solid,
        region_profiles: csgrs_curve_region_real_region_profiles
    }
);

/// Borrow the current thread's NUL-terminated diagnostic without clearing it.
///
/// Copy the text before the next ABI call or thread exit. Status-returning calls
/// replace it on both success and failure; free functions leave it unchanged.
#[unsafe(no_mangle)]
pub extern "C" fn csgrs_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

/// Allocate a native exact zero scalar.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_zero(out: *mut *mut CsgrsReal) -> CsgrsStatus {
    ffi_status(|| unsafe {
        out_handle(
            out,
            CsgrsReal {
                inner: Real::zero(),
            },
        )
    })
}

/// Allocate a native exact one scalar.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_one(out: *mut *mut CsgrsReal) -> CsgrsStatus {
    ffi_status(|| unsafe { out_handle(out, CsgrsReal { inner: Real::one() }) })
}

/// Import the exact finite binary64 value, rejecting NaN and infinity.
///
/// For example, `0.1_f64` is not the rational one tenth.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_from_f64(
    value: f64,
    out: *mut *mut CsgrsReal,
) -> CsgrsStatus {
    ffi_status(|| {
        let inner = Real::try_from(value).map_err(AdapterError::from)?;
        unsafe { out_handle(out, CsgrsReal { inner }) }
    })
}

/// Import a split-word signed integer exactly.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_from_i128(
    value: CsgrsI128,
    out: *mut *mut CsgrsReal,
) -> CsgrsStatus {
    ffi_status(|| unsafe {
        out_handle(
            out,
            CsgrsReal {
                inner: Real::from(i128_from_parts(value)),
            },
        )
    })
}

/// Parse a NUL-terminated UTF-8 decimal integer in the signed 128-bit range.
///
/// This accepts integer syntax only, not fractional decimal reals. Whitespace,
/// invalid UTF-8 and overflow are validation failures.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
/// The non-null string must remain readable through its terminating NUL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_from_i128_decimal(
    value: *const c_char,
    out: *mut *mut CsgrsReal,
) -> CsgrsStatus {
    ffi_status(|| {
        let text = unsafe { c_str(value) }?;
        let value = parse_i128(&text)?;
        unsafe {
            out_handle(
                out,
                CsgrsReal {
                    inner: Real::from(value),
                },
            )
        }
    })
}

/// Create another owned handle sharing the native scalar's exact meaning.
///
/// The original remains borrowed and must retain its own lifetime/ownership.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `out` must be a writable handle
/// slot; success transfers one scalar owner to free with `csgrs_real_free`.
/// `value` must borrow a live scalar handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_clone(
    value: *const CsgrsReal,
    out: *mut *mut CsgrsReal,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        unsafe {
            out_handle(
                out,
                CsgrsReal {
                    inner: value.inner.clone(),
                },
            )
        }
    })
}

/// Return a finite lossy binary64 projection, or a conversion failure.
///
/// The output is not an exactness or directed-rounding certificate.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `value` must borrow a live scalar
/// handle and `out` must be a writable, nonaliasing output slot.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_to_f64(
    value: *const CsgrsReal,
    out: *mut f64,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = F64::from_real(&value.inner)?;
        Ok(())
    })
}

/// Extract a recognized rational integer into signed high and unsigned low words.
///
/// Unrecognized expressions, nonintegers and range overflow fail; there is no
/// refinement search to decide whether an unrecognized expression is integral.
///
/// # Safety
/// Follow the crate pointer/aliasing contract. `value` must borrow a live scalar
/// handle and `out` must be a writable, nonaliasing output slot.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_to_i128(
    value: *const CsgrsReal,
    out: *mut CsgrsI128,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = i128_to_parts(real_to_i128(&value.inner)?);
        Ok(())
    })
}

/// Release one opaque scalar owner; null is a no-op.
///
/// Does not clear the last-error string and has no unwind catcher.
///
/// # Safety
/// A non-null pointer must be a still-owned scalar allocation returned by this
/// library, with no outstanding use. Free it exactly once; do not pass stack
/// values, borrowed pointers or a handle already freed elsewhere.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_free(value: *mut CsgrsReal) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

/// Release one opaque mesh owner; null is a no-op.
///
/// Does not clear the last-error string and has no unwind catcher.
///
/// # Safety
/// A non-null pointer must be a still-owned mesh allocation returned by this
/// library, with no outstanding use. Free it exactly once; do not pass stack
/// values, borrowed pointers or a handle already freed elsewhere.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_triangle_mesh_free(value: *mut CsgrsTriangleMesh) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

/// Release one opaque region owner; null is a no-op.
///
/// Does not clear the last-error string and has no unwind catcher.
///
/// # Safety
/// A non-null pointer must be a still-owned region allocation returned by this
/// library, with no outstanding use. Free it exactly once; do not pass stack
/// values, borrowed pointers or a handle already freed elsewhere.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_curve_region_free(value: *mut CsgrsCurveRegion) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

/// Read a live mesh handle's boundary-family tag without converting geometry.
///
/// The tag describes ingress/egress, not the internal coordinate representation.
///
/// # Safety
/// Borrow a live mesh handle and provide a nonaliasing writable family slot,
/// following the crate pointer contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_triangle_mesh_family(
    value: *const CsgrsTriangleMesh,
    out: *mut CsgrsScalarFamily,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = match value.inner {
            TriangleMeshKind::F32(_) => CsgrsScalarFamily::F32,
            TriangleMeshKind::F64(_) => CsgrsScalarFamily::F64,
            TriangleMeshKind::I128(_) => CsgrsScalarFamily::I128,
            TriangleMeshKind::Real(_) => CsgrsScalarFamily::Real,
        };
        Ok(())
    })
}

/// Read a live region handle's boundary-family tag without converting geometry.
///
/// The tag describes ingress/egress, not the internal coordinate representation.
///
/// # Safety
/// Borrow a live region handle and provide a nonaliasing writable family slot,
/// following the crate pointer contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_curve_region_family(
    value: *const CsgrsCurveRegion,
    out: *mut CsgrsScalarFamily,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = match value.inner {
            CurveRegionKind::F32(_) => CsgrsScalarFamily::F32,
            CurveRegionKind::F64(_) => CsgrsScalarFamily::F64,
            CurveRegionKind::I128(_) => CsgrsScalarFamily::I128,
            CurveRegionKind::Real(_) => CsgrsScalarFamily::Real,
        };
        Ok(())
    })
}

/// Generate primitive-family array destructors; real-valued arrays need nested-handle cleanup.
macro_rules! export_free_buffers {
    ($mesh_free:ident, $graphics_free:ident, $region_free:ident, $mesh:ty, $graphics:ty, $region:ty, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        /// Release the primitive-family vertex and triangle arrays.
        ///
        /// No diagnostic is written and no unwind catcher is installed.
        ///
        /// # Safety
        /// The descriptor must retain the exact pointers and lengths returned by the
        /// matching successful export, still owned once with no outstanding references.
        /// Do not free a copied descriptor twice or substitute foreign allocations.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_free(value: $mesh) {
            unsafe {
                free_boxed_slice::<$vec3>(value.vertices, value.vertex_len);
                free_boxed_slice::<CsgrsTriangleU32>(value.indices, value.index_len);
            }
        }

        /// Release the primitive-family graphics vertex and flat index arrays.
        ///
        /// No diagnostic is written and no unwind catcher is installed.
        ///
        /// # Safety
        /// The descriptor must retain the exact pointers and lengths returned by the
        /// matching successful export, still owned once with no outstanding references.
        /// Do not free a copied descriptor twice or substitute foreign allocations.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $graphics_free(value: $graphics) {
            unsafe {
                free_boxed_slice::<$gvertex>(value.vertices, value.vertex_len);
                free_boxed_slice::<u32>(value.indices, value.index_len);
            }
        }

        /// Release primitive-family points and both profile offset arrays.
        ///
        /// No diagnostic is written and no unwind catcher is installed.
        ///
        /// # Safety
        /// The descriptor must retain the exact pointers and lengths returned by the
        /// matching successful export, still owned once with no outstanding references.
        /// Do not free a copied descriptor twice or substitute foreign allocations.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_free(value: $region) {
            unsafe {
                free_boxed_slice::<$vec2>(value.points, value.point_len);
                free_boxed_slice::<usize>(value.ring_offsets, value.ring_offset_len);
                free_boxed_slice::<usize>(value.region_offsets, value.region_offset_len);
            }
        }
    };
}

export_free_buffers!(
    csgrs_triangle_mesh_buffers_f32_free,
    csgrs_graphics_mesh_f32_free,
    csgrs_region_profiles_f32_free,
    CsgrsTriangleMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);

export_free_buffers!(
    csgrs_triangle_mesh_buffers_f64_free,
    csgrs_graphics_mesh_f64_free,
    csgrs_region_profiles_f64_free,
    CsgrsTriangleMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);

export_free_buffers!(
    csgrs_triangle_mesh_buffers_i128_free,
    csgrs_graphics_mesh_i128_free,
    csgrs_region_profiles_i128_free,
    CsgrsTriangleMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);

/// Release both owned scalar components; null components are no-ops.
///
/// # Safety
/// Each non-null component must be a distinct live allocation owned by this record,
/// with no outstanding use. A copied record does not grant another ownership.
unsafe fn free_real_vec2(value: CsgrsVec2Real) {
    unsafe {
        csgrs_real_free(value.x);
        csgrs_real_free(value.y);
    }
}

/// Release all three owned scalar components; null components are no-ops.
///
/// # Safety
/// Each non-null component must be a distinct live allocation owned by this record,
/// with no outstanding use. A copied record does not grant another ownership.
unsafe fn free_real_vec3(value: CsgrsVec3Real) {
    unsafe {
        csgrs_real_free(value.x);
        csgrs_real_free(value.y);
        csgrs_real_free(value.z);
    }
}

/// Release all six owned endpoint handles in an exported real-valued AABB.
///
/// The by-value record is not reset; copies of it retain dangling pointers.
///
/// # Safety
/// Each non-null component must be a distinct live scalar owner returned by this
/// library, unused elsewhere. Call exactly once for a successful bounds export.
/// Never pass borrowed vector components or free individual components first.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_aabb3_real_free_values(value: CsgrsAabb3Real) {
    unsafe {
        free_real_vec3(value.mins);
        free_real_vec3(value.maxs);
    }
}

/// Release every vertex scalar handle, then the vertex and triangle arrays.
///
/// The descriptor is consumed by value without resetting copies or changing the diagnostic.
///
/// # Safety
/// The descriptor must retain the exact pointers and lengths returned by the
/// matching successful export, still owned once with no outstanding references.
/// Do not free a copied descriptor twice or substitute foreign allocations.
/// Nested scalar handles must remain distinct, live and owned by this descriptor;
/// do not free them separately before this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_triangle_mesh_buffers_real_free(
    value: CsgrsTriangleMeshBuffersReal,
) {
    unsafe {
        if !value.vertices.is_null() {
            for vertex in slice::from_raw_parts(value.vertices, value.vertex_len) {
                free_real_vec3(*vertex);
            }
        }
        free_boxed_slice::<CsgrsVec3Real>(value.vertices, value.vertex_len);
        free_boxed_slice::<CsgrsTriangleU32>(value.indices, value.index_len);
    }
}

/// Release position/normal scalar handles, then the graphics vertex and index arrays.
///
/// The descriptor is consumed by value without resetting copies or changing the diagnostic.
///
/// # Safety
/// The descriptor must retain the exact pointers and lengths returned by the
/// matching successful export, still owned once with no outstanding references.
/// Do not free a copied descriptor twice or substitute foreign allocations.
/// Nested scalar handles must remain distinct, live and owned by this descriptor;
/// do not free them separately before this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_graphics_mesh_real_free(value: CsgrsGraphicsMeshReal) {
    unsafe {
        if !value.vertices.is_null() {
            for vertex in slice::from_raw_parts(value.vertices, value.vertex_len) {
                free_real_vec3(vertex.position);
                free_real_vec3(vertex.normal);
            }
        }
        free_boxed_slice::<CsgrsGraphicsVertexReal>(value.vertices, value.vertex_len);
        free_boxed_slice::<u32>(value.indices, value.index_len);
    }
}

/// Release each projected point's scalar handles, then point and offset arrays.
///
/// The descriptor is consumed by value without resetting copies or changing the diagnostic.
///
/// # Safety
/// The descriptor must retain the exact pointers and lengths returned by the
/// matching successful export, still owned once with no outstanding references.
/// Do not free a copied descriptor twice or substitute foreign allocations.
/// Nested scalar handles must remain distinct, live and owned by this descriptor;
/// do not free them separately before this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_region_profiles_real_free(value: CsgrsRegionProfilesReal) {
    unsafe {
        if !value.points.is_null() {
            for point in slice::from_raw_parts(value.points, value.point_len) {
                free_real_vec2(*point);
            }
        }
        free_boxed_slice::<CsgrsVec2Real>(value.points, value.point_len);
        free_boxed_slice::<usize>(value.ring_offsets, value.ring_offset_len);
        free_boxed_slice::<usize>(value.region_offsets, value.region_offset_len);
    }
}

/// ABI status, pointer and output-ownership regression witnesses; these do not certify all geometry.
#[cfg(test)]
mod tests {
    /// Keep unavailable storage distinct from configured exhaustion, including numeric ABI values.
    #[test]
    fn unavailable_storage_has_a_distinct_stable_status() {
        assert_eq!(
            super::status_for_validation_kind(
                csgrs::errors::ValidationErrorKind::ResourceUnavailable
            ),
            super::CsgrsStatus::ResourceUnavailable
        );
        assert_eq!(
            super::status_for_validation_kind(
                csgrs::errors::ValidationErrorKind::ResourceExhausted
            ),
            super::CsgrsStatus::ResourceExhausted
        );
        assert_eq!(super::CsgrsStatus::ResourceUnavailable as u32, 11);
        assert_eq!(super::CsgrsStatus::ResourceExhausted as u32, 6);
    }

    use super::*;

    // Preserve the original exact proof and keep the ABI's undecided controls
    // outside the admitted scalar normal form. The assertions below check the
    // public strict error status and unchanged output handles.
    /// Verify the original trigonometric zero proof, then return an unrecognized addition-form zero.
    ///
    /// The equivalent expression keeps strict-undecided controls outside the admitted
    /// normal form; its lack of recognition is asserted rather than assumed.
    fn terminal_zero(proved_zero: Real) -> Real {
        assert_eq!(
            proved_zero.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        let zero = (Real::e() + Real::one()).sin()
            - Real::e().sin() * Real::one().cos()
            - Real::e().cos() * Real::one().sin();
        assert!(zero.exact_rational_normal_form().is_none());
        zero
    }

    /// Distinguish degenerate from undecided normals, preserve buffer sentinels/text, then export empty buffers.
    #[test]
    fn undecided_native_normals_leave_abi_buffers_untouched_and_recover() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero = terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
        for (height, expected) in [
            (Real::zero(), CsgrsStatus::ValidationFailed),
            (zero, CsgrsStatus::PredicateUndecided),
        ] {
            let mut mesh = csgrs::solid::polyhedron(
                &[[0, 0, 0], [1, 0, 0], [0, 1, 0]].map(|row| row.map(Real::from)),
                &[&[0, 1, 2]],
            )
            .unwrap();
            std::sync::Arc::make_mut(&mut mesh.positions)[2].y = height;
            let handle = triangle_mesh_handle::<RealFamily>(ScalarMesh::from_native(mesh));
            let mut output = CsgrsGraphicsMeshReal {
                vertices: ptr::null_mut(),
                vertex_len: 17,
                indices: ptr::null_mut(),
                index_len: 19,
            };
            unsafe {
                assert_eq!(
                    csgrs_triangle_mesh_real_graphics_mesh(&handle, &mut output),
                    expected
                );
                assert!(output.vertices.is_null() && output.indices.is_null());
                assert_eq!((output.vertex_len, output.index_len), (17, 19));
                assert_eq!(
                    CStr::from_ptr(csgrs_last_error_message()).to_str().unwrap(),
                    "triangle 0 has no certifiable unit normal"
                );
            }
        }
        let empty =
            triangle_mesh_handle::<RealFamily>(ScalarMesh::from_native(csgrs::solid::empty()));
        let mut output = CsgrsGraphicsMeshReal {
            vertices: ptr::null_mut(),
            vertex_len: 17,
            indices: ptr::null_mut(),
            index_len: 19,
        };
        unsafe {
            assert_eq!(
                csgrs_triangle_mesh_real_graphics_mesh(&empty, &mut output),
                CsgrsStatus::Ok
            );
            assert_eq!((output.vertex_len, output.index_len), (0, 0));
            assert!(CStr::from_ptr(csgrs_last_error_message()).is_empty());
            csgrs_graphics_mesh_real_free(output);
        }
    }

    /// Require an undecided symbolic degeneracy to preserve its status, diagnostic and null output.
    #[test]
    fn undecided_polyhedron_does_not_publish_a_model_handle() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero = CsgrsReal {
            inner: Real::zero(),
        };
        let one = CsgrsReal { inner: Real::one() };
        let two = CsgrsReal { inner: 2.into() };
        let uncertain = CsgrsReal {
            inner: terminal_zero(&sine * &sine + &cosine * &cosine - Real::one()),
        };
        let (z, o, t, u) = (
            ptr::from_ref(&zero).cast_mut(),
            ptr::from_ref(&one).cast_mut(),
            ptr::from_ref(&two).cast_mut(),
            ptr::from_ref(&uncertain).cast_mut(),
        );
        let points = [
            CsgrsVec3Real { x: z, y: z, z },
            CsgrsVec3Real { x: o, y: z, z },
            CsgrsVec3Real { x: t, y: u, z },
        ];
        let indices = [0, 1, 2];
        let offsets = [0, 3];
        let mut output = ptr::null_mut();
        unsafe {
            assert_eq!(
                csgrs_triangle_mesh_real_polyhedron(
                    points.as_ptr(),
                    points.len(),
                    indices.as_ptr(),
                    indices.len(),
                    offsets.as_ptr(),
                    offsets.len(),
                    &mut output
                ),
                CsgrsStatus::PredicateUndecided
            );
            assert!(output.is_null());
            assert!(
                CStr::from_ptr(csgrs_last_error_message())
                    .to_str()
                    .unwrap()
                    .contains("3D triangle degeneracy")
            );
        }
    }

    /// Check both revolution paths reject undecided angles without ownership transfer, plus invalid resolution.
    #[test]
    fn undecided_revolve_preserves_status_and_output_ownership() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero = CsgrsReal {
            inner: terminal_zero(&sine * &sine + &cosine * &cosine - Real::one()),
        };
        let square = curve_region_handle::<RealFamily>(ScalarCurve::from_native(
            csgrs::curve::square(2).unwrap(),
        ));
        let mut output = ptr::null_mut();
        unsafe {
            assert_eq!(
                csgrs_curve_region_real_revolve(&square, &zero, 4, &mut output),
                CsgrsStatus::PredicateUndecided
            );
            assert!(output.is_null());
            assert!(
                CStr::from_ptr(csgrs_last_error_message())
                    .to_str()
                    .unwrap()
                    .contains("revolution angle sign is undecided")
            );
            assert_eq!(
                csgrs_curve_region_real_revolve_solid(&square, &zero, 4, &mut output),
                CsgrsStatus::PredicateUndecided
            );
            assert!(output.is_null());
            let full = CsgrsReal { inner: 360.into() };
            assert_eq!(
                csgrs_curve_region_real_revolve(&square, &full, 1, &mut output),
                CsgrsStatus::ValidationFailed
            );
            assert!(output.is_null());
        }
    }

    /// Separate empty bounds, null output and undecided extrema; check positioning failures and empty reuse.
    #[test]
    fn bounds_absence_and_uncertainty_do_not_publish_real_values() {
        let null_point = CsgrsVec3Real {
            x: ptr::null_mut(),
            y: ptr::null_mut(),
            z: ptr::null_mut(),
        };
        let sentinel = CsgrsAabb3Real {
            mins: null_point,
            maxs: null_point,
        };
        let empty =
            triangle_mesh_handle::<RealFamily>(ScalarMesh::from_native(csgrs::solid::empty()));
        let mut out = sentinel;
        unsafe {
            assert_eq!(
                csgrs_triangle_mesh_real_bounding_box(&empty, &mut out),
                CsgrsStatus::NoBounds
            );
            assert_eq!(out, sentinel);
            assert_eq!(
                csgrs_triangle_mesh_real_bounding_box(&empty, ptr::null_mut()),
                CsgrsStatus::NullPointer
            );
        }
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero = terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
        let mesh = csgrs::TriangleMesh::new(
            vec![
                hyperlattice::Point3::new(sine.clone(), 0.into(), sine.clone()),
                hyperlattice::Point3::new(&sine + &zero, 1.into(), sine + zero),
            ],
            Vec::new(),
        );
        let uncertain = triangle_mesh_handle::<RealFamily>(ScalarMesh::from_native(mesh));
        let mut placed = ptr::null_mut();
        unsafe {
            assert_eq!(
                csgrs_triangle_mesh_real_bounding_box(&uncertain, &mut out),
                CsgrsStatus::PredicateUndecided
            );
            assert_eq!(out, sentinel);
            assert_eq!(
                csgrs_triangle_mesh_real_center(&uncertain, &mut placed),
                CsgrsStatus::PredicateUndecided
            );
            assert!(placed.is_null());
            assert_eq!(
                csgrs_triangle_mesh_real_float(&uncertain, &mut placed),
                CsgrsStatus::PredicateUndecided
            );
            assert!(placed.is_null());
            assert_eq!(
                csgrs_triangle_mesh_real_center(&empty, &mut placed),
                CsgrsStatus::Ok
            );
            assert_eq!(
                csgrs_triangle_mesh_real_bounding_box(placed, &mut out),
                CsgrsStatus::NoBounds
            );
            assert_eq!(out, sentinel);
            csgrs_triangle_mesh_free(placed);
        }
    }

    /// Require an unrecognized zero scale to fail undecided without publishing transformed geometry.
    #[test]
    fn undecided_transform_does_not_publish_an_ffi_handle() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let zero = CsgrsReal {
            inner: terminal_zero(&sine * &sine + &cosine * &cosine - Real::one()),
        };
        let one = CsgrsReal { inner: Real::one() };
        let mut cube = ptr::null_mut();
        let mut result = ptr::null_mut();
        unsafe {
            assert_eq!(
                csgrs_triangle_mesh_real_cube(&one, &mut cube),
                CsgrsStatus::Ok
            );
            assert_eq!(
                csgrs_triangle_mesh_real_scale(cube, &zero, &one, &one, &mut result),
                CsgrsStatus::PredicateUndecided
            );
            assert!(result.is_null());
            assert!(
                CStr::from_ptr(csgrs_last_error_message())
                    .to_str()
                    .unwrap()
                    .contains("scale axis is undecided")
            );
            csgrs_triangle_mesh_free(cube);
        }
    }

    /// Keep undecided width distinct from zero-sized success and retain its diagnostic.
    #[test]
    fn undecided_native_constructor_does_not_publish_an_ffi_handle() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let width = CsgrsReal {
            inner: terminal_zero(&sine * &sine + &cosine * &cosine - Real::one()),
        };
        let mut mesh = ptr::null_mut();
        let status = unsafe { csgrs_triangle_mesh_real_cube(&width, &mut mesh) };
        assert_eq!(status, CsgrsStatus::PredicateUndecided);
        assert!(mesh.is_null());
        let message = unsafe { CStr::from_ptr(csgrs_last_error_message()) }
            .to_str()
            .unwrap();
        assert!(message.contains("width is undecided"));
    }

    /// Provide the all-zero homogeneous matrix as a singular-transform control.
    fn zero_matrix_f64() -> CsgrsMatrix4F64 {
        CsgrsMatrix4F64 { values: [0.0; 16] }
    }

    /// Accept a null input array only in the zero-length branch.
    #[test]
    fn null_pointer_is_a_valid_empty_input_slice() {
        let values = unsafe { c_slice::<u8>(ptr::null(), 0) }.expect("empty slice");
        assert!(values.is_empty());
    }

    /// Diagnose a null nonempty array before forming a Rust slice.
    #[test]
    fn nonempty_input_slice_requires_a_pointer() {
        assert!(matches!(
            unsafe { c_slice::<u8>(ptr::null(), 1) },
            Err(FfiError::NullPointer)
        ));
    }

    /// Exercise a valid partition plus missing endpoints, overflow and decreasing offset controls.
    #[test]
    fn face_offsets_must_cover_the_entire_index_buffer() {
        let indices = [0, 1, 2, 0, 2, 3];
        assert!(build_faces(&[0, 3, 6], &indices).is_ok());
        assert!(build_faces(&[1, 6], &indices).is_err());
        assert!(build_faces(&[0, 3], &indices).is_err());
        assert!(build_faces(&[0, 7], &indices).is_err());
        assert!(build_faces(&[0, 4, 3, 6], &indices).is_err());
    }

    /// Reject NaN as scalar conversion failure and preserve the null mesh output.
    #[test]
    fn nonfinite_float_ingress_fails_without_producing_a_handle() {
        let mut mesh = ptr::null_mut();
        let status = unsafe { csgrs_triangle_mesh_f64_cube(f64::NAN, &mut mesh) };
        assert_eq!(status, CsgrsStatus::ConversionFailed);
        assert!(mesh.is_null());
    }

    /// Reject uncovered trailing indices through the public polyhedron ABI without a partial handle.
    #[test]
    fn malformed_face_offsets_fail_without_producing_a_partial_mesh() {
        let points = [
            CsgrsVec3F64 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            CsgrsVec3F64 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            CsgrsVec3F64 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        ];
        let indices = [0, 1, 2, 0];
        let offsets = [0, 3];
        let mut mesh = ptr::null_mut();
        let status = unsafe {
            csgrs_triangle_mesh_f64_polyhedron(
                points.as_ptr(),
                points.len(),
                indices.as_ptr(),
                indices.len(),
                offsets.as_ptr(),
                offsets.len(),
                &mut mesh,
            )
        };
        assert_eq!(status, CsgrsStatus::ValidationFailed);
        assert!(mesh.is_null());
    }

    /// Reject the zero homogeneous mesh map and free the unchanged source owner.
    #[test]
    fn singular_mesh_transform_is_reported_instead_of_returning_geometry() {
        let mut source = ptr::null_mut();
        assert_eq!(
            unsafe { csgrs_triangle_mesh_f64_cube(1.0, &mut source) },
            CsgrsStatus::Ok
        );
        assert!(!source.is_null());

        let mut transformed = ptr::null_mut();
        let status = unsafe {
            csgrs_triangle_mesh_f64_transform(source, zero_matrix_f64(), &mut transformed)
        };
        assert_eq!(status, CsgrsStatus::ValidationFailed);
        assert!(transformed.is_null());

        unsafe { csgrs_triangle_mesh_free(source) };
    }

    /// Reject the zero homogeneous region map and free the unchanged source owner.
    #[test]
    fn singular_region_transform_is_reported_instead_of_returning_geometry() {
        let mut source = ptr::null_mut();
        assert_eq!(
            unsafe { csgrs_curve_region_f64_square(1.0, &mut source) },
            CsgrsStatus::Ok
        );
        assert!(!source.is_null());

        let mut transformed = ptr::null_mut();
        let status = unsafe {
            csgrs_curve_region_f64_transform(source, zero_matrix_f64(), &mut transformed)
        };
        assert_eq!(status, CsgrsStatus::ValidationFailed);
        assert!(transformed.is_null());

        unsafe { csgrs_curve_region_free(source) };
    }
}
