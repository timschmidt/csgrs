#![allow(unsafe_code)]
#![allow(private_bounds, private_interfaces)]
#![deny(unused)]

use csgrs_adapter::{
    Aabb3, AdapterError, F32, F64, I128, Mesh, Profile, RawReal, Real, ScalarAdapter,
};
use hyperlattice::Matrix4;
use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::fmt;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;
use std::slice;

type MeshOf<F> = Mesh<<F as Family>::Adapter, ()>;
type ProfileOf<F> = Profile<<F as Family>::Adapter>;
type AdapterScalar<F> = <<F as Family>::Adapter as ScalarAdapter>::Scalar;

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CsgrsStatus {
    Ok = 0,
    NullPointer = 1,
    WrongScalarFamily = 2,
    ConversionFailed = 3,
    ValidationFailed = 4,
    Panic = 255,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CsgrsScalarFamily {
    F32 = 1,
    F64 = 2,
    I128 = 3,
    Real = 4,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsI128 {
    pub hi: i64,
    pub lo: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2F32 {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3F32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2F64 {
    pub x: f64,
    pub y: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3F64 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsVec2I128 {
    pub x: CsgrsI128,
    pub y: CsgrsI128,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsVec3I128 {
    pub x: CsgrsI128,
    pub y: CsgrsI128,
    pub z: CsgrsI128,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec2Real {
    pub x: *mut CsgrsReal,
    pub y: *mut CsgrsReal,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsVec3Real {
    pub x: *mut CsgrsReal,
    pub y: *mut CsgrsReal,
    pub z: *mut CsgrsReal,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsTriangleU32 {
    pub a: u32,
    pub b: u32,
    pub c: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3F32 {
    pub mins: CsgrsVec3F32,
    pub maxs: CsgrsVec3F32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3F64 {
    pub mins: CsgrsVec3F64,
    pub maxs: CsgrsVec3F64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsAabb3I128 {
    pub mins: CsgrsVec3I128,
    pub maxs: CsgrsVec3I128,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsAabb3Real {
    pub mins: CsgrsVec3Real,
    pub maxs: CsgrsVec3Real,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4F32 {
    pub values: [f32; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4F64 {
    pub values: [f64; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsMatrix4I128 {
    pub values: [CsgrsI128; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsMatrix4Real {
    pub values: [*const CsgrsReal; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexF32 {
    pub position: CsgrsVec3F32,
    pub normal: CsgrsVec3F32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexF64 {
    pub position: CsgrsVec3F64,
    pub normal: CsgrsVec3F64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CsgrsGraphicsVertexI128 {
    pub position: CsgrsVec3I128,
    pub normal: CsgrsVec3I128,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsgrsGraphicsVertexReal {
    pub position: CsgrsVec3Real,
    pub normal: CsgrsVec3Real,
}

macro_rules! define_buffers {
    ($mesh:ident, $graphics:ident, $region:ident, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        #[repr(C)]
        #[derive(Debug)]
        pub struct $mesh {
            pub vertices: *mut $vec3,
            pub vertex_len: usize,
            pub indices: *mut CsgrsTriangleU32,
            pub index_len: usize,
        }

        #[repr(C)]
        #[derive(Debug)]
        pub struct $graphics {
            pub vertices: *mut $gvertex,
            pub vertex_len: usize,
            pub indices: *mut u32,
            pub index_len: usize,
        }

        #[repr(C)]
        #[derive(Debug)]
        pub struct $region {
            pub points: *mut $vec2,
            pub point_len: usize,
            pub ring_offsets: *mut usize,
            pub ring_offset_len: usize,
            pub profile_offsets: *mut usize,
            pub profile_offset_len: usize,
        }
    };
}

define_buffers!(
    CsgrsMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);
define_buffers!(
    CsgrsMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);
define_buffers!(
    CsgrsMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);
define_buffers!(
    CsgrsMeshBuffersReal,
    CsgrsGraphicsMeshReal,
    CsgrsRegionProfilesReal,
    CsgrsVec2Real,
    CsgrsVec3Real,
    CsgrsGraphicsVertexReal
);

pub struct CsgrsReal {
    inner: Real,
}

pub struct CsgrsMesh {
    inner: MeshKind,
}

pub struct CsgrsProfile {
    inner: ProfileKind,
}

enum MeshKind {
    F32(Mesh<F32, ()>),
    F64(Mesh<F64, ()>),
    I128(Mesh<I128, ()>),
    Real(Mesh<RawReal, ()>),
}

enum ProfileKind {
    F32(Profile<F32>),
    F64(Profile<F64>),
    I128(Profile<I128>),
    Real(Profile<RawReal>),
}

#[derive(Debug)]
enum FfiError {
    NullPointer,
    WrongScalarFamily,
    Adapter(AdapterError),
}

impl fmt::Display for FfiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NullPointer => f.write_str("null pointer"),
            Self::WrongScalarFamily => f.write_str("handle belongs to another scalar family"),
            Self::Adapter(error) => write!(f, "{error}"),
        }
    }
}

impl From<AdapterError> for FfiError {
    fn from(value: AdapterError) -> Self {
        Self::Adapter(value)
    }
}

type FfiResult<T> = Result<T, FfiError>;

thread_local! {
    static LAST_ERROR: RefCell<CString> =
        RefCell::new(CString::new("").expect("empty string has no interior nul"));
}

trait Family {
    type Adapter: ScalarAdapter;
    type CScalar: Copy;
    type CVec2: Copy;
    type CVec3: Copy;
    type CMatrix4: Copy;
    type CAabb: Copy;
    type CGraphicsVertex: Copy;
    type CMeshBuffers;
    type CGraphicsMesh;
    type CRegionProfiles;

    fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<AdapterScalar<Self>>;
    fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[AdapterScalar<Self>; 2]>;
    fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[AdapterScalar<Self>; 3]>;
    fn vec2_from_adapter(value: [AdapterScalar<Self>; 2]) -> FfiResult<Self::CVec2>;
    fn vec3_from_adapter(value: [AdapterScalar<Self>; 3]) -> FfiResult<Self::CVec3>;
    fn matrix4_to_core(value: Self::CMatrix4) -> FfiResult<Matrix4>;
    fn aabb_from_adapter(value: Aabb3<AdapterScalar<Self>>) -> FfiResult<Self::CAabb>;
    fn graphics_vertex_from_adapter(
        position: [AdapterScalar<Self>; 3],
        normal: [AdapterScalar<Self>; 3],
    ) -> FfiResult<Self::CGraphicsVertex>;
    fn mesh_kind(mesh: MeshOf<Self>) -> MeshKind;
    fn profile_kind(profile: ProfileOf<Self>) -> ProfileKind;
    fn mesh_ref(mesh: &CsgrsMesh) -> FfiResult<&MeshOf<Self>>;
    fn profile_ref(profile: &CsgrsProfile) -> FfiResult<&ProfileOf<Self>>;
    fn mesh_buffers(
        vertices: Vec<Self::CVec3>,
        indices: Vec<CsgrsTriangleU32>,
    ) -> Self::CMeshBuffers;
    fn graphics_mesh(
        vertices: Vec<Self::CGraphicsVertex>,
        indices: Vec<u32>,
    ) -> Self::CGraphicsMesh;
    fn region_profiles(
        points: Vec<Self::CVec2>,
        ring_offsets: Vec<usize>,
        profile_offsets: Vec<usize>,
    ) -> Self::CRegionProfiles;
}

struct F32Family;
struct F64Family;
struct I128Family;
struct RealFamily;

fn i128_from_parts(value: CsgrsI128) -> i128 {
    (i128::from(value.hi) << 64) | i128::from(value.lo)
}

fn i128_to_parts(value: i128) -> CsgrsI128 {
    CsgrsI128 {
        hi: (value >> 64) as i64,
        lo: value as u64,
    }
}

fn real_handle(value: Real) -> *mut CsgrsReal {
    Box::into_raw(Box::new(CsgrsReal { inner: value }))
}

unsafe fn real_from_ptr(ptr: *const CsgrsReal) -> FfiResult<Real> {
    if ptr.is_null() {
        return Err(FfiError::NullPointer);
    }
    Ok(unsafe { &*ptr }.inner.clone())
}

fn real_to_i128(value: &Real) -> FfiResult<i128> {
    let rational = value.exact_rational().ok_or(AdapterError::NotExactInteger)?;
    rational
        .try_into()
        .map_err(AdapterError::from)
        .map_err(Into::into)
}

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
        $profile_variant:ident,
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
            type Adapter = $adapter;
            type CScalar = $scalar;
            type CVec2 = $vec2;
            type CVec3 = $vec3;
            type CMatrix4 = $matrix;
            type CAabb = $aabb;
            type CGraphicsVertex = $gvertex;
            type CMeshBuffers = $mesh_buffers;
            type CGraphicsMesh = $graphics_mesh;
            type CRegionProfiles = $region_profiles;

            fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<AdapterScalar<Self>> {
                ($scalar_to_adapter)(value)
            }

            fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[AdapterScalar<Self>; 2]> {
                let [x, y] = ($vec2_to_array)(value);
                Ok([Self::scalar_to_adapter(x)?, Self::scalar_to_adapter(y)?])
            }

            fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[AdapterScalar<Self>; 3]> {
                let [x, y, z] = ($vec3_to_array)(value);
                Ok([
                    Self::scalar_to_adapter(x)?,
                    Self::scalar_to_adapter(y)?,
                    Self::scalar_to_adapter(z)?,
                ])
            }

            fn vec2_from_adapter(value: [AdapterScalar<Self>; 2]) -> FfiResult<Self::CVec2> {
                let [x, y] = value;
                ($vec2_from_array)([x, y])
            }

            fn vec3_from_adapter(value: [AdapterScalar<Self>; 3]) -> FfiResult<Self::CVec3> {
                let [x, y, z] = value;
                ($vec3_from_array)([x, y, z])
            }

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

            fn aabb_from_adapter(value: Aabb3<AdapterScalar<Self>>) -> FfiResult<Self::CAabb> {
                ($aabb_from_parts)(value)
            }

            fn graphics_vertex_from_adapter(
                position: [AdapterScalar<Self>; 3],
                normal: [AdapterScalar<Self>; 3],
            ) -> FfiResult<Self::CGraphicsVertex> {
                ($gvertex_from_parts)(position, normal)
            }

            fn mesh_kind(mesh: MeshOf<Self>) -> MeshKind {
                MeshKind::$mesh_variant(mesh)
            }

            fn profile_kind(profile: ProfileOf<Self>) -> ProfileKind {
                ProfileKind::$profile_variant(profile)
            }

            fn mesh_ref(mesh: &CsgrsMesh) -> FfiResult<&MeshOf<Self>> {
                match &mesh.inner {
                    MeshKind::$mesh_variant(mesh) => Ok(mesh),
                    _ => Err(FfiError::WrongScalarFamily),
                }
            }

            fn profile_ref(profile: &CsgrsProfile) -> FfiResult<&ProfileOf<Self>> {
                match &profile.inner {
                    ProfileKind::$profile_variant(profile) => Ok(profile),
                    _ => Err(FfiError::WrongScalarFamily),
                }
            }

            fn mesh_buffers(
                vertices: Vec<Self::CVec3>,
                indices: Vec<CsgrsTriangleU32>,
            ) -> Self::CMeshBuffers {
                let (vertices, vertex_len) = boxed_slice_parts(vertices);
                let (indices, index_len) = boxed_slice_parts(indices);
                <$mesh_buffers>::new(vertices, vertex_len, indices, index_len)
            }

            fn graphics_mesh(
                vertices: Vec<Self::CGraphicsVertex>,
                indices: Vec<u32>,
            ) -> Self::CGraphicsMesh {
                let (vertices, vertex_len) = boxed_slice_parts(vertices);
                let (indices, index_len) = boxed_slice_parts(indices);
                <$graphics_mesh>::new(vertices, vertex_len, indices, index_len)
            }

            fn region_profiles(
                points: Vec<Self::CVec2>,
                ring_offsets: Vec<usize>,
                profile_offsets: Vec<usize>,
            ) -> Self::CRegionProfiles {
                let (points, point_len) = boxed_slice_parts(points);
                let (ring_offsets, ring_offset_len) = boxed_slice_parts(ring_offsets);
                let (profile_offsets, profile_offset_len) = boxed_slice_parts(profile_offsets);
                <$region_profiles>::new(
                    points,
                    point_len,
                    ring_offsets,
                    ring_offset_len,
                    profile_offsets,
                    profile_offset_len,
                )
            }
        }
    };
}

macro_rules! impl_buffer_constructors {
    ($mesh:ty, $graphics:ty, $region:ty, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        impl $mesh {
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
            fn new(
                points: *mut $vec2,
                point_len: usize,
                ring_offsets: *mut usize,
                ring_offset_len: usize,
                profile_offsets: *mut usize,
                profile_offset_len: usize,
            ) -> Self {
                Self {
                    points,
                    point_len,
                    ring_offsets,
                    ring_offset_len,
                    profile_offsets,
                    profile_offset_len,
                }
            }
        }
    };
}

impl_buffer_constructors!(
    CsgrsMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);
impl_buffer_constructors!(
    CsgrsMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);
impl_buffer_constructors!(
    CsgrsMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);
impl_buffer_constructors!(
    CsgrsMeshBuffersReal,
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
    CsgrsMeshBuffersF32,
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
    CsgrsMeshBuffersF64,
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
    CsgrsMeshBuffersI128,
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
    type Adapter = RawReal;
    type CScalar = *const CsgrsReal;
    type CVec2 = CsgrsVec2Real;
    type CVec3 = CsgrsVec3Real;
    type CMatrix4 = CsgrsMatrix4Real;
    type CAabb = CsgrsAabb3Real;
    type CGraphicsVertex = CsgrsGraphicsVertexReal;
    type CMeshBuffers = CsgrsMeshBuffersReal;
    type CGraphicsMesh = CsgrsGraphicsMeshReal;
    type CRegionProfiles = CsgrsRegionProfilesReal;

    fn scalar_to_adapter(value: Self::CScalar) -> FfiResult<Real> {
        unsafe { real_from_ptr(value) }
    }

    fn vec2_to_adapter(value: Self::CVec2) -> FfiResult<[Real; 2]> {
        Ok([
            Self::scalar_to_adapter(value.x)?,
            Self::scalar_to_adapter(value.y)?,
        ])
    }

    fn vec3_to_adapter(value: Self::CVec3) -> FfiResult<[Real; 3]> {
        Ok([
            Self::scalar_to_adapter(value.x)?,
            Self::scalar_to_adapter(value.y)?,
            Self::scalar_to_adapter(value.z)?,
        ])
    }

    fn vec2_from_adapter(value: [Real; 2]) -> FfiResult<Self::CVec2> {
        let [x, y] = value;
        Ok(CsgrsVec2Real {
            x: real_handle(x),
            y: real_handle(y),
        })
    }

    fn vec3_from_adapter(value: [Real; 3]) -> FfiResult<Self::CVec3> {
        let [x, y, z] = value;
        Ok(CsgrsVec3Real {
            x: real_handle(x),
            y: real_handle(y),
            z: real_handle(z),
        })
    }

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

    fn graphics_vertex_from_adapter(
        position: [Real; 3],
        normal: [Real; 3],
    ) -> FfiResult<Self::CGraphicsVertex> {
        Ok(CsgrsGraphicsVertexReal {
            position: Self::vec3_from_adapter(position)?,
            normal: Self::vec3_from_adapter(normal)?,
        })
    }

    fn mesh_kind(mesh: MeshOf<Self>) -> MeshKind {
        MeshKind::Real(mesh)
    }

    fn profile_kind(profile: ProfileOf<Self>) -> ProfileKind {
        ProfileKind::Real(profile)
    }

    fn mesh_ref(mesh: &CsgrsMesh) -> FfiResult<&MeshOf<Self>> {
        match &mesh.inner {
            MeshKind::Real(mesh) => Ok(mesh),
            _ => Err(FfiError::WrongScalarFamily),
        }
    }

    fn profile_ref(profile: &CsgrsProfile) -> FfiResult<&ProfileOf<Self>> {
        match &profile.inner {
            ProfileKind::Real(profile) => Ok(profile),
            _ => Err(FfiError::WrongScalarFamily),
        }
    }

    fn mesh_buffers(
        vertices: Vec<Self::CVec3>,
        indices: Vec<CsgrsTriangleU32>,
    ) -> Self::CMeshBuffers {
        let (vertices, vertex_len) = boxed_slice_parts(vertices);
        let (indices, index_len) = boxed_slice_parts(indices);
        CsgrsMeshBuffersReal::new(vertices, vertex_len, indices, index_len)
    }

    fn graphics_mesh(
        vertices: Vec<Self::CGraphicsVertex>,
        indices: Vec<u32>,
    ) -> Self::CGraphicsMesh {
        let (vertices, vertex_len) = boxed_slice_parts(vertices);
        let (indices, index_len) = boxed_slice_parts(indices);
        CsgrsGraphicsMeshReal::new(vertices, vertex_len, indices, index_len)
    }

    fn region_profiles(
        points: Vec<Self::CVec2>,
        ring_offsets: Vec<usize>,
        profile_offsets: Vec<usize>,
    ) -> Self::CRegionProfiles {
        let (points, point_len) = boxed_slice_parts(points);
        let (ring_offsets, ring_offset_len) = boxed_slice_parts(ring_offsets);
        let (profile_offsets, profile_offset_len) = boxed_slice_parts(profile_offsets);
        CsgrsRegionProfilesReal::new(
            points,
            point_len,
            ring_offsets,
            ring_offset_len,
            profile_offsets,
            profile_offset_len,
        )
    }
}

fn boxed_slice_parts<T>(values: Vec<T>) -> (*mut T, usize) {
    let boxed = values.into_boxed_slice();
    let len = boxed.len();
    (Box::into_raw(boxed) as *mut T, len)
}

unsafe fn free_boxed_slice<T>(ptr: *mut T, len: usize) {
    if !ptr.is_null() {
        let slice = ptr::slice_from_raw_parts_mut(ptr, len);
        unsafe {
            drop(Box::from_raw(slice));
        }
    }
}

fn set_last_error(message: impl fmt::Display) {
    let sanitized = message.to_string().replace('\0', "\\0");
    let c_string = CString::new(sanitized).unwrap_or_else(|_| {
        CString::new("failed to format error").expect("literal has no interior nul")
    });
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = c_string;
    });
}

fn status_for_error(error: &FfiError) -> CsgrsStatus {
    match error {
        FfiError::NullPointer => CsgrsStatus::NullPointer,
        FfiError::WrongScalarFamily => CsgrsStatus::WrongScalarFamily,
        FfiError::Adapter(AdapterError::Validation(_)) => CsgrsStatus::ValidationFailed,
        FfiError::Adapter(_) => CsgrsStatus::ConversionFailed,
    }
}

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

unsafe fn out_ptr<'a, T>(out: *mut T) -> FfiResult<&'a mut T> {
    if out.is_null() {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { &mut *out })
    }
}

unsafe fn out_handle<T>(out: *mut *mut T, value: T) -> FfiResult<()> {
    let out = unsafe { out_ptr(out) }?;
    *out = Box::into_raw(Box::new(value));
    Ok(())
}

unsafe fn ptr_ref<'a, T>(ptr: *const T) -> FfiResult<&'a T> {
    if ptr.is_null() {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { &*ptr })
    }
}

unsafe fn c_slice<'a, T>(ptr: *const T, len: usize) -> FfiResult<&'a [T]> {
    if ptr.is_null() && len != 0 {
        Err(FfiError::NullPointer)
    } else {
        Ok(unsafe { slice::from_raw_parts(ptr, len) })
    }
}

unsafe fn c_str(ptr: *const c_char) -> FfiResult<String> {
    if ptr.is_null() {
        return Err(FfiError::NullPointer);
    }
    let text = unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .map_err(|err| AdapterError::Validation(err.to_string()))?;
    Ok(text.to_owned())
}

fn parse_i128(text: &str) -> FfiResult<i128> {
    text.parse::<i128>()
        .map_err(|err| AdapterError::Validation(err.to_string()).into())
}

fn mesh_handle<F: Family>(mesh: MeshOf<F>) -> CsgrsMesh {
    CsgrsMesh {
        inner: F::mesh_kind(mesh),
    }
}

fn profile_handle<F: Family>(profile: ProfileOf<F>) -> CsgrsProfile {
    CsgrsProfile {
        inner: F::profile_kind(profile),
    }
}

fn build_faces<'a>(offsets: &'a [usize], indices: &'a [usize]) -> FfiResult<Vec<&'a [usize]>> {
    if offsets.len() < 2 {
        return Err(AdapterError::Validation(
            "face_offsets must contain at least two values".into(),
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

fn create_mesh_buffers<F: Family>(mesh: &MeshOf<F>) -> FfiResult<F::CMeshBuffers> {
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

fn create_graphics_mesh<F: Family>(mesh: &MeshOf<F>) -> FfiResult<F::CGraphicsMesh> {
    let graphics = mesh.graphics_mesh()?;
    let vertices = graphics
        .vertices
        .into_iter()
        .map(|(position, normal)| F::graphics_vertex_from_adapter(position, normal))
        .collect::<FfiResult<Vec<_>>>()?;
    Ok(F::graphics_mesh(vertices, graphics.indices))
}

fn create_region_profiles<F: Family>(profile: &ProfileOf<F>) -> FfiResult<F::CRegionProfiles> {
    let profiles = profile.region_profiles()?;
    let mut points = Vec::new();
    let mut ring_offsets = vec![0];
    let mut profile_offsets = vec![0];
    let mut ring_count = 0;

    for profile in profiles {
        for point in profile.exterior {
            points.push(F::vec2_from_adapter(point)?);
        }
        ring_count += 1;
        ring_offsets.push(points.len());

        for hole in profile.holes {
            for point in hole {
                points.push(F::vec2_from_adapter(point)?);
            }
            ring_count += 1;
            ring_offsets.push(points.len());
        }
        profile_offsets.push(ring_count);
    }

    Ok(F::region_profiles(points, ring_offsets, profile_offsets))
}

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
        profile: {
            square: $profile_square:ident,
            rectangle: $profile_rectangle:ident,
            circle: $profile_circle:ident,
            polygon: $profile_polygon:ident,
            union: $profile_union:ident,
            difference: $profile_difference:ident,
            intersection: $profile_intersection:ident,
            xor: $profile_xor:ident,
            transform: $profile_transform:ident,
            translate: $profile_translate:ident,
            scale: $profile_scale:ident,
            rotate: $profile_rotate:ident,
            bounding_box: $profile_bounding_box:ident,
            extrude: $profile_extrude:ident,
            extrude_vector: $profile_extrude_vector:ident,
            revolve: $profile_revolve:ident,
            region_profiles: $profile_region_profiles:ident
        }
    ) => {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cube(
            width: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = MeshOf::<$family>::cube(
                    <$family as Family>::scalar_to_adapter(width)?,
                    (),
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cuboid(
            width: <$family as Family>::CScalar,
            length: <$family as Family>::CScalar,
            height: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = MeshOf::<$family>::cuboid(
                    <$family as Family>::scalar_to_adapter(width)?,
                    <$family as Family>::scalar_to_adapter(length)?,
                    <$family as Family>::scalar_to_adapter(height)?,
                    (),
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_sphere(
            radius: <$family as Family>::CScalar,
            segments: usize,
            stacks: usize,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = MeshOf::<$family>::sphere(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    segments,
                    stacks,
                    (),
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_cylinder(
            radius: <$family as Family>::CScalar,
            height: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = MeshOf::<$family>::cylinder(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    <$family as Family>::scalar_to_adapter(height)?,
                    segments,
                    (),
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_polyhedron(
            points: *const <$family as Family>::CVec3,
            point_len: usize,
            face_indices: *const usize,
            face_index_len: usize,
            face_offsets: *const usize,
            face_offset_len: usize,
            out: *mut *mut CsgrsMesh,
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
                let mesh = MeshOf::<$family>::polyhedron(&points, &faces, ())?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_union(
            lhs: *const CsgrsMesh,
            rhs: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(lhs.union(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_difference(
            lhs: *const CsgrsMesh,
            rhs: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(lhs.difference(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_intersection(
            lhs: *const CsgrsMesh,
            rhs: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(lhs.intersection(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_xor(
            lhs: *const CsgrsMesh,
            rhs: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::mesh_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(lhs.xor(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_transform(
            mesh: *const CsgrsMesh,
            matrix: <$family as Family>::CMatrix4,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let matrix = <$family as Family>::matrix4_to_core(matrix)?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh.transform(&matrix))) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_translate(
            mesh: *const CsgrsMesh,
            x: <$family as Family>::CScalar,
            y: <$family as Family>::CScalar,
            z: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.translate(
                    <$family as Family>::scalar_to_adapter(x)?,
                    <$family as Family>::scalar_to_adapter(y)?,
                    <$family as Family>::scalar_to_adapter(z)?,
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_scale(
            mesh: *const CsgrsMesh,
            sx: <$family as Family>::CScalar,
            sy: <$family as Family>::CScalar,
            sz: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.scale(
                    <$family as Family>::scalar_to_adapter(sx)?,
                    <$family as Family>::scalar_to_adapter(sy)?,
                    <$family as Family>::scalar_to_adapter(sz)?,
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_rotate(
            mesh: *const CsgrsMesh,
            x_degrees: <$family as Family>::CScalar,
            y_degrees: <$family as Family>::CScalar,
            z_degrees: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                let result = mesh.rotate(
                    <$family as Family>::scalar_to_adapter(x_degrees)?,
                    <$family as Family>::scalar_to_adapter(y_degrees)?,
                    <$family as Family>::scalar_to_adapter(z_degrees)?,
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_inverse(
            mesh: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh.inverse())) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_center(
            mesh: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh.center())) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_float(
            mesh: *const CsgrsMesh,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh.float())) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_bounding_box(
            mesh: *const CsgrsMesh,
            out: *mut <$family as Family>::CAabb,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                *unsafe { out_ptr(out) }? =
                    <$family as Family>::aabb_from_adapter(mesh.bounding_box()?)?;
                Ok(())
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_vertices_and_indices(
            mesh: *const CsgrsMesh,
            out: *mut <$family as Family>::CMeshBuffers,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                *unsafe { out_ptr(out) }? = create_mesh_buffers::<$family>(mesh)?;
                Ok(())
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_graphics_mesh(
            mesh: *const CsgrsMesh,
            out: *mut <$family as Family>::CGraphicsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let mesh = <$family as Family>::mesh_ref(unsafe { ptr_ref(mesh) }?)?;
                *unsafe { out_ptr(out) }? = create_graphics_mesh::<$family>(mesh)?;
                Ok(())
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_square(
            width: <$family as Family>::CScalar,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = ProfileOf::<$family>::square(
                    <$family as Family>::scalar_to_adapter(width)?,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(profile)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_rectangle(
            width: <$family as Family>::CScalar,
            length: <$family as Family>::CScalar,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = ProfileOf::<$family>::rectangle(
                    <$family as Family>::scalar_to_adapter(width)?,
                    <$family as Family>::scalar_to_adapter(length)?,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(profile)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_circle(
            radius: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = ProfileOf::<$family>::circle(
                    <$family as Family>::scalar_to_adapter(radius)?,
                    segments,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(profile)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_polygon(
            points: *const <$family as Family>::CVec2,
            point_len: usize,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let points = unsafe { c_slice(points, point_len) }?
                    .iter()
                    .copied()
                    .map(<$family as Family>::vec2_to_adapter)
                    .collect::<FfiResult<Vec<_>>>()?;
                let profile = ProfileOf::<$family>::polygon(&points )?;
                unsafe { out_handle(out, profile_handle::<$family>(profile)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_union(
            lhs: *const CsgrsProfile,
            rhs: *const CsgrsProfile,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::profile_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::profile_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, profile_handle::<$family>(lhs.union(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_difference(
            lhs: *const CsgrsProfile,
            rhs: *const CsgrsProfile,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::profile_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::profile_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, profile_handle::<$family>(lhs.difference(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_intersection(
            lhs: *const CsgrsProfile,
            rhs: *const CsgrsProfile,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::profile_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::profile_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, profile_handle::<$family>(lhs.intersection(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_xor(
            lhs: *const CsgrsProfile,
            rhs: *const CsgrsProfile,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let lhs = <$family as Family>::profile_ref(unsafe { ptr_ref(lhs) }?)?;
                let rhs = <$family as Family>::profile_ref(unsafe { ptr_ref(rhs) }?)?;
                unsafe { out_handle(out, profile_handle::<$family>(lhs.xor(rhs)?)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_transform(
            profile: *const CsgrsProfile,
            matrix: <$family as Family>::CMatrix4,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let matrix = <$family as Family>::matrix4_to_core(matrix)?;
                unsafe {
                    out_handle(out, profile_handle::<$family>(profile.transform(&matrix)))
                }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_translate(
            profile: *const CsgrsProfile,
            x: <$family as Family>::CScalar,
            y: <$family as Family>::CScalar,
            z: <$family as Family>::CScalar,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let result = profile.translate(
                    <$family as Family>::scalar_to_adapter(x)?,
                    <$family as Family>::scalar_to_adapter(y)?,
                    <$family as Family>::scalar_to_adapter(z)?,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_scale(
            profile: *const CsgrsProfile,
            sx: <$family as Family>::CScalar,
            sy: <$family as Family>::CScalar,
            sz: <$family as Family>::CScalar,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let result = profile.scale(
                    <$family as Family>::scalar_to_adapter(sx)?,
                    <$family as Family>::scalar_to_adapter(sy)?,
                    <$family as Family>::scalar_to_adapter(sz)?,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_rotate(
            profile: *const CsgrsProfile,
            x_degrees: <$family as Family>::CScalar,
            y_degrees: <$family as Family>::CScalar,
            z_degrees: <$family as Family>::CScalar,
            out: *mut *mut CsgrsProfile,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let result = profile.rotate(
                    <$family as Family>::scalar_to_adapter(x_degrees)?,
                    <$family as Family>::scalar_to_adapter(y_degrees)?,
                    <$family as Family>::scalar_to_adapter(z_degrees)?,
                )?;
                unsafe { out_handle(out, profile_handle::<$family>(result)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_bounding_box(
            profile: *const CsgrsProfile,
            out: *mut <$family as Family>::CAabb,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                *unsafe { out_ptr(out) }? =
                    <$family as Family>::aabb_from_adapter(profile.bounding_box()?)?;
                Ok(())
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_extrude(
            profile: *const CsgrsProfile,
            height: <$family as Family>::CScalar,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let mesh = profile.extrude(<$family as Family>::scalar_to_adapter(height)?, ())?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_extrude_vector(
            profile: *const CsgrsProfile,
            direction: <$family as Family>::CVec3,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let mesh = profile
                    .extrude_vector(<$family as Family>::vec3_to_adapter(direction)?, ())?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_revolve(
            profile: *const CsgrsProfile,
            angle_degrees: <$family as Family>::CScalar,
            segments: usize,
            out: *mut *mut CsgrsMesh,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                let mesh = profile.revolve(
                    <$family as Family>::scalar_to_adapter(angle_degrees)?,
                    segments,
                    (),
                )?;
                unsafe { out_handle(out, mesh_handle::<$family>(mesh)) }
            })
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $profile_region_profiles(
            profile: *const CsgrsProfile,
            out: *mut <$family as Family>::CRegionProfiles,
        ) -> CsgrsStatus {
            ffi_status(|| {
                let profile = <$family as Family>::profile_ref(unsafe { ptr_ref(profile) }?)?;
                *unsafe { out_ptr(out) }? = create_region_profiles::<$family>(profile)?;
                Ok(())
            })
        }
    };
}

export_family!(
    F32Family,
    mesh: {
        cube: csgrs_mesh_f32_cube,
        cuboid: csgrs_mesh_f32_cuboid,
        sphere: csgrs_mesh_f32_sphere,
        cylinder: csgrs_mesh_f32_cylinder,
        polyhedron: csgrs_mesh_f32_polyhedron,
        union: csgrs_mesh_f32_union,
        difference: csgrs_mesh_f32_difference,
        intersection: csgrs_mesh_f32_intersection,
        xor: csgrs_mesh_f32_xor,
        transform: csgrs_mesh_f32_transform,
        translate: csgrs_mesh_f32_translate,
        scale: csgrs_mesh_f32_scale,
        rotate: csgrs_mesh_f32_rotate,
        inverse: csgrs_mesh_f32_inverse,
        center: csgrs_mesh_f32_center,
        float: csgrs_mesh_f32_float,
        bounding_box: csgrs_mesh_f32_bounding_box,
        vertices_and_indices: csgrs_mesh_f32_vertices_and_indices,
        graphics_mesh: csgrs_mesh_f32_graphics_mesh
    },
    profile: {
        square: csgrs_profile_f32_square,
        rectangle: csgrs_profile_f32_rectangle,
        circle: csgrs_profile_f32_circle,
        polygon: csgrs_profile_f32_polygon,
        union: csgrs_profile_f32_union,
        difference: csgrs_profile_f32_difference,
        intersection: csgrs_profile_f32_intersection,
        xor: csgrs_profile_f32_xor,
        transform: csgrs_profile_f32_transform,
        translate: csgrs_profile_f32_translate,
        scale: csgrs_profile_f32_scale,
        rotate: csgrs_profile_f32_rotate,
        bounding_box: csgrs_profile_f32_bounding_box,
        extrude: csgrs_profile_f32_extrude,
        extrude_vector: csgrs_profile_f32_extrude_vector,
        revolve: csgrs_profile_f32_revolve,
        region_profiles: csgrs_profile_f32_region_profiles
    }
);

export_family!(
    F64Family,
    mesh: {
        cube: csgrs_mesh_f64_cube,
        cuboid: csgrs_mesh_f64_cuboid,
        sphere: csgrs_mesh_f64_sphere,
        cylinder: csgrs_mesh_f64_cylinder,
        polyhedron: csgrs_mesh_f64_polyhedron,
        union: csgrs_mesh_f64_union,
        difference: csgrs_mesh_f64_difference,
        intersection: csgrs_mesh_f64_intersection,
        xor: csgrs_mesh_f64_xor,
        transform: csgrs_mesh_f64_transform,
        translate: csgrs_mesh_f64_translate,
        scale: csgrs_mesh_f64_scale,
        rotate: csgrs_mesh_f64_rotate,
        inverse: csgrs_mesh_f64_inverse,
        center: csgrs_mesh_f64_center,
        float: csgrs_mesh_f64_float,
        bounding_box: csgrs_mesh_f64_bounding_box,
        vertices_and_indices: csgrs_mesh_f64_vertices_and_indices,
        graphics_mesh: csgrs_mesh_f64_graphics_mesh
    },
    profile: {
        square: csgrs_profile_f64_square,
        rectangle: csgrs_profile_f64_rectangle,
        circle: csgrs_profile_f64_circle,
        polygon: csgrs_profile_f64_polygon,
        union: csgrs_profile_f64_union,
        difference: csgrs_profile_f64_difference,
        intersection: csgrs_profile_f64_intersection,
        xor: csgrs_profile_f64_xor,
        transform: csgrs_profile_f64_transform,
        translate: csgrs_profile_f64_translate,
        scale: csgrs_profile_f64_scale,
        rotate: csgrs_profile_f64_rotate,
        bounding_box: csgrs_profile_f64_bounding_box,
        extrude: csgrs_profile_f64_extrude,
        extrude_vector: csgrs_profile_f64_extrude_vector,
        revolve: csgrs_profile_f64_revolve,
        region_profiles: csgrs_profile_f64_region_profiles
    }
);

export_family!(
    I128Family,
    mesh: {
        cube: csgrs_mesh_i128_cube,
        cuboid: csgrs_mesh_i128_cuboid,
        sphere: csgrs_mesh_i128_sphere,
        cylinder: csgrs_mesh_i128_cylinder,
        polyhedron: csgrs_mesh_i128_polyhedron,
        union: csgrs_mesh_i128_union,
        difference: csgrs_mesh_i128_difference,
        intersection: csgrs_mesh_i128_intersection,
        xor: csgrs_mesh_i128_xor,
        transform: csgrs_mesh_i128_transform,
        translate: csgrs_mesh_i128_translate,
        scale: csgrs_mesh_i128_scale,
        rotate: csgrs_mesh_i128_rotate,
        inverse: csgrs_mesh_i128_inverse,
        center: csgrs_mesh_i128_center,
        float: csgrs_mesh_i128_float,
        bounding_box: csgrs_mesh_i128_bounding_box,
        vertices_and_indices: csgrs_mesh_i128_vertices_and_indices,
        graphics_mesh: csgrs_mesh_i128_graphics_mesh
    },
    profile: {
        square: csgrs_profile_i128_square,
        rectangle: csgrs_profile_i128_rectangle,
        circle: csgrs_profile_i128_circle,
        polygon: csgrs_profile_i128_polygon,
        union: csgrs_profile_i128_union,
        difference: csgrs_profile_i128_difference,
        intersection: csgrs_profile_i128_intersection,
        xor: csgrs_profile_i128_xor,
        transform: csgrs_profile_i128_transform,
        translate: csgrs_profile_i128_translate,
        scale: csgrs_profile_i128_scale,
        rotate: csgrs_profile_i128_rotate,
        bounding_box: csgrs_profile_i128_bounding_box,
        extrude: csgrs_profile_i128_extrude,
        extrude_vector: csgrs_profile_i128_extrude_vector,
        revolve: csgrs_profile_i128_revolve,
        region_profiles: csgrs_profile_i128_region_profiles
    }
);

export_family!(
    RealFamily,
    mesh: {
        cube: csgrs_mesh_real_cube,
        cuboid: csgrs_mesh_real_cuboid,
        sphere: csgrs_mesh_real_sphere,
        cylinder: csgrs_mesh_real_cylinder,
        polyhedron: csgrs_mesh_real_polyhedron,
        union: csgrs_mesh_real_union,
        difference: csgrs_mesh_real_difference,
        intersection: csgrs_mesh_real_intersection,
        xor: csgrs_mesh_real_xor,
        transform: csgrs_mesh_real_transform,
        translate: csgrs_mesh_real_translate,
        scale: csgrs_mesh_real_scale,
        rotate: csgrs_mesh_real_rotate,
        inverse: csgrs_mesh_real_inverse,
        center: csgrs_mesh_real_center,
        float: csgrs_mesh_real_float,
        bounding_box: csgrs_mesh_real_bounding_box,
        vertices_and_indices: csgrs_mesh_real_vertices_and_indices,
        graphics_mesh: csgrs_mesh_real_graphics_mesh
    },
    profile: {
        square: csgrs_profile_real_square,
        rectangle: csgrs_profile_real_rectangle,
        circle: csgrs_profile_real_circle,
        polygon: csgrs_profile_real_polygon,
        union: csgrs_profile_real_union,
        difference: csgrs_profile_real_difference,
        intersection: csgrs_profile_real_intersection,
        xor: csgrs_profile_real_xor,
        transform: csgrs_profile_real_transform,
        translate: csgrs_profile_real_translate,
        scale: csgrs_profile_real_scale,
        rotate: csgrs_profile_real_rotate,
        bounding_box: csgrs_profile_real_bounding_box,
        extrude: csgrs_profile_real_extrude,
        extrude_vector: csgrs_profile_real_extrude_vector,
        revolve: csgrs_profile_real_revolve,
        region_profiles: csgrs_profile_real_region_profiles
    }
);

#[unsafe(no_mangle)]
pub extern "C" fn csgrs_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_one(out: *mut *mut CsgrsReal) -> CsgrsStatus {
    ffi_status(|| unsafe { out_handle(out, CsgrsReal { inner: Real::one() }) })
}

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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_real_free(value: *mut CsgrsReal) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_mesh_free(value: *mut CsgrsMesh) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_profile_free(value: *mut CsgrsProfile) {
    if !value.is_null() {
        unsafe {
            drop(Box::from_raw(value));
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_mesh_family(
    value: *const CsgrsMesh,
    out: *mut CsgrsScalarFamily,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = match value.inner {
            MeshKind::F32(_) => CsgrsScalarFamily::F32,
            MeshKind::F64(_) => CsgrsScalarFamily::F64,
            MeshKind::I128(_) => CsgrsScalarFamily::I128,
            MeshKind::Real(_) => CsgrsScalarFamily::Real,
        };
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_profile_family(
    value: *const CsgrsProfile,
    out: *mut CsgrsScalarFamily,
) -> CsgrsStatus {
    ffi_status(|| {
        let value = unsafe { ptr_ref(value) }?;
        *unsafe { out_ptr(out) }? = match value.inner {
            ProfileKind::F32(_) => CsgrsScalarFamily::F32,
            ProfileKind::F64(_) => CsgrsScalarFamily::F64,
            ProfileKind::I128(_) => CsgrsScalarFamily::I128,
            ProfileKind::Real(_) => CsgrsScalarFamily::Real,
        };
        Ok(())
    })
}

macro_rules! export_free_buffers {
    ($mesh_free:ident, $graphics_free:ident, $region_free:ident, $mesh:ty, $graphics:ty, $region:ty, $vec2:ty, $vec3:ty, $gvertex:ty) => {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $mesh_free(value: $mesh) {
            unsafe {
                free_boxed_slice::<$vec3>(value.vertices, value.vertex_len);
                free_boxed_slice::<CsgrsTriangleU32>(value.indices, value.index_len);
            }
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $graphics_free(value: $graphics) {
            unsafe {
                free_boxed_slice::<$gvertex>(value.vertices, value.vertex_len);
                free_boxed_slice::<u32>(value.indices, value.index_len);
            }
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $region_free(value: $region) {
            unsafe {
                free_boxed_slice::<$vec2>(value.points, value.point_len);
                free_boxed_slice::<usize>(value.ring_offsets, value.ring_offset_len);
                free_boxed_slice::<usize>(value.profile_offsets, value.profile_offset_len);
            }
        }
    };
}

export_free_buffers!(
    csgrs_mesh_buffers_f32_free,
    csgrs_graphics_mesh_f32_free,
    csgrs_region_profiles_f32_free,
    CsgrsMeshBuffersF32,
    CsgrsGraphicsMeshF32,
    CsgrsRegionProfilesF32,
    CsgrsVec2F32,
    CsgrsVec3F32,
    CsgrsGraphicsVertexF32
);

export_free_buffers!(
    csgrs_mesh_buffers_f64_free,
    csgrs_graphics_mesh_f64_free,
    csgrs_region_profiles_f64_free,
    CsgrsMeshBuffersF64,
    CsgrsGraphicsMeshF64,
    CsgrsRegionProfilesF64,
    CsgrsVec2F64,
    CsgrsVec3F64,
    CsgrsGraphicsVertexF64
);

export_free_buffers!(
    csgrs_mesh_buffers_i128_free,
    csgrs_graphics_mesh_i128_free,
    csgrs_region_profiles_i128_free,
    CsgrsMeshBuffersI128,
    CsgrsGraphicsMeshI128,
    CsgrsRegionProfilesI128,
    CsgrsVec2I128,
    CsgrsVec3I128,
    CsgrsGraphicsVertexI128
);

unsafe fn free_real_vec2(value: CsgrsVec2Real) {
    unsafe {
        csgrs_real_free(value.x);
        csgrs_real_free(value.y);
    }
}

unsafe fn free_real_vec3(value: CsgrsVec3Real) {
    unsafe {
        csgrs_real_free(value.x);
        csgrs_real_free(value.y);
        csgrs_real_free(value.z);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_aabb3_real_free_values(value: CsgrsAabb3Real) {
    unsafe {
        free_real_vec3(value.mins);
        free_real_vec3(value.maxs);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn csgrs_mesh_buffers_real_free(value: CsgrsMeshBuffersReal) {
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
        free_boxed_slice::<usize>(value.profile_offsets, value.profile_offset_len);
    }
}
