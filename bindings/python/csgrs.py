"""ctypes facade over native csgrs-ffi, with deferred library loading.

Handle and buffer owners use best-effort __del__ cleanup without explicit close.
Raw adoption/copying is unchecked; do not create multiple owners for one native
allocation, and retain original Real owners while passing borrowed components.
Raw real bounds are not finalizer-wrapped and require the native bounds free.

Python integer conversion is not universally exact ingress: I128.from_int wraps
modulo 2**128 and I128Family.scalar first applies int(). ctypes float conversion
can round before native ingress. Negative size_t counts wrap. Matrix packing
bypasses scalar coercion, and functions do not declare argtypes; geometry calls
therefore require correct family/layout arguments rather than relying on Python
signature checking. See load() for its cached legacy-path lookup behavior.

Native kernels remain authoritative for geometry. Mesh construction history is
queryable, but there is no policy selector or separate local certainty receipt.
Every exported region profile is sampled/projected through binary64 even in the
real family; it is not exact native curve replay. No total-work, retained-memory
or cancellation bound is added by this wrapper.
"""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
from typing import Iterable, Sequence


class CsgrsError(RuntimeError):
    """Python exception carrying diagnostic text and an optional numeric C status.

    Wrapper-authored failures may have status None; ctypes/loader/type errors can
    escape as their own Python exceptions rather than CsgrsError.
    """
    def __init__(self, message: str, status: int | None = None):
        """Store the message and optional C category without parsing the diagnostic."""
        super().__init__(message)
        #: Optional numeric C category; None denotes a wrapper-authored error.
        self.status = status


class _RealHandle(ctypes.Structure):
    """Opaque ctypes scalar pointee; never allocate or inspect native storage through it."""
    pass


class _MeshHandle(ctypes.Structure):
    """Opaque ctypes mesh pointee; live storage must originate from csgrs-ffi."""
    pass


class _CurveRegionHandle(ctypes.Structure):
    """Opaque ctypes filled-region pointee; live storage must originate from csgrs-ffi."""
    pass


#: Opaque native pointer layout; a pointer object alone does not own its allocation.
RealPtr = ctypes.POINTER(_RealHandle)
#: Opaque native pointer layout; a pointer object alone does not own its allocation.
MeshPtr = ctypes.POINTER(_MeshHandle)
#: Opaque native pointer layout; a pointer object alone does not own its allocation.
CurveRegionPtr = ctypes.POINTER(_CurveRegionHandle)


class ConstructionEvidence(ctypes.Structure):
    """Five native construction-history flags, each encoded as uint8 zero or one.

    source_history_missing records absent history; approximate_512_consumed records
    terminal use; coordinates_projected records projection; source_sampled records
    sampling; buffers_changed records native buffer changes. A false approximate
    flag cannot certify missing history, topology or analytic-source fidelity.
    """
    _fields_ = [(name, ctypes.c_uint8) for name in (
        "source_history_missing", "approximate_512_consumed", "coordinates_projected",
        "source_sampled", "buffers_changed",
    )]


class I128(ctypes.Structure):
    """Two-word signed integer layout with signed hi and unsigned lo 64-bit fields.

    from_int deliberately retains only the low 128 bits; it is not range validation.
    """
    _fields_ = [("hi", ctypes.c_int64), ("lo", ctypes.c_uint64)]

    @classmethod
    def from_int(cls, value: int) -> "I128":
        """Wrap a Python integer modulo 2**128 and encode the signed two-word result.

        Out-of-range values do not raise: 2**127 becomes -2**127, and 2**128 becomes zero.
        """
        value &= (1 << 128) - 1
        hi = value >> 64
        if hi >= (1 << 63):
            hi -= 1 << 64
        return cls(hi, value & ((1 << 64) - 1))

    def to_int(self) -> int:
        """Reassemble the signed high word and unsigned low word as a Python integer."""
        value = (int(self.hi) << 64) | int(self.lo)
        if value >= (1 << 127):
            value -= 1 << 128
        return value


class Vec2F32(ctypes.Structure):
    """2 coordinates named x, y in the binary32 ctypes layout."""
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float)]


class Vec3F32(ctypes.Structure):
    """3 coordinates named x, y, z in the binary32 ctypes layout."""
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Vec2F64(ctypes.Structure):
    """2 coordinates named x, y in the binary64 ctypes layout."""
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class Vec3F64(ctypes.Structure):
    """3 coordinates named x, y, z in the binary64 ctypes layout."""
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double)]


class Vec2I128(ctypes.Structure):
    """2 coordinates named x, y in the wrapping signed integer ctypes layout."""
    _fields_ = [("x", I128), ("y", I128)]


class Vec3I128(ctypes.Structure):
    """3 coordinates named x, y, z in the wrapping signed integer ctypes layout."""
    _fields_ = [("x", I128), ("y", I128), ("z", I128)]


class Vec2Real(ctypes.Structure):
    """2 coordinates named x, y in the opaque native scalar ctypes layout.

    Input handles are borrowed; exported handles belong to their native output
    owner. The record itself neither retains Real owners nor frees handles.
    """
    _fields_ = [("x", RealPtr), ("y", RealPtr)]


class Vec3Real(ctypes.Structure):
    """3 coordinates named x, y, z in the opaque native scalar ctypes layout.

    Input handles are borrowed; exported handles belong to their native output
    owner. The record itself neither retains Real owners nor frees handles.
    """
    _fields_ = [("x", RealPtr), ("y", RealPtr), ("z", RealPtr)]


class TriangleU32(ctypes.Structure):
    """Ordered vertex indices a, b, c for one exported triangle; order preserves winding."""
    _fields_ = [("a", ctypes.c_uint32), ("b", ctypes.c_uint32), ("c", ctypes.c_uint32)]


class Aabb3F32(ctypes.Structure):
    """Axis-aligned bounds with mins/maxs XYZ records in the binary32 layout.
    Float coordinates are projections, not certified outward enclosures.
    """
    _fields_ = [("mins", Vec3F32), ("maxs", Vec3F32)]


class Aabb3F64(ctypes.Structure):
    """Axis-aligned bounds with mins/maxs XYZ records in the binary64 layout.
    Float coordinates are projections, not certified outward enclosures.
    """
    _fields_ = [("mins", Vec3F64), ("maxs", Vec3F64)]


class Aabb3I128(ctypes.Structure):
    """Axis-aligned bounds with mins/maxs XYZ records in the wrapping signed integer layout.
    Every exported endpoint must be a recognized in-range integer.
    """
    _fields_ = [("mins", Vec3I128), ("maxs", Vec3I128)]


class Aabb3Real(ctypes.Structure):
    """Axis-aligned bounds with mins/maxs XYZ records in the opaque native scalar layout.

    Owns six native handles on successful export, without an automatic destructor.
    Use the matching native bounds free once; copies do not clone ownership.
    """
    _fields_ = [("mins", Vec3Real), ("maxs", Vec3Real)]


class Matrix4F32(ctypes.Structure):
    """Row-major homogeneous matrix: values contains sixteen binary32 coefficients."""
    _fields_ = [("values", ctypes.c_float * 16)]


class Matrix4F64(ctypes.Structure):
    """Row-major homogeneous matrix: values contains sixteen binary64 coefficients."""
    _fields_ = [("values", ctypes.c_double * 16)]


class Matrix4I128(ctypes.Structure):
    """Row-major homogeneous matrix: values contains sixteen wrapping signed integer coefficients."""
    _fields_ = [("values", I128 * 16)]


class Matrix4Real(ctypes.Structure):
    """Row-major homogeneous matrix: values contains sixteen opaque native scalar coefficients.
    Scalar handles borrow their original Real owners, which must remain live.
    """
    _fields_ = [("values", RealPtr * 16)]


def _graphics_vertex(name: str, vec3_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    """Create a ctypes position/normal record for one scalar layout.

    Normal certification happens natively; scalar projection can change unit length.
    Real-valued output components own scalar handles through their graphics owner.
    """
    return type(
        name,
        (ctypes.Structure,),
        {"__doc__": 'Exported position and native-normal coordinate record; scalar projection can change normal length. Real components are owned by the containing output.', "_fields_": [("position", vec3_type), ("normal", vec3_type)]},
    )



GraphicsVertexF32 = _graphics_vertex("GraphicsVertexF32", Vec3F32)
GraphicsVertexF64 = _graphics_vertex("GraphicsVertexF64", Vec3F64)
GraphicsVertexI128 = _graphics_vertex("GraphicsVertexI128", Vec3I128)
GraphicsVertexReal = _graphics_vertex("GraphicsVertexReal", Vec3Real)


def _mesh_buffers(name: str, vec3_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    """Create an owning C position/triangle array descriptor with no Python destructor.

    vertices/vertex_len count position rows; indices/index_len count TriangleU32
    rows, not individual indices. MeshBuffers supplies the eventual native cleanup.
    """
    return type(
        name,
        (ctypes.Structure,),
        {
            "__doc__": 'Owned vertices/vertex_len and indices/index_len arrays. Indices are triangle records; use a MeshBuffers owner for cleanup. Copying does not duplicate ownership.',
            "_fields_": [
                ("vertices", ctypes.POINTER(vec3_type)),
                ("vertex_len", ctypes.c_size_t),
                ("indices", ctypes.POINTER(TriangleU32)),
                ("index_len", ctypes.c_size_t),
            ]
        },
    )



def _graphics_mesh(name: str, vertex_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    """Create an owning C graphics vertex/flat-index descriptor without a destructor.

    vertex_len counts position/normal records; index_len counts individual uint32
    indices. GraphicsMesh supplies cleanup, including nested real handles.
    """
    return type(
        name,
        (ctypes.Structure,),
        {
            "__doc__": 'Owned vertices/vertex_len graphics records and indices/index_len flat uint32 indices. Use a GraphicsMesh owner for cleanup.',
            "_fields_": [
                ("vertices", ctypes.POINTER(vertex_type)),
                ("vertex_len", ctypes.c_size_t),
                ("indices", ctypes.POINTER(ctypes.c_uint32)),
                ("index_len", ctypes.c_size_t),
            ]
        },
    )



def _region_profiles(name: str, vec2_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    """Create the owning sampled-profile descriptor without an automatic destructor.

    points/point_len hold XY rows. ring_offsets indexes points; region_offsets
    indexes rings. Both offset lengths include an end sentinel, with exterior
    then holes for each region. Empty profiles have one zero per offset array.
    Even Real profiles have passed through binary64 projection.
    """
    return type(
        name,
        (ctypes.Structure,),
        {
            "__doc__": 'Owned points/point_len and two sentinel-offset arrays. ring_offsets indexes points, region_offsets indexes exterior-first rings; lengths include end sentinels. Use RegionProfiles for cleanup. Even real points are binary64-derived.',
            "_fields_": [
                ("points", ctypes.POINTER(vec2_type)),
                ("point_len", ctypes.c_size_t),
                ("ring_offsets", ctypes.POINTER(ctypes.c_size_t)),
                ("ring_offset_len", ctypes.c_size_t),
                ("region_offsets", ctypes.POINTER(ctypes.c_size_t)),
                ("region_offset_len", ctypes.c_size_t),
            ]
        },
    )



MeshBuffersF32 = _mesh_buffers("MeshBuffersF32", Vec3F32)
MeshBuffersF64 = _mesh_buffers("MeshBuffersF64", Vec3F64)
MeshBuffersI128 = _mesh_buffers("MeshBuffersI128", Vec3I128)
MeshBuffersReal = _mesh_buffers("MeshBuffersReal", Vec3Real)
GraphicsMeshF32 = _graphics_mesh("GraphicsMeshF32", GraphicsVertexF32)
GraphicsMeshF64 = _graphics_mesh("GraphicsMeshF64", GraphicsVertexF64)
GraphicsMeshI128 = _graphics_mesh("GraphicsMeshI128", GraphicsVertexI128)
GraphicsMeshReal = _graphics_mesh("GraphicsMeshReal", GraphicsVertexReal)
RegionProfilesF32 = _region_profiles("RegionProfilesF32", Vec2F32)
RegionProfilesF64 = _region_profiles("RegionProfilesF64", Vec2F64)
RegionProfilesI128 = _region_profiles("RegionProfilesI128", Vec2I128)
RegionProfilesReal = _region_profiles("RegionProfilesReal", Vec2Real)


#: Cached first CDLL selection, shared by all owners and calls; initialized without locking.
_lib: ctypes.CDLL | None = None


def load(path: str | os.PathLike[str] | None = None) -> ctypes.CDLL:
    """Load and cache the first selected native library, returning it for later calls.

    Selection is explicit path, then CSGRS_LIBRARY, then legacy repository-relative
    ffi/target/release filenames for Linux/macOS/Windows. That fallback is not the
    current workspace target path; use an explicit path or environment setting.
    Once cached, a different path is ignored. Missing selection raises CsgrsError;
    CDLL/symbol failures can raise other exceptions. Only the last-error restype
    is configured here; argument signatures are not installed and initialization
    has no lock. The cache is assigned before diagnostic-symbol configuration;
    a failure at that step leaves the library cached.
    """
    global _lib
    if _lib is not None:
        return _lib
    if path is None:
        env = os.environ.get("CSGRS_LIBRARY")
        if env:
            path = env
        else:
            here = Path(__file__).resolve()
            candidates = [
                here.parents[2] / "ffi" / "target" / "release" / "libcsgrs_ffi.so",
                here.parents[2] / "ffi" / "target" / "release" / "libcsgrs_ffi.dylib",
                here.parents[2] / "ffi" / "target" / "release" / "csgrs_ffi.dll",
            ]
            path = next((candidate for candidate in candidates if candidate.exists()), None)
    if path is None:
        raise CsgrsError("set CSGRS_LIBRARY to the csgrs-ffi shared library")
    _lib = ctypes.CDLL(str(path))
    _lib.csgrs_last_error_message.restype = ctypes.c_char_p
    return _lib


def _check(status: int) -> None:
    """Raise CsgrsError for nonzero status using the current thread's C diagnostic.

    Call immediately after the ABI action; absent/empty text gets a status fallback.
    The numeric category is retained, while decoding failures remain Python errors.
    """
    if status != 0:
        message = load().csgrs_last_error_message()
        raise CsgrsError(message.decode() if message else f"csgrs status {status}", status)


def _call(name: str, *args):
    """Resolve a named ABI function, set its integer result type, call it and check status.

    No argtypes are declared: callers must provide correctly typed ctypes arguments.
    This is not a signature/argument-count validator or a native panic/memory-safety guard.
    """
    fn = getattr(load(), name)
    fn.restype = ctypes.c_int
    status = fn(*args)
    _check(status)


class Real:
    """Owner of one native scalar pointer, released by a best-effort finalizer.

    Raw adoption is unchecked. Do not wrap one pointer in multiple owners or
    copy ownership. Keep this Python object alive while vectors/matrices borrow
    its pointer. No explicit close or context-manager protocol is provided.
    """
    def __init__(self, ptr: RealPtr):
        """Adopt the supplied native scalar pointer without cloning or validating it."""
        #: Adopted native handle; do not duplicate ownership.
        self.ptr = ptr

    @classmethod
    def zero(cls) -> "Real":
        """Allocate native exact zero and return a new scalar owner."""
        out = RealPtr()
        _call("csgrs_real_zero", ctypes.byref(out))
        return cls(out)

    @classmethod
    def one(cls) -> "Real":
        """Allocate native exact one and return a new scalar owner."""
        out = RealPtr()
        _call("csgrs_real_one", ctypes.byref(out))
        return cls(out)

    @classmethod
    def from_f64(cls, value: float) -> "Real":
        """Convert to binary64 and import that finite binary value exactly.

        NaN/infinity fail natively; 0.1 is not rational one tenth.
        """
        out = RealPtr()
        _call("csgrs_real_from_f64", ctypes.c_double(value), ctypes.byref(out))
        return cls(out)

    @classmethod
    def from_i128(cls, value: int) -> "Real":
        """Wrap the Python integer modulo 2**128, then import the resulting signed value.

        This path does not reject out-of-range Python integers.
        """
        out = RealPtr()
        _call("csgrs_real_from_i128", I128.from_int(value), ctypes.byref(out))
        return cls(out)

    def to_f64(self) -> float:
        """Return a finite lossy binary64 projection or raise on conversion failure."""
        out = ctypes.c_double()
        _call("csgrs_real_to_f64", self.ptr, ctypes.byref(out))
        return float(out.value)

    def to_i128(self) -> int:
        """Export a recognized in-range native integer, rejecting other representations/values."""
        out = I128()
        _call("csgrs_real_to_i128", self.ptr, ctypes.byref(out))
        return out.to_int()

    def __del__(self):
        """Free a non-null owned scalar and clear ptr after success.

        Finalizer execution/timing is not guaranteed, and exceptions during shutdown
        are not reliable cleanup. Borrowed aliases must not outlive this owner.
        """
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_real_free(ptr)
            self.ptr = RealPtr()


class Family:
    """Scalar conversion and ctypes layout strategy used by concrete geometry subclasses.

    The attributes select ABI prefixes/layouts, not independent geometry kernels.
    Only the exact I128Family and RealFamily classes get their specialized scalar
    conversion branches. Changing strategy does not convert an existing handle.
    """
    #: Native symbol-family suffix; changing it does not convert handles.
    prefix: str
    #: ctypes scalar argument layout.
    scalar_type: type
    #: XY coordinate record layout.
    vec2_type: type[ctypes.Structure]
    #: XYZ coordinate record layout.
    vec3_type: type[ctypes.Structure]
    #: Row-major homogeneous-matrix record layout.
    matrix4_type: type[ctypes.Structure]
    #: Exported bounds layout; real records own endpoint handles.
    aabb_type: type[ctypes.Structure]
    #: Exported owning position/triangle descriptor layout.
    mesh_buffers_type: type[ctypes.Structure]
    #: Exported owning graphics descriptor layout.
    graphics_mesh_type: type[ctypes.Structure]
    #: Exported sampled-profile/offset descriptor layout.
    region_profiles_type: type[ctypes.Structure]

    @classmethod
    def scalar(cls, value):
        """Build this family's scalar argument using its current Python coercion rules.

        I128Family first calls int(value), which can truncate/coerce, then wraps modulo
        2**128. RealFamily requires a Real instance and borrows its pointer. Other
        classes call scalar_type directly, with any resulting ctypes rounding/errors.
        """
        if cls is I128Family:
            return I128.from_int(int(value))
        if cls is RealFamily:
            if not isinstance(value, Real):
                raise TypeError("raw-real APIs expect Real values")
            return value.ptr
        return cls.scalar_type(value)

    @classmethod
    def vec2(cls, value):
        """Unpack two values and convert X then Y into the ctypes coordinate layout.

        Real pointers borrow their original Real owners; retain those owners separately.
        """
        x, y = value
        return cls.vec2_type(cls.scalar(x), cls.scalar(y))

    @classmethod
    def vec3(cls, value):
        """Unpack three values and convert X, Y then Z into the ctypes coordinate layout.

        Real pointers borrow their original Real owners; retain those owners separately.
        """
        x, y, z = value
        return cls.vec3_type(cls.scalar(x), cls.scalar(y), cls.scalar(z))

    @classmethod
    def matrix4(cls, values: Sequence):
        """Pack exactly sixteen already-compatible coefficients in row-major order.

        This bypasses scalar(): I128Family accepts I128 records or (hi, lo)
        initializers, not ordinary integers. RealFamily accepts RealPtr values
        or None (null), not Real wrappers. Null coefficients fail native scalar
        admission if used. Length mismatch raises ValueError and incompatible
        elements raise ctypes errors. Original Real owners must stay live.
        """
        if len(values) != 16:
            raise ValueError("matrix4 expects 16 row-major values")
        return cls.matrix4_type((cls.scalar_type * 16)(*values))


class F32Family(Family):
    """Binary32 boundary strategy; ctypes rounds inputs before native finite-value ingress."""
    #: Native symbol-family suffix; changing it does not convert handles.
    prefix = "f32"
    #: ctypes scalar argument layout.
    scalar_type = ctypes.c_float
    #: XY coordinate record layout.
    vec2_type = Vec2F32
    #: XYZ coordinate record layout.
    vec3_type = Vec3F32
    #: Row-major homogeneous-matrix record layout.
    matrix4_type = Matrix4F32
    #: Exported bounds layout; real records own endpoint handles.
    aabb_type = Aabb3F32
    #: Exported owning position/triangle descriptor layout.
    mesh_buffers_type = MeshBuffersF32
    #: Exported owning graphics descriptor layout.
    graphics_mesh_type = GraphicsMeshF32
    #: Exported sampled-profile/offset descriptor layout.
    region_profiles_type = RegionProfilesF32


class F64Family(Family):
    """Binary64 boundary strategy; native import preserves the resulting finite binary value."""
    #: Native symbol-family suffix; changing it does not convert handles.
    prefix = "f64"
    #: ctypes scalar argument layout.
    scalar_type = ctypes.c_double
    #: XY coordinate record layout.
    vec2_type = Vec2F64
    #: XYZ coordinate record layout.
    vec3_type = Vec3F64
    #: Row-major homogeneous-matrix record layout.
    matrix4_type = Matrix4F64
    #: Exported bounds layout; real records own endpoint handles.
    aabb_type = Aabb3F64
    #: Exported owning position/triangle descriptor layout.
    mesh_buffers_type = MeshBuffersF64
    #: Exported owning graphics descriptor layout.
    graphics_mesh_type = GraphicsMeshF64
    #: Exported sampled-profile/offset descriptor layout.
    region_profiles_type = RegionProfilesF64


class I128Family(Family):
    """Signed 128-bit boundary strategy with Python int coercion and modulo wrapping."""
    #: Native symbol-family suffix; changing it does not convert handles.
    prefix = "i128"
    #: ctypes scalar argument layout.
    scalar_type = I128
    #: XY coordinate record layout.
    vec2_type = Vec2I128
    #: XYZ coordinate record layout.
    vec3_type = Vec3I128
    #: Row-major homogeneous-matrix record layout.
    matrix4_type = Matrix4I128
    #: Exported bounds layout; real records own endpoint handles.
    aabb_type = Aabb3I128
    #: Exported owning position/triangle descriptor layout.
    mesh_buffers_type = MeshBuffersI128
    #: Exported owning graphics descriptor layout.
    graphics_mesh_type = GraphicsMeshI128
    #: Exported sampled-profile/offset descriptor layout.
    region_profiles_type = RegionProfilesI128


class RealFamily(Family):
    """Borrowed native scalar input strategy and native scalar output layout.

    Display-profile export still passes through sampled binary64 points.
    """
    #: Native symbol-family suffix; changing it does not convert handles.
    prefix = "real"
    #: ctypes scalar argument layout.
    scalar_type = RealPtr
    #: XY coordinate record layout.
    vec2_type = Vec2Real
    #: XYZ coordinate record layout.
    vec3_type = Vec3Real
    #: Row-major homogeneous-matrix record layout.
    matrix4_type = Matrix4Real
    #: Exported bounds layout; real records own endpoint handles.
    aabb_type = Aabb3Real
    #: Exported owning position/triangle descriptor layout.
    mesh_buffers_type = MeshBuffersReal
    #: Exported owning graphics descriptor layout.
    graphics_mesh_type = GraphicsMeshReal
    #: Exported sampled-profile/offset descriptor layout.
    region_profiles_type = RegionProfilesReal


class TriangleMesh:
    """Native mesh owner whose concrete subclass selects the boundary scalar family.

    Raw adoption is unchecked; keep ptr/family consistent and do not duplicate
    ownership. Calls borrow inputs and return new owners. Cleanup uses __del__,
    not an explicit close/context manager. Separate local receipts are not exposed.
    """
    #: Boundary strategy used for native dispatch and matching cleanup.
    family: type[Family]

    def __init__(self, ptr: MeshPtr):
        """Adopt one native mesh allocation without checking its provenance or family."""
        #: Adopted native handle; do not duplicate ownership.
        self.ptr = ptr

    @classmethod
    def cube(cls, width):
        """Construct an origin-anchored cube with strict nonnegative width validation.

        Validated zero width succeeds as canonical empty geometry after scalar conversion.
        """
        out = MeshPtr()
        _call(f"csgrs_triangle_mesh_{cls.family.prefix}_cube", cls.family.scalar(width), ctypes.byref(out))
        return cls(out)

    @classmethod
    def cuboid(cls, width, length, height):
        """Construct an origin-anchored box after validating all nonnegative dimensions.

        A zero dimension yields canonical empty geometry only after companion validation.
        """
        out = MeshPtr()
        _call(
            f"csgrs_triangle_mesh_{cls.family.prefix}_cuboid",
            cls.family.scalar(width),
            cls.family.scalar(length),
            cls.family.scalar(height),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def sphere(cls, radius, segments: int, stacks: int):
        """Construct a sampled sphere centered at the origin, with Y-axis poles.

        Native minimum counts are three segments and two stacks; radius is nonnegative.
        Counts are explicitly cast to size_t here, so negative values wrap. Sampling
        history does not certify analytic-sphere fidelity.
        """
        out = MeshPtr()
        _call(
            f"csgrs_triangle_mesh_{cls.family.prefix}_sphere",
            cls.family.scalar(radius),
            ctypes.c_size_t(segments),
            ctypes.c_size_t(stacks),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def cylinder(cls, radius, height, segments: int):
        """Construct a sampled capped cylinder from Z zero to signed height.

        Radius must be nonnegative; the native minimum segment count is three. size_t
        conversion wraps negative counts; validation precedes zero-size empty success.
        """
        out = MeshPtr()
        _call(
            f"csgrs_triangle_mesh_{cls.family.prefix}_cylinder",
            cls.family.scalar(radius),
            cls.family.scalar(height),
            ctypes.c_size_t(segments),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def polyhedron(cls, points: Iterable[Sequence], faces: Sequence[Sequence[int]]):
        """Flatten polygon faces, convert points and call native strict triangulation.

        Faces must be a repeatable sequence: this method iterates them twice. Offsets
        cover all flattened indices; native construction validates simple planar faces
        and preserves authored winding, without requiring a closed solid. Index entries
        are cast to size_t. Array lengths are plain Python ints with no argtypes, so
        this wrapper does not establish size_t-correct transport for arbitrary lengths.
        Keep original Real point owners live throughout conversion and the native call.
        """
        point_values = [cls.family.vec3(point) for point in points]
        face_indices = [index for face in faces for index in face]
        offsets = [0]
        for face in faces:
            offsets.append(offsets[-1] + len(face))
        point_array = (cls.family.vec3_type * len(point_values))(*point_values)
        index_array = (ctypes.c_size_t * len(face_indices))(*face_indices)
        offset_array = (ctypes.c_size_t * len(offsets))(*offsets)
        out = MeshPtr()
        _call(
            f"csgrs_triangle_mesh_{cls.family.prefix}_polyhedron",
            point_array,
            len(point_values),
            index_array,
            len(face_indices),
            offset_array,
            len(offsets),
            ctypes.byref(out),
        )
        return cls(out)

    def _binary(self, other: "TriangleMesh", op: str):
        """Dispatch a named same-family mesh operation, check status and adopt its new handle.

        No Python family/type check is added; the ABI checks actual handle families.
        """
        out = MeshPtr()
        _call(f"csgrs_triangle_mesh_{self.family.prefix}_{op}", self.ptr, other.ptr, ctypes.byref(out))
        return self.__class__(out)

    def union(self, other):
        """Return the regularized union as a new owner.
        Mesh decisions are strict. Actual families must match.
        """
        return self._binary(other, "union")
    def difference(self, other):
        """Return the regularized receiver material outside other as a new owner.
        Mesh decisions are strict. Actual families must match.
        """
        return self._binary(other, "difference")
    def intersection(self, other):
        """Return the regularized common material as a new owner.
        Mesh decisions are strict. Actual families must match.
        """
        return self._binary(other, "intersection")
    def xor(self, other):
        """Return the regularized material in exactly one operand as a new owner.
        Mesh decisions are strict. Actual families must match.
        """
        return self._binary(other, "xor")

    def transform(self, matrix):
        """Apply an already-typed row-major matrix through strict native transform admission.

        Supply the matching family matrix and keep borrowed real coefficients alive.
        Singular, undecided or unbounded images can fail; no separate receipt is returned.
        """
        out = MeshPtr()
        _call(f"csgrs_triangle_mesh_{self.family.prefix}_transform", self.ptr, matrix, ctypes.byref(out))
        return self.__class__(out)

    def translate(self, x, y, z):
        """Translate by the converted XYZ displacement.
        Returns a new owner without a separate operation receipt.
        """
        return self._triple("translate", x, y, z)
    def scale(self, sx, sy, sz):
        """Scale XYZ through strict native transform admission.
        Zero mesh scale axes fail; reflections preserve winding.
        """
        return self._triple("scale", sx, sy, sz)
    def rotate(self, x, y, z):
        """Rotate about X, then Y, then Z with angles in degrees.
        Returns a new owner without a separate operation receipt.
        """
        return self._triple("rotate", x, y, z)

    def _triple(self, op, x, y, z):
        """Convert three scalar arguments, invoke the named mesh transform and adopt its output."""
        out = MeshPtr()
        _call(
            f"csgrs_triangle_mesh_{self.family.prefix}_{op}",
            self.ptr,
            self.family.scalar(x),
            self.family.scalar(y),
            self.family.scalar(z),
            ctypes.byref(out),
        )
        return self.__class__(out)

    def inverse(self):
        """Reverse triangle winding without constructing an unbounded set complement."""
        return self._unary("inverse")
    def center(self):
        """Move the native bounds center to the origin; empty geometry is preserved."""
        return self._unary("center")
    def floating(self):
        """Move the native minimum Z bound to zero; this is placement, not scalar conversion."""
        return self._unary("float")

    def construction_evidence(self):
        """Read retained history flags, not a topology or analytic-fidelity certificate."""
        out = ConstructionEvidence()
        _call("csgrs_triangle_mesh_construction_evidence", self.ptr, ctypes.byref(out))
        return out

    def _unary(self, op):
        """Invoke a named unary mesh operation on the borrowed input and adopt its output."""
        out = MeshPtr()
        _call(f"csgrs_triangle_mesh_{self.family.prefix}_{op}", self.ptr, ctypes.byref(out))
        return self.__class__(out)

    def bounding_box(self):
        """Return exported native bounds, None for empty geometry, or raise on failure.

        Float endpoints are not outward-enclosure certificates. Real-family output
        is a raw record owning six handles without a Python cleanup wrapper; callers
        are responsible for the matching native bounds free.
        """
        out = self.family.aabb_type()
        status = getattr(load(), f"csgrs_triangle_mesh_{self.family.prefix}_bounding_box")(self.ptr, ctypes.byref(out))
        if status == 10:  # CSGRS_STATUS_NO_BOUNDS: completed empty query
            return None
        _check(status)
        return out

    def vertices_and_indices(self):
        """Export scalar-family positions and triangle rows in a finalizer-backed buffer owner.

        Float coordinates are lossy, integer export can fail, and real coordinates
        retain native scalars. Borrow raw arrays only while the returned owner is live.
        """
        out = self.family.mesh_buffers_type()
        _call(f"csgrs_triangle_mesh_{self.family.prefix}_vertices_and_indices", self.ptr, ctypes.byref(out))
        return MeshBuffers(self.family, out)

    def graphics_mesh(self):
        """Export position/normal records and flat indices in a finalizer-backed owner.

        Native normal certification or scalar export can fail; projection can change
        normal lengths. Borrow raw arrays only while the owner is live.
        """
        out = self.family.graphics_mesh_type()
        _call(f"csgrs_triangle_mesh_{self.family.prefix}_graphics_mesh", self.ptr, ctypes.byref(out))
        return GraphicsMesh(self.family, out)

    def __del__(self):
        """Free the non-null mesh handle and clear it after success; finalization is best effort."""
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_triangle_mesh_free(ptr)
            self.ptr = MeshPtr()


class CurveRegion:
    """Native filled-region owner whose concrete subclass selects an ABI scalar family.

    Calls borrow inputs and return new owners; raw adoption does not validate
    provenance/family. Cleanup uses a best-effort __del__, without explicit close.
    No policy selector or separate local certainty receipt is exposed.
    """
    #: Boundary strategy used for native dispatch and matching cleanup.
    family: type[Family]

    def __init__(self, ptr: CurveRegionPtr):
        """Adopt one native region allocation without checking its provenance or family."""
        #: Adopted native handle; do not duplicate ownership.
        self.ptr = ptr

    @classmethod
    def square(cls, width):
        """Construct an origin-anchored XY square with strict nonnegative width validation.
        Validated zero width returns canonical empty geometry.
        """
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{cls.family.prefix}_square", cls.family.scalar(width), ctypes.byref(out))
        return cls(out)

    @classmethod
    def rectangle(cls, width, length):
        """Construct an origin-anchored XY rectangle, validating both nonnegative dimensions.
        A zero dimension does not bypass companion validation.
        """
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{cls.family.prefix}_rectangle", cls.family.scalar(width), cls.family.scalar(length), ctypes.byref(out))
        return cls(out)

    @classmethod
    def circle(cls, radius, segments: int):
        """Construct a sampled regular XY polygon, not an analytic circular boundary.

        Requires nonnegative radius and at least three native segments, even for zero
        radius. Negative Python counts wrap when converted to size_t.
        """
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{cls.family.prefix}_circle", cls.family.scalar(radius), ctypes.c_size_t(segments), ctypes.byref(out))
        return cls(out)

    @classmethod
    def polygon(cls, points: Iterable[Sequence]):
        """Convert XY points and construct a valid filled polygon under strict predicates.

        Empty/malformed rings fail. The array length is a plain Python int without
        argtypes, not an explicit size_t argument. Keep original Real owners alive
        through conversion and the call when using the real family.
        """
        values = [cls.family.vec2(point) for point in points]
        array = (cls.family.vec2_type * len(values))(*values)
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{cls.family.prefix}_polygon", array, len(values), ctypes.byref(out))
        return cls(out)

    def _binary(self, other: "CurveRegion", op: str):
        """Dispatch a named region Boolean on borrowed handles and adopt the new result.

        Actual family validation is native; Hypercurve uses its active policy scope.
        """
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_{op}", self.ptr, other.ptr, ctypes.byref(out))
        return self.__class__(out)

    def union(self, other):
        """Return the regularized union as a new owner.
        Uses Hypercurve's active scope, strict outside preview/provisional evaluation. Actual families must match.
        """
        return self._binary(other, "union")
    def difference(self, other):
        """Return the regularized receiver material outside other as a new owner.
        Uses Hypercurve's active scope, strict outside preview/provisional evaluation. Actual families must match.
        """
        return self._binary(other, "difference")
    def intersection(self, other):
        """Return the regularized common material as a new owner.
        Uses Hypercurve's active scope, strict outside preview/provisional evaluation. Actual families must match.
        """
        return self._binary(other, "intersection")
    def xor(self, other):
        """Return the regularized material in exactly one operand as a new owner.
        Uses Hypercurve's active scope, strict outside preview/provisional evaluation. Actual families must match.
        """
        return self._binary(other, "xor")

    def transform(self, matrix):
        """Apply a matching typed matrix whose XY restriction is affine and nonsingular.

        Strict planar admission rejects unsupported perspective/out-of-plane images.
        Real coefficient owners must stay live; no separate receipt is returned.
        """
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_transform", self.ptr, matrix, ctypes.byref(out))
        return self.__class__(out)

    def translate(self, x, y, z):
        """Translate by the converted XYZ displacement.
        Nonzero Z cannot remain in the XY carrier.
        """
        return self._triple("translate", x, y, z)
    def scale(self, sx, sy, sz):
        """Scale XYZ through strict native transform admission.
        The XY image must remain planar and nonsingular; Z scale has no planar effect.
        """
        return self._triple("scale", sx, sy, sz)
    def rotate(self, x, y, z):
        """Rotate about X, then Y, then Z with angles in degrees.
        The image must remain in the XY carrier.
        """
        return self._triple("rotate", x, y, z)

    def _triple(self, op, x, y, z):
        """Convert three scalar arguments, invoke the named planar transform and adopt its output."""
        out = CurveRegionPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_{op}", self.ptr, self.family.scalar(x), self.family.scalar(y), self.family.scalar(z), ctypes.byref(out))
        return self.__class__(out)

    def bounding_box(self):
        """Return exported region bounds, None for completed absence, or raise on failure.

        Trimmed fragments may use a looser certified source envelope; float endpoints
        are not outward enclosures. Raw real bounds own six manually freed handles
        without a Python cleanup wrapper.
        """
        out = self.family.aabb_type()
        status = getattr(load(), f"csgrs_curve_region_{self.family.prefix}_bounding_box")(self.ptr, ctypes.byref(out))
        if status == 10:  # CSGRS_STATUS_NO_BOUNDS: completed empty query
            return None
        _check(status)
        return out

    def extrude(self, height):
        """Extrude along Z by a nonzero signed height using strict predicates.

        Returns a matching-family mesh owner. Nonlinear interiors may be sampled with
        construction history; the separate local receipt is discarded at the ABI.
        """
        out = MeshPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_extrude", self.ptr, self.family.scalar(height), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def extrude_vector(self, direction):
        """Extrude along a converted XYZ vector whose Z component must be nonzero.

        Uses strict predicates and the native profile-lowering fidelity contract.
        Real scalar owners must remain live through the call.
        """
        out = MeshPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_extrude_vector", self.ptr, self.family.vec3(direction), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def revolve(self, angle, segments: int):
        """Build a segmented swept boundary about Y with X as signed radius.

        Native angle rules reject zero and oversized noninteger turns; partial/full
        turns require at least two/three segments. Partial axis-crossing loops fail
        and opposite-side pieces may overlap. size_t counts can wrap; strict decisions
        and retained construction history do not certify analytic-surface fidelity.
        """
        out = MeshPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_revolve", self.ptr, self.family.scalar(angle), ctypes.c_size_t(segments), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def revolve_solid(self, angle, segments: int):
        """Regularize the union of sampled revolution pieces about Y.

        Uses the same angle/resolution domain as revolve, admits axis-crossing partial
        profiles and counts overlapping material once. Closed point/edge contacts may
        remain nonmanifold; no separate operation receipt is returned.
        """
        out = MeshPtr()
        _call(f"csgrs_curve_region_{self.family.prefix}_revolve_solid", self.ptr, self.family.scalar(angle), ctypes.c_size_t(segments), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def region_profiles(self):
        """Export sampled exterior/hole rows in a finalizer-backed profile owner.

        Every family, including Real, first passes through binary64 display points.
        This is not exact native replay. Borrow raw arrays only while the owner is live.
        """
        out = self.family.region_profiles_type()
        _call(f"csgrs_curve_region_{self.family.prefix}_region_profiles", self.ptr, ctypes.byref(out))
        return RegionProfiles(self.family, out)

    def __del__(self):
        """Free the non-null region handle and clear it after success; finalization is best effort."""
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_curve_region_free(ptr)
            self.ptr = CurveRegionPtr()


class TriangleMeshF32(TriangleMesh):
    """TriangleMesh owner selecting the binary32 boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = F32Family
class TriangleMeshF64(TriangleMesh):
    """TriangleMesh owner selecting the binary64 boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = F64Family
class TriangleMeshI128(TriangleMesh):
    """TriangleMesh owner selecting the wrapping signed integer boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = I128Family
class TriangleMeshReal(TriangleMesh):
    """TriangleMesh owner selecting the opaque native scalar boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = RealFamily
class CurveRegionF32(CurveRegion):
    """CurveRegion owner selecting the binary32 boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = F32Family
class CurveRegionF64(CurveRegion):
    """CurveRegion owner selecting the binary64 boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = F64Family
class CurveRegionI128(CurveRegion):
    """CurveRegion owner selecting the wrapping signed integer boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = I128Family
class CurveRegionReal(CurveRegion):
    """CurveRegion owner selecting the opaque native scalar boundary strategy through family."""
    #: Boundary strategy used for native dispatch and matching cleanup.
    family = RealFamily


def mesh_class_for(family):
    """Select one of the four concrete mesh classes by exact family-class identity.
    Unknown/subclass family keys raise KeyError; this does not convert geometry.
    """
    return {F32Family: TriangleMeshF32, F64Family: TriangleMeshF64, I128Family: TriangleMeshI128, RealFamily: TriangleMeshReal}[family]


class MeshBuffers:
    """Finalizer-backed owner of exported position/triangle arrays.

    family selects the matching native free and raw is the owning C descriptor.
    Borrowed views must not outlive this object; do not copy or separately free
    its arrays/nested handles. No explicit close/context manager is supplied.
    """
    def __init__(self, family: type[Family], raw):
        """Adopt a successful raw descriptor with its exact family, without validating either."""
        #: Strategy used to select the matching native destructor.
        self.family = family
        #: Adopted owning native array descriptor.
        self.raw = raw

    def __del__(self):
        """Free the descriptor and nested real handles with its family function, then clear raw.

        Finalization is best effort; exceptions can prevent clearing/reclamation.
        Copied descriptors are not independent owners.
        """
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_triangle_mesh_buffers_{self.family.prefix}_free")(raw)
            self.raw = None


class GraphicsMesh:
    """Finalizer-backed owner of exported graphics vertex/index arrays.

    family selects the matching native free and raw is the owning C descriptor.
    Borrowed views must not outlive this object; do not copy or separately free
    its arrays/nested handles. No explicit close/context manager is supplied.
    """
    def __init__(self, family: type[Family], raw):
        """Adopt a successful raw descriptor with its exact family, without validating either."""
        #: Strategy used to select the matching native destructor.
        self.family = family
        #: Adopted owning native array descriptor.
        self.raw = raw

    def __del__(self):
        """Free the descriptor and nested real handles with its family function, then clear raw.

        Finalization is best effort; exceptions can prevent clearing/reclamation.
        Copied descriptors are not independent owners.
        """
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_graphics_mesh_{self.family.prefix}_free")(raw)
            self.raw = None


class RegionProfiles:
    """Finalizer-backed owner of exported sampled profile/offset arrays.

    family selects the matching native free and raw is the owning C descriptor.
    Borrowed views must not outlive this object; do not copy or separately free
    its arrays/nested handles. No explicit close/context manager is supplied.
    """
    def __init__(self, family: type[Family], raw):
        """Adopt a successful raw descriptor with its exact family, without validating either."""
        #: Strategy used to select the matching native destructor.
        self.family = family
        #: Adopted owning native array descriptor.
        self.raw = raw

    def __del__(self):
        """Free the descriptor and nested real handles with its family function, then clear raw.

        Finalization is best effort; exceptions can prevent clearing/reclamation.
        Copied descriptors are not independent owners.
        """
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_region_profiles_{self.family.prefix}_free")(raw)
            self.raw = None
