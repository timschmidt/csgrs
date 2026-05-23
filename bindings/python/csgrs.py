from __future__ import annotations

import ctypes
import os
from pathlib import Path
from typing import Iterable, Sequence


class CsgrsError(RuntimeError):
    pass


class _RealHandle(ctypes.Structure):
    pass


class _MeshHandle(ctypes.Structure):
    pass


class _ProfileHandle(ctypes.Structure):
    pass


RealPtr = ctypes.POINTER(_RealHandle)
MeshPtr = ctypes.POINTER(_MeshHandle)
ProfilePtr = ctypes.POINTER(_ProfileHandle)


class I128(ctypes.Structure):
    _fields_ = [("hi", ctypes.c_int64), ("lo", ctypes.c_uint64)]

    @classmethod
    def from_int(cls, value: int) -> "I128":
        value &= (1 << 128) - 1
        hi = value >> 64
        if hi >= (1 << 63):
            hi -= 1 << 64
        return cls(hi, value & ((1 << 64) - 1))

    def to_int(self) -> int:
        value = (int(self.hi) << 64) | int(self.lo)
        if value >= (1 << 127):
            value -= 1 << 128
        return value


class Vec2F32(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float)]


class Vec3F32(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Vec2F64(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class Vec3F64(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double)]


class Vec2I128(ctypes.Structure):
    _fields_ = [("x", I128), ("y", I128)]


class Vec3I128(ctypes.Structure):
    _fields_ = [("x", I128), ("y", I128), ("z", I128)]


class Vec2Real(ctypes.Structure):
    _fields_ = [("x", RealPtr), ("y", RealPtr)]


class Vec3Real(ctypes.Structure):
    _fields_ = [("x", RealPtr), ("y", RealPtr), ("z", RealPtr)]


class TriangleU32(ctypes.Structure):
    _fields_ = [("a", ctypes.c_uint32), ("b", ctypes.c_uint32), ("c", ctypes.c_uint32)]


class Aabb3F32(ctypes.Structure):
    _fields_ = [("mins", Vec3F32), ("maxs", Vec3F32)]


class Aabb3F64(ctypes.Structure):
    _fields_ = [("mins", Vec3F64), ("maxs", Vec3F64)]


class Aabb3I128(ctypes.Structure):
    _fields_ = [("mins", Vec3I128), ("maxs", Vec3I128)]


class Aabb3Real(ctypes.Structure):
    _fields_ = [("mins", Vec3Real), ("maxs", Vec3Real)]


class Matrix4F32(ctypes.Structure):
    _fields_ = [("values", ctypes.c_float * 16)]


class Matrix4F64(ctypes.Structure):
    _fields_ = [("values", ctypes.c_double * 16)]


class Matrix4I128(ctypes.Structure):
    _fields_ = [("values", I128 * 16)]


class Matrix4Real(ctypes.Structure):
    _fields_ = [("values", RealPtr * 16)]


def _graphics_vertex(name: str, vec3_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    return type(
        name,
        (ctypes.Structure,),
        {"_fields_": [("position", vec3_type), ("normal", vec3_type)]},
    )


GraphicsVertexF32 = _graphics_vertex("GraphicsVertexF32", Vec3F32)
GraphicsVertexF64 = _graphics_vertex("GraphicsVertexF64", Vec3F64)
GraphicsVertexI128 = _graphics_vertex("GraphicsVertexI128", Vec3I128)
GraphicsVertexReal = _graphics_vertex("GraphicsVertexReal", Vec3Real)


def _mesh_buffers(name: str, vec3_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    return type(
        name,
        (ctypes.Structure,),
        {
            "_fields_": [
                ("vertices", ctypes.POINTER(vec3_type)),
                ("vertex_len", ctypes.c_size_t),
                ("indices", ctypes.POINTER(TriangleU32)),
                ("index_len", ctypes.c_size_t),
            ]
        },
    )


def _graphics_mesh(name: str, vertex_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    return type(
        name,
        (ctypes.Structure,),
        {
            "_fields_": [
                ("vertices", ctypes.POINTER(vertex_type)),
                ("vertex_len", ctypes.c_size_t),
                ("indices", ctypes.POINTER(ctypes.c_uint32)),
                ("index_len", ctypes.c_size_t),
            ]
        },
    )


def _region_profiles(name: str, vec2_type: type[ctypes.Structure]) -> type[ctypes.Structure]:
    return type(
        name,
        (ctypes.Structure,),
        {
            "_fields_": [
                ("points", ctypes.POINTER(vec2_type)),
                ("point_len", ctypes.c_size_t),
                ("ring_offsets", ctypes.POINTER(ctypes.c_size_t)),
                ("ring_offset_len", ctypes.c_size_t),
                ("profile_offsets", ctypes.POINTER(ctypes.c_size_t)),
                ("profile_offset_len", ctypes.c_size_t),
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


_lib: ctypes.CDLL | None = None


def load(path: str | os.PathLike[str] | None = None) -> ctypes.CDLL:
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
    if status != 0:
        message = load().csgrs_last_error_message()
        raise CsgrsError(message.decode() if message else f"csgrs status {status}")


def _call(name: str, *args):
    fn = getattr(load(), name)
    fn.restype = ctypes.c_int
    status = fn(*args)
    _check(status)


class Real:
    def __init__(self, ptr: RealPtr):
        self.ptr = ptr

    @classmethod
    def zero(cls) -> "Real":
        out = RealPtr()
        _call("csgrs_real_zero", ctypes.byref(out))
        return cls(out)

    @classmethod
    def one(cls) -> "Real":
        out = RealPtr()
        _call("csgrs_real_one", ctypes.byref(out))
        return cls(out)

    @classmethod
    def from_f64(cls, value: float) -> "Real":
        out = RealPtr()
        _call("csgrs_real_from_f64", ctypes.c_double(value), ctypes.byref(out))
        return cls(out)

    @classmethod
    def from_i128(cls, value: int) -> "Real":
        out = RealPtr()
        _call("csgrs_real_from_i128", I128.from_int(value), ctypes.byref(out))
        return cls(out)

    def to_f64(self) -> float:
        out = ctypes.c_double()
        _call("csgrs_real_to_f64", self.ptr, ctypes.byref(out))
        return float(out.value)

    def to_i128(self) -> int:
        out = I128()
        _call("csgrs_real_to_i128", self.ptr, ctypes.byref(out))
        return out.to_int()

    def __del__(self):
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_real_free(ptr)
            self.ptr = RealPtr()


class Family:
    prefix: str
    scalar_type: type
    vec2_type: type[ctypes.Structure]
    vec3_type: type[ctypes.Structure]
    matrix4_type: type[ctypes.Structure]
    aabb_type: type[ctypes.Structure]
    mesh_buffers_type: type[ctypes.Structure]
    graphics_mesh_type: type[ctypes.Structure]
    region_profiles_type: type[ctypes.Structure]

    @classmethod
    def scalar(cls, value):
        if cls is I128Family:
            return I128.from_int(int(value))
        if cls is RealFamily:
            if not isinstance(value, Real):
                raise TypeError("raw-real APIs expect Real values")
            return value.ptr
        return cls.scalar_type(value)

    @classmethod
    def vec2(cls, value):
        x, y = value
        return cls.vec2_type(cls.scalar(x), cls.scalar(y))

    @classmethod
    def vec3(cls, value):
        x, y, z = value
        return cls.vec3_type(cls.scalar(x), cls.scalar(y), cls.scalar(z))

    @classmethod
    def matrix4(cls, values: Sequence):
        if len(values) != 16:
            raise ValueError("matrix4 expects 16 row-major values")
        return cls.matrix4_type((cls.scalar_type * 16)(*values))


class F32Family(Family):
    prefix = "f32"
    scalar_type = ctypes.c_float
    vec2_type = Vec2F32
    vec3_type = Vec3F32
    matrix4_type = Matrix4F32
    aabb_type = Aabb3F32
    mesh_buffers_type = MeshBuffersF32
    graphics_mesh_type = GraphicsMeshF32
    region_profiles_type = RegionProfilesF32


class F64Family(Family):
    prefix = "f64"
    scalar_type = ctypes.c_double
    vec2_type = Vec2F64
    vec3_type = Vec3F64
    matrix4_type = Matrix4F64
    aabb_type = Aabb3F64
    mesh_buffers_type = MeshBuffersF64
    graphics_mesh_type = GraphicsMeshF64
    region_profiles_type = RegionProfilesF64


class I128Family(Family):
    prefix = "i128"
    scalar_type = I128
    vec2_type = Vec2I128
    vec3_type = Vec3I128
    matrix4_type = Matrix4I128
    aabb_type = Aabb3I128
    mesh_buffers_type = MeshBuffersI128
    graphics_mesh_type = GraphicsMeshI128
    region_profiles_type = RegionProfilesI128


class RealFamily(Family):
    prefix = "real"
    scalar_type = RealPtr
    vec2_type = Vec2Real
    vec3_type = Vec3Real
    matrix4_type = Matrix4Real
    aabb_type = Aabb3Real
    mesh_buffers_type = MeshBuffersReal
    graphics_mesh_type = GraphicsMeshReal
    region_profiles_type = RegionProfilesReal


class Mesh:
    family: type[Family]

    def __init__(self, ptr: MeshPtr):
        self.ptr = ptr

    @classmethod
    def cube(cls, width):
        out = MeshPtr()
        _call(f"csgrs_mesh_{cls.family.prefix}_cube", cls.family.scalar(width), ctypes.byref(out))
        return cls(out)

    @classmethod
    def cuboid(cls, width, length, height):
        out = MeshPtr()
        _call(
            f"csgrs_mesh_{cls.family.prefix}_cuboid",
            cls.family.scalar(width),
            cls.family.scalar(length),
            cls.family.scalar(height),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def sphere(cls, radius, segments: int, stacks: int):
        out = MeshPtr()
        _call(
            f"csgrs_mesh_{cls.family.prefix}_sphere",
            cls.family.scalar(radius),
            ctypes.c_size_t(segments),
            ctypes.c_size_t(stacks),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def cylinder(cls, radius, height, segments: int):
        out = MeshPtr()
        _call(
            f"csgrs_mesh_{cls.family.prefix}_cylinder",
            cls.family.scalar(radius),
            cls.family.scalar(height),
            ctypes.c_size_t(segments),
            ctypes.byref(out),
        )
        return cls(out)

    @classmethod
    def polyhedron(cls, points: Iterable[Sequence], faces: Sequence[Sequence[int]]):
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
            f"csgrs_mesh_{cls.family.prefix}_polyhedron",
            point_array,
            len(point_values),
            index_array,
            len(face_indices),
            offset_array,
            len(offsets),
            ctypes.byref(out),
        )
        return cls(out)

    def _binary(self, other: "Mesh", op: str):
        out = MeshPtr()
        _call(f"csgrs_mesh_{self.family.prefix}_{op}", self.ptr, other.ptr, ctypes.byref(out))
        return self.__class__(out)

    def union(self, other): return self._binary(other, "union")
    def difference(self, other): return self._binary(other, "difference")
    def intersection(self, other): return self._binary(other, "intersection")
    def xor(self, other): return self._binary(other, "xor")

    def transform(self, matrix):
        out = MeshPtr()
        _call(f"csgrs_mesh_{self.family.prefix}_transform", self.ptr, matrix, ctypes.byref(out))
        return self.__class__(out)

    def translate(self, x, y, z): return self._triple("translate", x, y, z)
    def scale(self, sx, sy, sz): return self._triple("scale", sx, sy, sz)
    def rotate(self, x, y, z): return self._triple("rotate", x, y, z)

    def _triple(self, op, x, y, z):
        out = MeshPtr()
        _call(
            f"csgrs_mesh_{self.family.prefix}_{op}",
            self.ptr,
            self.family.scalar(x),
            self.family.scalar(y),
            self.family.scalar(z),
            ctypes.byref(out),
        )
        return self.__class__(out)

    def inverse(self): return self._unary("inverse")
    def center(self): return self._unary("center")
    def floating(self): return self._unary("float")

    def _unary(self, op):
        out = MeshPtr()
        _call(f"csgrs_mesh_{self.family.prefix}_{op}", self.ptr, ctypes.byref(out))
        return self.__class__(out)

    def bounding_box(self):
        out = self.family.aabb_type()
        _call(f"csgrs_mesh_{self.family.prefix}_bounding_box", self.ptr, ctypes.byref(out))
        return out

    def vertices_and_indices(self):
        out = self.family.mesh_buffers_type()
        _call(f"csgrs_mesh_{self.family.prefix}_vertices_and_indices", self.ptr, ctypes.byref(out))
        return MeshBuffers(self.family, out)

    def graphics_mesh(self):
        out = self.family.graphics_mesh_type()
        _call(f"csgrs_mesh_{self.family.prefix}_graphics_mesh", self.ptr, ctypes.byref(out))
        return GraphicsMesh(self.family, out)

    def __del__(self):
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_mesh_free(ptr)
            self.ptr = MeshPtr()


class Profile:
    family: type[Family]

    def __init__(self, ptr: ProfilePtr):
        self.ptr = ptr

    @classmethod
    def square(cls, width):
        out = ProfilePtr()
        _call(f"csgrs_profile_{cls.family.prefix}_square", cls.family.scalar(width), ctypes.byref(out))
        return cls(out)

    @classmethod
    def rectangle(cls, width, length):
        out = ProfilePtr()
        _call(f"csgrs_profile_{cls.family.prefix}_rectangle", cls.family.scalar(width), cls.family.scalar(length), ctypes.byref(out))
        return cls(out)

    @classmethod
    def circle(cls, radius, segments: int):
        out = ProfilePtr()
        _call(f"csgrs_profile_{cls.family.prefix}_circle", cls.family.scalar(radius), ctypes.c_size_t(segments), ctypes.byref(out))
        return cls(out)

    @classmethod
    def polygon(cls, points: Iterable[Sequence]):
        values = [cls.family.vec2(point) for point in points]
        array = (cls.family.vec2_type * len(values))(*values)
        out = ProfilePtr()
        _call(f"csgrs_profile_{cls.family.prefix}_polygon", array, len(values), ctypes.byref(out))
        return cls(out)

    def _binary(self, other: "Profile", op: str):
        out = ProfilePtr()
        _call(f"csgrs_profile_{self.family.prefix}_{op}", self.ptr, other.ptr, ctypes.byref(out))
        return self.__class__(out)

    def union(self, other): return self._binary(other, "union")
    def difference(self, other): return self._binary(other, "difference")
    def intersection(self, other): return self._binary(other, "intersection")
    def xor(self, other): return self._binary(other, "xor")

    def transform(self, matrix):
        out = ProfilePtr()
        _call(f"csgrs_profile_{self.family.prefix}_transform", self.ptr, matrix, ctypes.byref(out))
        return self.__class__(out)

    def translate(self, x, y, z): return self._triple("translate", x, y, z)
    def scale(self, sx, sy, sz): return self._triple("scale", sx, sy, sz)
    def rotate(self, x, y, z): return self._triple("rotate", x, y, z)

    def _triple(self, op, x, y, z):
        out = ProfilePtr()
        _call(f"csgrs_profile_{self.family.prefix}_{op}", self.ptr, self.family.scalar(x), self.family.scalar(y), self.family.scalar(z), ctypes.byref(out))
        return self.__class__(out)

    def bounding_box(self):
        out = self.family.aabb_type()
        _call(f"csgrs_profile_{self.family.prefix}_bounding_box", self.ptr, ctypes.byref(out))
        return out

    def extrude(self, height):
        out = MeshPtr()
        _call(f"csgrs_profile_{self.family.prefix}_extrude", self.ptr, self.family.scalar(height), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def extrude_vector(self, direction):
        out = MeshPtr()
        _call(f"csgrs_profile_{self.family.prefix}_extrude_vector", self.ptr, self.family.vec3(direction), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def revolve(self, angle, segments: int):
        out = MeshPtr()
        _call(f"csgrs_profile_{self.family.prefix}_revolve", self.ptr, self.family.scalar(angle), ctypes.c_size_t(segments), ctypes.byref(out))
        return mesh_class_for(self.family)(out)

    def region_profiles(self):
        out = self.family.region_profiles_type()
        _call(f"csgrs_profile_{self.family.prefix}_region_profiles", self.ptr, ctypes.byref(out))
        return RegionProfiles(self.family, out)

    def __del__(self):
        ptr = getattr(self, "ptr", None)
        if ptr:
            load().csgrs_profile_free(ptr)
            self.ptr = ProfilePtr()


class MeshF32(Mesh): family = F32Family
class MeshF64(Mesh): family = F64Family
class MeshI128(Mesh): family = I128Family
class MeshReal(Mesh): family = RealFamily
class ProfileF32(Profile): family = F32Family
class ProfileF64(Profile): family = F64Family
class ProfileI128(Profile): family = I128Family
class ProfileReal(Profile): family = RealFamily


def mesh_class_for(family):
    return {F32Family: MeshF32, F64Family: MeshF64, I128Family: MeshI128, RealFamily: MeshReal}[family]


class MeshBuffers:
    def __init__(self, family: type[Family], raw):
        self.family = family
        self.raw = raw

    def __del__(self):
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_mesh_buffers_{self.family.prefix}_free")(raw)
            self.raw = None


class GraphicsMesh:
    def __init__(self, family: type[Family], raw):
        self.family = family
        self.raw = raw

    def __del__(self):
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_graphics_mesh_{self.family.prefix}_free")(raw)
            self.raw = None


class RegionProfiles:
    def __init__(self, family: type[Family], raw):
        self.family = family
        self.raw = raw

    def __del__(self):
        raw = getattr(self, "raw", None)
        if raw is not None:
            getattr(load(), f"csgrs_region_profiles_{self.family.prefix}_free")(raw)
            self.raw = None
