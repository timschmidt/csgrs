package csgrs

/*
#cgo CFLAGS: -I../../ffi/include
#include "csgrs.h"
*/
import "C"

import (
	"errors"
	"unsafe"
)

type Family int

const (
	FamilyF32  Family = Family(C.CSGRS_SCALAR_F32)
	FamilyF64  Family = Family(C.CSGRS_SCALAR_F64)
	FamilyI128 Family = Family(C.CSGRS_SCALAR_I128)
	FamilyReal Family = Family(C.CSGRS_SCALAR_REAL)
)

type I128 = C.csgrs_i128_t
type Vec2F32 = C.csgrs_vec2_f32_t
type Vec3F32 = C.csgrs_vec3_f32_t
type Vec2F64 = C.csgrs_vec2_f64_t
type Vec3F64 = C.csgrs_vec3_f64_t
type Vec2I128 = C.csgrs_vec2_i128_t
type Vec3I128 = C.csgrs_vec3_i128_t
type Vec2Real = C.csgrs_vec2_real_t
type Vec3Real = C.csgrs_vec3_real_t
type Matrix4F32 = C.csgrs_matrix4_f32_t
type Matrix4F64 = C.csgrs_matrix4_f64_t
type Matrix4I128 = C.csgrs_matrix4_i128_t
type Matrix4Real = C.csgrs_matrix4_real_t
type Aabb3F32 = C.csgrs_aabb3_f32_t
type Aabb3F64 = C.csgrs_aabb3_f64_t
type Aabb3I128 = C.csgrs_aabb3_i128_t
type Aabb3Real = C.csgrs_aabb3_real_t
type MeshBuffersF32 = C.csgrs_mesh_buffers_f32_t
type MeshBuffersF64 = C.csgrs_mesh_buffers_f64_t
type MeshBuffersI128 = C.csgrs_mesh_buffers_i128_t
type MeshBuffersReal = C.csgrs_mesh_buffers_real_t
type GraphicsMeshF32 = C.csgrs_graphics_mesh_f32_t
type GraphicsMeshF64 = C.csgrs_graphics_mesh_f64_t
type GraphicsMeshI128 = C.csgrs_graphics_mesh_i128_t
type GraphicsMeshReal = C.csgrs_graphics_mesh_real_t
type RegionProfilesF32 = C.csgrs_region_profiles_f32_t
type RegionProfilesF64 = C.csgrs_region_profiles_f64_t
type RegionProfilesI128 = C.csgrs_region_profiles_i128_t
type RegionProfilesReal = C.csgrs_region_profiles_real_t

type Real struct{ ptr *C.csgrs_real_t }
type Mesh struct {
	ptr    *C.csgrs_mesh_t
	Family Family
}
type Profile struct {
	ptr    *C.csgrs_profile_t
	Family Family
}

func check(status C.csgrs_status_t) error {
	if status == C.CSGRS_STATUS_OK {
		return nil
	}
	msg := C.csgrs_last_error_message()
	if msg == nil {
		return errors.New("csgrs error")
	}
	return errors.New(C.GoString(msg))
}

func I128FromInt64(value int64) I128 {
	hi := int64(0)
	if value < 0 {
		hi = -1
	}
	return I128{hi: C.int64_t(hi), lo: C.uint64_t(uint64(value))}
}

func RealZero() (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_zero(&out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

func RealOne() (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_one(&out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

func RealFromF64(value float64) (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_from_f64(C.double(value), &out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

func RealFromI128(value I128) (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_from_i128(value, &out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

func (r *Real) Free() {
	if r != nil && r.ptr != nil {
		C.csgrs_real_free(r.ptr)
		r.ptr = nil
	}
}

func (r *Real) ToF64() (float64, error) {
	var out C.double
	if err := check(C.csgrs_real_to_f64(r.ptr, &out)); err != nil {
		return 0, err
	}
	return float64(out), nil
}

func MeshF32Cube(width float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_cube(C.float(width), &out)); return meshOut(FamilyF32, out, err) }
func MeshF64Cube(width float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_cube(C.double(width), &out)); return meshOut(FamilyF64, out, err) }
func MeshI128Cube(width I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_cube(width, &out)); return meshOut(FamilyI128, out, err) }
func MeshRealCube(width *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_cube(width.ptr, &out)); return meshOut(FamilyReal, out, err) }

func MeshF32Cuboid(width, length, height float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_cuboid(C.float(width), C.float(length), C.float(height), &out)); return meshOut(FamilyF32, out, err) }
func MeshF64Cuboid(width, length, height float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_cuboid(C.double(width), C.double(length), C.double(height), &out)); return meshOut(FamilyF64, out, err) }
func MeshI128Cuboid(width, length, height I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_cuboid(width, length, height, &out)); return meshOut(FamilyI128, out, err) }
func MeshRealCuboid(width, length, height *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_cuboid(width.ptr, length.ptr, height.ptr, &out)); return meshOut(FamilyReal, out, err) }

func MeshF32Sphere(radius float32, segments, stacks int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_sphere(C.float(radius), C.size_t(segments), C.size_t(stacks), &out)); return meshOut(FamilyF32, out, err) }
func MeshF64Sphere(radius float64, segments, stacks int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_sphere(C.double(radius), C.size_t(segments), C.size_t(stacks), &out)); return meshOut(FamilyF64, out, err) }
func MeshI128Sphere(radius I128, segments, stacks int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_sphere(radius, C.size_t(segments), C.size_t(stacks), &out)); return meshOut(FamilyI128, out, err) }
func MeshRealSphere(radius *Real, segments, stacks int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_sphere(radius.ptr, C.size_t(segments), C.size_t(stacks), &out)); return meshOut(FamilyReal, out, err) }

func MeshF32Cylinder(radius, height float32, segments int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_cylinder(C.float(radius), C.float(height), C.size_t(segments), &out)); return meshOut(FamilyF32, out, err) }
func MeshF64Cylinder(radius, height float64, segments int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_cylinder(C.double(radius), C.double(height), C.size_t(segments), &out)); return meshOut(FamilyF64, out, err) }
func MeshI128Cylinder(radius, height I128, segments int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_cylinder(radius, height, C.size_t(segments), &out)); return meshOut(FamilyI128, out, err) }
func MeshRealCylinder(radius, height *Real, segments int) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_cylinder(radius.ptr, height.ptr, C.size_t(segments), &out)); return meshOut(FamilyReal, out, err) }

func meshOut(f Family, out *C.csgrs_mesh_t, err error) (*Mesh, error) {
	if err != nil {
		return nil, err
	}
	return &Mesh{ptr: out, Family: f}, nil
}

func profileOut(f Family, out *C.csgrs_profile_t, err error) (*Profile, error) {
	if err != nil {
		return nil, err
	}
	return &Profile{ptr: out, Family: f}, nil
}

func (m *Mesh) Free() {
	if m != nil && m.ptr != nil {
		C.csgrs_mesh_free(m.ptr)
		m.ptr = nil
	}
}

func (p *Profile) Free() {
	if p != nil && p.ptr != nil {
		C.csgrs_profile_free(p.ptr)
		p.ptr = nil
	}
}

func (m *Mesh) binary(other *Mesh, op string) (*Mesh, error) {
	var out *C.csgrs_mesh_t
	var status C.csgrs_status_t
	switch op {
	case "union":
		status = meshUnion(m, other, &out)
	case "difference":
		status = meshDifference(m, other, &out)
	case "intersection":
		status = meshIntersection(m, other, &out)
	case "xor":
		status = meshXor(m, other, &out)
	default:
		return nil, errors.New("unknown mesh op")
	}
	return meshOut(m.Family, out, check(status))
}

func (m *Mesh) Union(other *Mesh) (*Mesh, error) { return m.binary(other, "union") }
func (m *Mesh) Difference(other *Mesh) (*Mesh, error) { return m.binary(other, "difference") }
func (m *Mesh) Intersection(other *Mesh) (*Mesh, error) { return m.binary(other, "intersection") }
func (m *Mesh) Xor(other *Mesh) (*Mesh, error) { return m.binary(other, "xor") }

func meshUnion(m, o *Mesh, out **C.csgrs_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_mesh_f32_union(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_mesh_f64_union(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_mesh_i128_union(m.ptr, o.ptr, out)
	default: return C.csgrs_mesh_real_union(m.ptr, o.ptr, out)
	}
}

func meshDifference(m, o *Mesh, out **C.csgrs_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_mesh_f32_difference(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_mesh_f64_difference(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_mesh_i128_difference(m.ptr, o.ptr, out)
	default: return C.csgrs_mesh_real_difference(m.ptr, o.ptr, out)
	}
}

func meshIntersection(m, o *Mesh, out **C.csgrs_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_mesh_f32_intersection(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_mesh_f64_intersection(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_mesh_i128_intersection(m.ptr, o.ptr, out)
	default: return C.csgrs_mesh_real_intersection(m.ptr, o.ptr, out)
	}
}

func meshXor(m, o *Mesh, out **C.csgrs_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_mesh_f32_xor(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_mesh_f64_xor(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_mesh_i128_xor(m.ptr, o.ptr, out)
	default: return C.csgrs_mesh_real_xor(m.ptr, o.ptr, out)
	}
}

func (m *Mesh) TranslateF64(x, y, z float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_translate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return meshOut(FamilyF64, out, err) }
func (m *Mesh) ScaleF64(x, y, z float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_scale(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return meshOut(FamilyF64, out, err) }
func (m *Mesh) RotateF64(x, y, z float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f64_rotate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return meshOut(FamilyF64, out, err) }
func (m *Mesh) TranslateF32(x, y, z float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_translate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return meshOut(FamilyF32, out, err) }
func (m *Mesh) ScaleF32(x, y, z float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_scale(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return meshOut(FamilyF32, out, err) }
func (m *Mesh) RotateF32(x, y, z float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_f32_rotate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return meshOut(FamilyF32, out, err) }
func (m *Mesh) TranslateI128(x, y, z I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_translate(m.ptr, x, y, z, &out)); return meshOut(FamilyI128, out, err) }
func (m *Mesh) ScaleI128(x, y, z I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_scale(m.ptr, x, y, z, &out)); return meshOut(FamilyI128, out, err) }
func (m *Mesh) RotateI128(x, y, z I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_i128_rotate(m.ptr, x, y, z, &out)); return meshOut(FamilyI128, out, err) }
func (m *Mesh) TranslateReal(x, y, z *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_translate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return meshOut(FamilyReal, out, err) }
func (m *Mesh) ScaleReal(x, y, z *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_scale(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return meshOut(FamilyReal, out, err) }
func (m *Mesh) RotateReal(x, y, z *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_mesh_real_rotate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return meshOut(FamilyReal, out, err) }
func (m *Mesh) BoundingBoxF64() (Aabb3F64, error) { var out Aabb3F64; err := check(C.csgrs_mesh_f64_bounding_box(m.ptr, &out)); return out, err }
func (m *Mesh) BoundingBoxF32() (Aabb3F32, error) { var out Aabb3F32; err := check(C.csgrs_mesh_f32_bounding_box(m.ptr, &out)); return out, err }
func (m *Mesh) BoundingBoxI128() (Aabb3I128, error) { var out Aabb3I128; err := check(C.csgrs_mesh_i128_bounding_box(m.ptr, &out)); return out, err }
func (m *Mesh) BoundingBoxReal() (Aabb3Real, error) { var out Aabb3Real; err := check(C.csgrs_mesh_real_bounding_box(m.ptr, &out)); return out, err }
func (m *Mesh) VerticesAndIndicesF64() (MeshBuffersF64, error) { var out MeshBuffersF64; err := check(C.csgrs_mesh_f64_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *Mesh) VerticesAndIndicesF32() (MeshBuffersF32, error) { var out MeshBuffersF32; err := check(C.csgrs_mesh_f32_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *Mesh) VerticesAndIndicesI128() (MeshBuffersI128, error) { var out MeshBuffersI128; err := check(C.csgrs_mesh_i128_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *Mesh) VerticesAndIndicesReal() (MeshBuffersReal, error) { var out MeshBuffersReal; err := check(C.csgrs_mesh_real_vertices_and_indices(m.ptr, &out)); return out, err }
func FreeMeshBuffersF64(value MeshBuffersF64) { C.csgrs_mesh_buffers_f64_free(value) }
func FreeMeshBuffersF32(value MeshBuffersF32) { C.csgrs_mesh_buffers_f32_free(value) }
func FreeMeshBuffersI128(value MeshBuffersI128) { C.csgrs_mesh_buffers_i128_free(value) }
func FreeMeshBuffersReal(value MeshBuffersReal) { C.csgrs_mesh_buffers_real_free(value) }

func (m *Mesh) Inverse() (*Mesh, error) { return m.meshUnary("inverse") }
func (m *Mesh) Center() (*Mesh, error) { return m.meshUnary("center") }
func (m *Mesh) Float() (*Mesh, error) { return m.meshUnary("float") }

func (m *Mesh) meshUnary(op string) (*Mesh, error) {
	var out *C.csgrs_mesh_t
	var status C.csgrs_status_t
	switch op {
	case "inverse":
		switch m.Family { case FamilyF32: status = C.csgrs_mesh_f32_inverse(m.ptr, &out); case FamilyF64: status = C.csgrs_mesh_f64_inverse(m.ptr, &out); case FamilyI128: status = C.csgrs_mesh_i128_inverse(m.ptr, &out); default: status = C.csgrs_mesh_real_inverse(m.ptr, &out) }
	case "center":
		switch m.Family { case FamilyF32: status = C.csgrs_mesh_f32_center(m.ptr, &out); case FamilyF64: status = C.csgrs_mesh_f64_center(m.ptr, &out); case FamilyI128: status = C.csgrs_mesh_i128_center(m.ptr, &out); default: status = C.csgrs_mesh_real_center(m.ptr, &out) }
	case "float":
		switch m.Family { case FamilyF32: status = C.csgrs_mesh_f32_float(m.ptr, &out); case FamilyF64: status = C.csgrs_mesh_f64_float(m.ptr, &out); case FamilyI128: status = C.csgrs_mesh_i128_float(m.ptr, &out); default: status = C.csgrs_mesh_real_float(m.ptr, &out) }
	default:
		return nil, errors.New("unknown mesh unary op")
	}
	return meshOut(m.Family, out, check(status))
}

func ProfileF32Square(width float32) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f32_square(C.float(width), &out)); return profileOut(FamilyF32, out, err) }
func ProfileF64Square(width float64) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f64_square(C.double(width), &out)); return profileOut(FamilyF64, out, err) }
func ProfileI128Square(width I128) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_i128_square(width, &out)); return profileOut(FamilyI128, out, err) }
func ProfileRealSquare(width *Real) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_real_square(width.ptr, &out)); return profileOut(FamilyReal, out, err) }

func ProfileF32Rectangle(width, length float32) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f32_rectangle(C.float(width), C.float(length), &out)); return profileOut(FamilyF32, out, err) }
func ProfileF64Rectangle(width, length float64) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f64_rectangle(C.double(width), C.double(length), &out)); return profileOut(FamilyF64, out, err) }
func ProfileI128Rectangle(width, length I128) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_i128_rectangle(width, length, &out)); return profileOut(FamilyI128, out, err) }
func ProfileRealRectangle(width, length *Real) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_real_rectangle(width.ptr, length.ptr, &out)); return profileOut(FamilyReal, out, err) }

func ProfileF32Circle(radius float32, segments int) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f32_circle(C.float(radius), C.size_t(segments), &out)); return profileOut(FamilyF32, out, err) }
func ProfileF64Circle(radius float64, segments int) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f64_circle(C.double(radius), C.size_t(segments), &out)); return profileOut(FamilyF64, out, err) }
func ProfileI128Circle(radius I128, segments int) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_i128_circle(radius, C.size_t(segments), &out)); return profileOut(FamilyI128, out, err) }
func ProfileRealCircle(radius *Real, segments int) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_real_circle(radius.ptr, C.size_t(segments), &out)); return profileOut(FamilyReal, out, err) }

func (p *Profile) ExtrudeF64(height float64) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_profile_f64_extrude(p.ptr, C.double(height), &out)); return meshOut(FamilyF64, out, err) }
func (p *Profile) TranslateF64(x, y, z float64) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f64_translate(p.ptr, C.double(x), C.double(y), C.double(z), &out)); return profileOut(FamilyF64, out, err) }
func (p *Profile) BoundingBoxF64() (Aabb3F64, error) { var out Aabb3F64; err := check(C.csgrs_profile_f64_bounding_box(p.ptr, &out)); return out, err }
func (p *Profile) ExtrudeF32(height float32) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_profile_f32_extrude(p.ptr, C.float(height), &out)); return meshOut(FamilyF32, out, err) }
func (p *Profile) ExtrudeI128(height I128) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_profile_i128_extrude(p.ptr, height, &out)); return meshOut(FamilyI128, out, err) }
func (p *Profile) ExtrudeReal(height *Real) (*Mesh, error) { var out *C.csgrs_mesh_t; err := check(C.csgrs_profile_real_extrude(p.ptr, height.ptr, &out)); return meshOut(FamilyReal, out, err) }
func (p *Profile) TranslateF32(x, y, z float32) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_f32_translate(p.ptr, C.float(x), C.float(y), C.float(z), &out)); return profileOut(FamilyF32, out, err) }
func (p *Profile) TranslateI128(x, y, z I128) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_i128_translate(p.ptr, x, y, z, &out)); return profileOut(FamilyI128, out, err) }
func (p *Profile) TranslateReal(x, y, z *Real) (*Profile, error) { var out *C.csgrs_profile_t; err := check(C.csgrs_profile_real_translate(p.ptr, x.ptr, y.ptr, z.ptr, &out)); return profileOut(FamilyReal, out, err) }
func (p *Profile) BoundingBoxF32() (Aabb3F32, error) { var out Aabb3F32; err := check(C.csgrs_profile_f32_bounding_box(p.ptr, &out)); return out, err }
func (p *Profile) BoundingBoxI128() (Aabb3I128, error) { var out Aabb3I128; err := check(C.csgrs_profile_i128_bounding_box(p.ptr, &out)); return out, err }
func (p *Profile) BoundingBoxReal() (Aabb3Real, error) { var out Aabb3Real; err := check(C.csgrs_profile_real_bounding_box(p.ptr, &out)); return out, err }

func (p *Profile) Union(other *Profile) (*Profile, error) { return p.profileBinary(other, "union") }
func (p *Profile) Difference(other *Profile) (*Profile, error) { return p.profileBinary(other, "difference") }
func (p *Profile) Intersection(other *Profile) (*Profile, error) { return p.profileBinary(other, "intersection") }
func (p *Profile) Xor(other *Profile) (*Profile, error) { return p.profileBinary(other, "xor") }

func (p *Profile) profileBinary(other *Profile, op string) (*Profile, error) {
	var out *C.csgrs_profile_t
	var status C.csgrs_status_t
	switch op {
	case "union":
		switch p.Family { case FamilyF32: status = C.csgrs_profile_f32_union(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_profile_f64_union(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_profile_i128_union(p.ptr, other.ptr, &out); default: status = C.csgrs_profile_real_union(p.ptr, other.ptr, &out) }
	case "difference":
		switch p.Family { case FamilyF32: status = C.csgrs_profile_f32_difference(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_profile_f64_difference(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_profile_i128_difference(p.ptr, other.ptr, &out); default: status = C.csgrs_profile_real_difference(p.ptr, other.ptr, &out) }
	case "intersection":
		switch p.Family { case FamilyF32: status = C.csgrs_profile_f32_intersection(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_profile_f64_intersection(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_profile_i128_intersection(p.ptr, other.ptr, &out); default: status = C.csgrs_profile_real_intersection(p.ptr, other.ptr, &out) }
	case "xor":
		switch p.Family { case FamilyF32: status = C.csgrs_profile_f32_xor(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_profile_f64_xor(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_profile_i128_xor(p.ptr, other.ptr, &out); default: status = C.csgrs_profile_real_xor(p.ptr, other.ptr, &out) }
	default:
		return nil, errors.New("unknown profile op")
	}
	return profileOut(p.Family, out, check(status))
}

func MeshF64Polyhedron(points []Vec3F64, faceIndices []uint, faceOffsets []uint) (*Mesh, error) {
	var out *C.csgrs_mesh_t
	indices := make([]C.size_t, len(faceIndices))
	offsets := make([]C.size_t, len(faceOffsets))
	for i, v := range faceIndices { indices[i] = C.size_t(v) }
	for i, v := range faceOffsets { offsets[i] = C.size_t(v) }
	var pptr *C.csgrs_vec3_f64_t
	if len(points) > 0 { pptr = (*C.csgrs_vec3_f64_t)(unsafe.Pointer(&points[0])) }
	err := check(C.csgrs_mesh_f64_polyhedron(pptr, C.size_t(len(points)), ptrSizeT(indices), C.size_t(len(indices)), ptrSizeT(offsets), C.size_t(len(offsets)), &out))
	return meshOut(FamilyF64, out, err)
}

func ProfileF64Polygon(points []Vec2F64) (*Profile, error) {
	var out *C.csgrs_profile_t
	var pptr *C.csgrs_vec2_f64_t
	if len(points) > 0 { pptr = (*C.csgrs_vec2_f64_t)(unsafe.Pointer(&points[0])) }
	err := check(C.csgrs_profile_f64_polygon(pptr, C.size_t(len(points)), &out))
	return profileOut(FamilyF64, out, err)
}

func ptrSizeT(values []C.size_t) *C.size_t {
	if len(values) == 0 { return nil }
	return &values[0]
}
