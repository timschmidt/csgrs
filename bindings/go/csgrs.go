package csgrs

/*
#cgo CFLAGS: -I../../../csgrs-ffi/include
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
type MeshBuffersF32 = C.csgrs_triangle_mesh_buffers_f32_t
type MeshBuffersF64 = C.csgrs_triangle_mesh_buffers_f64_t
type MeshBuffersI128 = C.csgrs_triangle_mesh_buffers_i128_t
type MeshBuffersReal = C.csgrs_triangle_mesh_buffers_real_t
type GraphicsMeshF32 = C.csgrs_graphics_mesh_f32_t
type GraphicsMeshF64 = C.csgrs_graphics_mesh_f64_t
type GraphicsMeshI128 = C.csgrs_graphics_mesh_i128_t
type GraphicsMeshReal = C.csgrs_graphics_mesh_real_t
type RegionProfilesF32 = C.csgrs_region_profiles_f32_t
type RegionProfilesF64 = C.csgrs_region_profiles_f64_t
type RegionProfilesI128 = C.csgrs_region_profiles_i128_t
type RegionProfilesReal = C.csgrs_region_profiles_real_t

type Real struct{ ptr *C.csgrs_real_t }
type TriangleMesh struct {
	ptr    *C.csgrs_triangle_mesh_t
	Family Family
}
type CurveRegion struct {
	ptr    *C.csgrs_curve_region_t
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

func TriangleMeshF32Cube(width float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cube(C.float(width), &out)); return triangleMeshOut(FamilyF32, out, err) }
func TriangleMeshF64Cube(width float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cube(C.double(width), &out)); return triangleMeshOut(FamilyF64, out, err) }
func TriangleMeshI128Cube(width I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cube(width, &out)); return triangleMeshOut(FamilyI128, out, err) }
func TriangleMeshRealCube(width *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cube(width.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }

func TriangleMeshF32Cuboid(width, length, height float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cuboid(C.float(width), C.float(length), C.float(height), &out)); return triangleMeshOut(FamilyF32, out, err) }
func TriangleMeshF64Cuboid(width, length, height float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cuboid(C.double(width), C.double(length), C.double(height), &out)); return triangleMeshOut(FamilyF64, out, err) }
func TriangleMeshI128Cuboid(width, length, height I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cuboid(width, length, height, &out)); return triangleMeshOut(FamilyI128, out, err) }
func TriangleMeshRealCuboid(width, length, height *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cuboid(width.ptr, length.ptr, height.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }

func TriangleMeshF32Sphere(radius float32, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_sphere(C.float(radius), C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyF32, out, err) }
func TriangleMeshF64Sphere(radius float64, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_sphere(C.double(radius), C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyF64, out, err) }
func TriangleMeshI128Sphere(radius I128, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_sphere(radius, C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyI128, out, err) }
func TriangleMeshRealSphere(radius *Real, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_sphere(radius.ptr, C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyReal, out, err) }

func TriangleMeshF32Cylinder(radius, height float32, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cylinder(C.float(radius), C.float(height), C.size_t(segments), &out)); return triangleMeshOut(FamilyF32, out, err) }
func TriangleMeshF64Cylinder(radius, height float64, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cylinder(C.double(radius), C.double(height), C.size_t(segments), &out)); return triangleMeshOut(FamilyF64, out, err) }
func TriangleMeshI128Cylinder(radius, height I128, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cylinder(radius, height, C.size_t(segments), &out)); return triangleMeshOut(FamilyI128, out, err) }
func TriangleMeshRealCylinder(radius, height *Real, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cylinder(radius.ptr, height.ptr, C.size_t(segments), &out)); return triangleMeshOut(FamilyReal, out, err) }

func triangleMeshOut(f Family, out *C.csgrs_triangle_mesh_t, err error) (*TriangleMesh, error) {
	if err != nil {
		return nil, err
	}
	return &TriangleMesh{ptr: out, Family: f}, nil
}

func curveRegionOut(f Family, out *C.csgrs_curve_region_t, err error) (*CurveRegion, error) {
	if err != nil {
		return nil, err
	}
	return &CurveRegion{ptr: out, Family: f}, nil
}

func (m *TriangleMesh) Free() {
	if m != nil && m.ptr != nil {
		C.csgrs_triangle_mesh_free(m.ptr)
		m.ptr = nil
	}
}

func (p *CurveRegion) Free() {
	if p != nil && p.ptr != nil {
		C.csgrs_curve_region_free(p.ptr)
		p.ptr = nil
	}
}

func (m *TriangleMesh) binary(other *TriangleMesh, op string) (*TriangleMesh, error) {
	var out *C.csgrs_triangle_mesh_t
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
	return triangleMeshOut(m.Family, out, check(status))
}

func (m *TriangleMesh) Union(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "union") }
func (m *TriangleMesh) Difference(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "difference") }
func (m *TriangleMesh) Intersection(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "intersection") }
func (m *TriangleMesh) Xor(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "xor") }

func meshUnion(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_union(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_union(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_union(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_union(m.ptr, o.ptr, out)
	}
}

func meshDifference(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_difference(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_difference(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_difference(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_difference(m.ptr, o.ptr, out)
	}
}

func meshIntersection(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_intersection(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_intersection(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_intersection(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_intersection(m.ptr, o.ptr, out)
	}
}

func meshXor(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_xor(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_xor(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_xor(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_xor(m.ptr, o.ptr, out)
	}
}

func (m *TriangleMesh) TranslateF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_translate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
func (m *TriangleMesh) ScaleF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_scale(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
func (m *TriangleMesh) RotateF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_rotate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
func (m *TriangleMesh) TranslateF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_translate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
func (m *TriangleMesh) ScaleF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_scale(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
func (m *TriangleMesh) RotateF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_rotate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
func (m *TriangleMesh) TranslateI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_translate(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
func (m *TriangleMesh) ScaleI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_scale(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
func (m *TriangleMesh) RotateI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_rotate(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
func (m *TriangleMesh) TranslateReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_translate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
func (m *TriangleMesh) ScaleReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_scale(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
func (m *TriangleMesh) RotateReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_rotate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
func (m *TriangleMesh) BoundingBoxF64() (Aabb3F64, error) { var out Aabb3F64; err := check(C.csgrs_triangle_mesh_f64_bounding_box(m.ptr, &out)); return out, err }
func (m *TriangleMesh) BoundingBoxF32() (Aabb3F32, error) { var out Aabb3F32; err := check(C.csgrs_triangle_mesh_f32_bounding_box(m.ptr, &out)); return out, err }
func (m *TriangleMesh) BoundingBoxI128() (Aabb3I128, error) { var out Aabb3I128; err := check(C.csgrs_triangle_mesh_i128_bounding_box(m.ptr, &out)); return out, err }
func (m *TriangleMesh) BoundingBoxReal() (Aabb3Real, error) { var out Aabb3Real; err := check(C.csgrs_triangle_mesh_real_bounding_box(m.ptr, &out)); return out, err }
func (m *TriangleMesh) VerticesAndIndicesF64() (MeshBuffersF64, error) { var out MeshBuffersF64; err := check(C.csgrs_triangle_mesh_f64_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *TriangleMesh) VerticesAndIndicesF32() (MeshBuffersF32, error) { var out MeshBuffersF32; err := check(C.csgrs_triangle_mesh_f32_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *TriangleMesh) VerticesAndIndicesI128() (MeshBuffersI128, error) { var out MeshBuffersI128; err := check(C.csgrs_triangle_mesh_i128_vertices_and_indices(m.ptr, &out)); return out, err }
func (m *TriangleMesh) VerticesAndIndicesReal() (MeshBuffersReal, error) { var out MeshBuffersReal; err := check(C.csgrs_triangle_mesh_real_vertices_and_indices(m.ptr, &out)); return out, err }
func FreeMeshBuffersF64(value MeshBuffersF64) { C.csgrs_triangle_mesh_buffers_f64_free(value) }
func FreeMeshBuffersF32(value MeshBuffersF32) { C.csgrs_triangle_mesh_buffers_f32_free(value) }
func FreeMeshBuffersI128(value MeshBuffersI128) { C.csgrs_triangle_mesh_buffers_i128_free(value) }
func FreeMeshBuffersReal(value MeshBuffersReal) { C.csgrs_triangle_mesh_buffers_real_free(value) }

func (m *TriangleMesh) Inverse() (*TriangleMesh, error) { return m.meshUnary("inverse") }
func (m *TriangleMesh) Center() (*TriangleMesh, error) { return m.meshUnary("center") }
func (m *TriangleMesh) Float() (*TriangleMesh, error) { return m.meshUnary("float") }

func (m *TriangleMesh) meshUnary(op string) (*TriangleMesh, error) {
	var out *C.csgrs_triangle_mesh_t
	var status C.csgrs_status_t
	switch op {
	case "inverse":
		switch m.Family { case FamilyF32: status = C.csgrs_triangle_mesh_f32_inverse(m.ptr, &out); case FamilyF64: status = C.csgrs_triangle_mesh_f64_inverse(m.ptr, &out); case FamilyI128: status = C.csgrs_triangle_mesh_i128_inverse(m.ptr, &out); default: status = C.csgrs_triangle_mesh_real_inverse(m.ptr, &out) }
	case "center":
		switch m.Family { case FamilyF32: status = C.csgrs_triangle_mesh_f32_center(m.ptr, &out); case FamilyF64: status = C.csgrs_triangle_mesh_f64_center(m.ptr, &out); case FamilyI128: status = C.csgrs_triangle_mesh_i128_center(m.ptr, &out); default: status = C.csgrs_triangle_mesh_real_center(m.ptr, &out) }
	case "float":
		switch m.Family { case FamilyF32: status = C.csgrs_triangle_mesh_f32_float(m.ptr, &out); case FamilyF64: status = C.csgrs_triangle_mesh_f64_float(m.ptr, &out); case FamilyI128: status = C.csgrs_triangle_mesh_i128_float(m.ptr, &out); default: status = C.csgrs_triangle_mesh_real_float(m.ptr, &out) }
	default:
		return nil, errors.New("unknown mesh unary op")
	}
	return triangleMeshOut(m.Family, out, check(status))
}

func CurveRegionF32Square(width float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_square(C.float(width), &out)); return curveRegionOut(FamilyF32, out, err) }
func CurveRegionF64Square(width float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_square(C.double(width), &out)); return curveRegionOut(FamilyF64, out, err) }
func CurveRegionI128Square(width I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_square(width, &out)); return curveRegionOut(FamilyI128, out, err) }
func CurveRegionRealSquare(width *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_square(width.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }

func CurveRegionF32Rectangle(width, length float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_rectangle(C.float(width), C.float(length), &out)); return curveRegionOut(FamilyF32, out, err) }
func CurveRegionF64Rectangle(width, length float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_rectangle(C.double(width), C.double(length), &out)); return curveRegionOut(FamilyF64, out, err) }
func CurveRegionI128Rectangle(width, length I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_rectangle(width, length, &out)); return curveRegionOut(FamilyI128, out, err) }
func CurveRegionRealRectangle(width, length *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_rectangle(width.ptr, length.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }

func CurveRegionF32Circle(radius float32, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_circle(C.float(radius), C.size_t(segments), &out)); return curveRegionOut(FamilyF32, out, err) }
func CurveRegionF64Circle(radius float64, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_circle(C.double(radius), C.size_t(segments), &out)); return curveRegionOut(FamilyF64, out, err) }
func CurveRegionI128Circle(radius I128, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_circle(radius, C.size_t(segments), &out)); return curveRegionOut(FamilyI128, out, err) }
func CurveRegionRealCircle(radius *Real, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_circle(radius.ptr, C.size_t(segments), &out)); return curveRegionOut(FamilyReal, out, err) }

func (p *CurveRegion) ExtrudeF64(height float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_f64_extrude(p.ptr, C.double(height), &out)); return triangleMeshOut(FamilyF64, out, err) }
func (p *CurveRegion) TranslateF64(x, y, z float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_translate(p.ptr, C.double(x), C.double(y), C.double(z), &out)); return curveRegionOut(FamilyF64, out, err) }
func (p *CurveRegion) BoundingBoxF64() (Aabb3F64, error) { var out Aabb3F64; err := check(C.csgrs_curve_region_f64_bounding_box(p.ptr, &out)); return out, err }
func (p *CurveRegion) ExtrudeF32(height float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_f32_extrude(p.ptr, C.float(height), &out)); return triangleMeshOut(FamilyF32, out, err) }
func (p *CurveRegion) ExtrudeI128(height I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_i128_extrude(p.ptr, height, &out)); return triangleMeshOut(FamilyI128, out, err) }
func (p *CurveRegion) ExtrudeReal(height *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_real_extrude(p.ptr, height.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
func (p *CurveRegion) TranslateF32(x, y, z float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_translate(p.ptr, C.float(x), C.float(y), C.float(z), &out)); return curveRegionOut(FamilyF32, out, err) }
func (p *CurveRegion) TranslateI128(x, y, z I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_translate(p.ptr, x, y, z, &out)); return curveRegionOut(FamilyI128, out, err) }
func (p *CurveRegion) TranslateReal(x, y, z *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_translate(p.ptr, x.ptr, y.ptr, z.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }
func (p *CurveRegion) BoundingBoxF32() (Aabb3F32, error) { var out Aabb3F32; err := check(C.csgrs_curve_region_f32_bounding_box(p.ptr, &out)); return out, err }
func (p *CurveRegion) BoundingBoxI128() (Aabb3I128, error) { var out Aabb3I128; err := check(C.csgrs_curve_region_i128_bounding_box(p.ptr, &out)); return out, err }
func (p *CurveRegion) BoundingBoxReal() (Aabb3Real, error) { var out Aabb3Real; err := check(C.csgrs_curve_region_real_bounding_box(p.ptr, &out)); return out, err }

func (p *CurveRegion) Union(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "union") }
func (p *CurveRegion) Difference(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "difference") }
func (p *CurveRegion) Intersection(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "intersection") }
func (p *CurveRegion) Xor(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "xor") }

func (p *CurveRegion) profileBinary(other *CurveRegion, op string) (*CurveRegion, error) {
	var out *C.csgrs_curve_region_t
	var status C.csgrs_status_t
	switch op {
	case "union":
		switch p.Family { case FamilyF32: status = C.csgrs_curve_region_f32_union(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_curve_region_f64_union(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_curve_region_i128_union(p.ptr, other.ptr, &out); default: status = C.csgrs_curve_region_real_union(p.ptr, other.ptr, &out) }
	case "difference":
		switch p.Family { case FamilyF32: status = C.csgrs_curve_region_f32_difference(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_curve_region_f64_difference(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_curve_region_i128_difference(p.ptr, other.ptr, &out); default: status = C.csgrs_curve_region_real_difference(p.ptr, other.ptr, &out) }
	case "intersection":
		switch p.Family { case FamilyF32: status = C.csgrs_curve_region_f32_intersection(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_curve_region_f64_intersection(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_curve_region_i128_intersection(p.ptr, other.ptr, &out); default: status = C.csgrs_curve_region_real_intersection(p.ptr, other.ptr, &out) }
	case "xor":
		switch p.Family { case FamilyF32: status = C.csgrs_curve_region_f32_xor(p.ptr, other.ptr, &out); case FamilyF64: status = C.csgrs_curve_region_f64_xor(p.ptr, other.ptr, &out); case FamilyI128: status = C.csgrs_curve_region_i128_xor(p.ptr, other.ptr, &out); default: status = C.csgrs_curve_region_real_xor(p.ptr, other.ptr, &out) }
	default:
		return nil, errors.New("unknown curve-region op")
	}
	return curveRegionOut(p.Family, out, check(status))
}

func TriangleMeshF64Polyhedron(points []Vec3F64, faceIndices []uint, faceOffsets []uint) (*TriangleMesh, error) {
	var out *C.csgrs_triangle_mesh_t
	indices := make([]C.size_t, len(faceIndices))
	offsets := make([]C.size_t, len(faceOffsets))
	for i, v := range faceIndices { indices[i] = C.size_t(v) }
	for i, v := range faceOffsets { offsets[i] = C.size_t(v) }
	var pptr *C.csgrs_vec3_f64_t
	if len(points) > 0 { pptr = (*C.csgrs_vec3_f64_t)(unsafe.Pointer(&points[0])) }
	err := check(C.csgrs_triangle_mesh_f64_polyhedron(pptr, C.size_t(len(points)), ptrSizeT(indices), C.size_t(len(indices)), ptrSizeT(offsets), C.size_t(len(offsets)), &out))
	return triangleMeshOut(FamilyF64, out, err)
}

func CurveRegionF64Polygon(points []Vec2F64) (*CurveRegion, error) {
	var out *C.csgrs_curve_region_t
	var pptr *C.csgrs_vec2_f64_t
	if len(points) > 0 { pptr = (*C.csgrs_vec2_f64_t)(unsafe.Pointer(&points[0])) }
	err := check(C.csgrs_curve_region_f64_polygon(pptr, C.size_t(len(points)), &out))
	return curveRegionOut(FamilyF64, out, err)
}

func ptrSizeT(values []C.size_t) *C.size_t {
	if len(values) == 0 { return nil }
	return &values[0]
}
