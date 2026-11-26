// Package csgrs wraps part of the native C ABI through cgo, retaining native
// geometry behind scalar-family handles. The build must supply the csgrs-ffi
// library/linker configuration; the preamble below supplies the header path.
//
// Successful Real, TriangleMesh and CurveRegion owners require explicit Free;
// there are no finalizers. Do not copy owning structs by value, mutate their
// Family tags, or race use with Free. Zero-value wrappers contain null handles,
// not canonical empty geometry. Free accepts nil receivers; other receiver and
// *Real argument accesses generally require non-nil Go pointers and may panic
// before the C ABI is reached. Outputs remain separate owners from their inputs.
//
// Float ingress preserves the exact finite binary value; float export is lossy
// and integer export can fail. Aliases retain C field names such as mins and x,
// which are unexported Go fields; they are not ergonomic external-package value
// APIs. Real bounds own native endpoint handles, but this package currently has
// no matching bounds-free wrapper. Mesh buffer exports have explicit free helpers.
//
// Numeric status categories are preserved, but check fetches thread-local C text
// in a later cgo call without pinning the goroutine. Reliable text requires caller
// thread affinity across the operation; otherwise the copied message need not
// belong to the returned status. See https://pkg.go.dev/runtime#LockOSThread.
// There is no policy selector, separate certainty receipt or construction-history
// query in this wrapper, and no wrapper-wide work/memory/cancellation budget.
package csgrs

/*
#cgo CFLAGS: -I../../crates/csgrs-ffi/include
#include "csgrs.h"
*/
import "C"

import (
	"errors"
	"unsafe"
)

// Family selects the boundary layout used to dispatch native operations.
// It does not describe internal coordinates, which remain native.
type Family int

const (
	// FamilyF32 selects binary32 ingress and lossy binary32 egress.
	FamilyF32  Family = Family(C.CSGRS_SCALAR_F32)
	// FamilyF64 selects binary64 ingress and lossy binary64 egress.
	FamilyF64  Family = Family(C.CSGRS_SCALAR_F64)
	// FamilyI128 selects split-word integer ingress and restricted integer egress.
	FamilyI128 Family = Family(C.CSGRS_SCALAR_I128)
	// FamilyReal selects opaque native scalar ingress/egress.
	FamilyReal Family = Family(C.CSGRS_SCALAR_REAL)
)

// I128 aliases the signed high/unsigned low 64-bit word record; fields retain C names.
type I128 = C.csgrs_i128_t
// Vec2F32 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec2F32 = C.csgrs_vec2_f32_t
// Vec3F32 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec3F32 = C.csgrs_vec3_f32_t
// Vec2F64 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec2F64 = C.csgrs_vec2_f64_t
// Vec3F64 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec3F64 = C.csgrs_vec3_f64_t
// Vec2I128 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec2I128 = C.csgrs_vec2_i128_t
// Vec3I128 aliases the C coordinate layout; lowercase component fields are unexported.
type Vec3I128 = C.csgrs_vec3_i128_t
// Vec2Real aliases the C coordinate layout; lowercase component fields are unexported.
// Real-family input components borrow handles; exported components can own handles.
type Vec2Real = C.csgrs_vec2_real_t
// Vec3Real aliases the C coordinate layout; lowercase component fields are unexported.
// Real-family input components borrow handles; exported components can own handles.
type Vec3Real = C.csgrs_vec3_real_t
// Matrix4F32 aliases the row-major sixteen-coefficient C layout.
// The lowercase values field is unexported; real-family coefficients borrow handles.
type Matrix4F32 = C.csgrs_matrix4_f32_t
// Matrix4F64 aliases the row-major sixteen-coefficient C layout.
// The lowercase values field is unexported; real-family coefficients borrow handles.
type Matrix4F64 = C.csgrs_matrix4_f64_t
// Matrix4I128 aliases the row-major sixteen-coefficient C layout.
// The lowercase values field is unexported; real-family coefficients borrow handles.
type Matrix4I128 = C.csgrs_matrix4_i128_t
// Matrix4Real aliases the row-major sixteen-coefficient C layout.
// The lowercase values field is unexported; real-family coefficients borrow handles.
type Matrix4Real = C.csgrs_matrix4_real_t
// Aabb3F32 aliases the C mins/maxs record; lowercase fields are unexported.
// Float endpoints are projections, not certified outward enclosures.
type Aabb3F32 = C.csgrs_aabb3_f32_t
// Aabb3F64 aliases the C mins/maxs record; lowercase fields are unexported.
// Float endpoints are projections, not certified outward enclosures.
type Aabb3F64 = C.csgrs_aabb3_f64_t
// Aabb3I128 aliases the C mins/maxs record; lowercase fields are unexported.
// Integer export requires recognized in-range integral extrema.
type Aabb3I128 = C.csgrs_aabb3_i128_t
// Aabb3Real aliases the C mins/maxs record; lowercase fields are unexported.
// Real endpoint handles require native cleanup, for which this package lacks a wrapper.
type Aabb3Real = C.csgrs_aabb3_real_t
// MeshBuffersF32 aliases owned C vertex and triangle arrays.
// Use FreeMeshBuffersF32 once after successful export; copying the record does not clone ownership.
type MeshBuffersF32 = C.csgrs_triangle_mesh_buffers_f32_t
// MeshBuffersF64 aliases owned C vertex and triangle arrays.
// Use FreeMeshBuffersF64 once after successful export; copying the record does not clone ownership.
type MeshBuffersF64 = C.csgrs_triangle_mesh_buffers_f64_t
// MeshBuffersI128 aliases owned C vertex and triangle arrays.
// Use FreeMeshBuffersI128 once after successful export; copying the record does not clone ownership.
type MeshBuffersI128 = C.csgrs_triangle_mesh_buffers_i128_t
// MeshBuffersReal aliases owned C vertex and triangle arrays.
// Use FreeMeshBuffersReal once after successful export; copying the record does not clone ownership.
type MeshBuffersReal = C.csgrs_triangle_mesh_buffers_real_t
// GraphicsMeshF32 aliases the C graphics descriptor, including raw owned arrays.
// This package does not expose a matching graphics export/free wrapper.
type GraphicsMeshF32 = C.csgrs_graphics_mesh_f32_t
// GraphicsMeshF64 aliases the C graphics descriptor, including raw owned arrays.
// This package does not expose a matching graphics export/free wrapper.
type GraphicsMeshF64 = C.csgrs_graphics_mesh_f64_t
// GraphicsMeshI128 aliases the C graphics descriptor, including raw owned arrays.
// This package does not expose a matching graphics export/free wrapper.
type GraphicsMeshI128 = C.csgrs_graphics_mesh_i128_t
// GraphicsMeshReal aliases the C graphics descriptor, including raw owned arrays.
// This package does not expose a matching graphics export/free wrapper.
type GraphicsMeshReal = C.csgrs_graphics_mesh_real_t
// RegionProfilesF32 aliases the sampled C profile descriptor and owned offset arrays.
// This package does not expose a matching profile export/free wrapper.
type RegionProfilesF32 = C.csgrs_region_profiles_f32_t
// RegionProfilesF64 aliases the sampled C profile descriptor and owned offset arrays.
// This package does not expose a matching profile export/free wrapper.
type RegionProfilesF64 = C.csgrs_region_profiles_f64_t
// RegionProfilesI128 aliases the sampled C profile descriptor and owned offset arrays.
// This package does not expose a matching profile export/free wrapper.
type RegionProfilesI128 = C.csgrs_region_profiles_i128_t
// RegionProfilesReal aliases the sampled C profile descriptor and owned offset arrays.
// This package does not expose a matching profile export/free wrapper.
type RegionProfilesReal = C.csgrs_region_profiles_real_t

// Real owns one native scalar handle and requires explicit Free. Do not copy by value.
type Real struct {
	// ptr is the unique native allocation, or nil before creation/after Free.
	ptr *C.csgrs_real_t
}
// TriangleMesh owns one native mesh allocation; Free is explicit, not automatic.
// The zero value is a null owner, not a canonical empty operand. Do not copy by value.
type TriangleMesh struct {
	// ptr owns the native geometry and must not be used concurrently with Free.
	ptr    *C.csgrs_triangle_mesh_t
	// Family is the dispatch tag set at construction; changing it does not convert geometry.
	Family Family
}
// CurveRegion owns one native filled-region allocation; Free is explicit.
// The zero value is a null owner, not an empty region. Do not copy by value.
type CurveRegion struct {
	// ptr owns the native geometry and must not be used concurrently with Free.
	ptr    *C.csgrs_curve_region_t
	// Family is the dispatch tag set at construction; changing it does not convert geometry.
	Family Family
}

// StatusError retains the numeric C category and a copied diagnostic string.
// The current wrapper does not pin the status and text calls to one OS thread;
// without caller affinity, Message is not guaranteed to belong to Status.
type StatusError struct {
	// Status is the numeric C ABI result category.
	Status  int
	// Message is copied from the C thread-local slot after the operation.
	Message string
}

// Error returns the stored text without reconstructing or parsing its category.
func (e *StatusError) Error() string { return e.Message }

// check converts non-OK statuses to StatusError using a subsequent cgo text query.
//
// It does not lock the goroutine to the operation's OS thread, so diagnostic
// identity is not guaranteed without caller thread affinity. A null text pointer
// uses a fallback; a non-null empty string is retained as empty.
func check(status C.csgrs_status_t) error {
	if status == C.CSGRS_STATUS_OK {
		return nil
	}
	msg := C.csgrs_last_error_message()
	if msg == nil {
		return &StatusError{Status: int(status), Message: "csgrs error"}
	}
	return &StatusError{Status: int(status), Message: C.GoString(msg)}
}

// I128FromInt64 sign-extends an int64 into the two-word exact integer layout.
func I128FromInt64(value int64) I128 {
	hi := int64(0)
	if value < 0 {
		hi = -1
	}
	return I128{hi: C.int64_t(hi), lo: C.uint64_t(uint64(value))}
}

// RealZero allocates exact native zero.
// A successful returned owner requires Free.
func RealZero() (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_zero(&out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

// RealOne allocates exact native one.
// A successful returned owner requires Free.
func RealOne() (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_one(&out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

// RealFromF64 imports a finite binary64 value exactly; NaN/infinity return conversion errors.
// A successful returned owner requires Free.
func RealFromF64(value float64) (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_from_f64(C.double(value), &out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

// RealFromI128 imports the signed two-word integer exactly.
// A successful returned owner requires Free.
func RealFromI128(value I128) (*Real, error) {
	var out *C.csgrs_real_t
	if err := check(C.csgrs_real_from_i128(value, &out)); err != nil {
		return nil, err
	}
	return &Real{ptr: out}, nil
}

// Free releases this scalar once and clears its pointer. Nil/already-cleared receivers are no-ops.
// Copies of the owning struct retain stale pointers; Free is not concurrency-safe.
func (r *Real) Free() {
	if r != nil && r.ptr != nil {
		C.csgrs_real_free(r.ptr)
		r.ptr = nil
	}
}

// ToF64 returns a finite lossy projection, or zero with an error.
// A nil Go receiver panics; a freed scalar returns the C null-pointer error.
func (r *Real) ToF64() (float64, error) {
	var out C.double
	if err := check(C.csgrs_real_to_f64(r.ptr, &out)); err != nil {
		return 0, err
	}
	return float64(out), nil
}

// TriangleMeshF32Cube constructs an origin-anchored cube under strict predicates.
// Width must be nonnegative; validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func TriangleMeshF32Cube(width float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cube(C.float(width), &out)); return triangleMeshOut(FamilyF32, out, err) }
// TriangleMeshF64Cube constructs an origin-anchored cube under strict predicates.
// Width must be nonnegative; validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func TriangleMeshF64Cube(width float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cube(C.double(width), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TriangleMeshI128Cube constructs an origin-anchored cube under strict predicates.
// Width must be nonnegative; validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func TriangleMeshI128Cube(width I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cube(width, &out)); return triangleMeshOut(FamilyI128, out, err) }
// TriangleMeshRealCube constructs an origin-anchored cube under strict predicates.
// Width must be nonnegative; validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func TriangleMeshRealCube(width *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cube(width.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }

// TriangleMeshF32Cuboid constructs an origin-anchored XYZ box under strict predicates.
// All dimensions are validated as nonnegative before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshF32Cuboid(width, length, height float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cuboid(C.float(width), C.float(length), C.float(height), &out)); return triangleMeshOut(FamilyF32, out, err) }
// TriangleMeshF64Cuboid constructs an origin-anchored XYZ box under strict predicates.
// All dimensions are validated as nonnegative before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshF64Cuboid(width, length, height float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cuboid(C.double(width), C.double(length), C.double(height), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TriangleMeshI128Cuboid constructs an origin-anchored XYZ box under strict predicates.
// All dimensions are validated as nonnegative before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshI128Cuboid(width, length, height I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cuboid(width, length, height, &out)); return triangleMeshOut(FamilyI128, out, err) }
// TriangleMeshRealCuboid constructs an origin-anchored XYZ box under strict predicates.
// All dimensions are validated as nonnegative before zero-size empty success.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func TriangleMeshRealCuboid(width, length, height *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cuboid(width.ptr, length.ptr, height.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }

// TriangleMeshF32Sphere constructs a sampled sphere centered at the origin, with Y-axis poles.
// Radius must be nonnegative; native minimum counts are three segments and two stacks.
// Go int counts are cast directly to size_t: negative values wrap rather than being
// rejected here. Sampling does not establish analytic-sphere fidelity.
// Success returns a new owner requiring Free.
func TriangleMeshF32Sphere(radius float32, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_sphere(C.float(radius), C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyF32, out, err) }
// TriangleMeshF64Sphere constructs a sampled sphere centered at the origin, with Y-axis poles.
// Radius must be nonnegative; native minimum counts are three segments and two stacks.
// Go int counts are cast directly to size_t: negative values wrap rather than being
// rejected here. Sampling does not establish analytic-sphere fidelity.
// Success returns a new owner requiring Free.
func TriangleMeshF64Sphere(radius float64, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_sphere(C.double(radius), C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TriangleMeshI128Sphere constructs a sampled sphere centered at the origin, with Y-axis poles.
// Radius must be nonnegative; native minimum counts are three segments and two stacks.
// Go int counts are cast directly to size_t: negative values wrap rather than being
// rejected here. Sampling does not establish analytic-sphere fidelity.
// Success returns a new owner requiring Free.
func TriangleMeshI128Sphere(radius I128, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_sphere(radius, C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyI128, out, err) }
// TriangleMeshRealSphere constructs a sampled sphere centered at the origin, with Y-axis poles.
// Radius must be nonnegative; native minimum counts are three segments and two stacks.
// Go int counts are cast directly to size_t: negative values wrap rather than being
// rejected here. Sampling does not establish analytic-sphere fidelity.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func TriangleMeshRealSphere(radius *Real, segments, stacks int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_sphere(radius.ptr, C.size_t(segments), C.size_t(stacks), &out)); return triangleMeshOut(FamilyReal, out, err) }

// TriangleMeshF32Cylinder constructs a sampled capped cylinder from Z zero to signed height.
// Radius must be nonnegative and the native minimum segment count is three.
// The Go count is cast directly to size_t; negative values wrap. Parameters are
// validated before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshF32Cylinder(radius, height float32, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_cylinder(C.float(radius), C.float(height), C.size_t(segments), &out)); return triangleMeshOut(FamilyF32, out, err) }
// TriangleMeshF64Cylinder constructs a sampled capped cylinder from Z zero to signed height.
// Radius must be nonnegative and the native minimum segment count is three.
// The Go count is cast directly to size_t; negative values wrap. Parameters are
// validated before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshF64Cylinder(radius, height float64, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_cylinder(C.double(radius), C.double(height), C.size_t(segments), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TriangleMeshI128Cylinder constructs a sampled capped cylinder from Z zero to signed height.
// Radius must be nonnegative and the native minimum segment count is three.
// The Go count is cast directly to size_t; negative values wrap. Parameters are
// validated before zero-size empty success.
// Success returns a new owner requiring Free.
func TriangleMeshI128Cylinder(radius, height I128, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_cylinder(radius, height, C.size_t(segments), &out)); return triangleMeshOut(FamilyI128, out, err) }
// TriangleMeshRealCylinder constructs a sampled capped cylinder from Z zero to signed height.
// Radius must be nonnegative and the native minimum segment count is three.
// The Go count is cast directly to size_t; negative values wrap. Parameters are
// validated before zero-size empty success.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func TriangleMeshRealCylinder(radius, height *Real, segments int) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_cylinder(radius.ptr, height.ptr, C.size_t(segments), &out)); return triangleMeshOut(FamilyReal, out, err) }

// triangleMeshOut returns a prior error or wraps the successful mesh pointer with its tag.
// It does not independently validate a successful pointer or reclaim one accompanying an error.
func triangleMeshOut(f Family, out *C.csgrs_triangle_mesh_t, err error) (*TriangleMesh, error) {
	if err != nil {
		return nil, err
	}
	return &TriangleMesh{ptr: out, Family: f}, nil
}

// curveRegionOut returns a prior error or wraps the successful region pointer with its tag.
// It does not independently validate a successful pointer or reclaim one accompanying an error.
func curveRegionOut(f Family, out *C.csgrs_curve_region_t, err error) (*CurveRegion, error) {
	if err != nil {
		return nil, err
	}
	return &CurveRegion{ptr: out, Family: f}, nil
}

// Free releases this owner and clears its pointer; nil/already-cleared receivers are no-ops.
// Do not copy the struct by value or race Free with native access.
func (m *TriangleMesh) Free() {
	if m != nil && m.ptr != nil {
		C.csgrs_triangle_mesh_free(m.ptr)
		m.ptr = nil
	}
}

// Free releases this owner and clears its pointer; nil/already-cleared receivers are no-ops.
// Do not copy the struct by value or race Free with native access.
func (p *CurveRegion) Free() {
	if p != nil && p.ptr != nil {
		C.csgrs_curve_region_free(p.ptr)
		p.ptr = nil
	}
}

// binary selects a named mesh Boolean and wraps its status/output.
// Unknown operation names return an ordinary Go error; receivers must be non-nil.
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

// Union returns the regularized union.
// Uses strict native mesh predicates. Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (m *TriangleMesh) Union(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "union") }
// Difference returns the regularized receiver material outside other.
// Uses strict native mesh predicates. Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (m *TriangleMesh) Difference(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "difference") }
// Intersection returns the regularized common material.
// Uses strict native mesh predicates. Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (m *TriangleMesh) Intersection(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "intersection") }
// Xor returns the regularized material in exactly one operand.
// Uses strict native mesh predicates. Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (m *TriangleMesh) Xor(other *TriangleMesh) (*TriangleMesh, error) { return m.binary(other, "xor") }

// meshUnion dispatches by the left wrapper's Family, defaulting unknown tags to real.
// The C ABI validates both actual handle families; this helper returns its status unchanged.
func meshUnion(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_union(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_union(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_union(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_union(m.ptr, o.ptr, out)
	}
}

// meshDifference dispatches by the left wrapper's Family, defaulting unknown tags to real.
// The C ABI validates both actual handle families; this helper returns its status unchanged.
func meshDifference(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_difference(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_difference(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_difference(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_difference(m.ptr, o.ptr, out)
	}
}

// meshIntersection dispatches by the left wrapper's Family, defaulting unknown tags to real.
// The C ABI validates both actual handle families; this helper returns its status unchanged.
func meshIntersection(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_intersection(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_intersection(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_intersection(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_intersection(m.ptr, o.ptr, out)
	}
}

// meshXor dispatches by the left wrapper's Family, defaulting unknown tags to real.
// The C ABI validates both actual handle families; this helper returns its status unchanged.
func meshXor(m, o *TriangleMesh, out **C.csgrs_triangle_mesh_t) C.csgrs_status_t {
	switch m.Family {
	case FamilyF32: return C.csgrs_triangle_mesh_f32_xor(m.ptr, o.ptr, out)
	case FamilyF64: return C.csgrs_triangle_mesh_f64_xor(m.ptr, o.ptr, out)
	case FamilyI128: return C.csgrs_triangle_mesh_i128_xor(m.ptr, o.ptr, out)
	default: return C.csgrs_triangle_mesh_real_xor(m.ptr, o.ptr, out)
	}
}

// TranslateF64 returns a translated owner using the F64 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) TranslateF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_translate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
// ScaleF64 scales XYZ under strict predicates, rejects zero axes and corrects reflection winding.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) ScaleF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_scale(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
// RotateF64 rotates about X, then Y, then Z in degrees under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) RotateF64(x, y, z float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f64_rotate(m.ptr, C.double(x), C.double(y), C.double(z), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TranslateF32 returns a translated owner using the F32 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) TranslateF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_translate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
// ScaleF32 scales XYZ under strict predicates, rejects zero axes and corrects reflection winding.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) ScaleF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_scale(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
// RotateF32 rotates about X, then Y, then Z in degrees under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) RotateF32(x, y, z float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_f32_rotate(m.ptr, C.float(x), C.float(y), C.float(z), &out)); return triangleMeshOut(FamilyF32, out, err) }
// TranslateI128 returns a translated owner using the I128 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) TranslateI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_translate(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
// ScaleI128 scales XYZ under strict predicates, rejects zero axes and corrects reflection winding.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) ScaleI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_scale(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
// RotateI128 rotates about X, then Y, then Z in degrees under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
func (m *TriangleMesh) RotateI128(x, y, z I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_i128_rotate(m.ptr, x, y, z, &out)); return triangleMeshOut(FamilyI128, out, err) }
// TranslateReal returns a translated owner using the Real ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
// Scalar arguments must be non-nil live wrappers.
func (m *TriangleMesh) TranslateReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_translate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
// ScaleReal scales XYZ under strict predicates, rejects zero axes and corrects reflection winding.
// The native handle must already match this family; the Go Family field is not used.
// Scalar arguments must be non-nil live wrappers.
func (m *TriangleMesh) ScaleReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_scale(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
// RotateReal rotates about X, then Y, then Z in degrees under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
// Scalar arguments must be non-nil live wrappers.
func (m *TriangleMesh) RotateReal(x, y, z *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_triangle_mesh_real_rotate(m.ptr, x.ptr, y.ptr, z.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
// BoundingBoxF64 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Exported floats are not certified outward enclosures.
func (m *TriangleMesh) BoundingBoxF64() (*Aabb3F64, error) {
	var out Aabb3F64
	status := C.csgrs_triangle_mesh_f64_bounding_box(m.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// BoundingBoxF32 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Exported floats are not certified outward enclosures.
func (m *TriangleMesh) BoundingBoxF32() (*Aabb3F32, error) {
	var out Aabb3F32
	status := C.csgrs_triangle_mesh_f32_bounding_box(m.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// BoundingBoxI128 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Every endpoint must be a recognized in-range integer.
func (m *TriangleMesh) BoundingBoxI128() (*Aabb3I128, error) {
	var out Aabb3I128
	status := C.csgrs_triangle_mesh_i128_bounding_box(m.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// BoundingBoxReal returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// A present result owns six native handles; this package lacks a bounds-free wrapper.
func (m *TriangleMesh) BoundingBoxReal() (*Aabb3Real, error) {
	var out Aabb3Real
	status := C.csgrs_triangle_mesh_real_bounding_box(m.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// VerticesAndIndicesF64 exports owned position and triangle arrays through the F64 ABI.
// Only a nil error grants ownership; use FreeMeshBuffersF64 exactly once.
// The handle must already match this family; no scalar-family conversion occurs.
// Coordinate projection is lossy.
func (m *TriangleMesh) VerticesAndIndicesF64() (MeshBuffersF64, error) { var out MeshBuffersF64; err := check(C.csgrs_triangle_mesh_f64_vertices_and_indices(m.ptr, &out)); return out, err }
// VerticesAndIndicesF32 exports owned position and triangle arrays through the F32 ABI.
// Only a nil error grants ownership; use FreeMeshBuffersF32 exactly once.
// The handle must already match this family; no scalar-family conversion occurs.
// Coordinate projection is lossy.
func (m *TriangleMesh) VerticesAndIndicesF32() (MeshBuffersF32, error) { var out MeshBuffersF32; err := check(C.csgrs_triangle_mesh_f32_vertices_and_indices(m.ptr, &out)); return out, err }
// VerticesAndIndicesI128 exports owned position and triangle arrays through the I128 ABI.
// Only a nil error grants ownership; use FreeMeshBuffersI128 exactly once.
// The handle must already match this family; no scalar-family conversion occurs.
// Noninteger/unrecognized/out-of-range coordinates fail export.
func (m *TriangleMesh) VerticesAndIndicesI128() (MeshBuffersI128, error) { var out MeshBuffersI128; err := check(C.csgrs_triangle_mesh_i128_vertices_and_indices(m.ptr, &out)); return out, err }
// VerticesAndIndicesReal exports owned position and triangle arrays through the Real ABI.
// Only a nil error grants ownership; use FreeMeshBuffersReal exactly once.
// The handle must already match this family; no scalar-family conversion occurs.
// Each returned coordinate owns a native scalar handle freed with its array.
func (m *TriangleMesh) VerticesAndIndicesReal() (MeshBuffersReal, error) { var out MeshBuffersReal; err := check(C.csgrs_triangle_mesh_real_vertices_and_indices(m.ptr, &out)); return out, err }
// FreeMeshBuffersF64 releases the matching successful descriptor and nested real handles, if any.
// Preserve all original pointers/counts and free once; copies of the record are not owners.
func FreeMeshBuffersF64(value MeshBuffersF64) { C.csgrs_triangle_mesh_buffers_f64_free(value) }
// FreeMeshBuffersF32 releases the matching successful descriptor and nested real handles, if any.
// Preserve all original pointers/counts and free once; copies of the record are not owners.
func FreeMeshBuffersF32(value MeshBuffersF32) { C.csgrs_triangle_mesh_buffers_f32_free(value) }
// FreeMeshBuffersI128 releases the matching successful descriptor and nested real handles, if any.
// Preserve all original pointers/counts and free once; copies of the record are not owners.
func FreeMeshBuffersI128(value MeshBuffersI128) { C.csgrs_triangle_mesh_buffers_i128_free(value) }
// FreeMeshBuffersReal releases the matching successful descriptor and nested real handles, if any.
// Preserve all original pointers/counts and free once; copies of the record are not owners.
func FreeMeshBuffersReal(value MeshBuffersReal) { C.csgrs_triangle_mesh_buffers_real_free(value) }

// Inverse reverses triangle winding without constructing an unbounded complement.
// Uses Family dispatch and returns a new owner; unresolved native decisions remain errors.
func (m *TriangleMesh) Inverse() (*TriangleMesh, error) { return m.meshUnary("inverse") }
// Center moves the native bounds center to the origin, preserving empty geometry.
// Uses Family dispatch and returns a new owner; unresolved native decisions remain errors.
func (m *TriangleMesh) Center() (*TriangleMesh, error) { return m.meshUnary("center") }
// Float moves the native minimum Z bound to zero, preserving empty geometry.
// Uses Family dispatch and returns a new owner; unresolved native decisions remain errors.
func (m *TriangleMesh) Float() (*TriangleMesh, error) { return m.meshUnary("float") }

// meshUnary dispatches inverse/center/float by Family, defaulting unknown tags to real.
// Unknown operation names return an ordinary Go error; no scalar family conversion occurs.
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

// CurveRegionF32Square constructs an origin-anchored XY square with strict nonnegative width validation.
// Validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func CurveRegionF32Square(width float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_square(C.float(width), &out)); return curveRegionOut(FamilyF32, out, err) }
// CurveRegionF64Square constructs an origin-anchored XY square with strict nonnegative width validation.
// Validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func CurveRegionF64Square(width float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_square(C.double(width), &out)); return curveRegionOut(FamilyF64, out, err) }
// CurveRegionI128Square constructs an origin-anchored XY square with strict nonnegative width validation.
// Validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free.
func CurveRegionI128Square(width I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_square(width, &out)); return curveRegionOut(FamilyI128, out, err) }
// CurveRegionRealSquare constructs an origin-anchored XY square with strict nonnegative width validation.
// Validated zero width produces canonical empty geometry.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func CurveRegionRealSquare(width *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_square(width.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }

// CurveRegionF32Rectangle constructs an origin-anchored XY rectangle under strict predicates.
// Both dimensions must be nonnegative, including when one is zero.
// Success returns a new owner requiring Free.
func CurveRegionF32Rectangle(width, length float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_rectangle(C.float(width), C.float(length), &out)); return curveRegionOut(FamilyF32, out, err) }
// CurveRegionF64Rectangle constructs an origin-anchored XY rectangle under strict predicates.
// Both dimensions must be nonnegative, including when one is zero.
// Success returns a new owner requiring Free.
func CurveRegionF64Rectangle(width, length float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_rectangle(C.double(width), C.double(length), &out)); return curveRegionOut(FamilyF64, out, err) }
// CurveRegionI128Rectangle constructs an origin-anchored XY rectangle under strict predicates.
// Both dimensions must be nonnegative, including when one is zero.
// Success returns a new owner requiring Free.
func CurveRegionI128Rectangle(width, length I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_rectangle(width, length, &out)); return curveRegionOut(FamilyI128, out, err) }
// CurveRegionRealRectangle constructs an origin-anchored XY rectangle under strict predicates.
// Both dimensions must be nonnegative, including when one is zero.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func CurveRegionRealRectangle(width, length *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_rectangle(width.ptr, length.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }

// CurveRegionF32Circle constructs a sampled regular XY polygon centered at the origin.
// Radius must be nonnegative and the native minimum segment count is three, even
// for zero radius. Negative Go counts wrap on size_t conversion. This is not an
// analytic circular boundary.
// Success returns a new owner requiring Free.
func CurveRegionF32Circle(radius float32, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_circle(C.float(radius), C.size_t(segments), &out)); return curveRegionOut(FamilyF32, out, err) }
// CurveRegionF64Circle constructs a sampled regular XY polygon centered at the origin.
// Radius must be nonnegative and the native minimum segment count is three, even
// for zero radius. Negative Go counts wrap on size_t conversion. This is not an
// analytic circular boundary.
// Success returns a new owner requiring Free.
func CurveRegionF64Circle(radius float64, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_circle(C.double(radius), C.size_t(segments), &out)); return curveRegionOut(FamilyF64, out, err) }
// CurveRegionI128Circle constructs a sampled regular XY polygon centered at the origin.
// Radius must be nonnegative and the native minimum segment count is three, even
// for zero radius. Negative Go counts wrap on size_t conversion. This is not an
// analytic circular boundary.
// Success returns a new owner requiring Free.
func CurveRegionI128Circle(radius I128, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_circle(radius, C.size_t(segments), &out)); return curveRegionOut(FamilyI128, out, err) }
// CurveRegionRealCircle constructs a sampled regular XY polygon centered at the origin.
// Radius must be nonnegative and the native minimum segment count is three, even
// for zero radius. Negative Go counts wrap on size_t conversion. This is not an
// analytic circular boundary.
// Success returns a new owner requiring Free. Real arguments must be non-nil live wrappers.
func CurveRegionRealCircle(radius *Real, segments int) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_circle(radius.ptr, C.size_t(segments), &out)); return curveRegionOut(FamilyReal, out, err) }

// ExtrudeF64 extrudes along Z by a nonzero signed height under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
// Success is a new mesh owner; nonlinear interiors may be sampled and the separate receipt is discarded.
func (p *CurveRegion) ExtrudeF64(height float64) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_f64_extrude(p.ptr, C.double(height), &out)); return triangleMeshOut(FamilyF64, out, err) }
// TranslateF64 returns a translated owner using the F64 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
// Nonzero Z cannot remain in the XY carrier.
func (p *CurveRegion) TranslateF64(x, y, z float64) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f64_translate(p.ptr, C.double(x), C.double(y), C.double(z), &out)); return curveRegionOut(FamilyF64, out, err) }
// BoundingBoxF64 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Exported floats are not certified outward enclosures.
func (p *CurveRegion) BoundingBoxF64() (*Aabb3F64, error) {
	var out Aabb3F64
	status := C.csgrs_curve_region_f64_bounding_box(p.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// ExtrudeF32 extrudes along Z by a nonzero signed height under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
// Success is a new mesh owner; nonlinear interiors may be sampled and the separate receipt is discarded.
func (p *CurveRegion) ExtrudeF32(height float32) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_f32_extrude(p.ptr, C.float(height), &out)); return triangleMeshOut(FamilyF32, out, err) }
// ExtrudeI128 extrudes along Z by a nonzero signed height under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
// Success is a new mesh owner; nonlinear interiors may be sampled and the separate receipt is discarded.
func (p *CurveRegion) ExtrudeI128(height I128) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_i128_extrude(p.ptr, height, &out)); return triangleMeshOut(FamilyI128, out, err) }
// ExtrudeReal extrudes along Z by a nonzero signed height under strict predicates.
// The native handle must already match this family; the Go Family field is not used.
// Success is a new mesh owner; nonlinear interiors may be sampled and the separate receipt is discarded.
// Scalar arguments must be non-nil live wrappers.
func (p *CurveRegion) ExtrudeReal(height *Real) (*TriangleMesh, error) { var out *C.csgrs_triangle_mesh_t; err := check(C.csgrs_curve_region_real_extrude(p.ptr, height.ptr, &out)); return triangleMeshOut(FamilyReal, out, err) }
// TranslateF32 returns a translated owner using the F32 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
// Nonzero Z cannot remain in the XY carrier.
func (p *CurveRegion) TranslateF32(x, y, z float32) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_f32_translate(p.ptr, C.float(x), C.float(y), C.float(z), &out)); return curveRegionOut(FamilyF32, out, err) }
// TranslateI128 returns a translated owner using the I128 ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
// Nonzero Z cannot remain in the XY carrier.
func (p *CurveRegion) TranslateI128(x, y, z I128) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_i128_translate(p.ptr, x, y, z, &out)); return curveRegionOut(FamilyI128, out, err) }
// TranslateReal returns a translated owner using the Real ABI, not a family conversion.
// The native handle must already match this family; the Go Family field is not used.
// Nonzero Z cannot remain in the XY carrier.
// Scalar arguments must be non-nil live wrappers.
func (p *CurveRegion) TranslateReal(x, y, z *Real) (*CurveRegion, error) { var out *C.csgrs_curve_region_t; err := check(C.csgrs_curve_region_real_translate(p.ptr, x.ptr, y.ptr, z.ptr, &out)); return curveRegionOut(FamilyReal, out, err) }
// BoundingBoxF32 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Exported floats are not certified outward enclosures.
func (p *CurveRegion) BoundingBoxF32() (*Aabb3F32, error) {
	var out Aabb3F32
	status := C.csgrs_curve_region_f32_bounding_box(p.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// BoundingBoxI128 returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// Every endpoint must be a recognized in-range integer.
func (p *CurveRegion) BoundingBoxI128() (*Aabb3I128, error) {
	var out Aabb3I128
	status := C.csgrs_curve_region_i128_bounding_box(p.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}
// BoundingBoxReal returns nil without error when the geometry is empty.
// The native handle must match this family; query/conversion failures remain errors.
// A present result owns six native handles; this package lacks a bounds-free wrapper.
func (p *CurveRegion) BoundingBoxReal() (*Aabb3Real, error) {
	var out Aabb3Real
	status := C.csgrs_curve_region_real_bounding_box(p.ptr, &out)
	if status == C.CSGRS_STATUS_NO_BOUNDS {
		return nil, nil
	}
	if err := check(status); err != nil {
		return nil, err
	}
	return &out, nil
}

// Union returns the regularized union.
// Uses Hypercurve's active scope (strict outside preview/provisional evaluation). Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (p *CurveRegion) Union(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "union") }
// Difference returns the regularized receiver material outside other.
// Uses Hypercurve's active scope (strict outside preview/provisional evaluation). Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (p *CurveRegion) Difference(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "difference") }
// Intersection returns the regularized common material.
// Uses Hypercurve's active scope (strict outside preview/provisional evaluation). Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (p *CurveRegion) Intersection(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "intersection") }
// Xor returns the regularized material in exactly one operand.
// Uses Hypercurve's active scope (strict outside preview/provisional evaluation). Non-nil operands are borrowed;
// success returns a new owner and no separate certainty receipt.
func (p *CurveRegion) Xor(other *CurveRegion) (*CurveRegion, error) { return p.profileBinary(other, "xor") }

// profileBinary selects a named region Boolean using the receiver's family tag.
// Unrecognized family values route to the real ABI; unknown operation names return
// an ordinary Go error. The native handles still enforce their actual family.
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

// TriangleMeshF64Polyhedron copies face ranges and borrows binary64 point rows for native triangulation.
// Offsets must partition all indices from zero to their count; native validation
// checks simple planar faces and preserves authored winding. Success need not be
// a closed solid. Slice memory is borrowed only during the C call; output requires Free.
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

// CurveRegionF64Polygon borrows XY rows to construct a filled polygon under strict predicates.
// A valid ring has at least three vertices; empty or malformed rings fail. Binary64
// coordinates enter exactly, and a successful new region requires Free.
func CurveRegionF64Polygon(points []Vec2F64) (*CurveRegion, error) {
	var out *C.csgrs_curve_region_t
	var pptr *C.csgrs_vec2_f64_t
	if len(points) > 0 { pptr = (*C.csgrs_vec2_f64_t)(unsafe.Pointer(&points[0])) }
	err := check(C.csgrs_curve_region_f64_polygon(pptr, C.size_t(len(points)), &out))
	return curveRegionOut(FamilyF64, out, err)
}

// ptrSizeT borrows the first size_t element or returns nil for an empty slice.
// The pointer may only be used while the slice remains live during the synchronous C call.
func ptrSizeT(values []C.size_t) *C.size_t {
	if len(values) == 0 { return nil }
	return &values[0]
}
