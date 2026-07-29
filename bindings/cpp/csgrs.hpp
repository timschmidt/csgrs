#pragma once

#include "../../../csgrs-ffi/include/csgrs.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace csgrs {

class error : public std::runtime_error {
public:
  explicit error(const char *message) : std::runtime_error(message ? message : "csgrs error") {}
};

inline void check(csgrs_status_t status) {
  if (status != CSGRS_STATUS_OK) {
    throw error(csgrs_last_error_message());
  }
}

class Real {
public:
  Real() = default;
  explicit Real(csgrs_real_t *handle) : handle_(handle) {}
  Real(const Real &other) {
    if (other.handle_) {
      check(csgrs_real_clone(other.handle_, &handle_));
    }
  }
  Real(Real &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  Real &operator=(Real other) noexcept {
    swap(other);
    return *this;
  }
  ~Real() { csgrs_real_free(handle_); }

  static Real zero() {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_zero(&out));
    return Real(out);
  }

  static Real one() {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_one(&out));
    return Real(out);
  }

  static Real from_f64(double value) {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_from_f64(value, &out));
    return Real(out);
  }

  static Real from_i128(csgrs_i128_t value) {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_from_i128(value, &out));
    return Real(out);
  }

  double to_f64() const {
    double out = 0.0;
    check(csgrs_real_to_f64(handle_, &out));
    return out;
  }

  csgrs_i128_t to_i128() const {
    csgrs_i128_t out{};
    check(csgrs_real_to_i128(handle_, &out));
    return out;
  }

  const csgrs_real_t *get() const { return handle_; }
  csgrs_real_t *release() { return std::exchange(handle_, nullptr); }
  void swap(Real &other) noexcept { std::swap(handle_, other.handle_); }

private:
  csgrs_real_t *handle_ = nullptr;
};

struct F32;
struct F64;
struct I128;
struct RawReal;

#define CSGRS_CPP_TRAITS(Traits, Name, ScalarType, Vec2Type, Vec3Type, MatrixType, AabbType, MeshBuffersType, GraphicsMeshType, RegionProfilesType) \
  struct Traits { \
    using scalar_type = ScalarType; \
    using vec2_type = Vec2Type; \
    using vec3_type = Vec3Type; \
    using matrix4_type = MatrixType; \
    using aabb_type = AabbType; \
    using mesh_buffers_type = MeshBuffersType; \
    using graphics_mesh_type = GraphicsMeshType; \
    using region_profiles_type = RegionProfilesType; \
    static csgrs_status_t mesh_cube(scalar_type width, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cube(width, out); } \
    static csgrs_status_t mesh_cuboid(scalar_type width, scalar_type length, scalar_type height, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cuboid(width, length, height, out); } \
    static csgrs_status_t mesh_sphere(scalar_type radius, std::size_t segments, std::size_t stacks, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_sphere(radius, segments, stacks, out); } \
    static csgrs_status_t mesh_cylinder(scalar_type radius, scalar_type height, std::size_t segments, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cylinder(radius, height, segments, out); } \
    static csgrs_status_t mesh_polyhedron(const vec3_type *points, std::size_t point_len, const std::size_t *face_indices, std::size_t face_index_len, const std::size_t *face_offsets, std::size_t face_offset_len, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_polyhedron(points, point_len, face_indices, face_index_len, face_offsets, face_offset_len, out); } \
    static csgrs_status_t mesh_union(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_union(lhs, rhs, out); } \
    static csgrs_status_t mesh_difference(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_difference(lhs, rhs, out); } \
    static csgrs_status_t mesh_intersection(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_intersection(lhs, rhs, out); } \
    static csgrs_status_t mesh_xor(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_xor(lhs, rhs, out); } \
    static csgrs_status_t mesh_transform(const csgrs_triangle_mesh_t *mesh, matrix4_type matrix, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_transform(mesh, matrix, out); } \
    static csgrs_status_t mesh_translate(const csgrs_triangle_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_translate(mesh, x, y, z, out); } \
    static csgrs_status_t mesh_scale(const csgrs_triangle_mesh_t *mesh, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_scale(mesh, sx, sy, sz, out); } \
    static csgrs_status_t mesh_rotate(const csgrs_triangle_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_rotate(mesh, x, y, z, out); } \
    static csgrs_status_t mesh_inverse(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_inverse(mesh, out); } \
    static csgrs_status_t mesh_center(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_center(mesh, out); } \
    static csgrs_status_t mesh_float(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_float(mesh, out); } \
    static csgrs_status_t mesh_bounding_box(const csgrs_triangle_mesh_t *mesh, aabb_type *out) { return csgrs_triangle_mesh_##Name##_bounding_box(mesh, out); } \
    static csgrs_status_t mesh_vertices_and_indices(const csgrs_triangle_mesh_t *mesh, mesh_buffers_type *out) { return csgrs_triangle_mesh_##Name##_vertices_and_indices(mesh, out); } \
    static csgrs_status_t mesh_graphics_mesh(const csgrs_triangle_mesh_t *mesh, graphics_mesh_type *out) { return csgrs_triangle_mesh_##Name##_graphics_mesh(mesh, out); } \
    static csgrs_status_t region_square(scalar_type width, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_square(width, out); } \
    static csgrs_status_t region_rectangle(scalar_type width, scalar_type length, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_rectangle(width, length, out); } \
    static csgrs_status_t region_circle(scalar_type radius, std::size_t segments, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_circle(radius, segments, out); } \
    static csgrs_status_t region_polygon(const vec2_type *points, std::size_t point_len, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_polygon(points, point_len, out); } \
    static csgrs_status_t region_union(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_union(lhs, rhs, out); } \
    static csgrs_status_t region_difference(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_difference(lhs, rhs, out); } \
    static csgrs_status_t region_intersection(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_intersection(lhs, rhs, out); } \
    static csgrs_status_t region_xor(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_xor(lhs, rhs, out); } \
    static csgrs_status_t region_transform(const csgrs_curve_region_t *region, matrix4_type matrix, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_transform(region, matrix, out); } \
    static csgrs_status_t region_translate(const csgrs_curve_region_t *region, scalar_type x, scalar_type y, scalar_type z, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_translate(region, x, y, z, out); } \
    static csgrs_status_t region_scale(const csgrs_curve_region_t *region, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_scale(region, sx, sy, sz, out); } \
    static csgrs_status_t region_rotate(const csgrs_curve_region_t *region, scalar_type x, scalar_type y, scalar_type z, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_rotate(region, x, y, z, out); } \
    static csgrs_status_t region_bounding_box(const csgrs_curve_region_t *region, aabb_type *out) { return csgrs_curve_region_##Name##_bounding_box(region, out); } \
    static csgrs_status_t region_extrude(const csgrs_curve_region_t *region, scalar_type height, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_extrude(region, height, out); } \
    static csgrs_status_t region_extrude_vector(const csgrs_curve_region_t *region, vec3_type direction, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_extrude_vector(region, direction, out); } \
    static csgrs_status_t region_revolve(const csgrs_curve_region_t *region, scalar_type angle, std::size_t segments, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_revolve(region, angle, segments, out); } \
    static csgrs_status_t region_profiles(const csgrs_curve_region_t *region, region_profiles_type *out) { return csgrs_curve_region_##Name##_region_profiles(region, out); } \
    static void mesh_buffers_free(mesh_buffers_type value) { csgrs_triangle_mesh_buffers_##Name##_free(value); } \
    static void graphics_mesh_free(graphics_mesh_type value) { csgrs_graphics_mesh_##Name##_free(value); } \
    static void region_profiles_free(region_profiles_type value) { csgrs_region_profiles_##Name##_free(value); } \
  };

CSGRS_CPP_TRAITS(F32, f32, float, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_matrix4_f32_t, csgrs_aabb3_f32_t, csgrs_triangle_mesh_buffers_f32_t, csgrs_graphics_mesh_f32_t, csgrs_region_profiles_f32_t)
CSGRS_CPP_TRAITS(F64, f64, double, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_matrix4_f64_t, csgrs_aabb3_f64_t, csgrs_triangle_mesh_buffers_f64_t, csgrs_graphics_mesh_f64_t, csgrs_region_profiles_f64_t)
CSGRS_CPP_TRAITS(I128, i128, csgrs_i128_t, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_matrix4_i128_t, csgrs_aabb3_i128_t, csgrs_triangle_mesh_buffers_i128_t, csgrs_graphics_mesh_i128_t, csgrs_region_profiles_i128_t)
CSGRS_CPP_TRAITS(RawReal, real, const csgrs_real_t *, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_matrix4_real_t, csgrs_aabb3_real_t, csgrs_triangle_mesh_buffers_real_t, csgrs_graphics_mesh_real_t, csgrs_region_profiles_real_t)

template <class Traits> class TriangleMesh {
public:
  using scalar_type = typename Traits::scalar_type;
  using vec3_type = typename Traits::vec3_type;
  using matrix4_type = typename Traits::matrix4_type;
  using aabb_type = typename Traits::aabb_type;
  using mesh_buffers_type = typename Traits::mesh_buffers_type;
  using graphics_mesh_type = typename Traits::graphics_mesh_type;

  TriangleMesh() = default;
  explicit TriangleMesh(csgrs_triangle_mesh_t *handle) : handle_(handle) {}
  TriangleMesh(const TriangleMesh &) = delete;
  TriangleMesh &operator=(const TriangleMesh &) = delete;
  TriangleMesh(TriangleMesh &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  TriangleMesh &operator=(TriangleMesh &&other) noexcept {
    if (this != &other) {
      csgrs_triangle_mesh_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  ~TriangleMesh() { csgrs_triangle_mesh_free(handle_); }

  static TriangleMesh cube(scalar_type width) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cube(width, &out)); return TriangleMesh(out); }
  static TriangleMesh cuboid(scalar_type width, scalar_type length, scalar_type height) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cuboid(width, length, height, &out)); return TriangleMesh(out); }
  static TriangleMesh sphere(scalar_type radius, std::size_t segments, std::size_t stacks) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_sphere(radius, segments, stacks, &out)); return TriangleMesh(out); }
  static TriangleMesh cylinder(scalar_type radius, scalar_type height, std::size_t segments) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cylinder(radius, height, segments, &out)); return TriangleMesh(out); }
  static TriangleMesh polyhedron(const std::vector<vec3_type> &points, const std::vector<std::size_t> &face_indices, const std::vector<std::size_t> &face_offsets) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_polyhedron(points.data(), points.size(), face_indices.data(), face_indices.size(), face_offsets.data(), face_offsets.size(), &out)); return TriangleMesh(out); }

  TriangleMesh union_with(const TriangleMesh &other) const { return binary(other, Traits::mesh_union); }
  TriangleMesh difference(const TriangleMesh &other) const { return binary(other, Traits::mesh_difference); }
  TriangleMesh intersection(const TriangleMesh &other) const { return binary(other, Traits::mesh_intersection); }
  TriangleMesh xor_with(const TriangleMesh &other) const { return binary(other, Traits::mesh_xor); }
  TriangleMesh transform(matrix4_type matrix) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_transform(handle_, matrix, &out)); return TriangleMesh(out); }
  TriangleMesh translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_translate(handle_, x, y, z, &out)); return TriangleMesh(out); }
  TriangleMesh scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_scale(handle_, sx, sy, sz, &out)); return TriangleMesh(out); }
  TriangleMesh rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_rotate(handle_, x, y, z, &out)); return TriangleMesh(out); }
  TriangleMesh inverse() const { return unary(Traits::mesh_inverse); }
  TriangleMesh center() const { return unary(Traits::mesh_center); }
  TriangleMesh floating() const { return unary(Traits::mesh_float); }
  aabb_type bounding_box() const { aabb_type out{}; check(Traits::mesh_bounding_box(handle_, &out)); return out; }
  mesh_buffers_type vertices_and_indices() const { mesh_buffers_type out{}; check(Traits::mesh_vertices_and_indices(handle_, &out)); return out; }
  graphics_mesh_type graphics_mesh() const { graphics_mesh_type out{}; check(Traits::mesh_graphics_mesh(handle_, &out)); return out; }
  csgrs_triangle_mesh_t *get() const { return handle_; }

private:
  using binary_fn = csgrs_status_t (*)(const csgrs_triangle_mesh_t *, const csgrs_triangle_mesh_t *, csgrs_triangle_mesh_t **);
  using unary_fn = csgrs_status_t (*)(const csgrs_triangle_mesh_t *, csgrs_triangle_mesh_t **);
  TriangleMesh binary(const TriangleMesh &other, binary_fn fn) const { csgrs_triangle_mesh_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return TriangleMesh(out); }
  TriangleMesh unary(unary_fn fn) const { csgrs_triangle_mesh_t *out = nullptr; check(fn(handle_, &out)); return TriangleMesh(out); }
  csgrs_triangle_mesh_t *handle_ = nullptr;
};

template <class Traits> class CurveRegion {
public:
  using scalar_type = typename Traits::scalar_type;
  using vec2_type = typename Traits::vec2_type;
  using vec3_type = typename Traits::vec3_type;
  using matrix4_type = typename Traits::matrix4_type;
  using aabb_type = typename Traits::aabb_type;
  using region_profiles_type = typename Traits::region_profiles_type;

  CurveRegion() = default;
  explicit CurveRegion(csgrs_curve_region_t *handle) : handle_(handle) {}
  CurveRegion(const CurveRegion &) = delete;
  CurveRegion &operator=(const CurveRegion &) = delete;
  CurveRegion(CurveRegion &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  CurveRegion &operator=(CurveRegion &&other) noexcept {
    if (this != &other) {
      csgrs_curve_region_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  ~CurveRegion() { csgrs_curve_region_free(handle_); }

  static CurveRegion square(scalar_type width) { csgrs_curve_region_t *out = nullptr; check(Traits::region_square(width, &out)); return CurveRegion(out); }
  static CurveRegion rectangle(scalar_type width, scalar_type length) { csgrs_curve_region_t *out = nullptr; check(Traits::region_rectangle(width, length, &out)); return CurveRegion(out); }
  static CurveRegion circle(scalar_type radius, std::size_t segments) { csgrs_curve_region_t *out = nullptr; check(Traits::region_circle(radius, segments, &out)); return CurveRegion(out); }
  static CurveRegion polygon(const std::vector<vec2_type> &points) { csgrs_curve_region_t *out = nullptr; check(Traits::region_polygon(points.data(), points.size(), &out)); return CurveRegion(out); }

  CurveRegion union_with(const CurveRegion &other) const { return binary(other, Traits::region_union); }
  CurveRegion difference(const CurveRegion &other) const { return binary(other, Traits::region_difference); }
  CurveRegion intersection(const CurveRegion &other) const { return binary(other, Traits::region_intersection); }
  CurveRegion xor_with(const CurveRegion &other) const { return binary(other, Traits::region_xor); }
  CurveRegion transform(matrix4_type matrix) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_transform(handle_, matrix, &out)); return CurveRegion(out); }
  CurveRegion translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_translate(handle_, x, y, z, &out)); return CurveRegion(out); }
  CurveRegion scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_scale(handle_, sx, sy, sz, &out)); return CurveRegion(out); }
  CurveRegion rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_rotate(handle_, x, y, z, &out)); return CurveRegion(out); }
  aabb_type bounding_box() const { aabb_type out{}; check(Traits::region_bounding_box(handle_, &out)); return out; }
  TriangleMesh<Traits> extrude(scalar_type height) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_extrude(handle_, height, &out)); return TriangleMesh<Traits>(out); }
  TriangleMesh<Traits> extrude_vector(vec3_type direction) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_extrude_vector(handle_, direction, &out)); return TriangleMesh<Traits>(out); }
  TriangleMesh<Traits> revolve(scalar_type angle, std::size_t segments) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_revolve(handle_, angle, segments, &out)); return TriangleMesh<Traits>(out); }
  region_profiles_type region_profiles() const { region_profiles_type out{}; check(Traits::region_profiles(handle_, &out)); return out; }
  csgrs_curve_region_t *get() const { return handle_; }

private:
  using binary_fn = csgrs_status_t (*)(const csgrs_curve_region_t *, const csgrs_curve_region_t *, csgrs_curve_region_t **);
  CurveRegion binary(const CurveRegion &other, binary_fn fn) const { csgrs_curve_region_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return CurveRegion(out); }
  csgrs_curve_region_t *handle_ = nullptr;
};

using TriangleMeshF32 = TriangleMesh<F32>;
using TriangleMeshF64 = TriangleMesh<F64>;
using TriangleMeshI128 = TriangleMesh<I128>;
using TriangleMeshReal = TriangleMesh<RawReal>;
using CurveRegionF32 = CurveRegion<F32>;
using CurveRegionF64 = CurveRegion<F64>;
using CurveRegionI128 = CurveRegion<I128>;
using CurveRegionReal = CurveRegion<RawReal>;

} // namespace csgrs
