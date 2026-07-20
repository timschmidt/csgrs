#pragma once

#include "../../ffi/include/csgrs.h"

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
    static csgrs_status_t mesh_cube(scalar_type width, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_cube(width, out); } \
    static csgrs_status_t mesh_cuboid(scalar_type width, scalar_type length, scalar_type height, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_cuboid(width, length, height, out); } \
    static csgrs_status_t mesh_sphere(scalar_type radius, std::size_t segments, std::size_t stacks, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_sphere(radius, segments, stacks, out); } \
    static csgrs_status_t mesh_cylinder(scalar_type radius, scalar_type height, std::size_t segments, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_cylinder(radius, height, segments, out); } \
    static csgrs_status_t mesh_polyhedron(const vec3_type *points, std::size_t point_len, const std::size_t *face_indices, std::size_t face_index_len, const std::size_t *face_offsets, std::size_t face_offset_len, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_polyhedron(points, point_len, face_indices, face_index_len, face_offsets, face_offset_len, out); } \
    static csgrs_status_t mesh_union(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_union(lhs, rhs, out); } \
    static csgrs_status_t mesh_difference(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_difference(lhs, rhs, out); } \
    static csgrs_status_t mesh_intersection(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_intersection(lhs, rhs, out); } \
    static csgrs_status_t mesh_xor(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_xor(lhs, rhs, out); } \
    static csgrs_status_t mesh_transform(const csgrs_mesh_t *mesh, matrix4_type matrix, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_transform(mesh, matrix, out); } \
    static csgrs_status_t mesh_translate(const csgrs_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_translate(mesh, x, y, z, out); } \
    static csgrs_status_t mesh_scale(const csgrs_mesh_t *mesh, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_scale(mesh, sx, sy, sz, out); } \
    static csgrs_status_t mesh_rotate(const csgrs_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_rotate(mesh, x, y, z, out); } \
    static csgrs_status_t mesh_inverse(const csgrs_mesh_t *mesh, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_inverse(mesh, out); } \
    static csgrs_status_t mesh_center(const csgrs_mesh_t *mesh, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_center(mesh, out); } \
    static csgrs_status_t mesh_float(const csgrs_mesh_t *mesh, csgrs_mesh_t **out) { return csgrs_mesh_##Name##_float(mesh, out); } \
    static csgrs_status_t mesh_bounding_box(const csgrs_mesh_t *mesh, aabb_type *out) { return csgrs_mesh_##Name##_bounding_box(mesh, out); } \
    static csgrs_status_t mesh_vertices_and_indices(const csgrs_mesh_t *mesh, mesh_buffers_type *out) { return csgrs_mesh_##Name##_vertices_and_indices(mesh, out); } \
    static csgrs_status_t mesh_graphics_mesh(const csgrs_mesh_t *mesh, graphics_mesh_type *out) { return csgrs_mesh_##Name##_graphics_mesh(mesh, out); } \
    static csgrs_status_t profile_square(scalar_type width, csgrs_profile_t **out) { return csgrs_profile_##Name##_square(width, out); } \
    static csgrs_status_t profile_rectangle(scalar_type width, scalar_type length, csgrs_profile_t **out) { return csgrs_profile_##Name##_rectangle(width, length, out); } \
    static csgrs_status_t profile_circle(scalar_type radius, std::size_t segments, csgrs_profile_t **out) { return csgrs_profile_##Name##_circle(radius, segments, out); } \
    static csgrs_status_t profile_polygon(const vec2_type *points, std::size_t point_len, csgrs_profile_t **out) { return csgrs_profile_##Name##_polygon(points, point_len, out); } \
    static csgrs_status_t profile_union(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out) { return csgrs_profile_##Name##_union(lhs, rhs, out); } \
    static csgrs_status_t profile_difference(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out) { return csgrs_profile_##Name##_difference(lhs, rhs, out); } \
    static csgrs_status_t profile_intersection(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out) { return csgrs_profile_##Name##_intersection(lhs, rhs, out); } \
    static csgrs_status_t profile_xor(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out) { return csgrs_profile_##Name##_xor(lhs, rhs, out); } \
    static csgrs_status_t profile_transform(const csgrs_profile_t *profile, matrix4_type matrix, csgrs_profile_t **out) { return csgrs_profile_##Name##_transform(profile, matrix, out); } \
    static csgrs_status_t profile_translate(const csgrs_profile_t *profile, scalar_type x, scalar_type y, scalar_type z, csgrs_profile_t **out) { return csgrs_profile_##Name##_translate(profile, x, y, z, out); } \
    static csgrs_status_t profile_scale(const csgrs_profile_t *profile, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_profile_t **out) { return csgrs_profile_##Name##_scale(profile, sx, sy, sz, out); } \
    static csgrs_status_t profile_rotate(const csgrs_profile_t *profile, scalar_type x, scalar_type y, scalar_type z, csgrs_profile_t **out) { return csgrs_profile_##Name##_rotate(profile, x, y, z, out); } \
    static csgrs_status_t profile_bounding_box(const csgrs_profile_t *profile, aabb_type *out) { return csgrs_profile_##Name##_bounding_box(profile, out); } \
    static csgrs_status_t profile_extrude(const csgrs_profile_t *profile, scalar_type height, csgrs_mesh_t **out) { return csgrs_profile_##Name##_extrude(profile, height, out); } \
    static csgrs_status_t profile_extrude_vector(const csgrs_profile_t *profile, vec3_type direction, csgrs_mesh_t **out) { return csgrs_profile_##Name##_extrude_vector(profile, direction, out); } \
    static csgrs_status_t profile_revolve(const csgrs_profile_t *profile, scalar_type angle, std::size_t segments, csgrs_mesh_t **out) { return csgrs_profile_##Name##_revolve(profile, angle, segments, out); } \
    static csgrs_status_t profile_region_profiles(const csgrs_profile_t *profile, region_profiles_type *out) { return csgrs_profile_##Name##_region_profiles(profile, out); } \
    static void mesh_buffers_free(mesh_buffers_type value) { csgrs_mesh_buffers_##Name##_free(value); } \
    static void graphics_mesh_free(graphics_mesh_type value) { csgrs_graphics_mesh_##Name##_free(value); } \
    static void region_profiles_free(region_profiles_type value) { csgrs_region_profiles_##Name##_free(value); } \
  };

CSGRS_CPP_TRAITS(F32, f32, float, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_matrix4_f32_t, csgrs_aabb3_f32_t, csgrs_mesh_buffers_f32_t, csgrs_graphics_mesh_f32_t, csgrs_region_profiles_f32_t)
CSGRS_CPP_TRAITS(F64, f64, double, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_matrix4_f64_t, csgrs_aabb3_f64_t, csgrs_mesh_buffers_f64_t, csgrs_graphics_mesh_f64_t, csgrs_region_profiles_f64_t)
CSGRS_CPP_TRAITS(I128, i128, csgrs_i128_t, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_matrix4_i128_t, csgrs_aabb3_i128_t, csgrs_mesh_buffers_i128_t, csgrs_graphics_mesh_i128_t, csgrs_region_profiles_i128_t)
CSGRS_CPP_TRAITS(RawReal, real, const csgrs_real_t *, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_matrix4_real_t, csgrs_aabb3_real_t, csgrs_mesh_buffers_real_t, csgrs_graphics_mesh_real_t, csgrs_region_profiles_real_t)

template <class Traits> class Mesh {
public:
  using scalar_type = typename Traits::scalar_type;
  using vec3_type = typename Traits::vec3_type;
  using matrix4_type = typename Traits::matrix4_type;
  using aabb_type = typename Traits::aabb_type;
  using mesh_buffers_type = typename Traits::mesh_buffers_type;
  using graphics_mesh_type = typename Traits::graphics_mesh_type;

  Mesh() = default;
  explicit Mesh(csgrs_mesh_t *handle) : handle_(handle) {}
  Mesh(const Mesh &) = delete;
  Mesh &operator=(const Mesh &) = delete;
  Mesh(Mesh &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  Mesh &operator=(Mesh &&other) noexcept {
    if (this != &other) {
      csgrs_mesh_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  ~Mesh() { csgrs_mesh_free(handle_); }

  static Mesh cube(scalar_type width) { csgrs_mesh_t *out = nullptr; check(Traits::mesh_cube(width, &out)); return Mesh(out); }
  static Mesh cuboid(scalar_type width, scalar_type length, scalar_type height) { csgrs_mesh_t *out = nullptr; check(Traits::mesh_cuboid(width, length, height, &out)); return Mesh(out); }
  static Mesh sphere(scalar_type radius, std::size_t segments, std::size_t stacks) { csgrs_mesh_t *out = nullptr; check(Traits::mesh_sphere(radius, segments, stacks, &out)); return Mesh(out); }
  static Mesh cylinder(scalar_type radius, scalar_type height, std::size_t segments) { csgrs_mesh_t *out = nullptr; check(Traits::mesh_cylinder(radius, height, segments, &out)); return Mesh(out); }
  static Mesh polyhedron(const std::vector<vec3_type> &points, const std::vector<std::size_t> &face_indices, const std::vector<std::size_t> &face_offsets) { csgrs_mesh_t *out = nullptr; check(Traits::mesh_polyhedron(points.data(), points.size(), face_indices.data(), face_indices.size(), face_offsets.data(), face_offsets.size(), &out)); return Mesh(out); }

  Mesh union_with(const Mesh &other) const { return binary(other, Traits::mesh_union); }
  Mesh difference(const Mesh &other) const { return binary(other, Traits::mesh_difference); }
  Mesh intersection(const Mesh &other) const { return binary(other, Traits::mesh_intersection); }
  Mesh xor_with(const Mesh &other) const { return binary(other, Traits::mesh_xor); }
  Mesh transform(matrix4_type matrix) const { csgrs_mesh_t *out = nullptr; check(Traits::mesh_transform(handle_, matrix, &out)); return Mesh(out); }
  Mesh translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_mesh_t *out = nullptr; check(Traits::mesh_translate(handle_, x, y, z, &out)); return Mesh(out); }
  Mesh scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_mesh_t *out = nullptr; check(Traits::mesh_scale(handle_, sx, sy, sz, &out)); return Mesh(out); }
  Mesh rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_mesh_t *out = nullptr; check(Traits::mesh_rotate(handle_, x, y, z, &out)); return Mesh(out); }
  Mesh inverse() const { return unary(Traits::mesh_inverse); }
  Mesh center() const { return unary(Traits::mesh_center); }
  Mesh floating() const { return unary(Traits::mesh_float); }
  aabb_type bounding_box() const { aabb_type out{}; check(Traits::mesh_bounding_box(handle_, &out)); return out; }
  mesh_buffers_type vertices_and_indices() const { mesh_buffers_type out{}; check(Traits::mesh_vertices_and_indices(handle_, &out)); return out; }
  graphics_mesh_type graphics_mesh() const { graphics_mesh_type out{}; check(Traits::mesh_graphics_mesh(handle_, &out)); return out; }
  csgrs_mesh_t *get() const { return handle_; }

private:
  using binary_fn = csgrs_status_t (*)(const csgrs_mesh_t *, const csgrs_mesh_t *, csgrs_mesh_t **);
  using unary_fn = csgrs_status_t (*)(const csgrs_mesh_t *, csgrs_mesh_t **);
  Mesh binary(const Mesh &other, binary_fn fn) const { csgrs_mesh_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return Mesh(out); }
  Mesh unary(unary_fn fn) const { csgrs_mesh_t *out = nullptr; check(fn(handle_, &out)); return Mesh(out); }
  csgrs_mesh_t *handle_ = nullptr;
};

template <class Traits> class Profile {
public:
  using scalar_type = typename Traits::scalar_type;
  using vec2_type = typename Traits::vec2_type;
  using vec3_type = typename Traits::vec3_type;
  using matrix4_type = typename Traits::matrix4_type;
  using aabb_type = typename Traits::aabb_type;
  using region_profiles_type = typename Traits::region_profiles_type;

  Profile() = default;
  explicit Profile(csgrs_profile_t *handle) : handle_(handle) {}
  Profile(const Profile &) = delete;
  Profile &operator=(const Profile &) = delete;
  Profile(Profile &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  Profile &operator=(Profile &&other) noexcept {
    if (this != &other) {
      csgrs_profile_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  ~Profile() { csgrs_profile_free(handle_); }

  static Profile square(scalar_type width) { csgrs_profile_t *out = nullptr; check(Traits::profile_square(width, &out)); return Profile(out); }
  static Profile rectangle(scalar_type width, scalar_type length) { csgrs_profile_t *out = nullptr; check(Traits::profile_rectangle(width, length, &out)); return Profile(out); }
  static Profile circle(scalar_type radius, std::size_t segments) { csgrs_profile_t *out = nullptr; check(Traits::profile_circle(radius, segments, &out)); return Profile(out); }
  static Profile polygon(const std::vector<vec2_type> &points) { csgrs_profile_t *out = nullptr; check(Traits::profile_polygon(points.data(), points.size(), &out)); return Profile(out); }

  Profile union_with(const Profile &other) const { return binary(other, Traits::profile_union); }
  Profile difference(const Profile &other) const { return binary(other, Traits::profile_difference); }
  Profile intersection(const Profile &other) const { return binary(other, Traits::profile_intersection); }
  Profile xor_with(const Profile &other) const { return binary(other, Traits::profile_xor); }
  Profile transform(matrix4_type matrix) const { csgrs_profile_t *out = nullptr; check(Traits::profile_transform(handle_, matrix, &out)); return Profile(out); }
  Profile translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_profile_t *out = nullptr; check(Traits::profile_translate(handle_, x, y, z, &out)); return Profile(out); }
  Profile scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_profile_t *out = nullptr; check(Traits::profile_scale(handle_, sx, sy, sz, &out)); return Profile(out); }
  Profile rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_profile_t *out = nullptr; check(Traits::profile_rotate(handle_, x, y, z, &out)); return Profile(out); }
  aabb_type bounding_box() const { aabb_type out{}; check(Traits::profile_bounding_box(handle_, &out)); return out; }
  Mesh<Traits> extrude(scalar_type height) const { csgrs_mesh_t *out = nullptr; check(Traits::profile_extrude(handle_, height, &out)); return Mesh<Traits>(out); }
  Mesh<Traits> extrude_vector(vec3_type direction) const { csgrs_mesh_t *out = nullptr; check(Traits::profile_extrude_vector(handle_, direction, &out)); return Mesh<Traits>(out); }
  Mesh<Traits> revolve(scalar_type angle, std::size_t segments) const { csgrs_mesh_t *out = nullptr; check(Traits::profile_revolve(handle_, angle, segments, &out)); return Mesh<Traits>(out); }
  region_profiles_type region_profiles() const { region_profiles_type out{}; check(Traits::profile_region_profiles(handle_, &out)); return out; }
  csgrs_profile_t *get() const { return handle_; }

private:
  using binary_fn = csgrs_status_t (*)(const csgrs_profile_t *, const csgrs_profile_t *, csgrs_profile_t **);
  Profile binary(const Profile &other, binary_fn fn) const { csgrs_profile_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return Profile(out); }
  csgrs_profile_t *handle_ = nullptr;
};

using MeshF32 = Mesh<F32>;
using MeshF64 = Mesh<F64>;
using MeshI128 = Mesh<I128>;
using MeshReal = Mesh<RawReal>;
using ProfileF32 = Profile<F32>;
using ProfileF64 = Profile<F64>;
using ProfileI128 = Profile<I128>;
using ProfileReal = Profile<RawReal>;

} // namespace csgrs
