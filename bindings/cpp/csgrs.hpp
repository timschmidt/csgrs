/**
 * @file
 * C++17 ownership wrappers over csgrs-ffi, with geometry delegated to its C ABI.
 *
 * Real is copyable by native scalar cloning. Meshes and regions are move-only.
 * Default/moved-from wrappers contain null handles, not canonical empty geometry.
 * Raw-handle constructors adopt ownership without checking provenance or family;
 * callers must supply a compatible, uniquely owned library allocation.
 *
 * Failed status calls throw csgrs::error with the C category and copied diagnostic.
 * Bounds translate NO_BOUNDS into std::nullopt. Other raw buffer records are not
 * RAII owners: use Traits::mesh_buffers_free, Traits::graphics_mesh_free or
 * Traits::region_profiles_free. RawReal bounds require csgrs_aabb3_real_free_values.
 * Copying these descriptors or optionals does not duplicate native ownership.
 *
 * Scalar families control boundaries, not internal geometry. Float export is
 * lossy; integer export may fail; all region profiles pass through sampled
 * binary64 points even for RawReal. Wrappers add no policy selector, separate
 * local certainty receipt, total-work budget or cancellation mechanism. Mesh
 * construction history is queryable but does not certify analytic fidelity or
 * topology. See the C header for operation domains and pointer obligations.
 */
#pragma once

#include "../../crates/csgrs-ffi/include/csgrs.h"

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

/// C++ facade sharing the native C ABI's scalar, geometry and failure contracts.
namespace csgrs {

/// Exception retaining a C status code and an owned copy of its diagnostic text.
class error : public std::runtime_error {
public:
  /// Copy diagnostic text, substituting a fallback for null, and retain the supplied status.
  /// The default category is validation failure for wrapper-authored errors.
  explicit error(const char *message, csgrs_status_t status = CSGRS_STATUS_VALIDATION_FAILED)
      : std::runtime_error(message ? message : "csgrs error"), status_(status) {}
  /// Read the original native category without parsing its message.
  csgrs_status_t status() const noexcept { return status_; }
private:
  /// Native category retained alongside the runtime_error message.
  csgrs_status_t status_;
};

/// Throw on every status except OK, copying this thread's current ABI diagnostic.
///
/// Call immediately after the status-producing call on the same thread. Bounds
/// wrappers intercept NO_BOUNDS before using this generic checker.
inline void check(csgrs_status_t status) {
  if (status != CSGRS_STATUS_OK) {
    throw error(csgrs_last_error_message(), status);
  }
}

/// Owned opaque native scalar, with cloning copy semantics and transferring moves.
///
/// A default instance has no scalar handle; use zero() for numeric zero. Raw-handle
/// adoption requires unique valid ownership. get() only borrows that ownership.
class Real {
public:
  /// Create an empty owner containing null, not a numeric zero.
  Real() = default;
  /// Adopt a uniquely owned native scalar allocation, or null, without validating it.
  explicit Real(csgrs_real_t *handle) : handle_(handle) {}
  /// Clone a live scalar handle; copying an empty owner stays empty. Native failure throws.
  Real(const Real &other) {
    if (other.handle_) {
      check(csgrs_real_clone(other.handle_, &handle_));
    }
  }
  /// Transfer the handle and leave other empty; no scalar cloning occurs.
  Real(Real &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  /// Assign by swap and release the previous owner when the parameter is destroyed.
  ///
  /// The body is noexcept; copying the by-value argument can throw before entry.
  Real &operator=(Real other) noexcept {
    swap(other);
    return *this;
  }
  /// Release the scalar once; null is accepted by the C destructor.
  ~Real() { csgrs_real_free(handle_); }

  /// Allocate a native exact zero scalar, throwing on ABI failure.
  static Real zero() {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_zero(&out));
    return Real(out);
  }

  /// Allocate a native exact one scalar, throwing on ABI failure.
  static Real one() {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_one(&out));
    return Real(out);
  }

  /// Import the exact finite binary64 value; NaN/infinity throw.
  /// The value 0.1 is not rational one tenth.
  static Real from_f64(double value) {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_from_f64(value, &out));
    return Real(out);
  }

  /// Import the signed two's-complement value from its high/low words exactly.
  static Real from_i128(csgrs_i128_t value) {
    csgrs_real_t *out = nullptr;
    check(csgrs_real_from_i128(value, &out));
    return Real(out);
  }

  /// Project to a finite lossy binary64 value or throw; this is not an exactness certificate.
  double to_f64() const {
    double out = 0.0;
    check(csgrs_real_to_f64(handle_, &out));
    return out;
  }

  /// Require a recognized in-range integer and export its two words, or throw.
  csgrs_i128_t to_i128() const {
    csgrs_i128_t out{};
    check(csgrs_real_to_i128(handle_, &out));
    return out;
  }

  /// Borrow the raw handle (possibly null); do not free it or outlive this owner.
  const csgrs_real_t *get() const { return handle_; }
  /// Transfer ownership to the caller and leave this wrapper empty; caller must free once.
  csgrs_real_t *release() { return std::exchange(handle_, nullptr); }
  /// Exchange scalar ownership without allocation or cloning.
  void swap(Real &other) noexcept { std::swap(handle_, other.handle_); }

private:
  /// Unique owned native scalar allocation, or null for an empty/moved-from owner.
  csgrs_real_t *handle_ = nullptr;
};

/// Binary32 scalar/layout strategy for native calls.
struct F32;
/// Binary64 scalar/layout strategy for native calls.
struct F64;
/// Split-word signed integer scalar/layout strategy for native calls.
struct I128;
/// Borrowed opaque scalar input and owned opaque scalar output strategy.
struct RawReal;

/// Generate layout aliases and direct C forwarding functions for one scalar family.
///
/// These functions return statuses without throwing and retain all C pointer and
/// ownership obligations. The wrapper classes apply check() and own geometry handles.
#define CSGRS_CPP_TRAITS(Traits, Name, ScalarType, Vec2Type, Vec3Type, MatrixType, AabbType, MeshBuffersType, GraphicsMeshType, RegionProfilesType) \
  /** One family's ABI layouts and status-returning forwarders. */ \
  struct Traits { \
    /** C scalar input representation for this family. */ \
    using scalar_type = ScalarType; \
    /** C XY coordinate layout; RawReal components are borrowed on input. */ \
    using vec2_type = Vec2Type; \
    /** C XYZ coordinate layout; RawReal components are borrowed on input. */ \
    using vec3_type = Vec3Type; \
    /** C row-major 4x4 coefficient layout. */ \
    using matrix4_type = MatrixType; \
    /** C bounds record; RawReal success owns six manually freed handles. */ \
    using aabb_type = AabbType; \
    /** C owned position/triangle arrays, requiring the matching manual free. */ \
    using mesh_buffers_type = MeshBuffersType; \
    /** C owned graphics vertex/index arrays, requiring the matching manual free. */ \
    using graphics_mesh_type = GraphicsMeshType; \
    /** C owned sampled profile/offset arrays, requiring the matching manual free. */ \
    using region_profiles_type = RegionProfilesType; \
    /** Construct `[0,width]^3` with strict nonnegative dimension validation. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_cube(scalar_type width, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cube(width, out); } \
    /** Construct an origin-anchored box with XYZ dimensions `width`, `length`, `height`. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_cuboid(scalar_type width, scalar_type length, scalar_type height, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cuboid(width, length, height, out); } \
    /** Construct the sampled triangle sphere centered at the origin, with Y-axis poles. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_sphere(scalar_type radius, std::size_t segments, std::size_t stacks, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_sphere(radius, segments, stacks, out); } \
    /** Construct a sampled capped cylinder from Z zero to `height`. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_cylinder(scalar_type radius, scalar_type height, std::size_t segments, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_cylinder(radius, height, segments, out); } \
    /** Triangulate indexed planar polygon faces with authored winding under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_polyhedron(const vec3_type *points, std::size_t point_len, const std::size_t *face_indices, std::size_t face_index_len, const std::size_t *face_offsets, std::size_t face_offset_len, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_polyhedron(points, point_len, face_indices, face_index_len, face_offsets, face_offset_len, out); } \
    /** Return the strict regularized union of two same-family meshes. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_union(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_union(lhs, rhs, out); } \
    /** Return the strict regularized material of `lhs` outside `rhs`. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_difference(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_difference(lhs, rhs, out); } \
    /** Return the strict regularized common material of two same-family meshes. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_intersection(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_intersection(lhs, rhs, out); } \
    /** Return the strict regularized material belonging to exactly one mesh operand. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_xor(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_xor(lhs, rhs, out); } \
    /** Apply an imported row-major homogeneous matrix through the checked mesh transform. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_transform(const csgrs_triangle_mesh_t *mesh, matrix4_type matrix, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_transform(mesh, matrix, out); } \
    /** Translate native mesh coordinates by the imported XYZ displacement. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_translate(const csgrs_triangle_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_translate(mesh, x, y, z, out); } \
    /** Scale XYZ independently under strict predicates and correct winding for reflections. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_scale(const csgrs_triangle_mesh_t *mesh, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_scale(mesh, sx, sy, sz, out); } \
    /** Rotate about X, then Y, then Z using imported angles in degrees. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_rotate(const csgrs_triangle_mesh_t *mesh, scalar_type x, scalar_type y, scalar_type z, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_rotate(mesh, x, y, z, out); } \
    /** Return a mesh with every triangle winding reversed and positions preserved. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_inverse(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_inverse(mesh, out); } \
    /** Move the native bounds center to the origin under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_center(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_center(mesh, out); } \
    /** Move the native minimum Z bound to zero under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_float(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out) { return csgrs_triangle_mesh_##Name##_float(mesh, out); } \
    /** Query strict native extrema, then export them through this scalar family. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_bounding_box(const csgrs_triangle_mesh_t *mesh, aabb_type *out) { return csgrs_triangle_mesh_##Name##_bounding_box(mesh, out); } \
    /** Export indexed positions and ordered triangle rows into owned arrays. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_vertices_and_indices(const csgrs_triangle_mesh_t *mesh, mesh_buffers_type *out) { return csgrs_triangle_mesh_##Name##_vertices_and_indices(mesh, out); } \
    /** Export positions, certified native normals and flat indices into owned graphics arrays. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t mesh_graphics_mesh(const csgrs_triangle_mesh_t *mesh, graphics_mesh_type *out) { return csgrs_triangle_mesh_##Name##_graphics_mesh(mesh, out); } \
    /** Construct an origin-anchored XY square with strict nonnegative width validation. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_square(scalar_type width, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_square(width, out); } \
    /** Construct an origin-anchored XY rectangle with nonnegative width and length. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_rectangle(scalar_type width, scalar_type length, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_rectangle(width, length, out); } \
    /** Construct a regular sampled XY polygon centered at the origin. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_circle(scalar_type radius, std::size_t segments, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_circle(radius, segments, out); } \
    /** Construct a filled XY polygon from a valid ring of at least three vertices. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_polygon(const vec2_type *points, std::size_t point_len, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_polygon(points, point_len, out); } \
    /** Return the regularized union of same-family filled regions. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_union(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_union(lhs, rhs, out); } \
    /** Return the regularized part of `lhs` outside `rhs`. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_difference(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_difference(lhs, rhs, out); } \
    /** Return the regularized common material of two same-family filled regions. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_intersection(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_intersection(lhs, rhs, out); } \
    /** Return regularized material belonging to exactly one filled-region operand. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_xor(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_xor(lhs, rhs, out); } \
    /** Apply a row-major homogeneous transform with a nonsingular affine XY restriction. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_transform(const csgrs_curve_region_t *region, matrix4_type matrix, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_transform(region, matrix, out); } \
    /** Translate the planar region under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_translate(const csgrs_curve_region_t *region, scalar_type x, scalar_type y, scalar_type z, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_translate(region, x, y, z, out); } \
    /** Scale the planar XY image under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_scale(const csgrs_curve_region_t *region, scalar_type sx, scalar_type sy, scalar_type sz, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_scale(region, sx, sy, sz, out); } \
    /** Apply X, then Y, then Z rotations in degrees when the image remains in XY. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_rotate(const csgrs_curve_region_t *region, scalar_type x, scalar_type y, scalar_type z, csgrs_curve_region_t **out) { return csgrs_curve_region_##Name##_rotate(region, x, y, z, out); } \
    /** Query strict native region bounds and export the endpoints through this family. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_bounding_box(const csgrs_curve_region_t *region, aabb_type *out) { return csgrs_curve_region_##Name##_bounding_box(region, out); } \
    /** Extrude the region along Z by a nonzero signed height under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_extrude(const csgrs_curve_region_t *region, scalar_type height, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_extrude(region, height, out); } \
    /** Extrude along the imported vector, requiring a nonzero Z component. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_extrude_vector(const csgrs_curve_region_t *region, vec3_type direction, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_extrude_vector(region, direction, out); } \
    /** Build a segmented swept boundary about Y, with profile X as signed radius. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_revolve(const csgrs_curve_region_t *region, scalar_type angle, std::size_t segments, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_revolve(region, angle, segments, out); } \
    /** Regularize the union of segmented revolution pieces about Y. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_revolve_solid(const csgrs_curve_region_t *region, scalar_type angle, std::size_t segments, csgrs_triangle_mesh_t **out) { return csgrs_curve_region_##Name##_revolve_solid(region, angle, segments, out); } \
    /** Export sampled exterior/hole rings with sentinel offsets under strict predicates. C pointer/ownership obligations apply; no exception translation. */ \
    static csgrs_status_t region_profiles(const csgrs_curve_region_t *region, region_profiles_type *out) { return csgrs_curve_region_##Name##_region_profiles(region, out); } \
    /** Release the primitive-family vertex and triangle arrays. C pointer/ownership obligations apply; no exception translation. */ \
    static void mesh_buffers_free(mesh_buffers_type value) { csgrs_triangle_mesh_buffers_##Name##_free(value); } \
    /** Release the primitive-family graphics vertex and flat index arrays. C pointer/ownership obligations apply; no exception translation. */ \
    static void graphics_mesh_free(graphics_mesh_type value) { csgrs_graphics_mesh_##Name##_free(value); } \
    /** Release primitive-family points and both profile offset arrays. C pointer/ownership obligations apply; no exception translation. */ \
    static void region_profiles_free(region_profiles_type value) { csgrs_region_profiles_##Name##_free(value); } \
  };

CSGRS_CPP_TRAITS(F32, f32, float, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_matrix4_f32_t, csgrs_aabb3_f32_t, csgrs_triangle_mesh_buffers_f32_t, csgrs_graphics_mesh_f32_t, csgrs_region_profiles_f32_t)
CSGRS_CPP_TRAITS(F64, f64, double, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_matrix4_f64_t, csgrs_aabb3_f64_t, csgrs_triangle_mesh_buffers_f64_t, csgrs_graphics_mesh_f64_t, csgrs_region_profiles_f64_t)
CSGRS_CPP_TRAITS(I128, i128, csgrs_i128_t, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_matrix4_i128_t, csgrs_aabb3_i128_t, csgrs_triangle_mesh_buffers_i128_t, csgrs_graphics_mesh_i128_t, csgrs_region_profiles_i128_t)
CSGRS_CPP_TRAITS(RawReal, real, const csgrs_real_t *, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_matrix4_real_t, csgrs_aabb3_real_t, csgrs_triangle_mesh_buffers_real_t, csgrs_graphics_mesh_real_t, csgrs_region_profiles_real_t)

/// Move-only native mesh owner parameterized by a boundary scalar family.
///
/// Operations borrow inputs and return new owners or raw output descriptors.
/// Default/moved-from objects hold null, not canonical empty geometry. Raw-handle
/// adoption does not verify family; operations diagnose a mismatched live handle.
template <class Traits> class TriangleMesh {
public:
  /// C scalar input representation for this family.
  using scalar_type = typename Traits::scalar_type;
  /// C XYZ coordinate layout; RawReal components are borrowed on input.
  using vec3_type = typename Traits::vec3_type;
  /// C row-major 4x4 coefficient layout.
  using matrix4_type = typename Traits::matrix4_type;
  /// C bounds record; RawReal success owns six manually freed handles.
  using aabb_type = typename Traits::aabb_type;
  /// C owned position/triangle arrays, requiring the matching manual free.
  using mesh_buffers_type = typename Traits::mesh_buffers_type;
  /// C owned graphics vertex/index arrays, requiring the matching manual free.
  using graphics_mesh_type = typename Traits::graphics_mesh_type;

  /// Create a null owner, not an empty geometric operand.
  TriangleMesh() = default;
  /// Adopt one compatible native mesh allocation (or null), without validating ownership/family.
  explicit TriangleMesh(csgrs_triangle_mesh_t *handle) : handle_(handle) {}
  /// Disable copying so two wrappers cannot free one native allocation.
  TriangleMesh(const TriangleMesh &) = delete;
  /// Disable copy assignment; geometry ownership transfers only by moves.
  TriangleMesh &operator=(const TriangleMesh &) = delete;
  /// Transfer ownership and leave other with a null handle.
  TriangleMesh(TriangleMesh &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  /// Free the old owner, transfer the new one and null other; self-move is unchanged.
  TriangleMesh &operator=(TriangleMesh &&other) noexcept {
    if (this != &other) {
      csgrs_triangle_mesh_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  /// Free the owned native mesh once; a null handle is a no-op.
  ~TriangleMesh() { csgrs_triangle_mesh_free(handle_); }

  /// Construct `[0,width]^3` with strict nonnegative dimension validation.
  /// Native failure throws error; success returns a new geometry owner.
  static TriangleMesh cube(scalar_type width) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cube(width, &out)); return TriangleMesh(out); }
  /// Construct an origin-anchored box with XYZ dimensions `width`, `length`, `height`.
  /// Native failure throws error; success returns a new geometry owner.
  static TriangleMesh cuboid(scalar_type width, scalar_type length, scalar_type height) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cuboid(width, length, height, &out)); return TriangleMesh(out); }
  /// Construct the sampled triangle sphere centered at the origin, with Y-axis poles.
  /// Native failure throws error; success returns a new geometry owner.
  static TriangleMesh sphere(scalar_type radius, std::size_t segments, std::size_t stacks) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_sphere(radius, segments, stacks, &out)); return TriangleMesh(out); }
  /// Construct a sampled capped cylinder from Z zero to `height`.
  /// Native failure throws error; success returns a new geometry owner.
  static TriangleMesh cylinder(scalar_type radius, scalar_type height, std::size_t segments) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_cylinder(radius, height, segments, &out)); return TriangleMesh(out); }
  /// Triangulate indexed planar polygon faces with authored winding under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  static TriangleMesh polyhedron(const std::vector<vec3_type> &points, const std::vector<std::size_t> &face_indices, const std::vector<std::size_t> &face_offsets) { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_polyhedron(points.data(), points.size(), face_indices.data(), face_indices.size(), face_offsets.data(), face_offsets.size(), &out)); return TriangleMesh(out); }

  /// Return the strict regularized union of two same-family meshes.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh union_with(const TriangleMesh &other) const { return binary(other, Traits::mesh_union); }
  /// Return the strict regularized material of `lhs` outside `rhs`.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh difference(const TriangleMesh &other) const { return binary(other, Traits::mesh_difference); }
  /// Return the strict regularized common material of two same-family meshes.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh intersection(const TriangleMesh &other) const { return binary(other, Traits::mesh_intersection); }
  /// Return the strict regularized material belonging to exactly one mesh operand.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh xor_with(const TriangleMesh &other) const { return binary(other, Traits::mesh_xor); }
  /// Apply an imported row-major homogeneous matrix through the checked mesh transform.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh transform(matrix4_type matrix) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_transform(handle_, matrix, &out)); return TriangleMesh(out); }
  /// Translate native mesh coordinates by the imported XYZ displacement.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_translate(handle_, x, y, z, &out)); return TriangleMesh(out); }
  /// Scale XYZ independently under strict predicates and correct winding for reflections.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_scale(handle_, sx, sy, sz, &out)); return TriangleMesh(out); }
  /// Rotate about X, then Y, then Z using imported angles in degrees.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::mesh_rotate(handle_, x, y, z, &out)); return TriangleMesh(out); }
  /// Return a mesh with every triangle winding reversed and positions preserved.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh inverse() const { return unary(Traits::mesh_inverse); }
  /// Move the native bounds center to the origin under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh center() const { return unary(Traits::mesh_center); }
  /// Move the native minimum Z bound to zero under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh floating() const { return unary(Traits::mesh_float); }
  /// Read retained construction-history flags; missing history cannot be certified
  /// by a false approximate flag, and this is not a topology/fidelity certificate.
  csgrs_construction_evidence_t construction_evidence() const {
    csgrs_construction_evidence_t out{};
    check(csgrs_triangle_mesh_construction_evidence(handle_, &out));
    return out;
  }
  /// Return exported bounds, std::nullopt for completed absence, or throw on failure.
  ///
  /// Float bounds are not certified outward enclosures. RawReal bounds own six
  /// handles: call csgrs_aabb3_real_free_values on the present record exactly once;
  /// std::optional itself does not free them.
  std::optional<aabb_type> bounding_box() const {
    aabb_type out{};
    const auto status = Traits::mesh_bounding_box(handle_, &out);
    if (status == CSGRS_STATUS_NO_BOUNDS) return std::nullopt;
    check(status);
    return out;
  }
  /// Export indexed positions and ordered triangle rows into owned arrays.
  /// Caller must release the descriptor with Traits::mesh_buffers_free; it has no RAII destructor.
  mesh_buffers_type vertices_and_indices() const { mesh_buffers_type out{}; check(Traits::mesh_vertices_and_indices(handle_, &out)); return out; }
  /// Export positions, certified native normals and flat indices into owned graphics arrays.
  /// Caller must release the descriptor with Traits::graphics_mesh_free; it has no RAII destructor.
  graphics_mesh_type graphics_mesh() const { graphics_mesh_type out{}; check(Traits::mesh_graphics_mesh(handle_, &out)); return out; }
  /// Borrow the raw native handle, possibly null; do not free it or outlive this owner.
  csgrs_triangle_mesh_t *get() const { return handle_; }

private:
  /// Same-family C binary operation returning a new handle via an output slot.
  using binary_fn = csgrs_status_t (*)(const csgrs_triangle_mesh_t *, const csgrs_triangle_mesh_t *, csgrs_triangle_mesh_t **);
  /// Same-family C unary operation returning a new handle via an output slot.
  using unary_fn = csgrs_status_t (*)(const csgrs_triangle_mesh_t *, csgrs_triangle_mesh_t **);
  /// Call the binary C operation on borrowed handles, throw on failure and adopt its new result.
  TriangleMesh binary(const TriangleMesh &other, binary_fn fn) const { csgrs_triangle_mesh_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return TriangleMesh(out); }
  /// Call the unary C operation on borrowed handles, throw on failure and adopt its new result.
  TriangleMesh unary(unary_fn fn) const { csgrs_triangle_mesh_t *out = nullptr; check(fn(handle_, &out)); return TriangleMesh(out); }
  /// Unique native mesh owner, or null after default construction/move.
  csgrs_triangle_mesh_t *handle_ = nullptr;
};

/// Move-only native region owner parameterized by a boundary scalar family.
///
/// Operations borrow inputs and return new owners or raw output descriptors.
/// Default/moved-from objects hold null, not canonical empty geometry. Raw-handle
/// adoption does not verify family; operations diagnose a mismatched live handle.
template <class Traits> class CurveRegion {
public:
  /// C scalar input representation for this family.
  using scalar_type = typename Traits::scalar_type;
  /// C XY coordinate layout; RawReal components are borrowed on input.
  using vec2_type = typename Traits::vec2_type;
  /// C XYZ coordinate layout; RawReal components are borrowed on input.
  using vec3_type = typename Traits::vec3_type;
  /// C row-major 4x4 coefficient layout.
  using matrix4_type = typename Traits::matrix4_type;
  /// C bounds record; RawReal success owns six manually freed handles.
  using aabb_type = typename Traits::aabb_type;
  /// C owned sampled profile/offset arrays, requiring the matching manual free.
  using region_profiles_type = typename Traits::region_profiles_type;

  /// Create a null owner, not an empty geometric operand.
  CurveRegion() = default;
  /// Adopt one compatible native region allocation (or null), without validating ownership/family.
  explicit CurveRegion(csgrs_curve_region_t *handle) : handle_(handle) {}
  /// Disable copying so two wrappers cannot free one native allocation.
  CurveRegion(const CurveRegion &) = delete;
  /// Disable copy assignment; geometry ownership transfers only by moves.
  CurveRegion &operator=(const CurveRegion &) = delete;
  /// Transfer ownership and leave other with a null handle.
  CurveRegion(CurveRegion &&other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
  /// Free the old owner, transfer the new one and null other; self-move is unchanged.
  CurveRegion &operator=(CurveRegion &&other) noexcept {
    if (this != &other) {
      csgrs_curve_region_free(handle_);
      handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
  }
  /// Free the owned native region once; a null handle is a no-op.
  ~CurveRegion() { csgrs_curve_region_free(handle_); }

  /// Construct an origin-anchored XY square with strict nonnegative width validation.
  /// Native failure throws error; success returns a new geometry owner.
  static CurveRegion square(scalar_type width) { csgrs_curve_region_t *out = nullptr; check(Traits::region_square(width, &out)); return CurveRegion(out); }
  /// Construct an origin-anchored XY rectangle with nonnegative width and length.
  /// Native failure throws error; success returns a new geometry owner.
  static CurveRegion rectangle(scalar_type width, scalar_type length) { csgrs_curve_region_t *out = nullptr; check(Traits::region_rectangle(width, length, &out)); return CurveRegion(out); }
  /// Construct a regular sampled XY polygon centered at the origin.
  /// Native failure throws error; success returns a new geometry owner.
  static CurveRegion circle(scalar_type radius, std::size_t segments) { csgrs_curve_region_t *out = nullptr; check(Traits::region_circle(radius, segments, &out)); return CurveRegion(out); }
  /// Construct a filled XY polygon from a valid ring of at least three vertices.
  /// Native failure throws error; success returns a new geometry owner.
  static CurveRegion polygon(const std::vector<vec2_type> &points) { csgrs_curve_region_t *out = nullptr; check(Traits::region_polygon(points.data(), points.size(), &out)); return CurveRegion(out); }

  /// Return the regularized union of same-family filled regions.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion union_with(const CurveRegion &other) const { return binary(other, Traits::region_union); }
  /// Return the regularized part of `lhs` outside `rhs`.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion difference(const CurveRegion &other) const { return binary(other, Traits::region_difference); }
  /// Return the regularized common material of two same-family filled regions.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion intersection(const CurveRegion &other) const { return binary(other, Traits::region_intersection); }
  /// Return regularized material belonging to exactly one filled-region operand.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion xor_with(const CurveRegion &other) const { return binary(other, Traits::region_xor); }
  /// Apply a row-major homogeneous transform with a nonsingular affine XY restriction.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion transform(matrix4_type matrix) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_transform(handle_, matrix, &out)); return CurveRegion(out); }
  /// Translate the planar region under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion translate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_translate(handle_, x, y, z, &out)); return CurveRegion(out); }
  /// Scale the planar XY image under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion scale(scalar_type sx, scalar_type sy, scalar_type sz) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_scale(handle_, sx, sy, sz, &out)); return CurveRegion(out); }
  /// Apply X, then Y, then Z rotations in degrees when the image remains in XY.
  /// Native failure throws error; success returns a new geometry owner.
  CurveRegion rotate(scalar_type x, scalar_type y, scalar_type z) const { csgrs_curve_region_t *out = nullptr; check(Traits::region_rotate(handle_, x, y, z, &out)); return CurveRegion(out); }
  /// Return exported bounds, std::nullopt for completed absence, or throw on failure.
  ///
  /// Float bounds are not certified outward enclosures. RawReal bounds own six
  /// handles: call csgrs_aabb3_real_free_values on the present record exactly once;
  /// std::optional itself does not free them.
  std::optional<aabb_type> bounding_box() const {
    aabb_type out{};
    const auto status = Traits::region_bounding_box(handle_, &out);
    if (status == CSGRS_STATUS_NO_BOUNDS) return std::nullopt;
    check(status);
    return out;
  }
  /// Extrude the region along Z by a nonzero signed height under strict predicates.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh<Traits> extrude(scalar_type height) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_extrude(handle_, height, &out)); return TriangleMesh<Traits>(out); }
  /// Extrude along the imported vector, requiring a nonzero Z component.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh<Traits> extrude_vector(vec3_type direction) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_extrude_vector(handle_, direction, &out)); return TriangleMesh<Traits>(out); }
  /// Build a segmented swept boundary about Y, with profile X as signed radius.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh<Traits> revolve(scalar_type angle, std::size_t segments) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_revolve(handle_, angle, segments, &out)); return TriangleMesh<Traits>(out); }
  /// Regularize the union of segmented revolution pieces about Y.
  /// Native failure throws error; success returns a new geometry owner.
  TriangleMesh<Traits> revolve_solid(scalar_type angle, std::size_t segments) const { csgrs_triangle_mesh_t *out = nullptr; check(Traits::region_revolve_solid(handle_, angle, segments, &out)); return TriangleMesh<Traits>(out); }
  /// Export sampled exterior/hole rings with sentinel offsets under strict predicates.
  /// Even RawReal contains projected points. Caller must use Traits::region_profiles_free.
  region_profiles_type region_profiles() const { region_profiles_type out{}; check(Traits::region_profiles(handle_, &out)); return out; }
  /// Borrow the raw native handle, possibly null; do not free it or outlive this owner.
  csgrs_curve_region_t *get() const { return handle_; }

private:
  /// Same-family C binary operation returning a new handle via an output slot.
  using binary_fn = csgrs_status_t (*)(const csgrs_curve_region_t *, const csgrs_curve_region_t *, csgrs_curve_region_t **);
  /// Call the binary C operation on borrowed handles, throw on failure and adopt its new result.
  CurveRegion binary(const CurveRegion &other, binary_fn fn) const { csgrs_curve_region_t *out = nullptr; check(fn(handle_, other.handle_, &out)); return CurveRegion(out); }
  /// Unique native region owner, or null after default construction/move.
  csgrs_curve_region_t *handle_ = nullptr;
};

/// Mesh owner using the F32 boundary scalar strategy.
using TriangleMeshF32 = TriangleMesh<F32>;
/// Mesh owner using the F64 boundary scalar strategy.
using TriangleMeshF64 = TriangleMesh<F64>;
/// Mesh owner using the I128 boundary scalar strategy.
using TriangleMeshI128 = TriangleMesh<I128>;
/// Mesh owner using the Real boundary scalar strategy.
using TriangleMeshReal = TriangleMesh<RawReal>;
/// Filled-region owner using the F32 boundary scalar strategy.
using CurveRegionF32 = CurveRegion<F32>;
/// Filled-region owner using the F64 boundary scalar strategy.
using CurveRegionF64 = CurveRegion<F64>;
/// Filled-region owner using the I128 boundary scalar strategy.
using CurveRegionI128 = CurveRegion<I128>;
/// Filled-region owner using the Real boundary scalar strategy.
using CurveRegionReal = CurveRegion<RawReal>;

} // namespace csgrs
