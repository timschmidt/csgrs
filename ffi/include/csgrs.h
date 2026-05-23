#ifndef CSGRS_H
#define CSGRS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum csgrs_status_t {
  CSGRS_STATUS_OK = 0,
  CSGRS_STATUS_NULL_POINTER = 1,
  CSGRS_STATUS_WRONG_SCALAR_FAMILY = 2,
  CSGRS_STATUS_CONVERSION_FAILED = 3,
  CSGRS_STATUS_VALIDATION_FAILED = 4,
  CSGRS_STATUS_PANIC = 255,
} csgrs_status_t;

typedef enum csgrs_scalar_family_t {
  CSGRS_SCALAR_F32 = 1,
  CSGRS_SCALAR_F64 = 2,
  CSGRS_SCALAR_I128 = 3,
  CSGRS_SCALAR_REAL = 4,
} csgrs_scalar_family_t;

typedef struct csgrs_real_t csgrs_real_t;
typedef struct csgrs_mesh_t csgrs_mesh_t;
typedef struct csgrs_profile_t csgrs_profile_t;

typedef struct csgrs_i128_t {
  int64_t hi;
  uint64_t lo;
} csgrs_i128_t;

typedef struct csgrs_vec2_f32_t { float x, y; } csgrs_vec2_f32_t;
typedef struct csgrs_vec3_f32_t { float x, y, z; } csgrs_vec3_f32_t;
typedef struct csgrs_vec2_f64_t { double x, y; } csgrs_vec2_f64_t;
typedef struct csgrs_vec3_f64_t { double x, y, z; } csgrs_vec3_f64_t;
typedef struct csgrs_vec2_i128_t { csgrs_i128_t x, y; } csgrs_vec2_i128_t;
typedef struct csgrs_vec3_i128_t { csgrs_i128_t x, y, z; } csgrs_vec3_i128_t;
typedef struct csgrs_vec2_real_t { csgrs_real_t *x, *y; } csgrs_vec2_real_t;
typedef struct csgrs_vec3_real_t { csgrs_real_t *x, *y, *z; } csgrs_vec3_real_t;

typedef struct csgrs_triangle_u32_t { uint32_t a, b, c; } csgrs_triangle_u32_t;

typedef struct csgrs_aabb3_f32_t { csgrs_vec3_f32_t mins, maxs; } csgrs_aabb3_f32_t;
typedef struct csgrs_aabb3_f64_t { csgrs_vec3_f64_t mins, maxs; } csgrs_aabb3_f64_t;
typedef struct csgrs_aabb3_i128_t { csgrs_vec3_i128_t mins, maxs; } csgrs_aabb3_i128_t;
typedef struct csgrs_aabb3_real_t { csgrs_vec3_real_t mins, maxs; } csgrs_aabb3_real_t;

typedef struct csgrs_matrix4_f32_t { float values[16]; } csgrs_matrix4_f32_t;
typedef struct csgrs_matrix4_f64_t { double values[16]; } csgrs_matrix4_f64_t;
typedef struct csgrs_matrix4_i128_t { csgrs_i128_t values[16]; } csgrs_matrix4_i128_t;
typedef struct csgrs_matrix4_real_t { const csgrs_real_t *values[16]; } csgrs_matrix4_real_t;

typedef struct csgrs_graphics_vertex_f32_t {
  csgrs_vec3_f32_t position;
  csgrs_vec3_f32_t normal;
} csgrs_graphics_vertex_f32_t;

typedef struct csgrs_graphics_vertex_f64_t {
  csgrs_vec3_f64_t position;
  csgrs_vec3_f64_t normal;
} csgrs_graphics_vertex_f64_t;

typedef struct csgrs_graphics_vertex_i128_t {
  csgrs_vec3_i128_t position;
  csgrs_vec3_i128_t normal;
} csgrs_graphics_vertex_i128_t;

typedef struct csgrs_graphics_vertex_real_t {
  csgrs_vec3_real_t position;
  csgrs_vec3_real_t normal;
} csgrs_graphics_vertex_real_t;

#define CSGRS_DECLARE_BUFFERS(Name, Vec2, Vec3, GraphicsVertex) \
  typedef struct csgrs_mesh_buffers_##Name##_t { \
    Vec3 *vertices; \
    size_t vertex_len; \
    csgrs_triangle_u32_t *indices; \
    size_t index_len; \
  } csgrs_mesh_buffers_##Name##_t; \
  typedef struct csgrs_graphics_mesh_##Name##_t { \
    GraphicsVertex *vertices; \
    size_t vertex_len; \
    uint32_t *indices; \
    size_t index_len; \
  } csgrs_graphics_mesh_##Name##_t; \
  typedef struct csgrs_region_profiles_##Name##_t { \
    Vec2 *points; \
    size_t point_len; \
    size_t *ring_offsets; \
    size_t ring_offset_len; \
    size_t *profile_offsets; \
    size_t profile_offset_len; \
  } csgrs_region_profiles_##Name##_t;

CSGRS_DECLARE_BUFFERS(f32, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_graphics_vertex_f32_t)
CSGRS_DECLARE_BUFFERS(f64, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_graphics_vertex_f64_t)
CSGRS_DECLARE_BUFFERS(i128, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_graphics_vertex_i128_t)
CSGRS_DECLARE_BUFFERS(real, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_graphics_vertex_real_t)

const char *csgrs_last_error_message(void);

csgrs_status_t csgrs_real_zero(csgrs_real_t **out);
csgrs_status_t csgrs_real_one(csgrs_real_t **out);
csgrs_status_t csgrs_real_from_f64(double value, csgrs_real_t **out);
csgrs_status_t csgrs_real_from_i128(csgrs_i128_t value, csgrs_real_t **out);
csgrs_status_t csgrs_real_from_i128_decimal(const char *value, csgrs_real_t **out);
csgrs_status_t csgrs_real_clone(const csgrs_real_t *value, csgrs_real_t **out);
csgrs_status_t csgrs_real_to_f64(const csgrs_real_t *value, double *out);
csgrs_status_t csgrs_real_to_i128(const csgrs_real_t *value, csgrs_i128_t *out);
void csgrs_real_free(csgrs_real_t *value);

void csgrs_mesh_free(csgrs_mesh_t *value);
void csgrs_profile_free(csgrs_profile_t *value);
csgrs_status_t csgrs_mesh_family(const csgrs_mesh_t *value, csgrs_scalar_family_t *out);
csgrs_status_t csgrs_profile_family(const csgrs_profile_t *value, csgrs_scalar_family_t *out);

#define CSGRS_DECLARE_FAMILY(Name, Scalar, Vec2, Vec3, Matrix4, Aabb3, MeshBuffers, GraphicsMesh, RegionProfiles) \
  csgrs_status_t csgrs_mesh_##Name##_cube(Scalar width, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_cuboid(Scalar width, Scalar length, Scalar height, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_sphere(Scalar radius, size_t segments, size_t stacks, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_cylinder(Scalar radius, Scalar height, size_t segments, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_polyhedron(const Vec3 *points, size_t point_len, const size_t *face_indices, size_t face_index_len, const size_t *face_offsets, size_t face_offset_len, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_union(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_difference(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_intersection(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_xor(const csgrs_mesh_t *lhs, const csgrs_mesh_t *rhs, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_transform(const csgrs_mesh_t *mesh, Matrix4 matrix, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_translate(const csgrs_mesh_t *mesh, Scalar x, Scalar y, Scalar z, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_scale(const csgrs_mesh_t *mesh, Scalar sx, Scalar sy, Scalar sz, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_rotate(const csgrs_mesh_t *mesh, Scalar x_degrees, Scalar y_degrees, Scalar z_degrees, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_inverse(const csgrs_mesh_t *mesh, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_center(const csgrs_mesh_t *mesh, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_float(const csgrs_mesh_t *mesh, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_mesh_##Name##_bounding_box(const csgrs_mesh_t *mesh, Aabb3 *out); \
  csgrs_status_t csgrs_mesh_##Name##_vertices_and_indices(const csgrs_mesh_t *mesh, MeshBuffers *out); \
  csgrs_status_t csgrs_mesh_##Name##_graphics_mesh(const csgrs_mesh_t *mesh, GraphicsMesh *out); \
  csgrs_status_t csgrs_profile_##Name##_square(Scalar width, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_rectangle(Scalar width, Scalar length, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_circle(Scalar radius, size_t segments, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_polygon(const Vec2 *points, size_t point_len, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_union(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_difference(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_intersection(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_xor(const csgrs_profile_t *lhs, const csgrs_profile_t *rhs, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_transform(const csgrs_profile_t *profile, Matrix4 matrix, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_translate(const csgrs_profile_t *profile, Scalar x, Scalar y, Scalar z, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_scale(const csgrs_profile_t *profile, Scalar sx, Scalar sy, Scalar sz, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_rotate(const csgrs_profile_t *profile, Scalar x_degrees, Scalar y_degrees, Scalar z_degrees, csgrs_profile_t **out); \
  csgrs_status_t csgrs_profile_##Name##_bounding_box(const csgrs_profile_t *profile, Aabb3 *out); \
  csgrs_status_t csgrs_profile_##Name##_extrude(const csgrs_profile_t *profile, Scalar height, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_profile_##Name##_extrude_vector(const csgrs_profile_t *profile, Vec3 direction, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_profile_##Name##_revolve(const csgrs_profile_t *profile, Scalar angle_degrees, size_t segments, csgrs_mesh_t **out); \
  csgrs_status_t csgrs_profile_##Name##_region_profiles(const csgrs_profile_t *profile, RegionProfiles *out); \
  void csgrs_mesh_buffers_##Name##_free(MeshBuffers value); \
  void csgrs_graphics_mesh_##Name##_free(GraphicsMesh value); \
  void csgrs_region_profiles_##Name##_free(RegionProfiles value);

CSGRS_DECLARE_FAMILY(f32, float, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_matrix4_f32_t, csgrs_aabb3_f32_t, csgrs_mesh_buffers_f32_t, csgrs_graphics_mesh_f32_t, csgrs_region_profiles_f32_t)
CSGRS_DECLARE_FAMILY(f64, double, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_matrix4_f64_t, csgrs_aabb3_f64_t, csgrs_mesh_buffers_f64_t, csgrs_graphics_mesh_f64_t, csgrs_region_profiles_f64_t)
CSGRS_DECLARE_FAMILY(i128, csgrs_i128_t, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_matrix4_i128_t, csgrs_aabb3_i128_t, csgrs_mesh_buffers_i128_t, csgrs_graphics_mesh_i128_t, csgrs_region_profiles_i128_t)
CSGRS_DECLARE_FAMILY(real, const csgrs_real_t *, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_matrix4_real_t, csgrs_aabb3_real_t, csgrs_mesh_buffers_real_t, csgrs_graphics_mesh_real_t, csgrs_region_profiles_real_t)

void csgrs_aabb3_real_free_values(csgrs_aabb3_real_t value);

#ifdef __cplusplus
}
#endif

#endif
