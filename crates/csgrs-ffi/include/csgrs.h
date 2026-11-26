/**
 * @file
 * C declarations for csgrs-ffi's native-backed scalar adapters.
 *
 * Geometry/validation remain in the shared adapters and Hyper kernels. Function
 * prefixes select boundary scalar layouts, not separate geometry kernels. Finite
 * float inputs retain their exact binary value; float outputs are lossy. Integer
 * output requires a recognized in-range integer. Real mesh-coordinate output
 * clones native scalars, but all region-profile families first sample/project
 * through binary64 and cannot serve as exact native curve replay.
 *
 * There is no policy selector or separate local certainty receipt. Most decision
 * paths use strict predicates; region Booleans use Hypercurve's active scope,
 * strict outside a preview/provisional scope. Mesh construction-history flags
 * are queryable but do not certify topology or analytic-source fidelity.
 *
 * Pointer and ownership contract:
 * - Non-null pointers must be aligned, live and valid for the documented reads
 *   or writes, without conflicting access. Null checks do not validate addresses.
 * - Nonempty arrays must fit in one live allocation with byte size no greater
 *   than the target's Rust isize::MAX. Null is allowed for zero-length arrays.
 * - Input handles/components are borrowed and must originate from this library.
 *   Geometry must match the selected scalar family. Immutable operands may alias.
 * - Output slots must not alias borrowed inputs. OK writes a new owned result
 *   without releasing a previous slot value; ordinary errors leave slots intact.
 *   NO_BOUNDS is completed absence and writes no record or owned real endpoints.
 * - Free successful owners exactly once with the matching function. Preserve
 *   buffer pointers/lengths and nested real handles. Copying a descriptor does
 *   not duplicate ownership, and freeing requires no outstanding use.
 * - Copy borrowed diagnostic text before the next ABI call on that thread or
 *   thread exit. Status-returning calls replace it; free functions do not.
 *
 * Status calls catch unwinding panics in the action, not aborts, invalid memory
 * access or failure during diagnostic formatting. Free functions have no unwind
 * catcher. These wrappers add no total-work, memory or cancellation budget.
 */
#ifndef CSGRS_H
#define CSGRS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Stable numeric outcomes. Only OK transfers newly returned ownership;
 * NO_BOUNDS reports completed empty bounds. Diagnostic text is thread-local.
 */
typedef enum csgrs_status_t {
  /** Completed operation; declared output has been written. */
  CSGRS_STATUS_OK = 0,
  /** A required pointer was null; non-null validity is not checked. */
  CSGRS_STATUS_NULL_POINTER = 1,
  /** A live geometry handle carries another family tag. */
  CSGRS_STATUS_WRONG_SCALAR_FAMILY = 2,
  /** Scalar import/export failed outside native construction validation. */
  CSGRS_STATUS_CONVERSION_FAILED = 3,
  /** Invalid input or ABI encoding. */
  CSGRS_STATUS_VALIDATION_FAILED = 4,
  /** A required native decision could not be certified. */
  CSGRS_STATUS_PREDICATE_UNDECIDED = 5,
  /** A native operation reached a configured allowance. */
  CSGRS_STATUS_RESOURCE_EXHAUSTED = 6,
  /** The native path does not support the requested representation/operation. */
  CSGRS_STATUS_UNSUPPORTED = 7,
  /** Other native construction failure not assigned a specific status. */
  CSGRS_STATUS_GEOMETRY_FAILED = 8,
  /** Requested geometry cannot be represented as a finite result. */
  CSGRS_STATUS_UNBOUNDED = 9,
  /** Completed empty bounds query; the output AABB is untouched. */
  CSGRS_STATUS_NO_BOUNDS = 10,
  /** Storage is unavailable without reaching a configured work allowance. */
  CSGRS_STATUS_RESOURCE_UNAVAILABLE = 11,
  /** An unwinding action panic was caught; this is not memory-safety recovery. */
  CSGRS_STATUS_PANIC = 255,
} csgrs_status_t;

/**
 * Boundary scalar strategy stored on each geometry handle; internal coordinates stay native.
 */
typedef enum csgrs_scalar_family_t {
  /** Binary32 ingress and lossy binary32 egress. */
  CSGRS_SCALAR_F32 = 1,
  /** Binary64 ingress and lossy binary64 egress. */
  CSGRS_SCALAR_F64 = 2,
  /** Split-word integer ingress and restricted integer egress. */
  CSGRS_SCALAR_I128 = 3,
  /** Borrowed/owned opaque native scalar handles. */
  CSGRS_SCALAR_REAL = 4,
} csgrs_scalar_family_t;

/**
 * Opaque owned native scalar; never allocate or inspect its storage from C.
 */
typedef struct csgrs_real_t csgrs_real_t;
/**
 * Opaque native mesh plus scalar-family tag; operations borrow it.
 */
typedef struct csgrs_triangle_mesh_t csgrs_triangle_mesh_t;
/**
 * Opaque filled region plus scalar-family tag; operations borrow it.
 */
typedef struct csgrs_curve_region_t csgrs_curve_region_t;

/** Independent of local certainty, topology and geometric fidelity. Fields are
   0 or 1. A false approximate flag does not certify missing source history. */
typedef struct csgrs_construction_evidence_t {
  /** Some source history is unavailable. */
  uint8_t source_history_missing;
  /** Retained history records approximate-terminal consumption. */
  uint8_t approximate_512_consumed;
  /** Retained history records coordinate projection. */
  uint8_t coordinates_projected;
  /** Retained history records sampling of an intended source. */
  uint8_t source_sampled;
  /** Construction history records native buffer changes. */
  uint8_t buffers_changed;
} csgrs_construction_evidence_t;

/* Accepts any valid mesh family. Failure leaves the output untouched. */
/**
 * Read retained mesh construction-history flags for any scalar family.
 *
 * A false approximate flag cannot certify unknown history, topology or source
 * fidelity. Failure leaves the output record untouched.
 *
 * Caller obligations:
 * Borrow a live mesh and provide a nonaliasing writable output record, following
 * the header pointer contract. No ownership is transferred by this query.
 */
csgrs_status_t csgrs_triangle_mesh_construction_evidence(
    const csgrs_triangle_mesh_t *mesh, csgrs_construction_evidence_t *out);

/**
 * Two's-complement integer with signed high word and unsigned low word.
 */
typedef struct csgrs_i128_t {
  /** Signed high 64 bits. */
  int64_t hi;
  /** Unsigned low 64 bits. */
  uint64_t lo;
} csgrs_i128_t;

/**
 * 2 coordinates in binary32 form: x/y.
 */
typedef struct csgrs_vec2_f32_t { float x, y; } csgrs_vec2_f32_t;
/**
 * 3 coordinates in binary32 form: x/y/z.
 */
typedef struct csgrs_vec3_f32_t { float x, y, z; } csgrs_vec3_f32_t;
/**
 * 2 coordinates in binary64 form: x/y.
 */
typedef struct csgrs_vec2_f64_t { double x, y; } csgrs_vec2_f64_t;
/**
 * 3 coordinates in binary64 form: x/y/z.
 */
typedef struct csgrs_vec3_f64_t { double x, y, z; } csgrs_vec3_f64_t;
/**
 * 2 coordinates in split-word signed integer form: x/y.
 */
typedef struct csgrs_vec2_i128_t { csgrs_i128_t x, y; } csgrs_vec2_i128_t;
/**
 * 3 coordinates in split-word signed integer form: x/y/z.
 */
typedef struct csgrs_vec3_i128_t { csgrs_i128_t x, y, z; } csgrs_vec3_i128_t;
/**
 * 2 coordinates in opaque native scalar handle form: x/y.
 * Inputs borrow components; exported components own separate handles. A record
 * copy does not clone ownership.
 */
typedef struct csgrs_vec2_real_t { csgrs_real_t *x, *y; } csgrs_vec2_real_t;
/**
 * 3 coordinates in opaque native scalar handle form: x/y/z.
 * Inputs borrow components; exported components own separate handles. A record
 * copy does not clone ownership.
 */
typedef struct csgrs_vec3_real_t { csgrs_real_t *x, *y, *z; } csgrs_vec3_real_t;

/**
 * Ordered vertex indices a, b, c into the corresponding exported position array.
 */
typedef struct csgrs_triangle_u32_t { uint32_t a, b, c; } csgrs_triangle_u32_t;

/**
 * Exported axis-aligned bounds: mins and maxs are XYZ endpoint records.
 * Float endpoints are projections, not an outward-enclosure certificate.
 */
typedef struct csgrs_aabb3_f32_t { csgrs_vec3_f32_t mins, maxs; } csgrs_aabb3_f32_t;
/**
 * Exported axis-aligned bounds: mins and maxs are XYZ endpoint records.
 * Float endpoints are projections, not an outward-enclosure certificate.
 */
typedef struct csgrs_aabb3_f64_t { csgrs_vec3_f64_t mins, maxs; } csgrs_aabb3_f64_t;
/**
 * Exported axis-aligned bounds: mins and maxs are XYZ endpoint records.
 * Integer export can fail.
 */
typedef struct csgrs_aabb3_i128_t { csgrs_vec3_i128_t mins, maxs; } csgrs_aabb3_i128_t;
/**
 * Exported axis-aligned bounds: mins and maxs are XYZ endpoint records.
 * Owns six scalar handles; release once with csgrs_aabb3_real_free_values.
 */
typedef struct csgrs_aabb3_real_t { csgrs_vec3_real_t mins, maxs; } csgrs_aabb3_real_t;

/**
 * Homogeneous transform: values contains sixteen coefficients in row-major order.
 */
typedef struct csgrs_matrix4_f32_t { float values[16]; } csgrs_matrix4_f32_t;
/**
 * Homogeneous transform: values contains sixteen coefficients in row-major order.
 */
typedef struct csgrs_matrix4_f64_t { double values[16]; } csgrs_matrix4_f64_t;
/**
 * Homogeneous transform: values contains sixteen coefficients in row-major order.
 */
typedef struct csgrs_matrix4_i128_t { csgrs_i128_t values[16]; } csgrs_matrix4_i128_t;
/**
 * Homogeneous transform: values contains sixteen coefficients in row-major order.
 * All scalar handles are borrowed for the duration of the call.
 */
typedef struct csgrs_matrix4_real_t { const csgrs_real_t *values[16]; } csgrs_matrix4_real_t;

/**
 * Exported position and native-certified unit normal; scalar export can round
 * or fail, and projection can change normal length.
 */
typedef struct csgrs_graphics_vertex_f32_t {
  /** Exported vertex position. */
  csgrs_vec3_f32_t position;
  /** Exported unit-normal coordinates. */
  csgrs_vec3_f32_t normal;
} csgrs_graphics_vertex_f32_t;

/**
 * Exported position and native-certified unit normal; scalar export can round
 * or fail, and projection can change normal length.
 */
typedef struct csgrs_graphics_vertex_f64_t {
  /** Exported vertex position. */
  csgrs_vec3_f64_t position;
  /** Exported unit-normal coordinates. */
  csgrs_vec3_f64_t normal;
} csgrs_graphics_vertex_f64_t;

/**
 * Exported position and native-certified unit normal; scalar export can round
 * or fail, and projection can change normal length.
 */
typedef struct csgrs_graphics_vertex_i128_t {
  /** Exported vertex position. */
  csgrs_vec3_i128_t position;
  /** Exported unit-normal coordinates. */
  csgrs_vec3_i128_t normal;
} csgrs_graphics_vertex_i128_t;

/**
 * Exported position and native-certified unit normal; scalar export can round
 * or fail, and projection can change normal length. Real components are owned.
 */
typedef struct csgrs_graphics_vertex_real_t {
  /** Exported vertex position. */
  csgrs_vec3_real_t position;
  /** Exported unit-normal coordinates. */
  csgrs_vec3_real_t normal;
} csgrs_graphics_vertex_real_t;

/**
 * Declare owning array descriptors for one boundary family.
 * Mesh index_len counts triangle records; graphics index_len counts flat indices.
 * All arrays require the matching free function, including empty outputs. Real
 * arrays also own nested scalar handles. Profiles are sampled/projected output:
 * ring_offsets indexes points, region_offsets indexes rings, both include end
 * sentinels. Each region lists its exterior followed by its holes. Empty profiles
 * have zero points and one zero in each offset array.
 */
#define CSGRS_DECLARE_BUFFERS(Name, Vec2, Vec3, GraphicsVertex) \
  typedef struct csgrs_triangle_mesh_buffers_##Name##_t { \
    /** Owned vertex array. */ \
    Vec3 *vertices; \
    /** Number of vertex records. */ \
    size_t vertex_len; \
    /** Owned index array, in the indicated element type. */ \
    csgrs_triangle_u32_t *indices; \
    /** Number of index elements, not bytes. */ \
    size_t index_len; \
  } csgrs_triangle_mesh_buffers_##Name##_t; \
  typedef struct csgrs_graphics_mesh_##Name##_t { \
    /** Owned vertex array. */ \
    GraphicsVertex *vertices; \
    /** Number of vertex records. */ \
    size_t vertex_len; \
    /** Owned index array, in the indicated element type. */ \
    uint32_t *indices; \
    /** Number of index elements, not bytes. */ \
    size_t index_len; \
  } csgrs_graphics_mesh_##Name##_t; \
  typedef struct csgrs_region_profiles_##Name##_t { \
    /** Owned flattened XY point array. */ \
    Vec2 *points; \
    /** Number of points. */ \
    size_t point_len; \
    /** Owned point offsets, including first zero and final point_len. */ \
    size_t *ring_offsets; \
    /** One more than the number of rings. */ \
    size_t ring_offset_len; \
    /** Owned ring offsets, including first zero and final ring count. */ \
    size_t *region_offsets; \
    /** One more than the number of regions. */ \
    size_t region_offset_len; \
  } csgrs_region_profiles_##Name##_t;

CSGRS_DECLARE_BUFFERS(f32, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_graphics_vertex_f32_t)
CSGRS_DECLARE_BUFFERS(f64, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_graphics_vertex_f64_t)
CSGRS_DECLARE_BUFFERS(i128, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_graphics_vertex_i128_t)
CSGRS_DECLARE_BUFFERS(real, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_graphics_vertex_real_t)

/**
 * Borrow the current thread's NUL-terminated diagnostic without clearing it.
 *
 * Copy the text before the next ABI call or thread exit. Status-returning calls
 * replace it on both success and failure; free functions leave it unchanged.
 */
const char *csgrs_last_error_message(void);

/**
 * Allocate a native exact zero scalar.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 */
csgrs_status_t csgrs_real_zero(csgrs_real_t **out);
/**
 * Allocate a native exact one scalar.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 */
csgrs_status_t csgrs_real_one(csgrs_real_t **out);
/**
 * Import the exact finite binary64 value, rejecting NaN and infinity.
 *
 * For example, `0.1_f64` is not the rational one tenth.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 */
csgrs_status_t csgrs_real_from_f64(double value, csgrs_real_t **out);
/**
 * Import a split-word signed integer exactly.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 */
csgrs_status_t csgrs_real_from_i128(csgrs_i128_t value, csgrs_real_t **out);
/**
 * Parse a NUL-terminated UTF-8 decimal integer in the signed 128-bit range.
 *
 * This accepts integer syntax only, not fractional decimal reals. Whitespace,
 * invalid UTF-8 and overflow are validation failures.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 * The non-null string must remain readable through its terminating NUL.
 */
csgrs_status_t csgrs_real_from_i128_decimal(const char *value, csgrs_real_t **out);
/**
 * Create another owned handle sharing the native scalar's exact meaning.
 *
 * The original remains borrowed and must retain its own lifetime/ownership.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `out` must be a writable handle
 * slot; success transfers one scalar owner to free with `csgrs_real_free`.
 * `value` must borrow a live scalar handle.
 */
csgrs_status_t csgrs_real_clone(const csgrs_real_t *value, csgrs_real_t **out);
/**
 * Return a finite lossy binary64 projection, or a conversion failure.
 *
 * The output is not an exactness or directed-rounding certificate.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `value` must borrow a live scalar
 * handle and `out` must be a writable, nonaliasing output slot.
 */
csgrs_status_t csgrs_real_to_f64(const csgrs_real_t *value, double *out);
/**
 * Extract a recognized rational integer into signed high and unsigned low words.
 *
 * Unrecognized expressions, nonintegers and range overflow fail; there is no
 * refinement search to decide whether an unrecognized expression is integral.
 *
 * Caller obligations:
 * Follow the header pointer/aliasing contract. `value` must borrow a live scalar
 * handle and `out` must be a writable, nonaliasing output slot.
 */
csgrs_status_t csgrs_real_to_i128(const csgrs_real_t *value, csgrs_i128_t *out);
/**
 * Release one opaque scalar owner; null is a no-op.
 *
 * Does not clear the last-error string and has no unwind catcher.
 *
 * Caller obligations:
 * A non-null pointer must be a still-owned scalar allocation returned by this
 * library, with no outstanding use. Free it exactly once; do not pass stack
 * values, borrowed pointers or a handle already freed elsewhere.
 */
void csgrs_real_free(csgrs_real_t *value);

/**
 * Release one opaque mesh owner; null is a no-op.
 *
 * Does not clear the last-error string and has no unwind catcher.
 *
 * Caller obligations:
 * A non-null pointer must be a still-owned mesh allocation returned by this
 * library, with no outstanding use. Free it exactly once; do not pass stack
 * values, borrowed pointers or a handle already freed elsewhere.
 */
void csgrs_triangle_mesh_free(csgrs_triangle_mesh_t *value);
/**
 * Release one opaque region owner; null is a no-op.
 *
 * Does not clear the last-error string and has no unwind catcher.
 *
 * Caller obligations:
 * A non-null pointer must be a still-owned region allocation returned by this
 * library, with no outstanding use. Free it exactly once; do not pass stack
 * values, borrowed pointers or a handle already freed elsewhere.
 */
void csgrs_curve_region_free(csgrs_curve_region_t *value);
/**
 * Read a live mesh handle's boundary-family tag without converting geometry.
 *
 * The tag describes ingress/egress, not the internal coordinate representation.
 *
 * Caller obligations:
 * Borrow a live mesh handle and provide a nonaliasing writable family slot,
 * following the header pointer contract.
 */
csgrs_status_t csgrs_triangle_mesh_family(const csgrs_triangle_mesh_t *value, csgrs_scalar_family_t *out);
/**
 * Read a live region handle's boundary-family tag without converting geometry.
 *
 * The tag describes ingress/egress, not the internal coordinate representation.
 *
 * Caller obligations:
 * Borrow a live region handle and provide a nonaliasing writable family slot,
 * following the header pointer contract.
 */
csgrs_status_t csgrs_curve_region_family(const csgrs_curve_region_t *value, csgrs_scalar_family_t *out);

/* revolve retains partial swept boundaries; revolve_solid returns regularized
 * segmented material, admits axis crossings and permits closed PWN contacts.
 * Both use strict decisions and the mesh construction-evidence queries. */

/*
 * Array pointers may be NULL only when their corresponding length is zero.
 * Polyhedron face_offsets must start at zero and end at face_index_len, so
 * every supplied index belongs to exactly one declared face range.
 */
/**
 * Declare the borrowed-input/new-output operation surface and owning-buffer
 * destructors for one scalar family. Implementations share the Rust adapters.
 */
#define CSGRS_DECLARE_FAMILY(Name, Scalar, Vec2, Vec3, Matrix4, Aabb3, MeshBuffers, GraphicsMesh, RegionProfiles) \
  /** \
   * Construct `[0,width]^3` with strict nonnegative dimension validation. \
   * \
   * Zero width succeeds as canonical empty geometry; negative or undecided width fails. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_cube(Scalar width, csgrs_triangle_mesh_t **out); \
  /** \
   * Construct an origin-anchored box with XYZ dimensions `width`, `length`, `height`. \
   * \
   * All dimensions must be nonnegative under strict predicates. A zero dimension \
   * gives canonical empty geometry only after validation of all dimensions. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_cuboid(Scalar width, Scalar length, Scalar height, csgrs_triangle_mesh_t **out); \
  /** \
   * Construct the sampled triangle sphere centered at the origin, with Y-axis poles. \
   * \
   * Requires nonnegative radius, at least three segments and two stacks, even \
   * for zero radius. Construction history records sampling; success is not an \
   * analytic-sphere fidelity certificate. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_sphere(Scalar radius, size_t segments, size_t stacks, csgrs_triangle_mesh_t **out); \
  /** \
   * Construct a sampled capped cylinder from Z zero to `height`. \
   * \
   * Requires nonnegative radius and at least three segments; signed height \
   * may be negative. Parameters are validated before zero-size empty success. \
   * The boundary is polygonal. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_cylinder(Scalar radius, Scalar height, size_t segments, csgrs_triangle_mesh_t **out); \
  /** \
   * Triangulate indexed planar polygon faces with authored winding under strict predicates. \
   * \
   * Offsets must partition the complete index buffer from zero to its length. \
   * Native validation checks faces/indices; success need not be a closed solid. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   * Input arrays must cover their declared element counts (null only at zero \
   * length); real point components borrow live scalar handles. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_polyhedron(const Vec3 *points, size_t point_len, const size_t *face_indices, size_t face_index_len, const size_t *face_offsets, size_t face_offset_len, csgrs_triangle_mesh_t **out); \
  /** \
   * Return the strict regularized union of two same-family meshes. \
   * \
   * Native input validation and failure categories are preserved. No separate \
   * operation receipt is returned; retained mesh history remains queryable. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_union(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out); \
  /** \
   * Return the strict regularized material of `lhs` outside `rhs`. \
   * \
   * Both operands use the same scalar family; native validation/failures remain \
   * authoritative and no separate operation receipt is returned. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_difference(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out); \
  /** \
   * Return the strict regularized common material of two same-family meshes. \
   * \
   * Empty success remains a mesh handle; it is distinct from invalid or undecided \
   * input. No separate operation receipt is returned. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_intersection(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out); \
  /** \
   * Return the strict regularized material belonging to exactly one mesh operand. \
   * \
   * Native validation/failure categories are preserved without a separate \
   * operation receipt at this boundary. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_xor(const csgrs_triangle_mesh_t *lhs, const csgrs_triangle_mesh_t *rhs, csgrs_triangle_mesh_t **out); \
  /** \
   * Apply an imported row-major homogeneous matrix through the checked mesh transform. \
   * \
   * Strict validation rejects singular/undecided transforms and images reaching \
   * infinity. Winding correction preserves oriented material where admitted. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_transform(const csgrs_triangle_mesh_t *mesh, Matrix4 matrix, csgrs_triangle_mesh_t **out); \
  /** \
   * Translate native mesh coordinates by the imported XYZ displacement. \
   * \
   * Uses the native adapter translation without a separate operation receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_translate(const csgrs_triangle_mesh_t *mesh, Scalar x, Scalar y, Scalar z, csgrs_triangle_mesh_t **out); \
  /** \
   * Scale XYZ independently under strict predicates and correct winding for reflections. \
   * \
   * Zero axes are invalid and unresolved signs fail; input scalar conversion \
   * precedes native transform validation. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_scale(const csgrs_triangle_mesh_t *mesh, Scalar sx, Scalar sy, Scalar sz, csgrs_triangle_mesh_t **out); \
  /** \
   * Rotate about X, then Y, then Z using imported angles in degrees. \
   * \
   * Uses strict checked transforms without a separate operation receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_rotate(const csgrs_triangle_mesh_t *mesh, Scalar x_degrees, Scalar y_degrees, Scalar z_degrees, csgrs_triangle_mesh_t **out); \
  /** \
   * Return a mesh with every triangle winding reversed and positions preserved. \
   * \
   * This operation does not construct the unbounded set complement. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_inverse(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out); \
  /** \
   * Move the native bounds center to the origin under strict predicates. \
   * \
   * Canonical empty geometry is preserved; bounds errors propagate. Placement \
   * uses native coordinates without first exporting them to the scalar family. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_center(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out); \
  /** \
   * Move the native minimum Z bound to zero under strict predicates. \
   * \
   * This is geometric placement, not scalar conversion. Empty geometry is \
   * preserved and unresolved bounds fail without a new handle. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_float(const csgrs_triangle_mesh_t *mesh, csgrs_triangle_mesh_t **out); \
  /** \
   * Query strict native extrema, then export them through this scalar family. \
   * \
   * `CSGRS_STATUS_NO_BOUNDS` leaves a valid output record untouched and allocates no real \
   * endpoints. Float output is not an outward-enclosure certificate; integer \
   * export can fail. Only `CSGRS_STATUS_OK` writes a record. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. The input handle must be live and \
   * match this function's family; `out` must be a writable record slot. Preserve \
   * all returned pointers/lengths and use the matching free function exactly once \
   * for owned buffer outputs or real-valued bounds. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_bounding_box(const csgrs_triangle_mesh_t *mesh, Aabb3 *out); \
  /** \
   * Export indexed positions and ordered triangle rows into owned arrays. \
   * \
   * Float coordinates are lossy; integer conversion can fail. The real family \
   * clones native scalars. Counts describe rows, and zero rows are valid success. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. The input handle must be live and \
   * match this function's family; `out` must be a writable record slot. Preserve \
   * all returned pointers/lengths and use the matching free function exactly once \
   * for owned buffer outputs or real-valued bounds. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_vertices_and_indices(const csgrs_triangle_mesh_t *mesh, MeshBuffers *out); \
  /** \
   * Export positions, certified native normals and flat indices into owned graphics arrays. \
   * \
   * Normal certification and scalar export can fail. Projection may change normal \
   * lengths; the result has no separate fidelity or certainty receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. The input handle must be live and \
   * match this function's family; `out` must be a writable record slot. Preserve \
   * all returned pointers/lengths and use the matching free function exactly once \
   * for owned buffer outputs or real-valued bounds. \
   */ \
  csgrs_status_t csgrs_triangle_mesh_##Name##_graphics_mesh(const csgrs_triangle_mesh_t *mesh, GraphicsMesh *out); \
  /** \
   * Construct an origin-anchored XY square with strict nonnegative width validation. \
   * \
   * Zero width returns canonical empty geometry; negative or undecided width fails. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_square(Scalar width, csgrs_curve_region_t **out); \
  /** \
   * Construct an origin-anchored XY rectangle with nonnegative width and length. \
   * \
   * Both dimensions are validated under strict predicates before zero-size empty success. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_rectangle(Scalar width, Scalar length, csgrs_curve_region_t **out); \
  /** \
   * Construct a regular sampled XY polygon centered at the origin. \
   * \
   * Requires nonnegative radius and at least three segments, including zero-radius \
   * input. This is not an analytic circular boundary or a fidelity certificate. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_circle(Scalar radius, size_t segments, csgrs_curve_region_t **out); \
  /** \
   * Construct a filled XY polygon from a valid ring of at least three vertices. \
   * \
   * Native strict validation rejects empty/malformed rings. The input is copied \
   * through the selected scalar adapter before region construction. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Real-family scalar components \
   * must borrow live handles. `out` must be a writable handle slot; on success \
   * its new owner must be freed once with the matching geometry free function. \
   * The point array must cover `point_len` readable elements (null only at zero \
   * length); real components must borrow live scalar handles. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_polygon(const Vec2 *points, size_t point_len, csgrs_curve_region_t **out); \
  /** \
   * Return the regularized union of same-family filled regions. \
   * \
   * Delegates to Hypercurve's active policy scope, strict outside a preview or \
   * provisional scope, and returns no separate operation receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_union(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out); \
  /** \
   * Return the regularized part of `lhs` outside `rhs`. \
   * \
   * Uses Hypercurve's active policy scope (normally strict), with no separate receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_difference(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out); \
  /** \
   * Return the regularized common material of two same-family filled regions. \
   * \
   * Uses Hypercurve's active policy scope (normally strict), with no separate receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_intersection(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out); \
  /** \
   * Return regularized material belonging to exactly one filled-region operand. \
   * \
   * Uses Hypercurve's active policy scope (normally strict), with no separate receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_xor(const csgrs_curve_region_t *lhs, const csgrs_curve_region_t *rhs, csgrs_curve_region_t **out); \
  /** \
   * Apply a row-major homogeneous transform with a nonsingular affine XY restriction. \
   * \
   * Strict planar admission rejects perspective/out-of-plane maps and diagnoses \
   * invalid or unbounded weight cases. The separate outcome receipt is discarded. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_transform(const csgrs_curve_region_t *region, Matrix4 matrix, csgrs_curve_region_t **out); \
  /** \
   * Translate the planar region under strict predicates. \
   * \
   * Nonzero Z displacement cannot remain in the XY carrier. The adapter discards \
   * the separate local receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_translate(const csgrs_curve_region_t *region, Scalar x, Scalar y, Scalar z, csgrs_curve_region_t **out); \
  /** \
   * Scale the planar XY image under strict predicates. \
   * \
   * Z scale is imported but has no planar effect; a singular XY image is rejected. \
   * The adapter discards the separate local receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_scale(const csgrs_curve_region_t *region, Scalar sx, Scalar sy, Scalar sz, csgrs_curve_region_t **out); \
  /** \
   * Apply X, then Y, then Z rotations in degrees when the image remains in XY. \
   * \
   * Strict planar transform admission may reject the map; the adapter discards \
   * the separate local receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_rotate(const csgrs_curve_region_t *region, Scalar x_degrees, Scalar y_degrees, Scalar z_degrees, csgrs_curve_region_t **out); \
  /** \
   * Query strict native region bounds and export the endpoints through this family. \
   * \
   * Trimmed fragments may use a looser certified source envelope. Float endpoints \
   * are not an outward-enclosure certificate. `CSGRS_STATUS_NO_BOUNDS` leaves a valid output \
   * record untouched and allocates no real handles. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. The input handle must be live and \
   * match this function's family; `out` must be a writable record slot. Preserve \
   * all returned pointers/lengths and use the matching free function exactly once \
   * for owned buffer outputs or real-valued bounds. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_bounding_box(const csgrs_curve_region_t *region, Aabb3 *out); \
  /** \
   * Extrude the region along Z by a nonzero signed height under strict predicates. \
   * \
   * Nonlinear profile interiors may be sampled with mesh construction history. \
   * The separate outcome receipt is discarded; exact source fidelity is not implied. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_extrude(const csgrs_curve_region_t *region, Scalar height, csgrs_triangle_mesh_t **out); \
  /** \
   * Extrude along the imported vector, requiring a nonzero Z component. \
   * \
   * Uses strict predicates and the native profile-lowering fidelity contract; \
   * the separate outcome receipt is discarded. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_extrude_vector(const csgrs_curve_region_t *region, Vec3 direction, csgrs_triangle_mesh_t **out); \
  /** \
   * Build a segmented swept boundary about Y, with profile X as signed radius. \
   * \
   * Angles are in degrees; at least two segments (three for a full turn) are \
   * required. Zero and oversized noninteger-turn angles are invalid. Partial \
   * sweeps reject axis-crossing loops and may retain overlapping winding. \
   * Strict decisions do not certify analytic-surface fidelity; the separate \
   * outcome receipt is discarded. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_revolve(const csgrs_curve_region_t *region, Scalar angle_degrees, size_t segments, csgrs_triangle_mesh_t **out); \
  /** \
   * Regularize the union of segmented revolution pieces about Y. \
   * \
   * Uses the revolution angle/resolution domain, admits axis-crossing partial \
   * profiles and counts overlapping material once. Closed point/edge contacts \
   * may remain nonmanifold. Uses strict predicates and discards the separate receipt. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. Input geometry must be live and \
   * match this function's scalar family; real scalar components are borrowed. \
   * `out` must be a writable handle slot; success transfers one new owner. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_revolve_solid(const csgrs_curve_region_t *region, Scalar angle_degrees, size_t segments, csgrs_triangle_mesh_t **out); \
  /** \
   * Export sampled exterior/hole rings with sentinel offsets under strict predicates. \
   * \
   * All families, including real, pass through binary64 display profiles. This \
   * is not exact native replay. The separate receipt is discarded; empty output \
   * has zero points and one zero in each offset array. \
   * \
   * Caller obligations: \
   * Follow the header pointer/aliasing contract. The input handle must be live and \
   * match this function's family; `out` must be a writable record slot. Preserve \
   * all returned pointers/lengths and use the matching free function exactly once \
   * for owned buffer outputs or real-valued bounds. \
   */ \
  csgrs_status_t csgrs_curve_region_##Name##_region_profiles(const csgrs_curve_region_t *region, RegionProfiles *out); \
  /** \
   * Release the primitive-family vertex and triangle arrays. \
   * \
   * No diagnostic is written and no unwind catcher is installed. \
   * \
   * Caller obligations: \
   * The descriptor must retain the exact pointers and lengths returned by the \
   * matching successful export, still owned once with no outstanding references. \
   * Do not free a copied descriptor twice or substitute foreign allocations. \
   */ \
  void csgrs_triangle_mesh_buffers_##Name##_free(MeshBuffers value); \
  /** \
   * Release the primitive-family graphics vertex and flat index arrays. \
   * \
   * No diagnostic is written and no unwind catcher is installed. \
   * \
   * Caller obligations: \
   * The descriptor must retain the exact pointers and lengths returned by the \
   * matching successful export, still owned once with no outstanding references. \
   * Do not free a copied descriptor twice or substitute foreign allocations. \
   */ \
  void csgrs_graphics_mesh_##Name##_free(GraphicsMesh value); \
  /** \
   * Release primitive-family points and both profile offset arrays. \
   * \
   * No diagnostic is written and no unwind catcher is installed. \
   * \
   * Caller obligations: \
   * The descriptor must retain the exact pointers and lengths returned by the \
   * matching successful export, still owned once with no outstanding references. \
   * Do not free a copied descriptor twice or substitute foreign allocations. \
   */ \
  void csgrs_region_profiles_##Name##_free(RegionProfiles value);

CSGRS_DECLARE_FAMILY(f32, float, csgrs_vec2_f32_t, csgrs_vec3_f32_t, csgrs_matrix4_f32_t, csgrs_aabb3_f32_t, csgrs_triangle_mesh_buffers_f32_t, csgrs_graphics_mesh_f32_t, csgrs_region_profiles_f32_t)
CSGRS_DECLARE_FAMILY(f64, double, csgrs_vec2_f64_t, csgrs_vec3_f64_t, csgrs_matrix4_f64_t, csgrs_aabb3_f64_t, csgrs_triangle_mesh_buffers_f64_t, csgrs_graphics_mesh_f64_t, csgrs_region_profiles_f64_t)
CSGRS_DECLARE_FAMILY(i128, csgrs_i128_t, csgrs_vec2_i128_t, csgrs_vec3_i128_t, csgrs_matrix4_i128_t, csgrs_aabb3_i128_t, csgrs_triangle_mesh_buffers_i128_t, csgrs_graphics_mesh_i128_t, csgrs_region_profiles_i128_t)
CSGRS_DECLARE_FAMILY(real, const csgrs_real_t *, csgrs_vec2_real_t, csgrs_vec3_real_t, csgrs_matrix4_real_t, csgrs_aabb3_real_t, csgrs_triangle_mesh_buffers_real_t, csgrs_graphics_mesh_real_t, csgrs_region_profiles_real_t)

/**
 * Release all six owned endpoint handles in an exported real-valued AABB.
 *
 * The by-value record is not reset; copies of it retain dangling pointers.
 *
 * Caller obligations:
 * Each non-null component must be a distinct live scalar owner returned by this
 * library, unused elsewhere. Call exactly once for a successful bounds export.
 * Never pass borrowed vector components or free individual components first.
 */
void csgrs_aabb3_real_free_values(csgrs_aabb3_real_t value);

#ifdef __cplusplus
}
#endif

#endif
