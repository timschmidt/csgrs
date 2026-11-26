/**
 * Typed native-loader facade, separate from the generated WebAssembly bindings.
 *
 * A consumer supplies NativeLibrary; this file contains no actual FFI loader.
 * Calls use convenient JS values and return logical outputs, whereas C functions
 * use scalar/layout encodings, lengths, output pointers and numeric statuses.
 * The loader must bridge those contracts, preserve errors and own native outputs.
 * TypeScript generics/tuples do not perform runtime family, size or pointer checks.
 *
 * Wrappers retain opaque handles but have no finalizers or disposed-state guard.
 * Call free once and never use the handle afterward; free does not clear it.
 * Raw buffers/bounds/profiles are unknown loader-defined values with loader-owned
 * cleanup rules. Keep the same loader installed while any handles remain live.
 * This facade adds no policy selector, local receipt, construction-history query,
 * total-work budget or cancellation mechanism. Geometry remains in csgrs-ffi.
 */
/**
 * Native scalar-family labels; this union provides no runtime validation.
 */
export type Family = "f32" | "f64" | "i128" | "real";
/**
 * Host bigint alias; signed 128-bit range is not enforced by the type itself.
 */
export type I128 = bigint;
/**
 * Compile-time input mapping: number for float families, bigint for i128, Real for native scalars.
 * The loader must convert these values to the corresponding C argument layout.
 */
export type Scalar<F extends Family> =
  F extends "f32" | "f64" ? number :
  F extends "i128" ? bigint :
  Real;

/**
 * XY tuple for loader conversion; no runtime length/component check is installed here.
 */
export type Vec2<S> = [S, S];
/**
 * XYZ tuple for loader conversion; real components borrow their scalar owners.
 */
export type Vec3<S> = [S, S, S];
/**
 * Row-major sixteen-coefficient homogeneous matrix tuple.
 * Runtime length, coefficient admission and native layout conversion belong to the loader/ABI.
 */
export type Matrix4<S> = [
  S, S, S, S,
  S, S, S, S,
  S, S, S, S,
  S, S, S, S,
];

/**
 * Consumer-implemented bridge from logical JS calls to the C ABI.
 *
 * Implementations must marshal scalar/struct arrays and their lengths, unwrap Real
 * arguments, allocate output slots, check statuses before publishing results, and
 * translate diagnostics. NO_BOUNDS needs an explicit absence representation. Raw
 * buffer/bounds outputs need documented ownership/cleanup rules. Decimal scalar
 * ingress needs a C string; real_to_i128 must return a decimal string, although
 * the C ABI actually writes a two-word integer. No implementation is shipped here.
 */
export interface NativeLibrary {
  /**
   * Perform one named logical operation and return its decoded output or throw.
   * T is a compile-time expectation, not runtime validation or a C return-type declaration.
   */
  call<T = unknown>(name: string, ...args: unknown[]): T;
}

/**
 * Module-level current loader, not bound separately to individual handle owners.
 */
let native: NativeLibrary | undefined;

/**
 * Install or replace the global loader without validating it or migrating live handles.
 * Do not switch incompatible loaders while wrappers or raw outputs remain live.
 */
export function loadCsgrs(library: NativeLibrary): void {
  native = library;
}

/**
 * Return the installed loader or throw a plain Error before any native call.
 */
function lib(): NativeLibrary {
  if (!native) {
    throw new Error("loadCsgrs() must be called with a native csgrs-ffi loader");
  }
  return native;
}

/**
 * Delegate directly to the current loader; no marshalling or error translation occurs here.
 */
function call<T>(name: string, ...args: unknown[]): T {
  return lib().call<T>(name, ...args);
}

/**
 * Opaque native scalar owner mediated by the installed loader.
 * Copies of handle values do not clone ownership; use clone. Free is manual and
 * not guarded against repeated calls. Native errors/cleanup depend on the loader.
 */
export class Real {
  /**
   * Loader-defined native scalar reference; readonly does not enforce liveness.
   */
  readonly handle: unknown;

  /**
   * Adopt the loader result without runtime type, provenance or ownership validation.
   */
  private constructor(handle: unknown) {
    this.handle = handle;
  }

  /**
   * Ask the loader to allocate an exact native zero scalar.
   */
  static zero(): Real {
    return new Real(call("csgrs_real_zero"));
  }

  /**
   * Ask the loader to allocate an exact native one scalar.
   */
  static one(): Real {
    return new Real(call("csgrs_real_one"));
  }

  /**
   * Request exact ingress of the finite binary64 number; NaN/infinity must fail natively.
   * The number 0.1 is not rational one tenth.
   */
  static fromF64(value: number): Real {
    return new Real(call("csgrs_real_from_f64", value));
  }

  /**
   * Send a base-ten bigint string to native signed-128-bit integer parsing.
   * The loader must encode a C string and preserve out-of-range parse failures.
   */
  static fromI128(value: bigint): Real {
    return new Real(call("csgrs_real_from_i128_decimal", value.toString()));
  }

  /**
   * Request another owned native scalar handle without changing its represented value.
   */
  clone(): Real {
    return new Real(call("csgrs_real_clone", this.handle));
  }

  /**
   * Request a finite lossy binary64 projection, with conversion failure handled by the loader.
   */
  toF64(): number {
    return call("csgrs_real_to_f64", this.handle);
  }

  /**
   * Parse the loader's decimal-string result as bigint.
   * The loader must translate the C two-word integer output to that string; the
   * native conversion requires a recognized in-range integer.
   */
  toI128(): bigint {
    return BigInt(call<string>("csgrs_real_to_i128", this.handle));
  }

  /**
   * Release this native scalar through the loader exactly once.
   * The handle is not cleared; subsequent use/free is invalid and not detected here.
   */
  free(): void {
    call("csgrs_real_free", this.handle);
  }
}

/**
 * Typed mesh handle facade over consumer-provided native calls.
 * Generic family compatibility is compile-time only. Operations borrow inputs and
 * adopt new loader results; explicit free is required, without a liveness guard.
 */
export class TriangleMesh<F extends Family> {
  /**
   * Boundary-family label used to form native symbol names; it does not convert geometry.
   */
  readonly family: F;
  /**
   * Loader-defined native mesh reference; no ownership/liveness check accompanies access.
   */
  readonly handle: unknown;

  /**
   * Adopt a family label and loader result without verifying either.
   */
  private constructor(family: F, handle: unknown) {
    this.family = family;
    this.handle = handle;
  }

  /**
   * Adopt an unchecked handle returned by another binding operation.
   * This public escape hatch does not clone, validate the family or prevent double ownership.
   * @internal
   */
  static fromNativeHandle<F extends Family>(family: F, handle: unknown): TriangleMesh<F> {
    return new TriangleMesh(family, handle);
  }

  /**
   * Request an origin-anchored cube with native strict nonnegative width validation.
   * Validated zero width yields canonical empty geometry; loader scalar conversion precedes it.
   */
  static cube<F extends Family>(family: F, width: Scalar<F>): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cube`, width));
  }

  /**
   * Request an origin-anchored XYZ box with all dimensions validated before empty collapse.
   */
  static cuboid<F extends Family>(
    family: F,
    width: Scalar<F>,
    length: Scalar<F>,
    height: Scalar<F>,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cuboid`, width, length, height));
  }

  /**
   * Request a sampled sphere with nonnegative radius and native minimum counts 3 and 2.
   * Y-axis poles and sampling history come from the native constructor. JS number
   * counts are forwarded without integer/range checks; the loader must marshal size_t.
   */
  static sphere<F extends Family>(
    family: F,
    radius: Scalar<F>,
    segments: number,
    stacks: number,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_sphere`, radius, segments, stacks));
  }

  /**
   * Request a sampled capped cylinder from Z zero to signed height.
   * Native validation requires nonnegative radius and at least three segments;
   * the wrapper does not validate the JS count before loader marshalling.
   */
  static cylinder<F extends Family>(
    family: F,
    radius: Scalar<F>,
    height: Scalar<F>,
    segments: number,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cylinder`, radius, height, segments));
  }

  /**
   * Flatten face indices and build sentinel offsets before requesting native triangulation.
   * The loader must marshal arrays/counts and validate integer index transport. Native
   * validation checks simple planar faces and authored winding, not whole-solid closure.
   */
  static polyhedron<F extends Family>(
    family: F,
    points: Array<Vec3<Scalar<F>>>,
    faces: number[][],
  ): TriangleMesh<F> {
    const faceIndices = faces.flat();
    const faceOffsets = [0];
    for (const face of faces) {
      faceOffsets.push(faceOffsets[faceOffsets.length - 1] + face.length);
    }
    return new TriangleMesh(
      family,
      call(`csgrs_triangle_mesh_${family}_polyhedron`, points, faceIndices, faceOffsets),
    );
  }

  /**
   * Request the strict regularized union of borrowed same-family native meshes.
   */
  union(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("union", other);
  }

  /**
   * Request strict regularized receiver material outside the other mesh.
   */
  difference(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("difference", other);
  }

  /**
   * Request strict regularized common material, retaining empty success as a mesh handle.
   */
  intersection(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("intersection", other);
  }

  /**
   * Request strict regularized material belonging to exactly one mesh operand.
   */
  xor(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("xor", other);
  }

  /**
   * Forward the row-major matrix for native strict checked homogeneous transformation.
   * Singular, undecided and unbounded images may fail; the loader owns matrix marshalling.
   */
  transform(matrix: Matrix4<Scalar<F>>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_transform`, this.handle, matrix));
  }

  /**
   * Request XYZ translation after loader scalar conversion.
   */
  translate(x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): TriangleMesh<F> {
    return this.triple("translate", x, y, z);
  }

  /**
   * Request strict axis scaling, rejecting zero axes and correcting reflection winding.
   */
  scale(sx: Scalar<F>, sy: Scalar<F>, sz: Scalar<F>): TriangleMesh<F> {
    return this.triple("scale", sx, sy, sz);
  }

  /**
   * Request X-then-Y-then-Z rotations with angles in degrees.
   */
  rotate(xDegrees: Scalar<F>, yDegrees: Scalar<F>, zDegrees: Scalar<F>): TriangleMesh<F> {
    return this.triple("rotate", xDegrees, yDegrees, zDegrees);
  }

  /**
   * Request triangle winding reversal, not an unbounded set complement.
   */
  inverse(): TriangleMesh<F> {
    return this.unary("inverse");
  }

  /**
   * Request native bounds-center placement at the origin, preserving canonical empty geometry.
   */
  center(): TriangleMesh<F> {
    return this.unary("center");
  }

  /**
   * Request native minimum-Z placement at zero; this is not scalar conversion.
   */
  floating(): TriangleMesh<F> {
    return this.unary("float");
  }

  /**
   * Request exported bounds with loader-defined shape, absence and ownership.
   * The loader must map NO_BOUNDS to absence and preserve errors. Float endpoints
   * are not outward-enclosure certificates; real endpoints need native cleanup.
   */
  boundingBox(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_bounding_box`, this.handle);
  }

  /**
   * Request indexed native positions/triangles in a loader-defined owning result.
   * Float projections can be lossy, integer conversion can fail and real outputs
   * own scalar handles. This facade provides no buffer-free method.
   */
  verticesAndIndices(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_vertices_and_indices`, this.handle);
  }

  /**
   * Request exported position/normal records and indices with loader-defined ownership.
   * Native normal certification or scalar export can fail. This facade supplies
   * neither a buffer-free method nor an output certainty/fidelity receipt.
   */
  graphicsMesh(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_graphics_mesh`, this.handle);
  }

  /**
   * Release the native mesh exactly once through the loader.
   * The handle is not cleared; repeated free/use is not detected here.
   */
  free(): void {
    call("csgrs_triangle_mesh_free", this.handle);
  }

  /**
   * Form a named unary symbol, borrow this handle and adopt the logical loader output.
   */
  private unary(op: string): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle));
  }

  /**
   * Form a named Boolean symbol and forward both handles without a runtime family check.
   */
  private binary(op: string, other: TriangleMesh<F>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle, other.handle));
  }

  /**
   * Forward three logical scalars to a named transform and adopt its loader output.
   */
  private triple(op: string, x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle, x, y, z));
  }
}

/**
 * Typed filled-region handle facade with manual, unguarded native lifetime.
 * Operations borrow inputs and adopt loader outputs. Generic types do not enforce
 * runtime family/provenance; the C ABI and loader retain those obligations.
 */
export class CurveRegion<F extends Family> {
  /**
   * Boundary label used for native symbol dispatch; it does not convert an existing region.
   */
  readonly family: F;
  /**
   * Loader-defined region reference, with no disposed-state guard.
   */
  readonly handle: unknown;

  /**
   * Adopt a family label and logical loader result without validation.
   */
  private constructor(family: F, handle: unknown) {
    this.family = family;
    this.handle = handle;
  }

  /**
   * Request an origin-anchored XY square; native strict validation admits nonnegative width.
   */
  static square<F extends Family>(family: F, width: Scalar<F>): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_square`, width));
  }

  /**
   * Request an XY rectangle with both dimensions validated as nonnegative before empty success.
   */
  static rectangle<F extends Family>(
    family: F,
    width: Scalar<F>,
    length: Scalar<F>,
  ): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_rectangle`, width, length));
  }

  /**
   * Request a sampled regular XY polygon, not an analytic circle.
   * Native validation requires nonnegative radius and at least three segments; the
   * loader must transport the unchecked JS count correctly.
   */
  static circle<F extends Family>(family: F, radius: Scalar<F>, segments: number): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_circle`, radius, segments));
  }

  /**
   * Forward XY rows for native strict filled-polygon construction.
   * The loader marshals points/count; an empty or malformed ring must remain a failure.
   */
  static polygon<F extends Family>(family: F, points: Array<Vec2<Scalar<F>>>): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_polygon`, points));
  }

  /**
   * Request regularized union using Hypercurve's active scope (normally strict).
   */
  union(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("union", other);
  }

  /**
   * Request regularized receiver material outside other in Hypercurve's active scope.
   */
  difference(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("difference", other);
  }

  /**
   * Request regularized common material in Hypercurve's active scope.
   */
  intersection(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("intersection", other);
  }

  /**
   * Request regularized material in exactly one region in Hypercurve's active scope.
   */
  xor(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("xor", other);
  }

  /**
   * Forward a row-major matrix for strict admission of a nonsingular affine XY image.
   * Perspective/out-of-plane maps can fail; the loader owns scalar/layout conversion.
   */
  transform(matrix: Matrix4<Scalar<F>>): CurveRegion<F> {
    return new CurveRegion(this.family, call(`csgrs_curve_region_${this.family}_transform`, this.handle, matrix));
  }

  /**
   * Request planar translation; nonzero Z is unsupported by the XY carrier.
   */
  translate(x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): CurveRegion<F> {
    return this.triple("translate", x, y, z);
  }

  /**
   * Request strict planar scaling; Z scale is imported but has no XY effect.
   */
  scale(sx: Scalar<F>, sy: Scalar<F>, sz: Scalar<F>): CurveRegion<F> {
    return this.triple("scale", sx, sy, sz);
  }

  /**
   * Request X-then-Y-then-Z rotations in degrees when the image remains in XY.
   */
  rotate(xDegrees: Scalar<F>, yDegrees: Scalar<F>, zDegrees: Scalar<F>): CurveRegion<F> {
    return this.triple("rotate", xDegrees, yDegrees, zDegrees);
  }

  /**
   * Request exported region bounds with loader-defined absence/ownership.
   * NO_BOUNDS is completed absence, not an error or zero box. Float endpoints are
   * not outward enclosures; real endpoint handles require matching native cleanup.
   */
  boundingBox(): unknown {
    return call(`csgrs_curve_region_${this.family}_bounding_box`, this.handle);
  }

  /**
   * Request strict extrusion along Z by a nonzero signed height into a matching-family mesh.
   * Nonlinear interiors may be sampled; the ABI discards the separate local receipt.
   */
  extrude(height: Scalar<F>): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_extrude`, this.handle, height),
    );
  }

  /**
   * Request strict extrusion along a vector with nonzero Z component.
   * The loader marshals the vector; native profile-lowering qualifications remain.
   */
  extrudeVector(direction: Vec3<Scalar<F>>): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_extrude_vector`, this.handle, direction),
    );
  }

  /**
   * Request a segmented swept boundary about Y, with X as signed radius.
   * Native angle rules reject zero and oversized noninteger turns; partial/full
   * coverage requires at least two/three segments. Partial axis crossings fail and
   * opposite-side pieces may overlap. Count conversion and error transport belong
   * to the loader. This is not an analytic surface of revolution.
   */
  revolve(angleDegrees: Scalar<F>, segments: number): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_revolve`, this.handle, angleDegrees, segments),
    );
  }

  /**
   * Request sampled exterior/hole profiles with loader-defined shape and cleanup.
   * Every family, including real, first passes through binary64 display points.
   * This is not exact native replay, and this facade has no profile-free method.
   */
  regionProfiles(): unknown {
    return call(`csgrs_curve_region_${this.family}_region_profiles`, this.handle);
  }

  /**
   * Release the region exactly once through the loader without clearing its handle.
   * Do not use/free it again; this facade installs no finalizer or liveness guard.
   */
  free(): void {
    call("csgrs_curve_region_free", this.handle);
  }

  /**
   * Build a named region Boolean call without runtime family checking and adopt its output.
   */
  private binary(op: string, other: CurveRegion<F>): CurveRegion<F> {
    return new CurveRegion(
      this.family,
      call(`csgrs_curve_region_${this.family}_${op}`, this.handle, other.handle),
    );
  }

  /**
   * Forward three logical scalars to a named planar transform and adopt its loader output.
   */
  private triple(op: string, x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): CurveRegion<F> {
    return new CurveRegion(
      this.family,
      call(`csgrs_curve_region_${this.family}_${op}`, this.handle, x, y, z),
    );
  }
}

/**
 * Compile-time TriangleMesh specialization for the F32 boundary; no separate runtime class.
 */
export type TriangleMeshF32 = TriangleMesh<"f32">;
/**
 * Compile-time TriangleMesh specialization for the F64 boundary; no separate runtime class.
 */
export type TriangleMeshF64 = TriangleMesh<"f64">;
/**
 * Compile-time TriangleMesh specialization for the I128 boundary; no separate runtime class.
 */
export type TriangleMeshI128 = TriangleMesh<"i128">;
/**
 * Compile-time TriangleMesh specialization for the Real boundary; no separate runtime class.
 */
export type TriangleMeshReal = TriangleMesh<"real">;
/**
 * Compile-time CurveRegion specialization for the F32 boundary; no separate runtime class.
 */
export type CurveRegionF32 = CurveRegion<"f32">;
/**
 * Compile-time CurveRegion specialization for the F64 boundary; no separate runtime class.
 */
export type CurveRegionF64 = CurveRegion<"f64">;
/**
 * Compile-time CurveRegion specialization for the I128 boundary; no separate runtime class.
 */
export type CurveRegionI128 = CurveRegion<"i128">;
/**
 * Compile-time CurveRegion specialization for the Real boundary; no separate runtime class.
 */
export type CurveRegionReal = CurveRegion<"real">;
