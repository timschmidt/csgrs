export type Family = "f32" | "f64" | "i128" | "real";
export type I128 = bigint;
export type Scalar<F extends Family> =
  F extends "f32" | "f64" ? number :
  F extends "i128" ? bigint :
  Real;

export type Vec2<S> = [S, S];
export type Vec3<S> = [S, S, S];
export type Matrix4<S> = [
  S, S, S, S,
  S, S, S, S,
  S, S, S, S,
  S, S, S, S,
];

export interface NativeLibrary {
  call<T = unknown>(name: string, ...args: unknown[]): T;
}

let native: NativeLibrary | undefined;

export function loadCsgrs(library: NativeLibrary): void {
  native = library;
}

function lib(): NativeLibrary {
  if (!native) {
    throw new Error("loadCsgrs() must be called with a native csgrs-ffi loader");
  }
  return native;
}

function call<T>(name: string, ...args: unknown[]): T {
  return lib().call<T>(name, ...args);
}

export class Real {
  readonly handle: unknown;

  private constructor(handle: unknown) {
    this.handle = handle;
  }

  static zero(): Real {
    return new Real(call("csgrs_real_zero"));
  }

  static one(): Real {
    return new Real(call("csgrs_real_one"));
  }

  static fromF64(value: number): Real {
    return new Real(call("csgrs_real_from_f64", value));
  }

  static fromI128(value: bigint): Real {
    return new Real(call("csgrs_real_from_i128_decimal", value.toString()));
  }

  clone(): Real {
    return new Real(call("csgrs_real_clone", this.handle));
  }

  toF64(): number {
    return call("csgrs_real_to_f64", this.handle);
  }

  toI128(): bigint {
    return BigInt(call<string>("csgrs_real_to_i128", this.handle));
  }

  free(): void {
    call("csgrs_real_free", this.handle);
  }
}

export class TriangleMesh<F extends Family> {
  readonly family: F;
  readonly handle: unknown;

  private constructor(family: F, handle: unknown) {
    this.family = family;
    this.handle = handle;
  }

  /** @internal Wrap a handle returned by another typed binding operation. */
  static fromNativeHandle<F extends Family>(family: F, handle: unknown): TriangleMesh<F> {
    return new TriangleMesh(family, handle);
  }

  static cube<F extends Family>(family: F, width: Scalar<F>): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cube`, width));
  }

  static cuboid<F extends Family>(
    family: F,
    width: Scalar<F>,
    length: Scalar<F>,
    height: Scalar<F>,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cuboid`, width, length, height));
  }

  static sphere<F extends Family>(
    family: F,
    radius: Scalar<F>,
    segments: number,
    stacks: number,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_sphere`, radius, segments, stacks));
  }

  static cylinder<F extends Family>(
    family: F,
    radius: Scalar<F>,
    height: Scalar<F>,
    segments: number,
  ): TriangleMesh<F> {
    return new TriangleMesh(family, call(`csgrs_triangle_mesh_${family}_cylinder`, radius, height, segments));
  }

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

  union(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("union", other);
  }

  difference(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("difference", other);
  }

  intersection(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("intersection", other);
  }

  xor(other: TriangleMesh<F>): TriangleMesh<F> {
    return this.binary("xor", other);
  }

  transform(matrix: Matrix4<Scalar<F>>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_transform`, this.handle, matrix));
  }

  translate(x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): TriangleMesh<F> {
    return this.triple("translate", x, y, z);
  }

  scale(sx: Scalar<F>, sy: Scalar<F>, sz: Scalar<F>): TriangleMesh<F> {
    return this.triple("scale", sx, sy, sz);
  }

  rotate(xDegrees: Scalar<F>, yDegrees: Scalar<F>, zDegrees: Scalar<F>): TriangleMesh<F> {
    return this.triple("rotate", xDegrees, yDegrees, zDegrees);
  }

  inverse(): TriangleMesh<F> {
    return this.unary("inverse");
  }

  center(): TriangleMesh<F> {
    return this.unary("center");
  }

  floating(): TriangleMesh<F> {
    return this.unary("float");
  }

  boundingBox(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_bounding_box`, this.handle);
  }

  verticesAndIndices(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_vertices_and_indices`, this.handle);
  }

  graphicsMesh(): unknown {
    return call(`csgrs_triangle_mesh_${this.family}_graphics_mesh`, this.handle);
  }

  free(): void {
    call("csgrs_triangle_mesh_free", this.handle);
  }

  private unary(op: string): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle));
  }

  private binary(op: string, other: TriangleMesh<F>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle, other.handle));
  }

  private triple(op: string, x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): TriangleMesh<F> {
    return new TriangleMesh(this.family, call(`csgrs_triangle_mesh_${this.family}_${op}`, this.handle, x, y, z));
  }
}

export class CurveRegion<F extends Family> {
  readonly family: F;
  readonly handle: unknown;

  private constructor(family: F, handle: unknown) {
    this.family = family;
    this.handle = handle;
  }

  static square<F extends Family>(family: F, width: Scalar<F>): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_square`, width));
  }

  static rectangle<F extends Family>(
    family: F,
    width: Scalar<F>,
    length: Scalar<F>,
  ): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_rectangle`, width, length));
  }

  static circle<F extends Family>(family: F, radius: Scalar<F>, segments: number): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_circle`, radius, segments));
  }

  static polygon<F extends Family>(family: F, points: Array<Vec2<Scalar<F>>>): CurveRegion<F> {
    return new CurveRegion(family, call(`csgrs_curve_region_${family}_polygon`, points));
  }

  union(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("union", other);
  }

  difference(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("difference", other);
  }

  intersection(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("intersection", other);
  }

  xor(other: CurveRegion<F>): CurveRegion<F> {
    return this.binary("xor", other);
  }

  transform(matrix: Matrix4<Scalar<F>>): CurveRegion<F> {
    return new CurveRegion(this.family, call(`csgrs_curve_region_${this.family}_transform`, this.handle, matrix));
  }

  translate(x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): CurveRegion<F> {
    return this.triple("translate", x, y, z);
  }

  scale(sx: Scalar<F>, sy: Scalar<F>, sz: Scalar<F>): CurveRegion<F> {
    return this.triple("scale", sx, sy, sz);
  }

  rotate(xDegrees: Scalar<F>, yDegrees: Scalar<F>, zDegrees: Scalar<F>): CurveRegion<F> {
    return this.triple("rotate", xDegrees, yDegrees, zDegrees);
  }

  boundingBox(): unknown {
    return call(`csgrs_curve_region_${this.family}_bounding_box`, this.handle);
  }

  extrude(height: Scalar<F>): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_extrude`, this.handle, height),
    );
  }

  extrudeVector(direction: Vec3<Scalar<F>>): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_extrude_vector`, this.handle, direction),
    );
  }

  revolve(angleDegrees: Scalar<F>, segments: number): TriangleMesh<F> {
    return TriangleMesh.fromNativeHandle(
      this.family,
      call(`csgrs_curve_region_${this.family}_revolve`, this.handle, angleDegrees, segments),
    );
  }

  regionProfiles(): unknown {
    return call(`csgrs_curve_region_${this.family}_region_profiles`, this.handle);
  }

  free(): void {
    call("csgrs_curve_region_free", this.handle);
  }

  private binary(op: string, other: CurveRegion<F>): CurveRegion<F> {
    return new CurveRegion(
      this.family,
      call(`csgrs_curve_region_${this.family}_${op}`, this.handle, other.handle),
    );
  }

  private triple(op: string, x: Scalar<F>, y: Scalar<F>, z: Scalar<F>): CurveRegion<F> {
    return new CurveRegion(
      this.family,
      call(`csgrs_curve_region_${this.family}_${op}`, this.handle, x, y, z),
    );
  }
}

export type TriangleMeshF32 = TriangleMesh<"f32">;
export type TriangleMeshF64 = TriangleMesh<"f64">;
export type TriangleMeshI128 = TriangleMesh<"i128">;
export type TriangleMeshReal = TriangleMesh<"real">;
export type CurveRegionF32 = CurveRegion<"f32">;
export type CurveRegionF64 = CurveRegion<"f64">;
export type CurveRegionI128 = CurveRegion<"i128">;
export type CurveRegionReal = CurveRegion<"real">;
