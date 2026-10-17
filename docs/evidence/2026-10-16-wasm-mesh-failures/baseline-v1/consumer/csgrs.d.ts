/* tslint:disable */
/* eslint-disable */

/** A native validation category carried by an ordinary JavaScript Error. */
export type GeometryValidationError = Error & {
    readonly name: "GeometryError";
    readonly code: "invalid_input" | "unsupported" | "undecided" |
    "resource_exhausted" | "unbounded" | "geometry_failure";
};

/**
 * Native interchange failure. These are host Error objects, not Wasm handles.
 * Inspect code before reading variant-specific fields. Construction causes
 * retain the native validation category; no geometry is returned on failure.
 */
export type GeometryIoError = Error & {
    readonly name: "GeometryIoError";
    readonly format: string;
} & (
| { readonly code: "std_io"; readonly ioKind: string; readonly osCode?: number; readonly cause: Error }
| { readonly code: "parse_float"; readonly cause: Error }
| { readonly code: "malformed_input"; readonly detail: string }
| { readonly code: "unrepresentable_coordinate"; readonly field: string; readonly target: string }
| { readonly code: "invalid_metadata"; readonly field: string }
| { readonly code: "size_overflow"; readonly limit: string }
| { readonly code: "unsupported"; readonly detail: string }
| { readonly code: "geometry"; readonly detail: string }
| { readonly code: "construction"; readonly cause: GeometryValidationError }
| { readonly code: "dxf"; readonly cause: Error }
| { readonly code: "gerber_parsing"; readonly detail: string }
| { readonly code: "gerber_codegen"; readonly cause: Error }
);



/**
 * JavaScript-facing curve Boolean value and aggregate predicate certainty.
 */
export class CurveBooleanResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    intoRegion(): CurveRegionJs;
    readonly approximate512Consumed: boolean;
    readonly certainty: GeometryCertaintyJs;
}

export class CurveRegionJs {
    free(): void;
    [Symbol.dispose](): void;
    boundingBoxWithContext(approximate_512: boolean): GeometryBoundsResultJs;
    static circle(radius: number, segments: number): CurveRegionJs;
    /**
     * Strict membership. Only a certified boundary returns an absent value;
     * an undecidable query throws. Use `containsXYWithContext` for typed status.
     */
    containsXY(x: number, y: number): boolean | undefined;
    containsXYWithContext(x: number, y: number, approximate_512: boolean): GeometryBoolResultJs;
    static crescent(outer_radius: number, inner_radius: number, offset: number, segments: number, approximate_512: boolean): CurveBooleanResultJs;
    difference(other: CurveRegionJs, approximate_512: boolean): CurveBooleanResultJs;
    static egg(width: number, length: number, segments: number): CurveRegionJs;
    static ellipse(width: number, height: number, segments: number): CurveRegionJs;
    extrude(height: number, approximate_512: boolean): GeometryMeshResultJs;
    extrudeTwisted(height: number, twist_degrees: number, end_scale_x: number, end_scale_y: number, slices: number, approximate_512: boolean): GeometryMeshResultJs;
    static fromRegionProfiles(value: any, approximate_512: boolean): CurveBooleanResultJs;
    static heart(width: number, height: number, segments: number): CurveRegionJs;
    intersection(other: CurveRegionJs, approximate_512: boolean): CurveBooleanResultJs;
    isEmpty(): boolean;
    constructor();
    static rectangle(width: number, height: number): CurveRegionJs;
    static regularNgon(sides: number, radius: number): CurveRegionJs;
    revolve(angle_degrees: number, segments: number, approximate_512: boolean): GeometryMeshResultJs;
    static rightTriangle(width: number, height: number): CurveRegionJs;
    static ring(inner_diameter: number, thickness: number, segments: number, approximate_512: boolean): CurveBooleanResultJs;
    rotate(degrees: number): CurveRegionJs;
    rotateWithContext(degrees: number, approximate_512: boolean): CurveBooleanResultJs;
    scale(x: number, y: number): CurveRegionJs;
    scaleWithContext(x: number, y: number, approximate_512: boolean): CurveBooleanResultJs;
    static square(width: number): CurveRegionJs;
    static star(points: number, outer_radius: number, inner_radius: number): CurveRegionJs;
    sweep(path: Point3Js[], approximate_512: boolean): GeometryMeshResultJs;
    static teardrop(width: number, length: number, segments: number): CurveRegionJs;
    toArrays(approximate_512: boolean): object;
    toRegionProfiles(approximate_512: boolean): any;
    transform(matrix: Matrix4Js): CurveRegionJs;
    transformWithContext(matrix: Matrix4Js, approximate_512: boolean): CurveBooleanResultJs;
    translate(x: number, y: number): CurveRegionJs;
    translateWithContext(x: number, y: number, approximate_512: boolean): CurveBooleanResultJs;
    union(other: CurveRegionJs, approximate_512: boolean): CurveBooleanResultJs;
    xor(other: CurveRegionJs, approximate_512: boolean): CurveBooleanResultJs;
}

/**
 * Point-query value, status, and optional certainty receipt.
 *
 * Read `status` before interpreting an absent `value`: it can denote a curve
 * boundary, curve uncertainty, or an unknown mesh query. An unresolved query
 * has no certainty receipt. Mesh membership is strict containment, so mesh
 * boundary points have a decided `false` value.
 */
export class GeometryBoolResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * True only for a completed query that consumed the approximate terminal.
     * A false value alone does not imply that the query was decided.
     */
    readonly approximate512Consumed: boolean;
    /**
     * Absent for an uncertain or unknown query.
     */
    readonly certainty: GeometryCertaintyJs | undefined;
    /**
     * Diagnostic reason for an unresolved query; absent for decided values.
     */
    readonly reason: string | undefined;
    readonly status: GeometryQueryStatusJs;
    readonly value: boolean | undefined;
}

/**
 * Bounds and aggregate predicate certainty.
 */
export class GeometryBoundsResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    readonly approximate512Consumed: boolean;
    readonly certainty: GeometryCertaintyJs;
    /**
     * True for a completed bounds query on empty geometry.
     */
    readonly isEmpty: boolean;
    readonly max: Point3Js | undefined;
    readonly min: Point3Js | undefined;
}

/**
 * Weakest predicate certainty consumed by a completed geometry operation.
 *
 * This describes topology decisions, not the fidelity of a float projection.
 */
export enum GeometryCertaintyJs {
    /**
     * All consumed decisions have exact certificates.
     */
    Certified = 0,
    /**
     * At least one decision consumed the authorized 512-bit terminal.
     */
    Approximate512Consumed = 1,
}

/**
 * Reflection matrix and aggregate predicate certainty.
 */
export class GeometryMatrixResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    intoMatrix(): Matrix4Js;
    readonly approximate512Consumed: boolean;
    readonly certainty: GeometryCertaintyJs;
}

/**
 * JavaScript-facing mesh value and aggregate predicate certainty.
 */
export class GeometryMeshResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    intoMesh(): MeshJs;
    readonly approximate512Consumed: boolean;
    readonly certainty: GeometryCertaintyJs;
}

/**
 * Plane construction and aggregate predicate certainty.
 */
export class GeometryPlaneResultJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    intoPlane(): PlaneJs;
    readonly approximate512Consumed: boolean;
    readonly certainty: GeometryCertaintyJs;
}

/**
 * Outcome of a point query, separate from its predicate certainty.
 */
export enum GeometryQueryStatusJs {
    /**
     * A Boolean membership value is available.
     */
    Decided = 0,
    /**
     * The curve query placed the point on the boundary under the selected policy.
     */
    Boundary = 1,
    /**
     * Hypercurve could not decide the query under the selected policy.
     */
    Uncertain = 2,
    /**
     * Hypermesh could not decide the query under the selected policy.
     */
    Unknown = 3,
}

export class Matrix4Js {
    free(): void;
    [Symbol.dispose](): void;
    constructor(m11: number, m12: number, m13: number, m20: number, m21: number, m22: number, m23: number, m30: number, m31: number, m32: number, m33: number, m34: number, m41: number, m42: number, m43: number, m44: number);
    toArray(): Float64Array;
}

export class MeshJs {
    free(): void;
    [Symbol.dispose](): void;
    boundingBox(): any;
    boundingBoxWithContext(approximate_512: boolean): GeometryBoundsResultJs;
    center(): MeshJs;
    centerWithContext(approximate_512: boolean): GeometryMeshResultJs;
    containsPoint(point: Point3Js): boolean;
    containsPointWithContext(point: Point3Js, approximate_512: boolean): GeometryBoolResultJs;
    convexHull(): MeshJs;
    convexHullWithContext(approximate_512: boolean): GeometryMeshResultJs;
    static cube(size: number): MeshJs;
    static cuboid(width: number, length: number, height: number): MeshJs;
    static cylinder(radius: number, height: number, segments: number): MeshJs;
    difference(other: MeshJs): MeshJs;
    differenceWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
    static ellipsoid(rx: number, ry: number, rz: number, segments: number, stacks: number): MeshJs;
    floatOnZ(): MeshJs;
    floatOnZWithContext(approximate_512: boolean): GeometryMeshResultJs;
    /**
     * Import finite coordinate triples and in-range index triples.
     *
     * Invalid buffers throw `GeometryError` with code `invalid_input` before
     * constructing a native mesh. Topology is validated by consuming operations.
     */
    static fromArrays(positions: Float64Array, indices: Uint32Array): MeshJs;
    /**
     * Import binary or ASCII STL through strict native closed-PWN validation.
     *
     * Canonical empty geometry is accepted. Coordinates parse as binary32 and
     * are promoted exactly; validation certifies that parsed geometry, not the
     * original exact construction or analytic source. Manifoldness is a separate
     * claim. Failure throws `GeometryIoError`; native validation remains in its
     * typed `cause`, and no mesh is returned. The byte buffer is not retained.
     */
    static fromSTL(data: Uint8Array): MeshJs;
    static frustum(bottom_radius: number, top_radius: number, height: number, segments: number): MeshJs;
    indices(): Uint32Array;
    intersection(other: MeshJs): MeshJs;
    intersectionWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
    inverse(): MeshJs;
    isEmpty(): boolean;
    /**
     * Strict convex hull of stored-position sums, explicitly convexifying gaps.
     */
    minkowskiHull(other: MeshJs): MeshJs;
    /**
     * Convex hull of stored-position sums, including unused positions.
     */
    minkowskiHullWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
    minkowskiSum(other: MeshJs): MeshJs;
    minkowskiSumWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
    /**
     * Represented-set sum with count limits; these are not time or heap-byte bounds.
     * Counts must be finite integers from zero through `u32::MAX`.
     */
    minkowskiSumWithLimits(other: MeshJs, approximate_512: boolean, max_cells: number, max_pair_sums: number, max_pair_points: number): GeometryMeshResultJs;
    mirror(plane: PlaneJs): MeshJs;
    mirrorWithContext(plane: PlaneJs, approximate_512: boolean): GeometryMeshResultJs;
    constructor();
    normals(): Float64Array;
    positions(): Float64Array;
    rotate(x: number, y: number, z: number): MeshJs;
    rotateWithContext(x: number, y: number, z: number, approximate_512: boolean): GeometryMeshResultJs;
    scale(x: number, y: number, z: number): MeshJs;
    scaleWithContext(x: number, y: number, z: number, approximate_512: boolean): GeometryMeshResultJs;
    static sphere(radius: number, segments: number, stacks: number): MeshJs;
    subdivide(levels: number): MeshJs;
    toArrays(): object;
    /**
     * Serialize exact decimal spellings of the finite binary64 projection.
     *
     * Failure throws `GeometryIoError` with queryable native fields rather than
     * a string. Reimport subsequently quantizes coordinates to binary32.
     */
    toSTLASCII(): string;
    /**
     * Serialize binary32 STL, rejecting collapsed projected triangles.
     *
     * Failure throws an ordinary `GeometryIoError` object with a native `code`,
     * `format`, and variant fields. This replaces the former string exception;
     * inspect `message` for text and `code` for the machine-readable variant.
     */
    toSTLBinary(): Uint8Array;
    static torus(major_radius: number, minor_radius: number, major_segments: number, minor_segments: number): MeshJs;
    transform(matrix: Matrix4Js): MeshJs;
    transformWithContext(matrix: Matrix4Js, approximate_512: boolean): GeometryMeshResultJs;
    translate(x: number, y: number, z: number): MeshJs;
    translateWithContext(x: number, y: number, z: number, approximate_512: boolean): GeometryMeshResultJs;
    triangleCount(): number;
    union(other: MeshJs): MeshJs;
    unionWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
    vertexCount(): number;
    xor(other: MeshJs): MeshJs;
    xorWithContext(other: MeshJs, approximate_512: boolean): GeometryMeshResultJs;
}

export class PlaneJs {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    static FromComponents(ax: number, ay: number, az: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number): PlaneJs;
    static FromComponentsWithContext(ax: number, ay: number, az: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number, approximate_512: boolean): GeometryPlaneResultJs;
    static FromNormal(normal: Vector3Js, offset: number): PlaneJs;
    static FromNormalComponents(nx: number, ny: number, nz: number, offset: number): PlaneJs;
    static FromNormalComponentsWithContext(nx: number, ny: number, nz: number, offset: number, approximate_512: boolean): GeometryPlaneResultJs;
    static FromNormalWithContext(normal: Vector3Js, offset: number, approximate_512: boolean): GeometryPlaneResultJs;
    static FromPoints(a: Point3Js, b: Point3Js, c: Point3Js): PlaneJs;
    static FromPointsWithContext(a: Point3Js, b: Point3Js, c: Point3Js, approximate_512: boolean): GeometryPlaneResultJs;
    flip(): void;
    normal(): Vector3Js;
    offset(): number;
    reflectionMatrix(): Matrix4Js;
    reflectionMatrixWithContext(approximate_512: boolean): GeometryMatrixResultJs;
}

export class Point3Js {
    free(): void;
    [Symbol.dispose](): void;
    constructor(x: number, y: number, z: number);
    toString(): string;
    readonly x: number;
    readonly y: number;
    readonly z: number;
}

export class Vector3Js {
    free(): void;
    [Symbol.dispose](): void;
    abs(): Vector3Js;
    add(other: Vector3Js): Vector3Js;
    angle(other: Vector3Js): number;
    cross(other: Vector3Js): Vector3Js;
    dot(other: Vector3Js): number;
    equals(other: Vector3Js): boolean;
    isOrthogonal(other: Vector3Js): boolean;
    length(): number;
    constructor(x: number, y: number, z: number);
    normalize(): Vector3Js;
    reverse(): Vector3Js;
    rotate(axis: Vector3Js, angle: number): Vector3Js;
    rotateQuaternion(w: number, x: number, y: number, z: number): Vector3Js;
    rotationBetween(other: Vector3Js): any;
    scale(factor: number): Vector3Js;
    subtract(other: Vector3Js): Vector3Js;
    toString(): string;
    readonly x: number;
    readonly y: number;
    readonly z: number;
}
