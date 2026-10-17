/* @ts-self-types="./csgrs.d.ts" */

/**
 * JavaScript-facing curve Boolean value and aggregate predicate certainty.
 */
class CurveBooleanResultJs {
    static __wrap(ptr) {
        const obj = Object.create(CurveBooleanResultJs.prototype);
        obj.__wbg_ptr = ptr;
        CurveBooleanResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CurveBooleanResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_curvebooleanresultjs_free(ptr, 0);
    }
    /**
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.curvebooleanresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {GeometryCertaintyJs}
     */
    get certainty() {
        const ret = wasm.curvebooleanresultjs_certainty(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {CurveRegionJs}
     */
    intoRegion() {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.curvebooleanresultjs_intoRegion(ptr);
        return CurveRegionJs.__wrap(ret);
    }
}
if (Symbol.dispose) CurveBooleanResultJs.prototype[Symbol.dispose] = CurveBooleanResultJs.prototype.free;
exports.CurveBooleanResultJs = CurveBooleanResultJs;

class CurveRegionJs {
    static __wrap(ptr) {
        const obj = Object.create(CurveRegionJs.prototype);
        obj.__wbg_ptr = ptr;
        CurveRegionJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CurveRegionJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_curveregionjs_free(ptr, 0);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryBoundsResultJs}
     */
    boundingBoxWithContext(approximate_512) {
        const ret = wasm.curveregionjs_boundingBoxWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryBoundsResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} radius
     * @param {number} segments
     * @returns {CurveRegionJs}
     */
    static circle(radius, segments) {
        const ret = wasm.curveregionjs_circle(radius, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * Strict membership. Only a certified boundary returns an absent value;
     * an undecidable query throws. Use `containsXYWithContext` for typed status.
     * @param {number} x
     * @param {number} y
     * @returns {boolean | undefined}
     */
    containsXY(x, y) {
        const ret = wasm.curveregionjs_containsXY(this.__wbg_ptr, x, y);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] === 0xFFFFFF ? undefined : ret[0] !== 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {boolean} approximate_512
     * @returns {GeometryBoolResultJs}
     */
    containsXYWithContext(x, y, approximate_512) {
        const ret = wasm.curveregionjs_containsXYWithContext(this.__wbg_ptr, x, y, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryBoolResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} outer_radius
     * @param {number} inner_radius
     * @param {number} offset
     * @param {number} segments
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    static crescent(outer_radius, inner_radius, offset, segments, approximate_512) {
        const ret = wasm.curveregionjs_crescent(outer_radius, inner_radius, offset, segments, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {CurveRegionJs} other
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    difference(other, approximate_512) {
        _assertClass(other, CurveRegionJs);
        const ret = wasm.curveregionjs_difference(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} length
     * @param {number} segments
     * @returns {CurveRegionJs}
     */
    static egg(width, length, segments) {
        const ret = wasm.curveregionjs_egg(width, length, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} height
     * @param {number} segments
     * @returns {CurveRegionJs}
     */
    static ellipse(width, height, segments) {
        const ret = wasm.curveregionjs_ellipse(width, height, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} height
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    extrude(height, approximate_512) {
        const ret = wasm.curveregionjs_extrude(this.__wbg_ptr, height, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} height
     * @param {number} twist_degrees
     * @param {number} end_scale_x
     * @param {number} end_scale_y
     * @param {number} slices
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    extrudeTwisted(height, twist_degrees, end_scale_x, end_scale_y, slices, approximate_512) {
        const ret = wasm.curveregionjs_extrudeTwisted(this.__wbg_ptr, height, twist_degrees, end_scale_x, end_scale_y, slices, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {any} value
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    static fromRegionProfiles(value, approximate_512) {
        const ret = wasm.curveregionjs_fromRegionProfiles(value, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} height
     * @param {number} segments
     * @returns {CurveRegionJs}
     */
    static heart(width, height, segments) {
        const ret = wasm.curveregionjs_heart(width, height, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {CurveRegionJs} other
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    intersection(other, approximate_512) {
        _assertClass(other, CurveRegionJs);
        const ret = wasm.curveregionjs_intersection(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {boolean}
     */
    isEmpty() {
        const ret = wasm.curveregionjs_isEmpty(this.__wbg_ptr);
        return ret !== 0;
    }
    constructor() {
        const ret = wasm.curveregionjs_new();
        this.__wbg_ptr = ret;
        CurveRegionJsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} width
     * @param {number} height
     * @returns {CurveRegionJs}
     */
    static rectangle(width, height) {
        const ret = wasm.curveregionjs_rectangle(width, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} sides
     * @param {number} radius
     * @returns {CurveRegionJs}
     */
    static regularNgon(sides, radius) {
        const ret = wasm.curveregionjs_regularNgon(sides, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} angle_degrees
     * @param {number} segments
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    revolve(angle_degrees, segments, approximate_512) {
        const ret = wasm.curveregionjs_revolve(this.__wbg_ptr, angle_degrees, segments, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} height
     * @returns {CurveRegionJs}
     */
    static rightTriangle(width, height) {
        const ret = wasm.curveregionjs_rightTriangle(width, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} inner_diameter
     * @param {number} thickness
     * @param {number} segments
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    static ring(inner_diameter, thickness, segments, approximate_512) {
        const ret = wasm.curveregionjs_ring(inner_diameter, thickness, segments, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} degrees
     * @returns {CurveRegionJs}
     */
    rotate(degrees) {
        const ret = wasm.curveregionjs_rotate(this.__wbg_ptr, degrees);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} degrees
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    rotateWithContext(degrees, approximate_512) {
        const ret = wasm.curveregionjs_rotateWithContext(this.__wbg_ptr, degrees, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @returns {CurveRegionJs}
     */
    scale(x, y) {
        const ret = wasm.curveregionjs_scale(this.__wbg_ptr, x, y);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    scaleWithContext(x, y, approximate_512) {
        const ret = wasm.curveregionjs_scaleWithContext(this.__wbg_ptr, x, y, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @returns {CurveRegionJs}
     */
    static square(width) {
        const ret = wasm.curveregionjs_square(width);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} points
     * @param {number} outer_radius
     * @param {number} inner_radius
     * @returns {CurveRegionJs}
     */
    static star(points, outer_radius, inner_radius) {
        const ret = wasm.curveregionjs_star(points, outer_radius, inner_radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {Point3Js[]} path
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    sweep(path, approximate_512) {
        const ptr0 = passArrayJsValueToWasm0(path, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.curveregionjs_sweep(this.__wbg_ptr, ptr0, len0, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} length
     * @param {number} segments
     * @returns {CurveRegionJs}
     */
    static teardrop(width, length, segments) {
        const ret = wasm.curveregionjs_teardrop(width, length, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {object}
     */
    toArrays(approximate_512) {
        const ret = wasm.curveregionjs_toArrays(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {any}
     */
    toRegionProfiles(approximate_512) {
        const ret = wasm.curveregionjs_toRegionProfiles(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * @param {Matrix4Js} matrix
     * @returns {CurveRegionJs}
     */
    transform(matrix) {
        _assertClass(matrix, Matrix4Js);
        const ret = wasm.curveregionjs_transform(this.__wbg_ptr, matrix.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {Matrix4Js} matrix
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    transformWithContext(matrix, approximate_512) {
        _assertClass(matrix, Matrix4Js);
        const ret = wasm.curveregionjs_transformWithContext(this.__wbg_ptr, matrix.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @returns {CurveRegionJs}
     */
    translate(x, y) {
        const ret = wasm.curveregionjs_translate(this.__wbg_ptr, x, y);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveRegionJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    translateWithContext(x, y, approximate_512) {
        const ret = wasm.curveregionjs_translateWithContext(this.__wbg_ptr, x, y, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {CurveRegionJs} other
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    union(other, approximate_512) {
        _assertClass(other, CurveRegionJs);
        const ret = wasm.curveregionjs_union(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
    /**
     * @param {CurveRegionJs} other
     * @param {boolean} approximate_512
     * @returns {CurveBooleanResultJs}
     */
    xor(other, approximate_512) {
        _assertClass(other, CurveRegionJs);
        const ret = wasm.curveregionjs_xor(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return CurveBooleanResultJs.__wrap(ret[0]);
    }
}
if (Symbol.dispose) CurveRegionJs.prototype[Symbol.dispose] = CurveRegionJs.prototype.free;
exports.CurveRegionJs = CurveRegionJs;

/**
 * Point-query value, status, and optional certainty receipt.
 *
 * Read `status` before interpreting an absent `value`: it can denote a curve
 * boundary, curve uncertainty, or an unknown mesh query. An unresolved query
 * has no certainty receipt. Mesh membership is strict containment, so mesh
 * boundary points have a decided `false` value.
 */
class GeometryBoolResultJs {
    static __wrap(ptr) {
        const obj = Object.create(GeometryBoolResultJs.prototype);
        obj.__wbg_ptr = ptr;
        GeometryBoolResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        GeometryBoolResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_geometryboolresultjs_free(ptr, 0);
    }
    /**
     * True only for a completed query that consumed the approximate terminal.
     * A false value alone does not imply that the query was decided.
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.geometryboolresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Absent for an uncertain or unknown query.
     * @returns {GeometryCertaintyJs | undefined}
     */
    get certainty() {
        const ret = wasm.geometryboolresultjs_certainty(this.__wbg_ptr);
        return ret === 2 ? undefined : ret;
    }
    /**
     * Diagnostic reason for an unresolved query; absent for decided values.
     * @returns {string | undefined}
     */
    get reason() {
        const ret = wasm.geometryboolresultjs_reason(this.__wbg_ptr);
        let v1;
        if (ret[0] !== 0) {
            v1 = getStringFromWasm0(ret[0], ret[1]).slice();
            wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        }
        return v1;
    }
    /**
     * @returns {GeometryQueryStatusJs}
     */
    get status() {
        const ret = wasm.geometryboolresultjs_status(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {boolean | undefined}
     */
    get value() {
        const ret = wasm.geometryboolresultjs_value(this.__wbg_ptr);
        return ret === 0xFFFFFF ? undefined : ret !== 0;
    }
}
if (Symbol.dispose) GeometryBoolResultJs.prototype[Symbol.dispose] = GeometryBoolResultJs.prototype.free;
exports.GeometryBoolResultJs = GeometryBoolResultJs;

/**
 * Bounds and aggregate predicate certainty.
 */
class GeometryBoundsResultJs {
    static __wrap(ptr) {
        const obj = Object.create(GeometryBoundsResultJs.prototype);
        obj.__wbg_ptr = ptr;
        GeometryBoundsResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        GeometryBoundsResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_geometryboundsresultjs_free(ptr, 0);
    }
    /**
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.geometryboundsresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {GeometryCertaintyJs}
     */
    get certainty() {
        const ret = wasm.geometryboundsresultjs_certainty(this.__wbg_ptr);
        return ret;
    }
    /**
     * True for a completed bounds query on empty geometry.
     * @returns {boolean}
     */
    get isEmpty() {
        const ret = wasm.geometryboundsresultjs_isEmpty(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {Point3Js | undefined}
     */
    get max() {
        const ret = wasm.geometryboundsresultjs_max(this.__wbg_ptr);
        return ret === 0 ? undefined : Point3Js.__wrap(ret);
    }
    /**
     * @returns {Point3Js | undefined}
     */
    get min() {
        const ret = wasm.geometryboundsresultjs_min(this.__wbg_ptr);
        return ret === 0 ? undefined : Point3Js.__wrap(ret);
    }
}
if (Symbol.dispose) GeometryBoundsResultJs.prototype[Symbol.dispose] = GeometryBoundsResultJs.prototype.free;
exports.GeometryBoundsResultJs = GeometryBoundsResultJs;

/**
 * Weakest predicate certainty consumed by a completed geometry operation.
 *
 * This describes topology decisions, not the fidelity of a float projection.
 * @enum {0 | 1}
 */
const GeometryCertaintyJs = Object.freeze({
    /**
     * All consumed decisions have exact certificates.
     */
    Certified: 0, "0": "Certified",
    /**
     * At least one decision consumed the authorized 512-bit terminal.
     */
    Approximate512Consumed: 1, "1": "Approximate512Consumed",
});
exports.GeometryCertaintyJs = GeometryCertaintyJs;

/**
 * Reflection matrix and aggregate predicate certainty.
 */
class GeometryMatrixResultJs {
    static __wrap(ptr) {
        const obj = Object.create(GeometryMatrixResultJs.prototype);
        obj.__wbg_ptr = ptr;
        GeometryMatrixResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        GeometryMatrixResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_geometrymatrixresultjs_free(ptr, 0);
    }
    /**
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.geometrymatrixresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {GeometryCertaintyJs}
     */
    get certainty() {
        const ret = wasm.geometrymatrixresultjs_certainty(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Matrix4Js}
     */
    intoMatrix() {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.geometrymatrixresultjs_intoMatrix(ptr);
        return Matrix4Js.__wrap(ret);
    }
}
if (Symbol.dispose) GeometryMatrixResultJs.prototype[Symbol.dispose] = GeometryMatrixResultJs.prototype.free;
exports.GeometryMatrixResultJs = GeometryMatrixResultJs;

/**
 * JavaScript-facing mesh value and aggregate predicate certainty.
 */
class GeometryMeshResultJs {
    static __wrap(ptr) {
        const obj = Object.create(GeometryMeshResultJs.prototype);
        obj.__wbg_ptr = ptr;
        GeometryMeshResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        GeometryMeshResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_geometrymeshresultjs_free(ptr, 0);
    }
    /**
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.geometrymeshresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {GeometryCertaintyJs}
     */
    get certainty() {
        const ret = wasm.geometrymeshresultjs_certainty(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {MeshJs}
     */
    intoMesh() {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.geometrymeshresultjs_intoMesh(ptr);
        return MeshJs.__wrap(ret);
    }
}
if (Symbol.dispose) GeometryMeshResultJs.prototype[Symbol.dispose] = GeometryMeshResultJs.prototype.free;
exports.GeometryMeshResultJs = GeometryMeshResultJs;

/**
 * Plane construction and aggregate predicate certainty.
 */
class GeometryPlaneResultJs {
    static __wrap(ptr) {
        const obj = Object.create(GeometryPlaneResultJs.prototype);
        obj.__wbg_ptr = ptr;
        GeometryPlaneResultJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        GeometryPlaneResultJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_geometryplaneresultjs_free(ptr, 0);
    }
    /**
     * @returns {boolean}
     */
    get approximate512Consumed() {
        const ret = wasm.geometryplaneresultjs_approximate512Consumed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {GeometryCertaintyJs}
     */
    get certainty() {
        const ret = wasm.geometryplaneresultjs_certainty(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {PlaneJs}
     */
    intoPlane() {
        const ptr = this.__destroy_into_raw();
        const ret = wasm.geometryplaneresultjs_intoPlane(ptr);
        return PlaneJs.__wrap(ret);
    }
}
if (Symbol.dispose) GeometryPlaneResultJs.prototype[Symbol.dispose] = GeometryPlaneResultJs.prototype.free;
exports.GeometryPlaneResultJs = GeometryPlaneResultJs;

/**
 * Outcome of a point query, separate from its predicate certainty.
 * @enum {0 | 1 | 2 | 3}
 */
const GeometryQueryStatusJs = Object.freeze({
    /**
     * A Boolean membership value is available.
     */
    Decided: 0, "0": "Decided",
    /**
     * The curve query placed the point on the boundary under the selected policy.
     */
    Boundary: 1, "1": "Boundary",
    /**
     * Hypercurve could not decide the query under the selected policy.
     */
    Uncertain: 2, "2": "Uncertain",
    /**
     * Hypermesh could not decide the query under the selected policy.
     */
    Unknown: 3, "3": "Unknown",
});
exports.GeometryQueryStatusJs = GeometryQueryStatusJs;

class Matrix4Js {
    static __wrap(ptr) {
        const obj = Object.create(Matrix4Js.prototype);
        obj.__wbg_ptr = ptr;
        Matrix4JsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        Matrix4JsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_matrix4js_free(ptr, 0);
    }
    /**
     * @param {number} m11
     * @param {number} m12
     * @param {number} m13
     * @param {number} m20
     * @param {number} m21
     * @param {number} m22
     * @param {number} m23
     * @param {number} m30
     * @param {number} m31
     * @param {number} m32
     * @param {number} m33
     * @param {number} m34
     * @param {number} m41
     * @param {number} m42
     * @param {number} m43
     * @param {number} m44
     */
    constructor(m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, m34, m41, m42, m43, m44) {
        const ret = wasm.matrix4js_new(m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, m34, m41, m42, m43, m44);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        Matrix4JsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {Float64Array}
     */
    toArray() {
        const ret = wasm.matrix4js_toArray(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
}
if (Symbol.dispose) Matrix4Js.prototype[Symbol.dispose] = Matrix4Js.prototype.free;
exports.Matrix4Js = Matrix4Js;

class MeshJs {
    static __wrap(ptr) {
        const obj = Object.create(MeshJs.prototype);
        obj.__wbg_ptr = ptr;
        MeshJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        MeshJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_meshjs_free(ptr, 0);
    }
    /**
     * @returns {any}
     */
    boundingBox() {
        const ret = wasm.meshjs_boundingBox(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryBoundsResultJs}
     */
    boundingBoxWithContext(approximate_512) {
        const ret = wasm.meshjs_boundingBoxWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryBoundsResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {MeshJs}
     */
    center() {
        const ret = wasm.meshjs_center(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    centerWithContext(approximate_512) {
        const ret = wasm.meshjs_centerWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {Point3Js} point
     * @returns {boolean}
     */
    containsPoint(point) {
        _assertClass(point, Point3Js);
        const ret = wasm.meshjs_containsPoint(this.__wbg_ptr, point.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * @param {Point3Js} point
     * @param {boolean} approximate_512
     * @returns {GeometryBoolResultJs}
     */
    containsPointWithContext(point, approximate_512) {
        _assertClass(point, Point3Js);
        const ret = wasm.meshjs_containsPointWithContext(this.__wbg_ptr, point.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryBoolResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {MeshJs}
     */
    convexHull() {
        const ret = wasm.meshjs_convexHull(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    convexHullWithContext(approximate_512) {
        const ret = wasm.meshjs_convexHullWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} size
     * @returns {MeshJs}
     */
    static cube(size) {
        const ret = wasm.meshjs_cube(size);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} width
     * @param {number} length
     * @param {number} height
     * @returns {MeshJs}
     */
    static cuboid(width, length, height) {
        const ret = wasm.meshjs_cuboid(width, length, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} radius
     * @param {number} height
     * @param {number} segments
     * @returns {MeshJs}
     */
    static cylinder(radius, height, segments) {
        const ret = wasm.meshjs_cylinder(radius, height, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    difference(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_difference(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    differenceWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_differenceWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} rx
     * @param {number} ry
     * @param {number} rz
     * @param {number} segments
     * @param {number} stacks
     * @returns {MeshJs}
     */
    static ellipsoid(rx, ry, rz, segments, stacks) {
        const ret = wasm.meshjs_ellipsoid(rx, ry, rz, segments, stacks);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @returns {MeshJs}
     */
    floatOnZ() {
        const ret = wasm.meshjs_floatOnZ(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    floatOnZWithContext(approximate_512) {
        const ret = wasm.meshjs_floatOnZWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * Import finite coordinate triples and in-range index triples.
     *
     * Invalid buffers throw `GeometryError` with code `invalid_input` before
     * constructing a native mesh. Topology is validated by consuming operations.
     * @param {Float64Array} positions
     * @param {Uint32Array} indices
     * @returns {MeshJs}
     */
    static fromArrays(positions, indices) {
        const ptr0 = passArrayF64ToWasm0(positions, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArray32ToWasm0(indices, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.meshjs_fromArrays(ptr0, len0, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * Import binary or ASCII STL through strict native closed-PWN validation.
     *
     * Canonical empty geometry is accepted. Coordinates parse as binary32 and
     * are promoted exactly; validation certifies that parsed geometry, not the
     * original exact construction or analytic source. Manifoldness is a separate
     * claim. Failure throws `GeometryIoError`; native validation remains in its
     * typed `cause`, and no mesh is returned. The byte buffer is not retained.
     * @param {Uint8Array} data
     * @returns {MeshJs}
     */
    static fromSTL(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.meshjs_fromSTL(ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} bottom_radius
     * @param {number} top_radius
     * @param {number} height
     * @param {number} segments
     * @returns {MeshJs}
     */
    static frustum(bottom_radius, top_radius, height, segments) {
        const ret = wasm.meshjs_frustum(bottom_radius, top_radius, height, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @returns {Uint32Array}
     */
    indices() {
        const ret = wasm.meshjs_indices(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    intersection(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_intersection(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    intersectionWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_intersectionWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {MeshJs}
     */
    inverse() {
        const ret = wasm.meshjs_inverse(this.__wbg_ptr);
        return MeshJs.__wrap(ret);
    }
    /**
     * @returns {boolean}
     */
    isEmpty() {
        const ret = wasm.meshjs_isEmpty(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Strict convex hull of stored-position sums, explicitly convexifying gaps.
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    minkowskiHull(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_minkowskiHull(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * Convex hull of stored-position sums, including unused positions.
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    minkowskiHullWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_minkowskiHullWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    minkowskiSum(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_minkowskiSum(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    minkowskiSumWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_minkowskiSumWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * Represented-set sum with count limits; these are not time or heap-byte bounds.
     * Counts must be finite integers from zero through `u32::MAX`.
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @param {number} max_cells
     * @param {number} max_pair_sums
     * @param {number} max_pair_points
     * @returns {GeometryMeshResultJs}
     */
    minkowskiSumWithLimits(other, approximate_512, max_cells, max_pair_sums, max_pair_points) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_minkowskiSumWithLimits(this.__wbg_ptr, other.__wbg_ptr, approximate_512, max_cells, max_pair_sums, max_pair_points);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {PlaneJs} plane
     * @returns {MeshJs}
     */
    mirror(plane) {
        _assertClass(plane, PlaneJs);
        const ret = wasm.meshjs_mirror(this.__wbg_ptr, plane.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {PlaneJs} plane
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    mirrorWithContext(plane, approximate_512) {
        _assertClass(plane, PlaneJs);
        const ret = wasm.meshjs_mirrorWithContext(this.__wbg_ptr, plane.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    constructor() {
        const ret = wasm.meshjs_new();
        this.__wbg_ptr = ret;
        MeshJsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {Float64Array}
     */
    normals() {
        const ret = wasm.meshjs_normals(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Float64Array}
     */
    positions() {
        const ret = wasm.meshjs_positions(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {MeshJs}
     */
    rotate(x, y, z) {
        const ret = wasm.meshjs_rotate(this.__wbg_ptr, x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    rotateWithContext(x, y, z, approximate_512) {
        const ret = wasm.meshjs_rotateWithContext(this.__wbg_ptr, x, y, z, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {MeshJs}
     */
    scale(x, y, z) {
        const ret = wasm.meshjs_scale(this.__wbg_ptr, x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    scaleWithContext(x, y, z, approximate_512) {
        const ret = wasm.meshjs_scaleWithContext(this.__wbg_ptr, x, y, z, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} radius
     * @param {number} segments
     * @param {number} stacks
     * @returns {MeshJs}
     */
    static sphere(radius, segments, stacks) {
        const ret = wasm.meshjs_sphere(radius, segments, stacks);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} levels
     * @returns {MeshJs}
     */
    subdivide(levels) {
        const ret = wasm.meshjs_subdivide(this.__wbg_ptr, levels);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @returns {object}
     */
    toArrays() {
        const ret = wasm.meshjs_toArrays(this.__wbg_ptr);
        return ret;
    }
    /**
     * Serialize exact decimal spellings of the finite binary64 projection.
     *
     * Failure throws `GeometryIoError` with queryable native fields rather than
     * a string. Reimport subsequently quantizes coordinates to binary32.
     * @returns {string}
     */
    toSTLASCII() {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.meshjs_toSTLASCII(this.__wbg_ptr);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Serialize binary32 STL, rejecting collapsed projected triangles.
     *
     * Failure throws an ordinary `GeometryIoError` object with a native `code`,
     * `format`, and variant fields. This replaces the former string exception;
     * inspect `message` for text and `code` for the machine-readable variant.
     * @returns {Uint8Array}
     */
    toSTLBinary() {
        const ret = wasm.meshjs_toSTLBinary(this.__wbg_ptr);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * @param {number} major_radius
     * @param {number} minor_radius
     * @param {number} major_segments
     * @param {number} minor_segments
     * @returns {MeshJs}
     */
    static torus(major_radius, minor_radius, major_segments, minor_segments) {
        const ret = wasm.meshjs_torus(major_radius, minor_radius, major_segments, minor_segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {Matrix4Js} matrix
     * @returns {MeshJs}
     */
    transform(matrix) {
        _assertClass(matrix, Matrix4Js);
        const ret = wasm.meshjs_transform(this.__wbg_ptr, matrix.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {Matrix4Js} matrix
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    transformWithContext(matrix, approximate_512) {
        _assertClass(matrix, Matrix4Js);
        const ret = wasm.meshjs_transformWithContext(this.__wbg_ptr, matrix.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {MeshJs}
     */
    translate(x, y, z) {
        const ret = wasm.meshjs_translate(this.__wbg_ptr, x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    translateWithContext(x, y, z, approximate_512) {
        const ret = wasm.meshjs_translateWithContext(this.__wbg_ptr, x, y, z, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {number}
     */
    triangleCount() {
        const ret = wasm.meshjs_triangleCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    union(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_union(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    unionWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_unionWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
    /**
     * @returns {number}
     */
    vertexCount() {
        const ret = wasm.meshjs_vertexCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @param {MeshJs} other
     * @returns {MeshJs}
     */
    xor(other) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_xor(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return MeshJs.__wrap(ret[0]);
    }
    /**
     * @param {MeshJs} other
     * @param {boolean} approximate_512
     * @returns {GeometryMeshResultJs}
     */
    xorWithContext(other, approximate_512) {
        _assertClass(other, MeshJs);
        const ret = wasm.meshjs_xorWithContext(this.__wbg_ptr, other.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMeshResultJs.__wrap(ret[0]);
    }
}
if (Symbol.dispose) MeshJs.prototype[Symbol.dispose] = MeshJs.prototype.free;
exports.MeshJs = MeshJs;

class PlaneJs {
    static __wrap(ptr) {
        const obj = Object.create(PlaneJs.prototype);
        obj.__wbg_ptr = ptr;
        PlaneJsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        PlaneJsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_planejs_free(ptr, 0);
    }
    /**
     * @param {number} ax
     * @param {number} ay
     * @param {number} az
     * @param {number} bx
     * @param {number} by
     * @param {number} bz
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @returns {PlaneJs}
     */
    static FromComponents(ax, ay, az, bx, by, bz, cx, cy, cz) {
        const ret = wasm.planejs_FromComponents(ax, ay, az, bx, by, bz, cx, cy, cz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return PlaneJs.__wrap(ret[0]);
    }
    /**
     * @param {number} ax
     * @param {number} ay
     * @param {number} az
     * @param {number} bx
     * @param {number} by
     * @param {number} bz
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @param {boolean} approximate_512
     * @returns {GeometryPlaneResultJs}
     */
    static FromComponentsWithContext(ax, ay, az, bx, by, bz, cx, cy, cz, approximate_512) {
        const ret = wasm.planejs_FromComponentsWithContext(ax, ay, az, bx, by, bz, cx, cy, cz, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryPlaneResultJs.__wrap(ret[0]);
    }
    /**
     * @param {Vector3Js} normal
     * @param {number} offset
     * @returns {PlaneJs}
     */
    static FromNormal(normal, offset) {
        _assertClass(normal, Vector3Js);
        const ret = wasm.planejs_FromNormal(normal.__wbg_ptr, offset);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return PlaneJs.__wrap(ret[0]);
    }
    /**
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} offset
     * @returns {PlaneJs}
     */
    static FromNormalComponents(nx, ny, nz, offset) {
        const ret = wasm.planejs_FromNormalComponents(nx, ny, nz, offset);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return PlaneJs.__wrap(ret[0]);
    }
    /**
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} offset
     * @param {boolean} approximate_512
     * @returns {GeometryPlaneResultJs}
     */
    static FromNormalComponentsWithContext(nx, ny, nz, offset, approximate_512) {
        const ret = wasm.planejs_FromNormalComponentsWithContext(nx, ny, nz, offset, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryPlaneResultJs.__wrap(ret[0]);
    }
    /**
     * @param {Vector3Js} normal
     * @param {number} offset
     * @param {boolean} approximate_512
     * @returns {GeometryPlaneResultJs}
     */
    static FromNormalWithContext(normal, offset, approximate_512) {
        _assertClass(normal, Vector3Js);
        const ret = wasm.planejs_FromNormalWithContext(normal.__wbg_ptr, offset, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryPlaneResultJs.__wrap(ret[0]);
    }
    /**
     * @param {Point3Js} a
     * @param {Point3Js} b
     * @param {Point3Js} c
     * @returns {PlaneJs}
     */
    static FromPoints(a, b, c) {
        _assertClass(a, Point3Js);
        _assertClass(b, Point3Js);
        _assertClass(c, Point3Js);
        const ret = wasm.planejs_FromPoints(a.__wbg_ptr, b.__wbg_ptr, c.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return PlaneJs.__wrap(ret[0]);
    }
    /**
     * @param {Point3Js} a
     * @param {Point3Js} b
     * @param {Point3Js} c
     * @param {boolean} approximate_512
     * @returns {GeometryPlaneResultJs}
     */
    static FromPointsWithContext(a, b, c, approximate_512) {
        _assertClass(a, Point3Js);
        _assertClass(b, Point3Js);
        _assertClass(c, Point3Js);
        const ret = wasm.planejs_FromPointsWithContext(a.__wbg_ptr, b.__wbg_ptr, c.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryPlaneResultJs.__wrap(ret[0]);
    }
    flip() {
        wasm.planejs_flip(this.__wbg_ptr);
    }
    /**
     * @returns {Vector3Js}
     */
    normal() {
        const ret = wasm.planejs_normal(this.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @returns {number}
     */
    offset() {
        const ret = wasm.planejs_offset(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Matrix4Js}
     */
    reflectionMatrix() {
        const ret = wasm.planejs_reflectionMatrix(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return Matrix4Js.__wrap(ret[0]);
    }
    /**
     * @param {boolean} approximate_512
     * @returns {GeometryMatrixResultJs}
     */
    reflectionMatrixWithContext(approximate_512) {
        const ret = wasm.planejs_reflectionMatrixWithContext(this.__wbg_ptr, approximate_512);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return GeometryMatrixResultJs.__wrap(ret[0]);
    }
}
if (Symbol.dispose) PlaneJs.prototype[Symbol.dispose] = PlaneJs.prototype.free;
exports.PlaneJs = PlaneJs;

class Point3Js {
    static __wrap(ptr) {
        const obj = Object.create(Point3Js.prototype);
        obj.__wbg_ptr = ptr;
        Point3JsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    static __unwrap(jsValue) {
        if (!(jsValue instanceof Point3Js)) {
            return 0;
        }
        return jsValue.__destroy_into_raw();
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        Point3JsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_point3js_free(ptr, 0);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    constructor(x, y, z) {
        const ret = wasm.point3js_new(x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        Point3JsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {string}
     */
    toString() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.point3js_toString(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * @returns {number}
     */
    get x() {
        const ret = wasm.point3js_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get y() {
        const ret = wasm.point3js_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get z() {
        const ret = wasm.point3js_z(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) Point3Js.prototype[Symbol.dispose] = Point3Js.prototype.free;
exports.Point3Js = Point3Js;

class Vector3Js {
    static __wrap(ptr) {
        const obj = Object.create(Vector3Js.prototype);
        obj.__wbg_ptr = ptr;
        Vector3JsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        Vector3JsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_vector3js_free(ptr, 0);
    }
    /**
     * @returns {Vector3Js}
     */
    abs() {
        const ret = wasm.vector3js_abs(this.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @param {Vector3Js} other
     * @returns {Vector3Js}
     */
    add(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_add(this.__wbg_ptr, other.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @param {Vector3Js} other
     * @returns {number}
     */
    angle(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_angle(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
    /**
     * @param {Vector3Js} other
     * @returns {Vector3Js}
     */
    cross(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_cross(this.__wbg_ptr, other.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @param {Vector3Js} other
     * @returns {number}
     */
    dot(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_dot(this.__wbg_ptr, other.__wbg_ptr);
        return ret;
    }
    /**
     * @param {Vector3Js} other
     * @returns {boolean}
     */
    equals(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_equals(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * @param {Vector3Js} other
     * @returns {boolean}
     */
    isOrthogonal(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_isOrthogonal(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * @returns {number}
     */
    length() {
        const ret = wasm.vector3js_length(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    constructor(x, y, z) {
        const ret = wasm.vector3js_new(x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        Vector3JsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {Vector3Js}
     */
    normalize() {
        const ret = wasm.vector3js_normalize(this.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return Vector3Js.__wrap(ret[0]);
    }
    /**
     * @returns {Vector3Js}
     */
    reverse() {
        const ret = wasm.vector3js_reverse(this.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @param {Vector3Js} axis
     * @param {number} angle
     * @returns {Vector3Js}
     */
    rotate(axis, angle) {
        _assertClass(axis, Vector3Js);
        const ret = wasm.vector3js_rotate(this.__wbg_ptr, axis.__wbg_ptr, angle);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return Vector3Js.__wrap(ret[0]);
    }
    /**
     * @param {number} w
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {Vector3Js}
     */
    rotateQuaternion(w, x, y, z) {
        const ret = wasm.vector3js_rotateQuaternion(this.__wbg_ptr, w, x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return Vector3Js.__wrap(ret[0]);
    }
    /**
     * @param {Vector3Js} other
     * @returns {any}
     */
    rotationBetween(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_rotationBetween(this.__wbg_ptr, other.__wbg_ptr);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * @param {number} factor
     * @returns {Vector3Js}
     */
    scale(factor) {
        const ret = wasm.vector3js_scale(this.__wbg_ptr, factor);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return Vector3Js.__wrap(ret[0]);
    }
    /**
     * @param {Vector3Js} other
     * @returns {Vector3Js}
     */
    subtract(other) {
        _assertClass(other, Vector3Js);
        const ret = wasm.vector3js_subtract(this.__wbg_ptr, other.__wbg_ptr);
        return Vector3Js.__wrap(ret);
    }
    /**
     * @returns {string}
     */
    toString() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.vector3js_toString(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * @returns {number}
     */
    get x() {
        const ret = wasm.vector3js_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get y() {
        const ret = wasm.vector3js_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get z() {
        const ret = wasm.vector3js_z(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) Vector3Js.prototype[Symbol.dispose] = Vector3Js.prototype.free;
exports.Vector3Js = Vector3Js;
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg_Error_92b29b0548f8b746: function(arg0, arg1) {
            const ret = Error(getStringFromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_String_8564e559799eccda: function(arg0, arg1) {
            const ret = String(arg1);
            const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbg___wbindgen_boolean_get_fa956cfa2d1bd751: function(arg0) {
            const v = arg0;
            const ret = typeof(v) === 'boolean' ? v : undefined;
            return isLikeNone(ret) ? 0xFFFFFF : ret ? 1 : 0;
        },
        __wbg___wbindgen_debug_string_c25d447a39f5578f: function(arg0, arg1) {
            const ret = debugString(arg1);
            const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbg___wbindgen_in_aca499c5de7ff5e5: function(arg0, arg1) {
            const ret = arg0 in arg1;
            return ret;
        },
        __wbg___wbindgen_is_function_1ff95bcc5517c252: function(arg0) {
            const ret = typeof(arg0) === 'function';
            return ret;
        },
        __wbg___wbindgen_is_object_a27215656b807791: function(arg0) {
            const val = arg0;
            const ret = typeof(val) === 'object' && val !== null;
            return ret;
        },
        __wbg___wbindgen_is_undefined_c05833b95a3cf397: function(arg0) {
            const ret = arg0 === undefined;
            return ret;
        },
        __wbg___wbindgen_jsval_loose_eq_db4c3b15f63fc170: function(arg0, arg1) {
            const ret = arg0 == arg1;
            return ret;
        },
        __wbg___wbindgen_number_get_394265ed1e1b84ee: function(arg0, arg1) {
            const obj = arg1;
            const ret = typeof(obj) === 'number' ? obj : undefined;
            getDataViewMemory0().setFloat64(arg0 + 8 * 1, isLikeNone(ret) ? 0 : ret, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, !isLikeNone(ret), true);
        },
        __wbg___wbindgen_string_get_b0ca35b86a603356: function(arg0, arg1) {
            const obj = arg1;
            const ret = typeof(obj) === 'string' ? obj : undefined;
            var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            var len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbg___wbindgen_throw_344f42d3211c4765: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_call_8a2dd23819f8a60a: function() { return handleError(function (arg0, arg1) {
            const ret = arg0.call(arg1);
            return ret;
        }, arguments); },
        __wbg_done_89b2b13e91a60321: function(arg0) {
            const ret = arg0.done;
            return ret;
        },
        __wbg_get_78f252d074a84d0b: function() { return handleError(function (arg0, arg1) {
            const ret = Reflect.get(arg0, arg1);
            return ret;
        }, arguments); },
        __wbg_get_c7eb1f358a7654df: function() { return handleError(function (arg0, arg1) {
            const ret = Reflect.get(arg0, arg1);
            return ret;
        }, arguments); },
        __wbg_get_unchecked_6e0ad6d2a41b06f6: function(arg0, arg1) {
            const ret = arg0[arg1 >>> 0];
            return ret;
        },
        __wbg_get_with_ref_key_6412cf3094599694: function(arg0, arg1) {
            const ret = arg0[arg1];
            return ret;
        },
        __wbg_instanceof_ArrayBuffer_4480b9e0068a8adb: function(arg0) {
            let result;
            try {
                result = arg0 instanceof ArrayBuffer;
            } catch (_) {
                result = false;
            }
            const ret = result;
            return ret;
        },
        __wbg_instanceof_Uint8Array_309b927aaf7a3fc7: function(arg0) {
            let result;
            try {
                result = arg0 instanceof Uint8Array;
            } catch (_) {
                result = false;
            }
            const ret = result;
            return ret;
        },
        __wbg_isArray_0677c962b281d01a: function(arg0) {
            const ret = Array.isArray(arg0);
            return ret;
        },
        __wbg_iterator_6f722e4a93058b71: function() {
            const ret = Symbol.iterator;
            return ret;
        },
        __wbg_length_1f0964f4a5e2c6d8: function(arg0) {
            const ret = arg0.length;
            return ret;
        },
        __wbg_length_370319915dc99107: function(arg0) {
            const ret = arg0.length;
            return ret;
        },
        __wbg_new_32b398fb48b6d94a: function() {
            const ret = new Array();
            return ret;
        },
        __wbg_new_b667d279fd5aa943: function(arg0, arg1) {
            const ret = new Error(getStringFromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_cd45aabdf6073e84: function(arg0) {
            const ret = new Uint8Array(arg0);
            return ret;
        },
        __wbg_new_da52cf8fe3429cb2: function() {
            const ret = new Object();
            return ret;
        },
        __wbg_new_from_slice_7568ba55b4a7e81f: function(arg0, arg1) {
            const ret = new Uint32Array(getArrayU32FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_7e254b47c77fb8cc: function(arg0, arg1) {
            const ret = new Float64Array(getArrayF64FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_next_6dbf2c0ac8cde20f: function(arg0) {
            const ret = arg0.next;
            return ret;
        },
        __wbg_next_71f2aa1cb3d1e37e: function() { return handleError(function (arg0) {
            const ret = arg0.next();
            return ret;
        }, arguments); },
        __wbg_point3js_new: function(arg0) {
            const ret = Point3Js.__wrap(arg0);
            return ret;
        },
        __wbg_point3js_unwrap: function(arg0) {
            const ret = Point3Js.__unwrap(arg0);
            return ret;
        },
        __wbg_prototypesetcall_4770620bbe4688a0: function(arg0, arg1, arg2) {
            Uint8Array.prototype.set.call(getArrayU8FromWasm0(arg0, arg1), arg2);
        },
        __wbg_set_6be42768c690e380: function(arg0, arg1, arg2) {
            arg0[arg1] = arg2;
        },
        __wbg_set_8535240470bf2500: function() { return handleError(function (arg0, arg1, arg2) {
            const ret = Reflect.set(arg0, arg1, arg2);
            return ret;
        }, arguments); },
        __wbg_set_8a16b38e4805b298: function(arg0, arg1, arg2) {
            arg0[arg1 >>> 0] = arg2;
        },
        __wbg_set_name_3bbc583faefa4193: function(arg0, arg1, arg2) {
            arg0.name = getStringFromWasm0(arg1, arg2);
        },
        __wbg_value_a5d5488a9589444a: function(arg0) {
            const ret = arg0.value;
            return ret;
        },
        __wbindgen_cast_0000000000000001: function(arg0) {
            // Cast intrinsic for `F64 -> Externref`.
            const ret = arg0;
            return ret;
        },
        __wbindgen_cast_0000000000000002: function(arg0, arg1) {
            // Cast intrinsic for `Ref(String) -> Externref`.
            const ret = getStringFromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./csgrs_bg.js": import0,
    };
}

const CurveBooleanResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_curvebooleanresultjs_free(ptr, 1));
const CurveRegionJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_curveregionjs_free(ptr, 1));
const GeometryBoolResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_geometryboolresultjs_free(ptr, 1));
const GeometryBoundsResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_geometryboundsresultjs_free(ptr, 1));
const GeometryMatrixResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_geometrymatrixresultjs_free(ptr, 1));
const GeometryMeshResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_geometrymeshresultjs_free(ptr, 1));
const GeometryPlaneResultJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_geometryplaneresultjs_free(ptr, 1));
const Matrix4JsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_matrix4js_free(ptr, 1));
const MeshJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_meshjs_free(ptr, 1));
const PlaneJsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_planejs_free(ptr, 1));
const Point3JsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_point3js_free(ptr, 1));
const Vector3JsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_vector3js_free(ptr, 1));

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_externrefs.set(idx, obj);
    return idx;
}

function _assertClass(instance, klass) {
    if (!(instance instanceof klass)) {
        throw new Error(`expected instance of ${klass.name}`);
    }
}

function debugString(val) {
    // primitive types
    const type = typeof val;
    if (type == 'number' || type == 'boolean' || val == null) {
        return  `${val}`;
    }
    if (type == 'string') {
        return `"${val}"`;
    }
    if (type == 'symbol') {
        const description = val.description;
        if (description == null) {
            return 'Symbol';
        } else {
            return `Symbol(${description})`;
        }
    }
    if (type == 'function') {
        const name = val.name;
        if (typeof name == 'string' && name.length > 0) {
            return `Function(${name})`;
        } else {
            return 'Function';
        }
    }
    // objects
    if (Array.isArray(val)) {
        const length = val.length;
        let debug = '[';
        if (length > 0) {
            debug += debugString(val[0]);
        }
        for(let i = 1; i < length; i++) {
            debug += ', ' + debugString(val[i]);
        }
        debug += ']';
        return debug;
    }
    // Test for built-in
    const builtInMatches = /\[object ([^\]]+)\]/.exec(toString.call(val));
    let className;
    if (builtInMatches && builtInMatches.length > 1) {
        className = builtInMatches[1];
    } else {
        // Failed to match the standard '[object ClassName]'
        return toString.call(val);
    }
    if (className == 'Object') {
        // we're a user defined class or Object
        // JSON.stringify avoids problems with cycles, and is generally much
        // easier than looping through ownProperties of `val`.
        try {
            return 'Object(' + JSON.stringify(val) + ')';
        } catch (_) {
            return 'Object';
        }
    }
    // errors
    if (val instanceof Error) {
        return `${val.name}: ${val.message}\n${val.stack}`;
    }
    // TODO we could test for more things here, like `Set`s and `Map`s.
    return className;
}

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

function getArrayU32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint32ArrayMemory0 = null;
function getUint32ArrayMemory0() {
    if (cachedUint32ArrayMemory0 === null || cachedUint32ArrayMemory0.byteLength === 0) {
        cachedUint32ArrayMemory0 = new Uint32Array(wasm.memory.buffer);
    }
    return cachedUint32ArrayMemory0;
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        const idx = addToExternrefTable0(e);
        wasm.__wbindgen_exn_store(idx);
    }
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function passArray32ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 4, 4) >>> 0;
    getUint32ArrayMemory0().set(arg, ptr / 4);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArrayJsValueToWasm0(array, malloc) {
    const ptr = malloc(array.length * 4, 4) >>> 0;
    for (let i = 0; i < array.length; i++) {
        const add = addToExternrefTable0(array[i]);
        getDataViewMemory0().setUint32(ptr + 4 * i, add, true);
    }
    WASM_VECTOR_LEN = array.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
function decodeText(ptr, len) {
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

const wasmPath = `${__dirname}/csgrs_bg.wasm`;
const wasmBytes = require('fs').readFileSync(wasmPath);
const wasmModule = new WebAssembly.Module(wasmBytes);
let wasmInstance = new WebAssembly.Instance(wasmModule, __wbg_get_imports());
let wasm = wasmInstance.exports;
wasm.__wbindgen_start();
