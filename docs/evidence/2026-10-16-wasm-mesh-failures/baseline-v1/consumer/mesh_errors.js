"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const csgrs_1 = require("./csgrs");
function check(condition, message) {
    if (!condition)
        throw new Error(message);
}
function capture(operation) {
    let value;
    try {
        value = operation();
    }
    catch (error) {
        check(error instanceof Error, "native failure must be a host Error");
        check(error.name === "GeometryError" && "code" in error, "typed native category missing");
        const typed = error;
        const category = typed.code;
        check(category === "invalid_input", "incorrect native category");
        check(typed.message.length > 0, "native diagnostic missing");
        return typed;
    }
    if (typeof value === "object" && value !== null && "free" in value && typeof value.free === "function")
        value.free();
    throw new Error("failed operation returned a successful value");
}
const empty = new csgrs_1.MeshJs();
const bad = csgrs_1.MeshJs.fromArrays(Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1, 0]), Uint32Array.from([0, 1, 2]));
let diagnostic;
try {
    diagnostic = capture(() => bad.union(empty));
    capture(() => empty.intersectionWithContext(bad, false));
    capture(() => bad.convexHullWithContext(true));
    capture(() => empty.subdivide(0));
    const result = empty.unionWithContext(empty, false);
    check(result.certainty === csgrs_1.GeometryCertaintyJs.Certified, "empty composition lost certainty");
    const mesh = result.intoMesh();
    try {
        const imported = csgrs_1.MeshJs.fromSTL(mesh.toSTLBinary());
        try {
            check(imported.isEmpty(), "empty STL round trip changed geometry");
        }
        finally {
            imported.free();
        }
    }
    finally {
        mesh.free();
    }
}
finally {
    bad.free();
    empty.free();
}
check(diagnostic.name === "GeometryError" && diagnostic.message.length > 0, "diagnostic died with native handles");
