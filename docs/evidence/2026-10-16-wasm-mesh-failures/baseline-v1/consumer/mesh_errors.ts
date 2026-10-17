import { MeshJs, GeometryCertaintyJs } from "./csgrs";
import type { GeometryValidationError } from "./csgrs";

function check(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(message);
}
function capture(operation: () => unknown): GeometryValidationError {
  let value: unknown;
  try { value = operation(); }
  catch (error: unknown) {
    check(error instanceof Error, "native failure must be a host Error");
    check(error.name === "GeometryError" && "code" in error, "typed native category missing");
    const typed = error as GeometryValidationError;
    const category: GeometryValidationError["code"] = typed.code;
    check(category === "invalid_input", "incorrect native category");
    check(typed.message.length > 0, "native diagnostic missing");
    return typed;
  }
  if (typeof value === "object" && value !== null && "free" in value && typeof value.free === "function") value.free();
  throw new Error("failed operation returned a successful value");
}
const empty = new MeshJs();
const bad = MeshJs.fromArrays(Float64Array.from([0,0,0,1,0,0,0,1,0]),Uint32Array.from([0,1,2]));
let diagnostic: GeometryValidationError;
try {
  diagnostic = capture(() => bad.union(empty));
  capture(() => empty.intersectionWithContext(bad,false));
  capture(() => bad.convexHullWithContext(true));
  capture(() => empty.subdivide(0));
  const result = empty.unionWithContext(empty,false);
  check(result.certainty === GeometryCertaintyJs.Certified,"empty composition lost certainty");
  const mesh = result.intoMesh();
  try {
    const imported = MeshJs.fromSTL(mesh.toSTLBinary());
    try { check(imported.isEmpty(),"empty STL round trip changed geometry"); }
    finally { imported.free(); }
  } finally { mesh.free(); }
} finally { bad.free(); empty.free(); }
check(diagnostic.name === "GeometryError" && diagnostic.message.length > 0,"diagnostic died with native handles");
