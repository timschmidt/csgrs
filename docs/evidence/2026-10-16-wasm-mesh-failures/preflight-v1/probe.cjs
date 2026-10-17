const path = require("node:path");
const { MeshJs, Point3Js } = require(path.resolve(process.argv[2]));
const owned = [];
const keep = (value) => { owned.push(value); return value; };
const rows = [];
function inspect(name, operation) {
  try {
    const value = operation();
    const result = { name, outcome: "returned", type: typeof value };
    if (value && typeof value.isEmpty === "function") result.empty = value.isEmpty();
    if (value && "status" in value) result.status = value.status;
    if (typeof value === "boolean") result.value = value;
    if (value && typeof value.free === "function") value.free();
    rows.push(result);
  } catch (error) {
    rows.push({ name, outcome: "threw", type: typeof error, isError: error instanceof Error,
      errorName: error && error.name, code: error && error.code, diagnostic: String(error) });
  }
}
try {
  const empty = keep(new MeshJs());
  const open = keep(MeshJs.fromArrays(Float64Array.from([0,0,0, 1,0,0, 0,1,0]), Uint32Array.from([0,1,2])));
  const point = keep(new Point3Js(0.25,0.25,0));
  for (const operation of ["union", "intersection", "difference", "xor"]) {
    for (const [side, left, right] of [["open-empty",open,empty],["empty-open",empty,open]]) {
      inspect(operation+"/"+side, () => left[operation](right));
      for (const approximate of [false,true]) inspect(operation+"WithContext/"+side+"/"+approximate,
        () => left[operation+"WithContext"](right, approximate));
    }
  }
  inspect("containsPoint/open", () => open.containsPoint(point));
  for (const approximate of [false,true]) inspect("containsPointWithContext/open/"+approximate,
    () => open.containsPointWithContext(point,approximate));
  inspect("subdivide/zero", () => open.subdivide(0));
  inspect("convexHull/open", () => open.convexHull());
  for (const approximate of [false,true]) inspect("convexHullWithContext/open/"+approximate,
    () => open.convexHullWithContext(approximate));
} finally { for (const value of owned.reverse()) value.free(); }
process.stdout.write(JSON.stringify(rows,null,2)+"\n");
