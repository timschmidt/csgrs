const assert = require("node:assert/strict");
const { test } = require("node:test");
const path = require("node:path");
const { MeshJs, Point3Js, GeometryCertaintyJs, GeometryQueryStatusJs } = require(path.resolve(process.argv[2]));

function capture(operation) {
  let value;
  try { value = operation(); }
  catch (error) { assert.equal(value, undefined); return error; }
  if (value && typeof value.free === "function") value.free();
  assert.fail("failed operation returned a successful value");
}
function invalid(error) {
  assert.ok(error instanceof Error);
  assert.equal(error.name, "GeometryError");
  assert.equal(error.code, "invalid_input");
  assert.ok(error.message.length > 0);
  assert.equal(typeof error.free, "undefined");
}
function malformed(degenerate = false) {
  return MeshJs.fromArrays(Float64Array.from(degenerate
    ? [0,0,0, 1,0,0, 2,0,0] : [0,0,0, 1,0,0, 0,1,0]), Uint32Array.from([0,1,2]));
}
for (const operation of ["union", "intersection", "difference", "xor"]) {
  test(operation+" retains invalid-input errors for either operand beside empty", () => {
    const empty = new MeshJs();
    try {
      for (const degenerate of [false,true]) {
        const bad = malformed(degenerate);
        try {
          for (const [left,right] of [[bad,empty],[empty,bad]]) {
            invalid(capture(() => left[operation](right)));
            for (const approximate of [false,true]) invalid(capture(() => left[operation+"WithContext"](right,approximate)));
          }
        } finally { bad.free(); }
      }
    } finally { empty.free(); }
  });
}
test("insufficient hull geometry keeps its typed failure for every policy", () => {
  for (const construct of [() => new MeshJs(),() => malformed(),() => malformed(true)]) {
    const mesh = construct();
    try {
      invalid(capture(() => mesh.convexHull()));
      for (const approximate of [false,true]) invalid(capture(() => mesh.convexHullWithContext(approximate)));
    } finally { mesh.free(); }
  }
});
test("zero subdivision fails visibly even for canonical empty geometry", () => {
  for (const construct of [() => new MeshJs(),() => MeshJs.cube(1)]) {
    const mesh = construct();
    try { invalid(capture(() => mesh.subdivide(0))); }
    finally { mesh.free(); }
  }
});
test("mesh diagnostics remain ordinary host errors after all native inputs are freed", () => {
  const bad = malformed(),empty = new MeshJs();
  let error;
  try { error = capture(() => bad.union(empty)); }
  finally { bad.free(); empty.free(); }
  invalid(error);
  assert.ok(error.stack.length > 0);
});
test("successful empty outputs retain certainty and compose through STL reimport", () => {
  const empty = new MeshJs();
  try {
    for (const operation of ["union", "intersection", "difference", "xor"]) {
      for (const approximate of [false,true]) {
        const result = empty[operation+"WithContext"](empty,approximate);
        assert.equal(result.certainty,GeometryCertaintyJs.Certified);
        const mesh = result.intoMesh();
        try {
          assert.equal(mesh.isEmpty(),true);
          assert.deepEqual(Array.from(mesh.toArrays().positions),[]);
          assert.deepEqual(Array.from(mesh.toArrays().indices),[]);
          const imported = MeshJs.fromSTL(mesh.toSTLBinary());
          try {
            const next = imported.unionWithContext(empty,false);
            assert.equal(next.certainty,GeometryCertaintyJs.Certified);
            const consumed = next.intoMesh();
            try { assert.equal(consumed.isEmpty(),true); }
            finally { consumed.free(); }
          } finally { imported.free(); }
        } finally { mesh.free(); }
      }
    }
  } finally { empty.free(); }
});
test("successful membership retains its own query status and consumed certainty", () => {
  const mesh = MeshJs.cube(1);
  try {
    for (const [x,inside] of [[0.5,true],[2,false]]) {
      const point = new Point3Js(x,0.5,0.5);
      try {
        assert.equal(mesh.containsPoint(point),inside);
        for (const approximate of [false,true]) {
          const result = mesh.containsPointWithContext(point,approximate);
          try {
            assert.equal(result.status,GeometryQueryStatusJs.Decided);
            assert.equal(result.value,inside);
            assert.equal(result.certainty,GeometryCertaintyJs.Certified);
          } finally { result.free(); }
        }
      } finally { point.free(); }
    }
  } finally { mesh.free(); }
});

for (const [name, construct] of [
  ["open", () => malformed()],
  ["degenerate", () => malformed(true)],
  ["positions-only", () => MeshJs.fromArrays(Float64Array.from([0,0,0]), new Uint32Array())],
]) {
  test("membership rejects "+name+" input before bounds and boundary shortcuts", () => {
    const mesh = construct();
    try {
      for (const xyz of [[10,10,10],[0,0,0]]) {
        const point = new Point3Js(...xyz);
        try {
          invalid(capture(() => mesh.containsPoint(point)));
          for (const approximate of [false,true,false]) {
            invalid(capture(() => mesh.containsPointWithContext(point,approximate)));
          }
        } finally { point.free(); }
      }
    } finally { mesh.free(); }
  });
}
test("canonical empty membership is a decided certified false", () => {
  const mesh = new MeshJs(), point = new Point3Js(0,0,0);
  try {
    assert.equal(mesh.containsPoint(point),false);
    for (const approximate of [false,true]) {
      const result = mesh.containsPointWithContext(point,approximate);
      try {
        assert.equal(result.status,GeometryQueryStatusJs.Decided);
        assert.equal(result.value,false);
        assert.equal(result.certainty,GeometryCertaintyJs.Certified);
      } finally { result.free(); }
    }
  } finally { point.free(); mesh.free(); }
});
