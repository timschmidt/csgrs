// Independent binary fixtures for the actual native import entry point.
const assert = require("node:assert/strict");
const path = require("node:path");
const { performance } = require("node:perf_hooks");
const { MeshJs, Point3Js, GeometryCertaintyJs, GeometryQueryStatusJs } = require(path.resolve(process.argv[2]));
const owned = [];
const keep = (value) => { owned.push(value); return value; };
function takeMesh(result) {
  const index = owned.indexOf(result);
  assert.notEqual(index, -1);
  owned.splice(index, 1);
  return keep(result.intoMesh());
}
function cube(origin, side) {
  const [x, y, z] = origin;
  const p = [[x,y,z],[x+side,y,z],[x+side,y+side,z],[x,y+side,z],
    [x,y,z+side],[x+side,y,z+side],[x+side,y+side,z+side],[x,y+side,z+side]];
  return [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,4,7],[0,7,3],
    [1,2,6],[1,6,5],[0,1,5],[0,5,4],[3,7,6],[3,6,2]].map((f) => f.map((i) => p[i]));
}
function binary(faces) {
  const bytes = new Uint8Array(84 + 50 * faces.length);
  const view = new DataView(bytes.buffer);
  view.setUint32(80, faces.length, true);
  faces.forEach((face, index) => face.flat().forEach((value, coordinate) => {
    view.setFloat32(84 + 50 * index + 12 + 4 * coordinate, value, true);
  }));
  return bytes;
}
function rows(mesh) {
  const { positions, indices } = mesh.toArrays();
  const faces = [];
  for (let i = 0; i < indices.length; i += 3) {
    faces.push(Array.from(indices.slice(i, i + 3), (index) => {
      assert.ok(3 * index + 2 < positions.length);
      return Array.from(positions.slice(3 * index, 3 * index + 3));
    }));
  }
  return faces;
}
function canonical(faces) {
  return faces.map((face) => [0,1,2].map((start) =>
    JSON.stringify([face[start], face[(start+1)%3], face[(start+2)%3]]))
    .sort()[0]).sort();
}
function sixVolume(faces) {
  return faces.reduce((sum, [a,b,c]) => sum
    + a[0]*(b[1]*c[2]-b[2]*c[1]) + a[1]*(b[2]*c[0]-b[0]*c[2])
    + a[2]*(b[0]*c[1]-b[1]*c[0]), 0);
}
function member(mesh, point, expected) {
  const query = keep(mesh.containsPointWithContext(keep(new Point3Js(...point)), false));
  assert.equal(query.status, GeometryQueryStatusJs.Decided);
  assert.equal(query.certainty, GeometryCertaintyJs.Certified);
  assert.equal(query.value, expected);
}

try {
  const hostile = new Uint8Array(84);
  new DataView(hostile.buffer).setUint32(80, 0xffffffff, true);
  const start = performance.now();
  for (let repeat = 0; repeat < 128; repeat++) {
    let mesh;
    assert.throws(() => { mesh = MeshJs.fromSTL(hostile); }, (error) =>
      error instanceof Error && error.name === "GeometryIoError" && error.code === "std_io"
      && error.ioKind === "UnexpectedEof" && error.format === "STL");
    assert.equal(mesh, undefined);
  }
  console.log(JSON.stringify({ fixture: "maximum-declared-count-with-no-records", bytes: 84,
    repeats: 128, elapsed_ms: performance.now() - start, result: "typed UnexpectedEof; no model" }));

  const empty = keep(new MeshJs());
  for (const side of [2 ** -149, 2 ** -126, 2 ** 128 - 2 ** 104]) {
    const source = cube([0,0,0], side);
    const imported = keep(MeshJs.fromSTL(binary(source)));
    assert.deepEqual(canonical(rows(imported)), canonical(source));
    const outcome = keep(imported.unionWithContext(empty, false));
    assert.equal(outcome.certainty, GeometryCertaintyJs.Certified);
    const normalized = rows(takeMesh(outcome)).map((face) => face.map((point) =>
      point.map((value) => { assert.ok(value === 0 || value === side); return value === 0 ? 0 : 1; })));
    assert.equal(sixVolume(normalized), 6);
    console.log(JSON.stringify({ fixture: "binary32-extreme-cube", side, result: "certified import and composition", triangles: normalized.length }));
  }

  const unit = cube([0,0,0], 1);
  const cases = [
    ["disconnected", [...unit, ...cube([3,0,0], 1)], 12],
    ["nested-hole", [...cube([0,0,0], 3), ...cube([1,1,1], 1).map(([a,b,c]) => [a,c,b])], 156],
    ["inverted", unit.map(([a,b,c]) => [a,c,b]), -6],
    ["double-winding", [...unit, ...unit], 12],
  ];
  for (const [name, source, integral] of cases) {
    const imported = keep(MeshJs.fromSTL(binary(source)));
    assert.deepEqual(canonical(rows(imported)), canonical(source));
    assert.equal(sixVolume(rows(imported)), integral);
    if (name === "nested-hole" || name === "disconnected") {
      const outcome = keep(imported.unionWithContext(empty, false));
      assert.equal(outcome.certainty, GeometryCertaintyJs.Certified);
      const composed = takeMesh(outcome);
      assert.equal(sixVolume(rows(composed)), integral);
      if (name === "nested-hole") {
        member(composed, [0.5,0.5,0.5], true);
        member(composed, [1.5,1.5,1.5], false);
      } else {
        member(composed, [0.5,0.5,0.5], true);
        member(composed, [3.5,0.5,0.5], true);
        member(composed, [2,0.5,0.5], false);
      }
    }
    console.log(JSON.stringify({ fixture: name, result: "native closed-PWN representation preserved", triangles: source.length, six_volume: integral }));
  }
} finally {
  for (const value of owned.reverse()) value.free();
}
