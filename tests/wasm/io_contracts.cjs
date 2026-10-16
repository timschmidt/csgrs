// Real generated bindings; independent geometry and exception contracts.
const assert = require("node:assert/strict");
const { test } = require("node:test");
const path = require("node:path");
const { MeshJs, GeometryCertaintyJs } = require(path.resolve(process.argv[2]));
const encoder = new TextEncoder();

function scoped(name, operation) {
  test(name, () => {
    const owned = [];
    const keep = (value) => { owned.push(value); return value; };
    const remove = (value) => {
      const slot = owned.indexOf(value);
      assert.notEqual(slot, -1);
      owned.splice(slot, 1);
    };
    const takeMesh = (result) => { remove(result); return keep(result.intoMesh()); };
    const release = (value) => { remove(value); value.free(); };
    try { operation({ keep, takeMesh, release }); }
    finally { for (const value of owned.reverse()) value.free(); }
  });
}

function capture(operation) {
  let value;
  try { value = operation(); }
  catch (error) { assert.equal(value, undefined); return error; }
  if (value && typeof value.free === "function") value.free();
  assert.fail("failed operation returned an apparently successful value");
}

function geometryError(error) {
  assert.ok(error instanceof Error);
  assert.equal(error.name, "GeometryError");
  assert.equal(error.code, "invalid_input");
  assert.ok(error.message.length > 0);
}

function ioError(error, code) {
  assert.ok(error instanceof Error);
  assert.equal(error.name, "GeometryIoError");
  assert.equal(error.code, code);
  assert.equal(error.format, "STL");
  assert.ok(error.message.length > 0);
  assert.equal(typeof error.free, "undefined");
}

function triangle(points) {
  return MeshJs.fromArrays(Float64Array.from(points.flat()), Uint32Array.from([0, 1, 2]));
}

function assertCube(mesh, side) {
  assert.ok(side > 0 && Number.isFinite(side));
  const arrays = mesh.toArrays();
  const points = [];
  for (let offset = 0; offset < arrays.positions.length; offset += 3) {
    points.push(Array.from(arrays.positions.slice(offset, offset + 3), (coordinate) => {
      assert.ok(coordinate === 0 || coordinate === side);
      return coordinate === 0 ? 0 : 1;
    }));
  }
  assert.equal(new Set(points.map((point) => point.join(","))).size, 8);
  assert.equal(arrays.indices.length, 36);
  const edges = new Map();
  let sixVolume = 0;
  for (let offset = 0; offset < arrays.indices.length; offset += 3) {
    const [a, b, c] = Array.from(arrays.indices.slice(offset, offset + 3), (index) => {
      assert.ok(index < points.length);
      return points[index];
    });
    const u = b.map((v, i) => v - a[i]), v = c.map((v, i) => v - a[i]);
    const normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    assert.ok([0, 1, 2].some((axis) =>
      a[axis] === b[axis] && b[axis] === c[axis]
      && (a[axis] === 0 ? normal[axis] < 0 : normal[axis] > 0)));
    sixVolume += a[0] * (b[1] * c[2] - b[2] * c[1])
      + a[1] * (b[2] * c[0] - b[0] * c[2])
      + a[2] * (b[0] * c[1] - b[1] * c[0]);
    for (const [left, right] of [[a, b], [b, c], [c, a]]) {
      const l = left.join(","), r = right.join(",");
      assert.notEqual(l, r);
      const key = l < r ? `${l}|${r}` : `${r}|${l}`;
      const record = edges.get(key) || { uses: 0, winding: 0 };
      record.uses++;
      record.winding += l < r ? 1 : -1;
      edges.set(key, record);
    }
  }
  assert.equal(sixVolume, 6);
  assert.ok([...edges.values()].every(({ uses, winding }) => uses === 2 && winding === 0));
}

scoped("array length failures retain the invalid-input category", () => {
  for (const [positions, indices] of [ [[0, 0], []], [[0, 0, 0], [0]], [[], [0, 0]] ]) {
    geometryError(capture(() => MeshJs.fromArrays(Float64Array.from(positions), Uint32Array.from(indices))));
  }
});

scoped("array nonfinite failures retain the invalid-input category", () => {
  for (const value of [NaN, Infinity, -Infinity]) {
    geometryError(capture(() => MeshJs.fromArrays(Float64Array.from([value, 0, 0]), new Uint32Array())));
  }
});

scoped("array index failures retain the invalid-input category", () => {
  for (const index of [1, 0xffffffff]) {
    geometryError(capture(() => MeshJs.fromArrays(Float64Array.from([0, 0, 0]), Uint32Array.from([0, 0, index]))));
  }
});

scoped("binary projection collapse retains typed format detail", ({ keep }) => {
  const unit = keep(MeshJs.cube(1));
  const collapsed = keep(unit.translate(2 ** 24, 0, 0));
  const collinear = keep(triangle([[0, 0, 0], [1, 1, 0], [2, 2 + 2 ** -25, 0]]));
  for (const mesh of [collapsed, collinear, collapsed]) {
    const error = capture(() => mesh.toSTLBinary());
    ioError(error, "geometry");
    assert.equal(error.detail, "degenerate triangle after binary32 projection");
  }
});

scoped("binary overflow retains coordinate field and target", ({ keep }) => {
  const mesh = keep(triangle([[0, 0, 0], [1e50, 0, 0], [0, 1, 0]]));
  const error = capture(() => mesh.toSTLBinary());
  ioError(error, "unrepresentable_coordinate");
  assert.equal(error.field, "vertex x");
  assert.equal(error.target, "f32");
});

scoped("ASCII export failure retains typed format detail", ({ keep }) => {
  const mesh = keep(triangle([[0, 0, 0], [0, 0, 0], [0, 0, 0]]));
  const error = capture(() => mesh.toSTLASCII());
  ioError(error, "geometry");
  assert.equal(error.detail, "degenerate triangle after finite format projection");
});

scoped("successful exports and recovery remain unchanged", ({ keep }) => {
  const cube = keep(MeshJs.cube(1));
  const bytes = cube.toSTLBinary();
  assert.equal(bytes.length, 684);
  assert.equal(new DataView(bytes.buffer, bytes.byteOffset).getUint32(80, true), 12);
  assert.match(cube.toSTLASCII(), /^solid mesh\n/);
  const wrong = keep(cube.translate(2 ** 24, 0, 0));
  capture(() => wrong.toSTLBinary());
  assert.deepEqual(cube.toSTLBinary(), bytes);
  assertCube(cube, 1);
});

scoped("canonical empty native imports compose", ({ keep, takeMesh }) => {
  const cube = keep(MeshJs.cube(1));
  for (const bytes of [new Uint8Array(84), encoder.encode("solid empty\nendsolid empty\n")]) {
    const empty = keep(MeshJs.fromSTL(bytes));
    assert.equal(empty.vertexCount(), 0);
    assert.equal(empty.triangleCount(), 0);
    const outcome = keep(empty.unionWithContext(cube, false));
    assert.equal(outcome.certainty, GeometryCertaintyJs.Certified);
    assertCube(takeMesh(outcome), 1);
  }
});

scoped("both native STL encodings reimport and compose at ordinary and tiny scales", ({ keep, takeMesh }) => {
  const empty = keep(new MeshJs());
  for (const side of [0.25, 1.1, 2 ** -70]) {
    const source = keep(MeshJs.cube(side));
    for (const binary of [true, false]) {
      const bytes = binary ? source.toSTLBinary() : encoder.encode(source.toSTLASCII());
      const input = bytes.slice();
      const restored = keep(MeshJs.fromSTL(input));
      input.fill(0); // Imported geometry owns its data independently of the JS bytes.
      const projectedSide = Math.fround(side);
      assertCube(restored, projectedSide);
      if (side === 1.1) assert.notEqual(projectedSide, side);
      const outcome = keep(restored.unionWithContext(empty, false));
      assert.equal(outcome.certainty, GeometryCertaintyJs.Certified);
      const composed = takeMesh(outcome);
      assertCube(composed, projectedSide);
      const cut = keep(composed.differenceWithContext(composed, false));
      assert.equal(cut.certainty, GeometryCertaintyJs.Certified);
      const removed = takeMesh(cut);
      assert.equal(removed.vertexCount(), 0);
      assert.equal(removed.triangleCount(), 0);
      const recovered = keep(removed.unionWithContext(source, false));
      assert.equal(recovered.certainty, GeometryCertaintyJs.Certified);
      assertCube(takeMesh(recovered), side);
    }
  }
});

scoped("malformed bytes retain the parser IO category", () => {
  const error = capture(() => MeshJs.fromSTL(encoder.encode("not an STL")));
  ioError(error, "std_io");
  assert.equal(typeof error.ioKind, "string");
  assert.ok(error.ioKind.length > 0);
  assert.ok(error.cause instanceof Error);
});

scoped("nonfinite STL coordinates remain malformed input", ({ keep }) => {
  const bytes = keep(MeshJs.cube(1)).toSTLBinary();
  for (const value of [NaN, Infinity, -Infinity]) {
    const invalid = bytes.slice();
    new DataView(invalid.buffer, invalid.byteOffset).setFloat32(96, value, true);
    const error = capture(() => MeshJs.fromSTL(invalid));
    ioError(error, "malformed_input");
    assert.match(error.detail, /STL vertex/);
  }
});

scoped("invalid STL topology retains its validation cause after disposal", ({ keep, release }) => {
  const cube = keep(MeshJs.cube(1));
  const valid = cube.toSTLBinary();
  const open = new Uint8Array(valid.length - 50);
  open.set(valid.subarray(0, 84));
  open.set(valid.subarray(134), 84);
  new DataView(open.buffer).setUint32(80, 11, true);
  const face = keep(triangle([[0, 0, 0], [1, 0, 0], [0, 1, 0]]));
  const saved = [];
  for (const bytes of [open, encoder.encode(face.toSTLASCII())]) {
    const error = capture(() => MeshJs.fromSTL(bytes));
    ioError(error, "construction");
    geometryError(error.cause);
    saved.push([error, error.message, error.cause.message]);
  }
  release(cube);
  release(face);
  assertCube(keep(MeshJs.fromSTL(valid)), 1);
  for (const [error, message, causeMessage] of saved) {
    ioError(error, "construction");
    geometryError(error.cause);
    assert.equal(error.message, message);
    assert.equal(error.cause.message, causeMessage);
  }
});
