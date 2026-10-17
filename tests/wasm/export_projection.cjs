// Exercise actual wasm-bindgen output. No mocked loader or native substitute.
const assert = require("node:assert/strict");
const path = require("node:path");
const { MeshJs, GeometryCertaintyJs } = require(path.resolve(process.argv[2]));
const owned = [];
const keep = (value) => { owned.push(value); return value; };

function takeMesh(result) {
  const slot = owned.indexOf(result);
  assert.notEqual(slot, -1);
  owned.splice(slot, 1);
  // intoMesh consumes its Rust result handle; do not free that handle twice.
  return keep(result.intoMesh());
}

function exactDecimal(text, numerator, denominator) {
  const match = /^(-?)(\d+)(?:\.(\d+))?(?:[eE]([+-]?\d+))?$/.exec(text);
  assert.ok(match, `finite decimal required: ${text}`);
  const fractional = match[3] || "";
  let n = BigInt(match[2] + fractional) * (match[1] ? -1n : 1n);
  let d = 10n ** BigInt(fractional.length);
  const exponent = Number(match[4] || 0);
  assert.ok(Math.abs(exponent) <= 2000);
  if (exponent >= 0) n *= 10n ** BigInt(exponent);
  else d *= 10n ** BigInt(-exponent);
  assert.equal(n * denominator, numerator * d);
}

function vertices(text) {
  return text.split("\n").map((line) => line.trim())
    .filter((line) => line.startsWith("vertex "))
    .map((line) => line.slice(7).trim().split(/\s+/));
}

try {
  const tiny = 2 ** -70;
  const face = keep(MeshJs.fromArrays(
    Float64Array.from([0, 0, 0, 1.1, 0, 0, 0, 1, tiny]),
    Uint32Array.from([0, 1, 2]),
  ));
  const rows = vertices(face.toSTLASCII());
  assert.equal(rows.length, 3);
  // Independent exact ratios, rather than Number(text) agreeing after rounding.
  exactDecimal(rows[1][0], 2476979795053773n, 2251799813685248n);
  exactDecimal(rows[2][2], 1n, 1n << 70n);

  const normalFace = keep(MeshJs.fromArrays(
    Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1, tiny]),
    Uint32Array.from([0, 1, 2]),
  ));
  const normal = normalFace.toSTLASCII().split("\n")
    .map((line) => line.trim()).find((line) => line.startsWith("facet normal "))
    .slice(13).trim().split(/\s+/);
  exactDecimal(normal[0], 0n, 1n);
  exactDecimal(normal[1], -1n, 1n << 70n);
  exactDecimal(normal[2], 1n, 1n);

  const unit = keep(MeshJs.cube(1));
  const baseline = new Uint8Array(unit.toSTLBinary());
  const changedCopy = unit.toSTLBinary();
  changedCopy.fill(0);
  assert.deepEqual(unit.toSTLBinary(), baseline);
  const collapsed = keep(unit.translate(2 ** 24, 0, 0));
  const collinear = keep(MeshJs.fromArrays(
    Float64Array.from([0, 0, 0, 1, 1, 0, 2, 2 + 2 ** -25, 0]),
    Uint32Array.from([0, 1, 2]),
  ));
  for (const invalid of [collapsed, collinear, collapsed]) {
    let emitted;
    assert.throws(() => { emitted = invalid.toSTLBinary(); },
      (error) => String(error).includes("degenerate triangle after binary32 projection"));
    assert.equal(emitted, undefined);
  }
  for (let offset = 1; offset <= 5; offset++) {
    keep(unit.translate(2 * offset, 0, 0)).toSTLBinary();
  }
  assert.throws(() => collapsed.toSTLBinary(),
    (error) => String(error).includes("degenerate triangle after binary32 projection"));
  assert.deepEqual(unit.toSTLBinary(), baseline);

  const tinyCube = keep(MeshJs.cube(tiny));
  const tinyRows = vertices(tinyCube.toSTLASCII());
  assert.equal(tinyRows.length, 36);
  for (const row of tinyRows) {
    for (const field of row) {
      if (Number(field) === 0) exactDecimal(field, 0n, 1n);
      else exactDecimal(field, 1n, 1n << 70n);
    }
  }
  // Keep an independent decoder as an oracle alongside native import tests.
  // Apply binary32 quantization, then use the public mesh constructor.
  const restored = keep(MeshJs.fromArrays(
    Float64Array.from(tinyRows.flat().map((field) => Math.fround(Number(field)))),
    Uint32Array.from({ length: tinyRows.length }, (_, index) => index),
  ));
  const empty = keep(new MeshJs());
  const outcome = keep(restored.unionWithContext(empty, false));
  assert.equal(outcome.certainty, GeometryCertaintyJs.Certified);
  const composed = takeMesh(outcome);
  const arrays = composed.toArrays();
  for (const coordinate of arrays.positions) {
    assert.ok(coordinate === 0 || coordinate === tiny);
  }
  let sixVolume = 0;
  for (let index = 0; index < arrays.indices.length; index += 3) {
    const points = Array.from(arrays.indices.slice(index, index + 3),
      (vertex) => Array.from(arrays.positions.slice(3 * vertex, 3 * vertex + 3)));
    assert.ok([0, 1, 2].some((axis) => points.every((p) => p[axis] === 0)
      || points.every((p) => p[axis] === tiny)));
    const [a, b, c] = points;
    sixVolume += a[0] * (b[1] * c[2] - b[2] * c[1])
      + a[1] * (b[2] * c[0] - b[0] * c[2])
      + a[2] * (b[0] * c[1] - b[1] * c[0]);
  }
  assert.equal(sixVolume, 6 * tiny ** 3);
  const difference = keep(composed.differenceWithContext(composed, false));
  assert.equal(difference.certainty, GeometryCertaintyJs.Certified);
  assert.equal(takeMesh(difference).isEmpty(), true);
  console.log("WASM export projection, failure, cache and composition contracts passed");
} finally {
  for (const value of owned.reverse()) value.free();
}
