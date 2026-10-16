const assert = require("node:assert/strict");
const FACE_INDICES = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,4,7],[0,7,3],
  [1,2,6],[1,6,5],[0,1,5],[0,5,4],[3,7,6],[3,6,2]];

function fixture(cubes) {
  assert.ok([0, 1, 256].includes(cubes));
  const positions = [], indices = [];
  for (let i = 0; i < cubes; i++) {
    const x = 3 * i, offset = positions.length / 3;
    positions.push(...[[x,0,0],[x+1,0,0],[x+1,1,0],[x,1,0],
      [x,0,1],[x+1,0,1],[x+1,1,1],[x,1,1]].flat());
    indices.push(...FACE_INDICES.flat().map((index) => index + offset));
  }
  const faces = [];
  for (let i = 0; i < indices.length; i += 3) {
    faces.push(indices.slice(i, i + 3).map((index) => positions.slice(3 * index, 3 * index + 3)));
  }
  return { positions, indices, faces };
}

function encode(faces, encoding) {
  if (encoding === "ascii") {
    let text = "solid fixture\n";
    for (const face of faces) {
      text += "facet normal 0 0 0\nouter loop\n";
      for (const point of face) text += `vertex ${point.join(" ")}\n`;
      text += "endloop\nendfacet\n";
    }
    return Buffer.from(text + "endsolid fixture\n");
  }
  const bytes = new Uint8Array(84 + 50 * faces.length);
  const view = new DataView(bytes.buffer);
  view.setUint32(80, faces.length, true);
  faces.forEach((face, i) => face.flat().forEach((value, j) => view.setFloat32(96 + 50 * i + 4 * j, value, true)));
  return bytes;
}

function decode(output, encoding) {
  if (encoding === "ascii") {
    assert.ok(output.startsWith("solid mesh\n") && output.endsWith("endsolid mesh\n"));
    const vertices = output.split("\n").map((line) => line.trim())
      .filter((line) => line.startsWith("vertex ")).map((line) =>
        line.slice(7).split(/\s+/).map((field) => {
          // The fixtures are integers. Require exact integer decimal text,
          // rather than accepting a rounded Number comparison as an oracle.
          assert.match(field, /^-?\d+\.0+$/);
          const integer = BigInt(field.slice(0, field.indexOf(".")));
          const value = Number(integer);
          assert.equal(BigInt(value), integer);
          return value;
        }));
    assert.equal(vertices.length % 3, 0);
    const faces = [];
    for (let i = 0; i < vertices.length; i += 3) faces.push(vertices.slice(i, i + 3));
    return faces;
  }
  const view = new DataView(output.buffer, output.byteOffset, output.byteLength);
  const count = view.getUint32(80, true);
  assert.equal(output.length, 84 + 50 * count);
  return Array.from({ length: count }, (_, i) => Array.from({ length: 3 }, (_, p) =>
    Array.from({ length: 3 }, (_, axis) => view.getFloat32(96 + 50 * i + 12 * p + 4 * axis, true))));
}

function verify(faces, cubes) {
  assert.equal(faces.length, 12 * cubes); // Fixture's deterministic artifact check.
  const volumes = Array(cubes).fill(0), edges = new Map();
  for (const [a, b, c] of faces) {
    const component = Math.floor(a[0] / 3), x = 3 * component;
    assert.ok(component >= 0 && component < cubes);
    for (const point of [a, b, c]) {
      assert.ok(point[0] === x || point[0] === x + 1);
      assert.ok(point[1] === 0 || point[1] === 1);
      assert.ok(point[2] === 0 || point[2] === 1);
    }
    const u = b.map((v,i) => v-a[i]), v = c.map((v,i) => v-a[i]);
    const n = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    assert.ok([0,1,2].some((axis) => a[axis] === b[axis] && b[axis] === c[axis]
      && (a[axis] === (axis === 0 ? x : 0) ? n[axis] < 0 : n[axis] > 0)));
    volumes[component] += a[0]*(b[1]*c[2]-b[2]*c[1])
      + a[1]*(b[2]*c[0]-b[0]*c[2]) + a[2]*(b[0]*c[1]-b[1]*c[0]);
    for (const [left,right] of [[a,b],[b,c],[c,a]]) {
      const l=left.join(","), r=right.join(",");
      assert.notEqual(l,r);
      const key=l<r?`${l}|${r}`:`${r}|${l}`;
      const row=edges.get(key)||[0,0]; row[0]++;row[1]+=l<r?1:-1;edges.set(key,row);
    }
  }
  assert.ok(volumes.every((volume) => volume === 6));
  assert.ok([...edges.values()].every(([uses,winding]) => uses === 2 && winding === 0));
}

module.exports = { fixture, encode, decode, verify };
