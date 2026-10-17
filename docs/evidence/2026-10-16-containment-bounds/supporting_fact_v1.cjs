// Independent observations of the same retained inverse-rotated source.
const path = require("node:path");
const {
  MeshJs, Point3Js, GeometryCertaintyJs: Certainty, GeometryQueryStatusJs: Status,
} = require(path.resolve(process.argv[2]));
const owned = [];
const keep = value => { owned.push(value); return value; };
try {
  const cube = keep(MeshJs.cube(1));
  const rotated = keep(cube.rotate(0, 0, 13));
  const restored = keep(rotated.rotate(0, 0, -13));
  const bounds = keep(restored.boundingBoxWithContext(false));
  const min = keep(bounds.min), max = keep(bounds.max);
  const rows = [];
  for (const xyz of [[0.5, 0, 0.5], [1, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 0.5, 0.5]]) {
    const point = keep(new Point3Js(...xyz));
    for (const approximate of [false, true, false]) {
      const result = keep(restored.containsPointWithContext(point, approximate));
      rows.push({ xyz, approximate, status: Status[result.status],
        value: result.value ?? null, certainty: Certainty[result.certainty] ?? null,
        reason: result.reason ?? null });
    }
  }
  console.log(JSON.stringify({ bounds: {certainty: Certainty[bounds.certainty],
    projected_min: [min.x, min.y, min.z], projected_max: [max.x, max.y, max.z]}, rows }, null, 2));
} finally {
  for (const value of owned.reverse()) value.free();
}
