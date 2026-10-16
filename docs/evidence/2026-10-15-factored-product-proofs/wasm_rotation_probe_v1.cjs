const path = require('node:path');
const wasm = require(path.resolve(process.argv[2]));
const { CurveRegionJs, MeshJs, Point3Js } = wasm;

for (const angles of [[13, -13], [13, 17, -30]]) {
  const owned = [];
  const keep = value => { owned.push(value); return value; };
  try {
    let square = keep(CurveRegionJs.square(1));
    let cube = keep(MeshJs.cube(1));
    for (const angle of angles) {
      square = keep(square.rotate(angle));
      cube = keep(cube.rotate(0, 0, angle));
    }
    for (const [x, y] of [[0.5, 0], [0, 0.5], [1, 0.5], [0.5, 1], [0.5, 0.5], [2, 0.5]]) {
      let legacy;
      try { legacy = { value: square.containsXY(x, y) }; }
      catch (error) { legacy = { code: error.code, message: error.message }; }
      console.log(JSON.stringify({ angles, x, y, legacy }));
      const point = keep(new Point3Js(x, y, 0.5));
      for (const approximate of [false, true, false]) {
        for (const [kind, query] of [
          ['curve', square.containsXYWithContext(x, y, approximate)],
          ['mesh', cube.containsPointWithContext(point, approximate)],
        ]) {
          const row = { angles, x, y, kind, approximate, value: query.value,
            status: query.status, certainty: query.certainty, reason: query.reason,
            approximate512Consumed: query.approximate512Consumed };
          query.free();
          console.log(JSON.stringify(row));
        }
      }
    }
  } finally {
    for (const value of owned.reverse()) value.free();
  }
}
