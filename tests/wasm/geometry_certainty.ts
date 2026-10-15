// The runner copies this beside the generated csgrs.d.ts before type-checking.
import {
  CurveRegionJs, GeometryBoolResultJs, GeometryCertaintyJs,
  GeometryQueryStatusJs, MeshJs, Point3Js,
} from "./csgrs";

function assertNever(value: never): never {
  throw new Error(`Unexpected query status: ${value}`);
}

function describe(query: GeometryBoolResultJs): string {
  const certainty: GeometryCertaintyJs | undefined = query.certainty;
  const reason: string | undefined = query.reason;
  const value: boolean | undefined = query.value;
  const status = query.status;
  switch (status) {
    case GeometryQueryStatusJs.Decided:
      return `${value} (${certainty})`;
    case GeometryQueryStatusJs.Boundary:
      return `boundary (${certainty})`;
    case GeometryQueryStatusJs.Uncertain:
    case GeometryQueryStatusJs.Unknown:
      return `unresolved: ${reason}`;
    default:
      return assertNever(status);
  }
}

const square = CurveRegionJs.square(1);
const query = square.containsXYWithContext(0, 0.5, false);
describe(query);
query.free();
square.free();

for (const profile of [
  CurveRegionJs.star(4, 2, 1),
  CurveRegionJs.teardrop(2, 3, 2),
  CurveRegionJs.egg(2, 4, 4),
  CurveRegionJs.heart(2, 2, 8),
]) {
  const extrusion = profile.extrude(3, false);
  const receipt: GeometryCertaintyJs = extrusion.certainty;
  if (receipt !== GeometryCertaintyJs.Certified) throw new Error("Uncertified fixture");
  extrusion.free();
  profile.free();
}

const cube = MeshJs.cube(1);
const bounds = cube.boundingBoxWithContext(false);
const minimum: Point3Js | undefined = bounds.min;
const maximum: Point3Js | undefined = bounds.max;
const empty: boolean = bounds.isEmpty;
if (!empty && minimum && maximum) {
  minimum.free();
  maximum.free();
}
bounds.free();
const result = cube.unionWithContext(cube, false);
const certainty: GeometryCertaintyJs = result.certainty;
if (certainty === GeometryCertaintyJs.Certified) {
  const mesh = result.intoMesh(); // Consumes the result wrapper.
  mesh.free();
} else {
  result.free();
}
cube.free();
