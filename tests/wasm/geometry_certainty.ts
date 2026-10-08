// The runner copies this beside the generated csgrs.d.ts before type-checking.
import {
  CurveRegionJs, GeometryBoolResultJs, GeometryCertaintyJs,
  GeometryQueryStatusJs, MeshJs,
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

const cube = MeshJs.cube(1);
const result = cube.unionWithContext(cube, false);
const certainty: GeometryCertaintyJs = result.certainty;
if (certainty === GeometryCertaintyJs.Certified) {
  const mesh = result.intoMesh(); // Consumes the result wrapper.
  mesh.free();
} else {
  result.free();
}
cube.free();
