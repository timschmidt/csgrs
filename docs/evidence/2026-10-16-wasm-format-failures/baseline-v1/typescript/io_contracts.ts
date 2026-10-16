// Compile and execute beside the actual generated csgrs.js and csgrs.d.ts.
import { MeshJs, GeometryCertaintyJs, GeometryMeshResultJs } from "./csgrs";
import type { GeometryIoError, GeometryValidationError } from "./csgrs";

function unreachable(value: never): never {
  throw new Error(`Unexpected IO variant: ${value}`);
}

function describe(error: GeometryIoError): string {
  const format: string = error.format;
  switch (error.code) {
    case "std_io": {
      const kind: string = error.ioKind;
      const osCode: number | undefined = error.osCode;
      const cause: Error = error.cause;
      return `${format}: ${kind} (${osCode}): ${cause.message}`;
    }
    case "parse_float":
    case "dxf":
    case "gerber_codegen": {
      const cause: Error = error.cause;
      return `${format}: ${cause.message}`;
    }
    case "malformed_input":
    case "unsupported":
    case "geometry":
    case "gerber_parsing": {
      const detail: string = error.detail;
      return `${format}: ${detail}`;
    }
    case "unrepresentable_coordinate": {
      const field: string = error.field;
      const target: string = error.target;
      return `${format}: ${field} cannot use ${target}`;
    }
    case "invalid_metadata": {
      const field: string = error.field;
      return `${format}: invalid ${field}`;
    }
    case "size_overflow": {
      const limit: string = error.limit;
      return `${format}: ${limit}`;
    }
    case "construction": {
      const cause: GeometryValidationError = error.cause;
      return `${format}: ${cause.code}: ${cause.message}`;
    }
    default:
      return unreachable(error);
  }
}

function ensure(value: boolean, message: string): asserts value {
  if (!value) throw new Error(message);
}

function caughtIo(operation: () => unknown): GeometryIoError {
  let value: unknown;
  try { value = operation(); }
  catch (error: unknown) {
    ensure(error instanceof Error && error.name === "GeometryIoError", "typed format failure required");
    // This catch handles only documented CSGRS format entry points. The runtime
    // JS matrix independently verifies their discriminants and variant fields.
    const typed = error as GeometryIoError;
    ensure(describe(typed).length > 0, "diagnostic must survive");
    return typed;
  }
  if (value instanceof MeshJs) value.free();
  throw new Error("invalid format operation returned a model");
}

const owned: Array<{ free(): void }> = [];
function keep<T extends { free(): void }>(value: T): T {
  owned.push(value);
  return value;
}
function takeMesh(result: GeometryMeshResultJs): MeshJs {
  const index = owned.indexOf(result);
  ensure(index >= 0, "owned result required");
  owned.splice(index, 1);
  return keep(result.intoMesh());
}

try {
  const source = keep(MeshJs.cube(0.25));
  const bytes: Uint8Array = source.toSTLBinary();
  const restored: MeshJs = keep(MeshJs.fromSTL(bytes));
  const empty = keep(new MeshJs());
  const outcome = keep(restored.unionWithContext(empty, false));
  const certainty: GeometryCertaintyJs = outcome.certainty;
  ensure(certainty === GeometryCertaintyJs.Certified, "parsed geometry must certify independently");
  const mesh = takeMesh(outcome);
  ensure(mesh.triangleCount() === 12, "complete imported cube required");
  const invalid = caughtIo(() => MeshJs.fromSTL(new TextEncoder().encode("not an STL")));
  ensure(invalid.code === "std_io" && invalid.format === "STL", "parser category must survive");
  ensure(invalid.cause instanceof Error, "parser cause required");
  const open = keep(MeshJs.fromArrays(
    Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1, 0]), Uint32Array.from([0, 1, 2]),
  ));
  const topology = caughtIo(() => MeshJs.fromSTL(open.toSTLBinary()));
  ensure(topology.code === "construction", "native validation failure required");
  const cause: GeometryValidationError = topology.cause;
  ensure(cause.code === "invalid_input", "native category must survive");
  console.log("TypeScript native STL pipeline and discriminated errors passed");
} finally {
  for (const value of owned.reverse()) value.free();
}
