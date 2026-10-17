const path = require("node:path");
const { MeshJs } = require(path.resolve(process.argv[2]));
const successes = [], failures = [];
const start = process.hrtime.bigint(), cpu = process.cpuUsage();
function arrays(mesh) {
  const a = mesh.toArrays();
  return { positions:Array.from(a.positions), normals:Array.from(a.normals), indices:Array.from(a.indices) };
}
function fail(name, operation) {
  try {
    const value = operation();
    if (value && typeof value.free === "function") value.free();
    throw new Error("unexpected successful geometry from "+name);
  } catch (error) {
    if (error instanceof Error && error.message.startsWith("unexpected successful geometry")) throw error;
    failures.push({ name, type:typeof error, isError:error instanceof Error,
      errorName:error && error.name, code:error && error.code,
      diagnostic:error instanceof Error ? error.message : String(error),
      stack:error instanceof Error ? error.stack : null });
  }
}
for (let i=0;i<5;i++) {
  const empty = new MeshJs(), cube = MeshJs.cube(1), moved = cube.translate(2,3,4);
  try {
    const result = moved.unionWithContext(empty,Boolean(i%2));
    const certainty = result.certainty, mesh = result.intoMesh();
    try {
      const bytes = mesh.toSTLBinary(), imported = MeshJs.fromSTL(bytes);
      try {
        const subtracted = imported.differenceWithContext(imported,false);
        const emptyCertainty = subtracted.certainty, final = subtracted.intoMesh();
        try { successes.push({ iteration:i, certainty, mesh:arrays(mesh),
          stl:Buffer.from(bytes).toString("base64"), imported:arrays(imported),
          emptyCertainty, empty:final.isEmpty(), final:arrays(final) }); }
        finally { final.free(); }
      } finally { imported.free(); }
    } finally { mesh.free(); }
  } finally { moved.free(); cube.free(); empty.free(); }
}
const successfulPhase = { elapsedNs:String(process.hrtime.bigint()-start), cpuMicroseconds:process.cpuUsage(cpu) };
const errorStart = process.hrtime.bigint(), errorCpu = process.cpuUsage();
for (let i=0;i<5;i++) {
  const empty = new MeshJs();
  const bad = MeshJs.fromArrays(Float64Array.from([0,0,0,1,0,0,0,1,0]),Uint32Array.from([0,1,2]));
  try {
    fail("union",() => bad.union(empty));
    fail("intersectionWithContext",() => empty.intersectionWithContext(bad,Boolean(i%2)));
    fail("convexHull",() => bad.convexHull());
    fail("convexHullWithContext",() => bad.convexHullWithContext(Boolean(i%2)));
    fail("subdivide-zero",() => empty.subdivide(0));
  } finally { bad.free(); empty.free(); }
}
const diagnosticPhase = { elapsedNs:String(process.hrtime.bigint()-errorStart), cpuMicroseconds:process.cpuUsage(errorCpu) };
const usage = process.cpuUsage(cpu), elapsedNs = String(process.hrtime.bigint()-start);
process.stdout.write(JSON.stringify({ completed:successes.length, diagnosed:failures.length,
  elapsedNs, cpuMicroseconds:usage, successfulPhase, diagnosticPhase, successes, failures })+"\n");
