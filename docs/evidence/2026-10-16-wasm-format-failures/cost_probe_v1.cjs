// Complete retained API results. Module loading, fixture setup, GC and oracles
// are outside timing. Both variants use the same memory observation hook.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const { fixture, encode, decode, verify } = require("./cost_fixture_v1.cjs");
const [modulePath, directory, mode, encoding, count] = process.argv.slice(2);
assert.ok(global.gc, "run with --expose-gc");
let memory, instances = 0;
const NativeInstance = WebAssembly.Instance;
WebAssembly.Instance = new Proxy(NativeInstance, {
  construct(target, args) {
    const instance = Reflect.construct(target, args);
    memory = instance.exports.memory;
    instances++;
    return instance;
  },
});
let bindings;
try { bindings = require(path.resolve(modulePath)); }
finally { WebAssembly.Instance = NativeInstance; }
assert.equal(instances, 1);
assert.ok(memory instanceof WebAssembly.Memory);
const { MeshJs, GeometryCertaintyJs } = bindings;
const owned = [], outputs = [], meshes = [];
const keep = (value) => { owned.push(value); return value; };
const exportMesh = (mesh) => encoding === "binary" ? mesh.toSTLBinary() : mesh.toSTLASCII();
const bytes = (output) => typeof output === "string" ? Buffer.from(output) : Buffer.from(output.buffer, output.byteOffset, output.byteLength);
const observe = () => ({ ...process.memoryUsage(), wasm_linear_bytes: memory.buffer.byteLength });

try {
  const cubes = Number(count);
  assert.ok(["export", "pipeline", "errors"].includes(mode));
  let operation, iterations, input;
  const errorExpectations = [];
  if (mode === "errors") {
    iterations = 64;
    const unit = keep(MeshJs.cube(1));
    const collapsed = keep(unit.translate(2 ** 24, 0, 0));
    const face = (points) => keep(MeshJs.fromArrays(Float64Array.from(points.flat()), Uint32Array.from([0,1,2])));
    const overflow = face([[0,0,0],[1e50,0,0],[0,1,0]]);
    const degenerate = face([[0,0,0],[0,0,0],[0,0,0]]);
    const open = face([[0,0,0],[1,0,0],[0,1,0]]).toSTLBinary();
    const malformed = new TextEncoder().encode("not an STL");
    const cases = [
      [() => MeshJs.fromArrays(Float64Array.from([0,0]), new Uint32Array()), "GeometryError", "invalid_input"],
      [() => MeshJs.fromArrays(Float64Array.from([NaN,0,0]), new Uint32Array()), "GeometryError", "invalid_input"],
      [() => MeshJs.fromArrays(Float64Array.from([0,0,0]), Uint32Array.from([0,0,1])), "GeometryError", "invalid_input"],
      [() => collapsed.toSTLBinary(), "GeometryIoError", "geometry"],
      [() => overflow.toSTLBinary(), "GeometryIoError", "unrepresentable_coordinate"],
      [() => degenerate.toSTLASCII(), "GeometryIoError", "geometry"],
      [() => MeshJs.fromSTL(malformed), "GeometryIoError", "std_io"],
      [() => MeshJs.fromSTL(open), "GeometryIoError", "construction"],
    ];
    errorExpectations.push(...cases.map(([,name,code]) => ({ name, code })));
    operation = () => {
      for (const [invoke] of cases) {
        let returned;
        try { returned = invoke(); }
        catch (error) { outputs.push(error); continue; }
        if (returned && typeof returned.free === "function") returned.free();
        throw new Error("invalid operation returned a result");
      }
    };
  } else {
    assert.ok(["binary", "ascii"].includes(encoding));
    const data = fixture(cubes);
    fs.writeFileSync(path.join(directory, "input.json"), JSON.stringify(data));
    iterations = cubes === 256 ? 4 : 64;
    if (mode === "export") {
      const mesh = keep(MeshJs.fromArrays(Float64Array.from(data.positions), Uint32Array.from(data.indices)));
      operation = () => { outputs.push(exportMesh(mesh)); };
    } else {
      input = encode(data.faces, encoding);
      fs.writeFileSync(path.join(directory, "input.stl"), input);
      const empty = keep(new MeshJs());
      operation = () => {
        const imported = MeshJs.fromSTL(input);
        try {
          const outcome = imported.unionWithContext(empty, false);
          if (outcome.certainty !== GeometryCertaintyJs.Certified) {
            outcome.free();
            throw new Error("strict composition did not certify parsed geometry");
          }
          const mesh = keep(outcome.intoMesh()); // consumes the result wrapper
          meshes.push(mesh);
          outputs.push(exportMesh(mesh));
        } finally { imported.free(); }
      };
    }
  }
  for (const [phase, runs] of [["first", 1], ["warm", iterations]]) {
    global.gc();
    const before = observe();
    const start = process.hrtime.bigint();
    for (let i = 0; i < runs; i++) operation();
    const elapsed = Number(process.hrtime.bigint() - start);
    const after = observe();
    global.gc();
    const retained = observe();
    console.log(JSON.stringify({ mode, encoding, cubes, phase, iterations: runs,
      operations: runs * (mode === "errors" ? errorExpectations.length : 1), elapsed_ns: elapsed,
      before, after, retained, output_count: outputs.length,
      output_bytes: mode === "errors" ? null : bytes(outputs[outputs.length - 1]).byteLength,
      completion: mode === "errors" ? "typed_failures_without_models" : "verified_integer_geometry_projection" }));
  }
  if (mode === "errors") {
    outputs.forEach((error, index) => {
      assert.ok(error instanceof Error);
      const expected = errorExpectations[index % errorExpectations.length];
      assert.equal(error.name, expected.name);
      assert.equal(error.code, expected.code);
      assert.ok(error.message.length > 0);
      if (error.name === "GeometryIoError") assert.equal(error.format, "STL");
      if (error.code === "construction") {
        assert.ok(error.cause instanceof Error);
        assert.equal(error.cause.name, "GeometryError");
        assert.equal(error.cause.code, "invalid_input");
      }
    });
    const record = (error) => Object.fromEntries(Object.getOwnPropertyNames(error).map((name) =>
      [name, error[name] instanceof Error ? record(error[name]) : error[name]]));
    fs.writeFileSync(path.join(directory, "errors.json"), JSON.stringify(outputs.slice(0, errorExpectations.length).map(record), null, 2));
  } else {
    verify(decode(outputs[0], encoding), cubes);
    for (const output of outputs.slice(1)) assert.deepEqual(bytes(output), bytes(outputs[0]));
    fs.writeFileSync(path.join(directory, "output.data"), bytes(outputs[0]));
    if (mode === "pipeline") {
      // Certainty was checked during every complete pipeline; independent
      // decoded boundaries, winding and integer volume qualify the output.
      assert.equal(meshes.length, outputs.length);
    }
  }
} finally {
  for (const value of owned.reverse()) value.free();
}
