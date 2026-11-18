// Observe guest linear-stack writes during complete, independently checked calls.
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { verifyInstance } from "./check.mjs";

const [binary, disassembly, input] = process.argv.slice(2);
const module = await WebAssembly.compile(await readFile(binary));
const assembly = await readFile(disassembly, "utf8");
const corpus = JSON.parse(await readFile(input, "utf8"));
assert.deepEqual(WebAssembly.Module.imports(module), []);
assert.doesNotMatch(assembly, /^\s*\(start\s/m, "module initialization could leave live stack data");
const initialStack = /^\s*\(global \$__stack_pointer \(mut i32\) \(i32.const (\d+)\)\)$/m.exec(assembly);
assert.ok(initialStack, "unknown linker stack layout");
const stackBytes = Number(initialStack[1]);
assert.ok(stackBytes > 0);
for (const line of assembly.split("\n")) {
  if (!/^\s*\(data\b/.test(line)) continue;
  const start = /^\s*\(data (?:\$\S+ )?\(i32.const (\d+)\) "/.exec(line);
  assert.ok(start && Number(start[1]) >= stackBytes, "data segment could overlap the linker stack");
}

// Measure unwritten reservations on a second copy. wasm-dis names every
// reference to this global; replace each write with a guest-only observer.
async function measureReservations() {
  const writes = /^(\s*)\(global\.set \$__stack_pointer(?=\s)/gm;
  const writeCount = [...assembly.matchAll(writes)].length;
  assert.ok(writeCount > 0, "missing guest stack updates");
  assert.doesNotMatch(assembly, /interval_probe_/);
  const instrumented = assembly.trimEnd().replace(writes, "$1(call $interval_probe_set");
  assert.doesNotMatch(instrumented, /\(global\.set \$__stack_pointer(?=\s)/,
    "unrecognized guest stack write");
  assert.ok(instrumented.endsWith(")"));
  const source = instrumented.slice(0, -1) + `
    (global $interval_probe_min (mut i32) (i32.const ${stackBytes}))
    (export "interval_probe_min" (global $interval_probe_min))
    (export "interval_probe_sp" (global $__stack_pointer))
    (func $interval_probe_set (param $sp i32)
      (if (i32.gt_u (local.get $sp) (i32.const ${stackBytes})) (then (unreachable)))
      (global.set $__stack_pointer (local.get $sp))
      (if (i32.lt_u (local.get $sp) (global.get $interval_probe_min))
        (then (global.set $interval_probe_min (local.get $sp)))))
  )`;
  const dir = await mkdtemp(join(tmpdir(), "interval-stack-"));
  let tracked;
  try {
    const wat = join(dir, "tracked.wat"), wasm = join(dir, "tracked.wasm");
    const original = join(dir, "linked.wat");
    execFileSync("wasm-dis", [binary, "-o", original]);
    assert.ok(await readFile(original, "utf8") === assembly, "disassembly does not match linked binary");
    await writeFile(wat, source);
    execFileSync("wasm-as", [wat, "--all-features", "-o", wasm]);
    tracked = await WebAssembly.compile(await readFile(wasm));
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
  const { exports: api } = await WebAssembly.instantiate(tracked);
  const checked = {};
  for (const [name, fn] of Object.entries(api)) {
    if (typeof fn !== "function") continue;
    checked[name] = (...args) => {
      const result = fn(...args);
      assert.equal(api.interval_probe_sp.value, stackBytes, `${name} left an open stack frame`);
      return result;
    };
  }
  const depths = {};
  for (const temperature of ["cold", "warm"]) {
    api.interval_probe_min.value = stackBytes;
    verifyInstance(checked, corpus);
    depths[temperature] = stackBytes - api.interval_probe_min.value;
  }
  return depths;
}

const reservations = await measureReservations();
const observations = [];
for (const sentinel of [0x55, 0xaa]) {
  const { exports: api } = await WebAssembly.instantiate(module);
  const memory = api.memory;
  const linearBytes = memory.buffer.byteLength;
  for (const temperature of ["cold", "warm"]) {
    const stack = new Uint8Array(memory.buffer, 0, stackBytes);
    stack.fill(sentinel);
    const result = verifyInstance(api, corpus);
    assert.equal(memory.buffer.byteLength, linearBytes);
    const firstWrite = stack.findIndex(byte => byte !== sentinel);
    assert.ok(firstWrite >= 0, "no stack write observed");
    observations.push({ sentinel, temperature, linearBytes, stackBytes,
      touchedBytes: stackBytes - firstWrite, untouchedBytes: firstWrite,
      reservedBytes: reservations[temperature],
      cases: result.cases, properties: result.properties });
  }
}
console.log(JSON.stringify(observations, null, 2));
