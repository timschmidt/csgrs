// Observe guest linear-stack writes during complete, independently checked calls.
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
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
      cases: result.cases, properties: result.properties });
  }
}
console.log(JSON.stringify(observations, null, 2));
