// Negative control for WASM targets outside the integer IEEE qualification.
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const module = await WebAssembly.compile(await readFile(process.argv[2]));
const imports = WebAssembly.Module.imports(module);
let host = {};
if (imports.length) {
  assert.deepEqual(imports, [{ module: "env", name: "memory", kind: "memory" }]);
  host = { env: { memory: new WebAssembly.Memory({ initial: 17, maximum: 32, shared: true }) } };
}
const { exports: api } = await WebAssembly.instantiate(module, host);
assert.equal(api.qualification(), 0);
for (const [query, one] of [[api.query32, 0x3f800000], [api.query64, 0x3ff0000000000000n]]) {
  assert.equal(query(0, 0, 0, one, one, one, one, 0), 21n);
  assert.equal(query(0, 0, 1, one, one, one, one, 0), 1n);
  for (const selector of [1, 2]) {
    for (const empty of [0, 1]) {
      assert.throws(() => query(0, selector, empty, one, one, one, one, 0), WebAssembly.RuntimeError);
    }
  }
}
console.log(JSON.stringify({ widths: [32, 64], status: 21, empty: 1, endpoint_traps: 8, imports }));
