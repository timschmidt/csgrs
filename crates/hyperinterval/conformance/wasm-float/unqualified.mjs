// Negative control for WASM targets outside the integer IEEE qualification.
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const module = await WebAssembly.compile(await readFile(process.argv[2]));
const imports = WebAssembly.Module.imports(module);
if (imports.length) {
  assert.deepEqual(imports, [{ module: "env", name: "memory", kind: "memory" }]);
}
async function fresh() {
  const host = imports.length ? {
    env: { memory: new WebAssembly.Memory({ initial: 17, maximum: 32, shared: true }) },
  } : {};
  return (await WebAssembly.instantiate(module, host)).exports;
}
const api = await fresh();
assert.equal(api.qualification(), 0);
let endpointTraps = 0;
for (const [width, one] of [[32, 0x3f800000], [64, 0x3ff0000000000000n]]) {
  for (const call of [
    (guest, selector, empty) => guest[`query${width}`](0, selector, empty, one, one, one, one, 0),
    (guest, selector, empty) => guest[`convert${width}`](selector, empty, one, one),
  ]) {
    for (const [empty, status] of [[0, 21n], [1, 1n]]) {
      assert.equal(call(api, 0, empty), status);
      for (const selector of [1, 2]) {
        const guest = await fresh();
        assert.equal(call(guest, 0, empty), status);
        assert.throws(() => call(guest, selector, empty), WebAssembly.RuntimeError);
        endpointTraps++;
      }
    }
  }
}
console.log(JSON.stringify({ widths: [32, 64], status: 21, empty: 1, endpoint_traps: endpointTraps, imports }));
