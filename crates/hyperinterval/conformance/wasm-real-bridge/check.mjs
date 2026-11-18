import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const module = await WebAssembly.compile(await readFile(process.argv[2]));
assert.deepEqual(WebAssembly.Module.imports(module), []);
let calls = 0;
for (let instance = 0; instance < 2; instance++) {
  const { exports } = await WebAssembly.instantiate(module);
  for (let i = 0; i < 2; i++) {
    exports.verify_real_dyadic_retry();
    calls++;
  }
}
console.log(JSON.stringify({ instances: 2, calls }));
