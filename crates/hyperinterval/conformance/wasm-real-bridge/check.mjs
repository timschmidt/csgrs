import { instantiate, stdFunctions } from '../wasm-float/instantiate.mjs';
import { readFile } from 'node:fs/promises';

const module = await WebAssembly.compile(await readFile(process.argv[2]));
const wasiPreview1 = process.argv.includes("--wasi-p1");
let calls = 0;
for (let instance = 0; instance < 2; instance++) {
  const { exports } = await instantiate(module, wasiPreview1, [...stdFunctions, "random_get"]);
  for (let i = 0; i < 2; i++) {
    exports.verify_real_dyadic_retry();
    calls++;
  }
}
console.log(JSON.stringify({ instances: 2, calls, imports: WebAssembly.Module.imports(module) }));
