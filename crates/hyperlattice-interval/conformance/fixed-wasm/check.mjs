import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const module = await WebAssembly.compile(await readFile(process.argv[2]));
assert.deepEqual(WebAssembly.Module.imports(module), []);

// Affine images attain each coordinate extremum at a source corner. Apply
// the original point map at each step, independently of the guest's chain.
function boxDigest(corners) {
  const xs = corners.map(([x]) => x);
  const ys = corners.map(([, y]) => y);
  return Math.min(...xs) + 3 * Math.max(...xs) + 5 * Math.min(...ys) + 7 * Math.max(...ys);
}

const inputs = [-(2 ** 31), ...Array.from({ length: 2049 }, (_, i) => i - 1024), 2 ** 31 - 1];
const depths = [...Array.from({ length: 34 }, (_, n) => n), 255, 2 ** 32 - 1];
let calls = 0;
const memoryBytes = [];
for (let pass = 0; pass < 2; pass++) {
  const { exports } = await WebAssembly.instantiate(module);
  const bytes = exports.memory.buffer.byteLength;
  for (const input of pass === 0 ? inputs : [...inputs].reverse()) {
    const s = Math.max(-256, Math.min(256, input));
    let corners = [[s, -1], [s + 1, -1], [s, 2], [s + 1, 2]];
    function check(name, expected, ...args) {
      assert.equal(exports[name](input, ...args), expected, `${name}(${[input, ...args]})`);
      calls++;
    }
    check("affine_correlation_probe", 7);
    check("interval_affine_probe", boxDigest(corners.map(([x, y]) => [2 * x - y + 3, x + y - 4])));
    const images = [boxDigest(corners)];
    for (let step = 1; step <= 32; step++) {
      corners = corners.map(([x, y]) => [x + y + 1, y - 1]);
      images.push(boxDigest(corners));
    }
    for (const depth of depths) {
      check("affine_chain_probe", images[depth % 33], depth);
    }
    check("interval_norm_probe", 50 - 44 * Math.max(2, Math.min(256, input)));
    const width = Math.max(0, Math.min(255, input));
    check("interval_width_probe", width === 0 ? 0 : 3000 + width);
    check("midpoint_radius_probe", 2 * s + 31);
    const low = s === 0 || s === 1 ? 0 : Math.min(Math.abs(s), Math.abs(1 - s));
    const high = Math.max(Math.abs(s), Math.abs(1 - s));
    check("contraction_probe", 1 - Math.max(high, 1) - 3 * high + 7 * (1 - low));
  }
  assert.equal(exports.memory.buffer.byteLength, bytes);
  memoryBytes.push(bytes);
}
assert.equal(calls, 2 * inputs.length * (6 + depths.length));
console.log(JSON.stringify({ instances: 2, inputs_per_instance: inputs.length, calls, memory_bytes: memoryBytes }));
