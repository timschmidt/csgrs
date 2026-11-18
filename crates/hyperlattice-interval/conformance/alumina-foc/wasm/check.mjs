import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const file = process.argv[2] ?? new URL(
  "target/wasm32-unknown-unknown/release/interval_foc_wasm_consumer.wasm",
  import.meta.url,
);
const module = await WebAssembly.compile(await readFile(file));
assert.deepEqual(WebAssembly.Module.imports(module), []);

const min = -(2 ** 31);
const max = 2 ** 31 - 1;
const cardinal = [[0, 1], [1, 0], [-1, 0], [0, -1]];

// The ABI returns an endpoint digest. For the cardinal Park transforms,
// the exact map is a signed permutation of the original two intervals.
function parkDigest(index) {
  const n = (index % 33) - 16;
  const alpha = n * 2 ** 24;
  const beta = (16 - Math.abs(n)) * 2 ** 24;
  const [sin, cos] = cardinal[index % 5];
  const d = alpha * cos + beta * sin;
  const q = beta * cos - alpha * sin;
  const bytes = Buffer.alloc(25);
  bytes[0] = 2; // rotating output
  [d - 1, d + 1, q - 1, q + 1, 0, 0].forEach((endpoint, i) => {
    bytes.writeInt32LE(endpoint, 1 + 4 * i);
  });
  let hash = 0xcbf29ce484222325n;
  for (const byte of bytes) {
    hash = BigInt.asUintN(64, (hash ^ BigInt(byte)) * 0x100000001b3n);
  }
  return BigInt.asIntN(64, hash);
}

const previous = [];
for (let pass = 0; pass < 2; pass++) {
  const { exports } = await WebAssembly.instantiate(module);
  // The sample interval centers repeat every 33 indices and the rotations every 5.
  for (let i = 0; i < 165; i++) {
    const index = pass === 0 ? i : 164 - i;
    for (let action = 0; action < 5; action++) {
      const digest = exports.conformance_probe(index, action);
      if (action === 1 && index % 5 < cardinal.length) {
        assert.equal(digest, parkDigest(index), `cardinal Park at sample ${index}`);
      }
      const slot = 5 * index + action;
      if (pass === 0) previous[slot] = digest;
      else assert.equal(digest, previous[slot], `sample ${index}, action ${action}`);
    }
  }
  for (const a of [min, min + 1, -1, 0, 1, max]) {
    for (const b of [min, min + 1, -1, 0, 1, max]) {
      for (const [rotation, [s, c]] of cardinal.entries()) {
        // Park checks each product before combining it. Negating i32::MIN
        // can overflow even when the final mathematical component would fit.
        const values = [a * c, b * s, b * c, a * s, a * c + b * s, b * c - a * s];
        const overflow = values.some(v => v < min || v > max);
        assert.equal(exports.range_probe(a, b, rotation), Number(overflow),
          `Park range at (${a}, ${b}), rotation ${rotation}`);
      }
    }
  }
  // For the 3/5, 4/5 rotation, equal full-scale inputs exceed the d range.
  for (const [value, expected] of [[min, 1], [0, 0], [max, 1]]) {
    assert.equal(exports.range_probe(value, value, 4), expected);
  }
}
console.log("825 transform cases and 147 range cases passed in each of two fresh instances.");
