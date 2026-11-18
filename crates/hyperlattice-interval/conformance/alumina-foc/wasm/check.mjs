import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { transforms, parkRange } from "./oracle.mjs";

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
  return outputDigest(2, [[d - 1, d + 1], [q - 1, q + 1]]);
}

function outputDigest(tag, lanes) {
  const bytes = Buffer.alloc(25); // Unused third lanes are zero.
  bytes[0] = tag;
  lanes.flat().forEach((endpoint, i) => bytes.writeInt32LE(Number(endpoint), 1 + 4 * i));
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
    const outputs = transforms(index);
    for (let action = 0; action < 5; action++) {
      const digest = exports.conformance_probe(index, action);
      if (action === 1 && index % 5 < cardinal.length) {
        assert.equal(digest, parkDigest(index), `cardinal Park at sample ${index}`);
      }
      const [tag, lanes] = outputs[action];
      const at = `sample ${index}, action ${action}`;
      assert.equal(digest, outputDigest(tag, lanes), at);
      assert.equal(exports.output_word(index, action, 3), BigInt(tag), at);
      for (let word = 0; word < 3; word++) {
        const bits = exports.output_word(index, action, word);
        const actual = [BigInt.asIntN(32, bits), BigInt.asIntN(32, bits >> 32n)];
        assert.deepEqual(actual, lanes[word] ?? [0n, 0n], `${at}, lane ${word}`);
      }
      if (index === 0) {
        assert.equal(exports.output_word(index, action, 4), exports.output_word(index, action, 0));
        assert.equal(exports.output_word(index, action, 0xffffffff), BigInt(tag));
      }
      const slot = 5 * index + action;
      if (pass === 0) previous[slot] = digest;
      else assert.equal(digest, previous[slot], `sample ${index}, action ${action}`);
    }
  }
  for (const a of [min, min + 1, -1, 0, 1, max]) {
    for (const b of [min, min + 1, -1, 0, 1, max]) {
      assert.equal(exports.range_probe(a, b, 4), parkRange(BigInt(a), BigInt(b), 4),
        `fifths rotation at (${a}, ${b})`);
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
  // The sum of the two nominal rotation coefficients is 7/5. Exercise both
  // sides of the positive and negative representability boundaries.
  for (const extreme of [min, max]) {
    const center = BigInt(extreme) * 5n / 7n;
    for (let offset = -8n; offset <= 8n; offset++) {
      const a = center + offset;
      for (const b of [a, -a]) {
        assert.equal(exports.range_probe(Number(a), Number(b), 4), parkRange(a, b, 4),
          `fifths rotation at (${a}, ${b})`);
      }
    }
  }
  // For the 3/5, 4/5 rotation, equal full-scale inputs exceed the d range.
  for (const [value, expected] of [[min, 1], [0, 0], [max, 1]]) {
    assert.equal(exports.range_probe(value, value, 4), expected);
  }
}
console.log("825 full transform outputs and 251 range checks passed in each of two fresh instances.");
