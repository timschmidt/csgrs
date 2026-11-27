// Node-only conformance driver: independent full endpoints/tags plus digest and
// range-status checks in two fresh, import-free Wasm instances. The first optional
// positional argument selects the binary; the second requests an exclusive-create
// JSON call log after all assertions pass. No internal timeout or memory bound is
// imposed; the guest's panic handler can spin indefinitely, so callers contain it.
import assert from "node:assert/strict";
import { readFile, writeFile } from "node:fs/promises";
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

// Independent cardinal shortcut for indices whose rotation residue is 0..3.
// Park is a signed permutation with one raw unit of radius in each output.
// These bounded integer Number operations are exact; the fifths case instead
// uses the staged BigInt oracle and must not call this helper.
function parkDigest(index) {
  const n = (index % 33) - 16;
  const alpha = n * 2 ** 24;
  const beta = (16 - Math.abs(n)) * 2 ** 24;
  const [sin, cos] = cardinal[index % 5];
  const d = alpha * cos + beta * sin;
  const q = beta * cos - alpha * sin;
  return outputDigest(2, [[d - 1, d + 1], [q - 1, q + 1]]);
}

// Encode one byte of tag and six signed little-endian i32 endpoints, padding an
// absent third lane with zero. Signed i32 endpoints convert to Number exactly.
// Fold FNV-1a modulo 2^64 and return the signed i64 view used by the Wasm JS ABI;
// full endpoint assertions below remain necessary because a digest can collide.
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
// Capture actual call names, numeric arguments and decimal-string return values
// for Thumb replay. Calls execute before capture; no guessed result is logged.
const replayFile = process.argv[3];
const replay = [];
for (let pass = 0; pass < 2; pass++) {
  const instance = await WebAssembly.instantiate(module);
  const calls = [];
  const exports = replayFile ? Object.fromEntries(
    ["conformance_probe", "output_word", "range_probe"].map(name => [name, (...args) => {
      const value = instance.exports[name](...args);
      calls.push([name, args, value.toString()]);
      return value;
    }]),
  ) : instance.exports;
  // All 165 residue combinations, in ascending then descending sample order.
  // Each pass checks full endpoints/tags independently; the retained digest table
  // additionally compares instances. Range cases below keep the same order in both.
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
  if (replayFile) replay.push(calls);
}
// Two arrays of 4,391 calls: 825 digest, 3,315 word and 251 range queries each.
// Existing files fail here after validation rather than being overwritten. File
// creation/write failures propagate; a replay log is not a firmware execution proof.
if (replayFile) await writeFile(replayFile, JSON.stringify(replay), { flag: "wx" });
console.log("825 full transform outputs and 251 range checks passed in each of two fresh instances.");
