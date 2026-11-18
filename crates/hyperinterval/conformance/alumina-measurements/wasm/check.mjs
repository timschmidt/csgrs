import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const file = process.argv[2] ?? new URL(
  "target/wasm32-unknown-unknown/release/interval_measurement_wasm.wasm",
  import.meta.url,
);
const module = await WebAssembly.compile(await readFile(file));
const imports = Object.create(null);
for (const { module: namespace, name, kind } of WebAssembly.Module.imports(module)) {
  assert.equal(kind, "function");
  (imports[namespace] ??= Object.create(null))[name] = () => {
    throw new Error(`measurement called ${namespace}.${name}`);
  };
}

// The public Rational display uses signed mixed numbers with a reduced remainder.
function endpoint(numerator, denominator) {
  const magnitude = numerator < 0n ? -numerator : numerator;
  const sign = numerator < 0n ? "-" : "";
  const whole = magnitude / denominator;
  const remainder = magnitude % denominator;
  if (!remainder) return sign + whole;

  let a = denominator;
  let b = remainder;
  while (b) [a, b] = [b, a % b];
  return sign + (whole ? whole + " " : "") + `${remainder / a}/${denominator / a}`;
}

function expected(n, u, d) {
  if (d === 0n) return -1n;
  if (u < 0n) return 0n;
  const text = `${endpoint(n - u, d)} ${endpoint(n + u, d)} mm`;
  let hash = 0xcbf29ce484222325n;
  for (const byte of Buffer.from(text)) {
    hash = BigInt.asUintN(64, (hash ^ BigInt(byte)) * 0x100000001b3n);
  }
  return BigInt.asIntN(64, hash);
}

const cases = [];
for (let n = -32n; n <= 32n; n++) {
  for (let u = -4n; u <= 8n; u++) {
    for (const d of [1n, 2n, 3n, 7n, 31n]) cases.push([n, u, d]);
  }
}
const min = -(1n << 63n);
const max = -min - 1n;
for (const n of [min, min + 1n, -1n, 0n, 1n, max]) {
  for (const u of [-1n, 0n, 1n, max]) {
    for (const d of [0n, 1n, 2n, 3n, 7n, (1n << 64n) - 1n]) cases.push([n, u, d]);
  }
}

for (const ordered of [cases, [...cases].reverse()]) {
  const { exports } = await WebAssembly.instantiate(module, imports);
  for (const [n, u, d] of ordered) {
    const digest = exports.measurement(n, u, d);
    assert.notEqual(digest, -2n, `unexpected kernel error for (${n}, ${u}, ${d})`);
    assert.equal(digest, expected(n, u, d), `measurement(${n}, ${u}, ${d})`);
  }
}
console.log(`${cases.length} measurement cases passed in each of two fresh instances.`);
