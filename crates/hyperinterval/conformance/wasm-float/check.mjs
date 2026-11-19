// Shared Node/browser verifier. Expected endpoint bits come from oracle.py.
import { instantiate, stdFunctions } from "./instantiate.mjs";

// The corpus width identifies the input encoding for both kinds of query.
function endpointQuery(api, width, operation, flags, [a, b, c, d], exponent) {
  if (operation === 21) {
    const convert = api[`convert${width}`];
    return selector => convert(selector, flags & 1, a, b);
  }
  const query = api[`query${width}`];
  return selector => query(operation, selector, flags, a, b, c, d, exponent);
}

export async function verify(bytes, corpus, wasiPreview1 = false) {
  const module = await WebAssembly.compile(bytes);
  const imports = WebAssembly.Module.imports(module);
  const { exports: api } = await instantiate(module, wasiPreview1);
  const result = verifyInstance(api, corpus);
  let endpointTraps = 0;
  for (const width of [32, 64]) {
    // Empty, nonfinite, reversed, overflow, zero divisor and empty domain.
    const failures = [1, 16, 17, 18, 19, 20].map(status => [undefined, status]);
    // Widening always fits. Narrowing can overflow at either endpoint.
    const conversionFailures = width === 32 ? [1, 16, 17] : [1, 16, 17, 18];
    for (const status of conversionFailures) failures.push([21, status]);
    if (!corpus.cases.some(row => row[0] === width && row[1] === 21 && row[8] === 0)) {
      throw new Error(`Missing successful binary${width} format conversion`);
    }
    for (const [requiredOperation, status] of failures) {
      const row = corpus.cases.find(row => row[0] === width && row[8] === status &&
        (requiredOperation === undefined || row[1] === requiredOperation));
      if (!row) throw new Error(`Missing binary${width} oracle case for operation ${requiredOperation ?? "any"}, status ${status}`);
      const [, operation, flags, a, b, c, d, exponent] = row;
      const inputs = [a, b, c, d].map(width === 32 ? Number : BigInt);
      for (const selector of [1, 2]) {
        // A trap may leave the guest stack pointer inside the aborted call.
        const instance = await instantiate(module, wasiPreview1);
        const query = endpointQuery(instance.exports, width, operation, flags, inputs, exponent);
        if (query(0) !== BigInt(status)) {
          throw new Error(`Fresh binary${width} instance did not reproduce status ${status}`);
        }
        try {
          query(selector);
        } catch (error) {
          if (!(error instanceof WebAssembly.RuntimeError)) throw error;
          endpointTraps++;
          continue;
        }
        throw new Error(`Binary${width} returned endpoint ${selector} for status ${status}`);
      }
    }
  }
  return { ...result, endpoint_traps: endpointTraps, imports };
}

export function verifyInstance(api, corpus) {
  if (api.qualification() !== 3) throw new Error("Both IEEE backends must be qualified");
  const statuses = {};
  const counts = {};
  for (const [index, row] of corpus.cases.entries()) {
    const [width, operation, flags, a, b, c, d, exponent, expected, lower, upper, outerLower, outerUpper] = row;
    const inputs = [a, b, c, d].map(width === 32 ? Number : BigInt);
    const call = endpointQuery(api, width, operation, flags, inputs, exponent);
    const status = Number(call(0));
    if (status !== expected) throw new Error(`Case ${index}: status ${status} != ${expected}; ${JSON.stringify(row)}`);
    if (status === 0 || status === 2) {
      const lo = BigInt.asUintN(64, call(1)), hi = BigInt.asUintN(64, call(2));
      if (outerLower === undefined) {
        if (lo !== BigInt(lower) || hi !== BigInt(upper)) {
          throw new Error(`Case ${index}: bits [${lo.toString(16)},${hi.toString(16)}] != [${lower},${upper}]; ${JSON.stringify(row)}`);
        }
      } else {
        const sign = 1n << BigInt(width - 1);
        const rank = bits => bits < sign ? bits : -(bits - sign);
        if (lo === sign || hi === sign || lo >= 2n * sign || hi >= 2n * sign ||
            rank(lo) < rank(BigInt(outerLower)) || rank(lo) > rank(BigInt(lower)) ||
            rank(hi) < rank(BigInt(upper)) || rank(hi) > rank(BigInt(outerUpper))) {
          throw new Error(`Case ${index}: power bounds [${lo.toString(16)},${hi.toString(16)}] outside endpoint allowances; ${JSON.stringify(row)}`);
        }
      }
    }
    const key = `${width}/${operation}`;
    counts[key] = (counts[key] ?? 0) + 1;
    statuses[status] = (statuses[status] ?? 0) + 1;
  }
  for (const [index, row] of corpus.properties.entries()) {
    const [width, flags, a, b, c, d, point, expected] = row;
    const inputs = [a, b, c, d, point].map(width === 32 ? Number : BigInt);
    const actual = BigInt.asUintN(64, api[`properties${width}`](flags, ...inputs));
    if (actual !== BigInt(expected)) throw new Error(`Property ${index}: ${actual.toString(16)} != ${expected}; ${JSON.stringify(row)}`);
  }
  return { cases: corpus.cases.length, properties: corpus.properties.length, counts, statuses, seed: corpus.seed };
}

export async function verifyLattice(bytes, corpus, wasiPreview1 = false) {
  const module = await WebAssembly.compile(bytes);
  const { exports: api } = await instantiate(module, wasiPreview1, stdFunctions);
  if (api.float_qualification() !== 0) throw new Error("Lattice consumer still rejects IEEE ingress");
  let floating = 0, fixed = 0;
  for (const [width, dimension, seed, lower, upper, lo, hi] of corpus.lattice) {
    for (const soa of [0, 1]) {
      for (const endpoint of [0, 1]) {
        const actual = BigInt.asUintN(64, api.floating(soa, width === 64 ? 1 : 0, dimension, seed, endpoint));
        const expected = BigInt(endpoint ? upper : lower);
        if (actual !== expected) throw new Error(`Lattice IEEE mismatch: ${[width, dimension, seed, soa, endpoint, actual, expected]}`);
        const fixedActual = api.fixed(soa, width === 64 ? 1 : 0, dimension, seed, endpoint);
        const fixedExpected = BigInt(endpoint ? hi : lo) << (width === 64 ? 24n : 0n);
        if (fixedActual !== fixedExpected) throw new Error(`Lattice fixed mismatch: ${[width, dimension, seed, soa, endpoint, fixedActual, fixedExpected]}`);
      }
      floating++;
      fixed++;
    }
  }
  return { floating_chains: floating, fixed_chains: fixed, seed: corpus.seed };
}

export async function verifyEgress(bytes, _corpus, wasiPreview1 = false) {
  const module = await WebAssembly.compile(bytes);
  const imports = WebAssembly.Module.imports(module);
  const { exports: api } = await instantiate(module, wasiPreview1, stdFunctions);
  const cases = api.verify_dyadic_egress();
  if (cases !== 56) throw new Error(`Expected all 56 exact conversion cases, got ${cases}`);
  return { dyadic_egress_cases: cases, imports };
}

if (typeof process !== "undefined" && process.argv[1]?.endsWith("check.mjs")) {
  const fs = await import("node:fs/promises");
  const [wasm, corpus, mode] = process.argv.slice(2).filter(arg => arg !== "--wasi-p1");
  if (!wasm || !corpus) throw new Error("Usage: node check.mjs MODULE.wasm CORPUS.json");
  const run = mode === "--egress" ? verifyEgress : mode === "--lattice" ? verifyLattice : verify;
  const report = await run(await fs.readFile(wasm), JSON.parse(await fs.readFile(corpus, "utf8")), process.argv.includes("--wasi-p1"));
  console.log(JSON.stringify({ ...report, node: process.version, versions: process.versions }, null, 2));
}
