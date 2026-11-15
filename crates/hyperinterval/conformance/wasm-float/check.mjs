// Shared Node/browser verifier. Expected endpoint bits come from oracle.py.
export async function verify(bytes, corpus) {
  const module = await WebAssembly.compile(bytes);
  const imports = WebAssembly.Module.imports(module);
  if (imports.length) throw new Error(`Unexpected module imports: ${JSON.stringify(imports)}`);
  const { exports: api } = await WebAssembly.instantiate(module);
  if (api.qualification() !== 3) throw new Error("Both IEEE backends must be qualified");
  const statuses = {};
  const counts = {};
  for (const [index, row] of corpus.cases.entries()) {
    const [width, operation, flags, a, b, c, d, exponent, expected, lower, upper] = row;
    const inputs = [a, b, c, d].map(width === 32 ? Number : BigInt);
    const query = api[`query${width}`];
    const call = selector => BigInt.asUintN(64, query(operation, selector, flags, ...inputs, exponent));
    const status = Number(call(0));
    if (status !== expected) throw new Error(`Case ${index}: status ${status} != ${expected}; ${JSON.stringify(row)}`);
    if (status === 0 || status === 2) {
      const lo = call(1), hi = call(2);
      if (lo !== BigInt(lower) || hi !== BigInt(upper)) {
        throw new Error(`Case ${index}: bits [${lo.toString(16)},${hi.toString(16)}] != [${lower},${upper}]; ${JSON.stringify(row)}`);
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
  return { cases: corpus.cases.length, properties: corpus.properties.length, counts, statuses, imports, seed: corpus.seed };
}

export async function verifyLattice(bytes, corpus) {
  const module = await WebAssembly.compile(bytes);
  const { exports: api } = await WebAssembly.instantiate(module);
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

if (typeof process !== "undefined" && process.argv[1]?.endsWith("check.mjs")) {
  const fs = await import("node:fs/promises");
  const [wasm, corpus, mode] = process.argv.slice(2);
  if (!wasm || !corpus) throw new Error("Usage: node check.mjs MODULE.wasm CORPUS.json");
  const run = mode === "--lattice" ? verifyLattice : verify;
  const report = await run(await fs.readFile(wasm), JSON.parse(await fs.readFile(corpus, "utf8")));
  console.log(JSON.stringify({ ...report, node: process.version, versions: process.versions }, null, 2));
}
