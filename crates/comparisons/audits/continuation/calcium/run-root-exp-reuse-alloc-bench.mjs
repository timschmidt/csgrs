import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const [mode] = process.argv.slice(2);
assert.equal(mode, 'alloc');
const stem = `root-exp-reuse-confirm-${mode}`;
const variants = ['baseline', 'sign', 'reuse'];
const binaries = Object.fromEntries(variants.map(v => [v,
  `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-root-exp-opaque-${v}-bench`]));
const metadata = { mode, cpu: 6, started: new Date().toISOString(), binaries: {} };
for (const [v, path] of Object.entries(binaries)) metadata.binaries[v] = {
  path, bytes: statSync(path).size, sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
  size: execFileSync('size', [path], { encoding: 'utf8' }).trim(),
};
const cases = ['identity-1','identity-8','identity-32','identity-128',
  'unresolved-1','unresolved-8','unresolved-32','unresolved-128'];
const lives = ['fresh','warm-pair','warm-difference'];
const run = (v, name, life, n) => JSON.parse(execFileSync('taskset', ['-c', '6', binaries[v], name, life, String(n)],
  { encoding: 'utf8', timeout: 30000 }));
const median = a => { a = [...a].sort((x,y) => x-y); return (a[Math.floor((a.length-1)/2)] + a[Math.floor(a.length/2)])/2; };
const output = resolve(here, `results/${stem}.jsonl`);
writeFileSync(output, '', { flag: 'wx' });
let seed = 17911;
const rand = n => { seed = (Math.imul(seed, 1664525)+1013904223) >>> 0; return seed % n; };
const summaries = [];
for (const name of cases) for (const lifecycle of lives) {
  const pilots = variants.map(v => run(v, name, lifecycle, 20));
  const ns = Math.max(...pilots.map(p => p.elapsed_ns/p.iterations));
  const iterations = mode === 'alloc' ? (lifecycle === 'fresh' ? 10 : 100) : Math.max(20, Math.min(100000, Math.ceil(1e7/ns)));
  const blocks = mode === 'cpu' ? 12 : 3;
  const rows = [];
  for (let block = 0; block < blocks; block++) {
    // Rotate all three variants through the outside/middle/inside positions.
    const order = variants.map((_, i) => variants[(i + block) % 3]);
    for (const variant of [...order, ...order.toReversed()]) {
      const r = { variant, block, ...run(variant, name, lifecycle, iterations) };
      assert.equal(r.case, name); assert.equal(r.lifecycle, lifecycle); assert.equal(r.iterations, iterations);
      assert(r.elapsed_ns > 0);
      if (mode === 'cpu') assert.equal(r.alloc_calls, 0);
      const expected = name.startsWith('identity') && variant !== 'baseline' ? 0 : 2;
      assert.deepEqual(r.outcomes, [0,1,2].map(i => i === expected ? iterations : 0));
      rows.push(r); appendFileSync(output, JSON.stringify(r)+'\n');
    }
  }
  const comparisons = {};
  for (const control of ['baseline','sign']) {
    const pairs = Array.from({length: blocks}, (_, b) => {
      const get = v => median(rows.filter(r => r.block === b && r.variant === v).map(r => r.elapsed_ns));
      return get('reuse')/get(control);
    });
    const bootstrap = Array.from({length: 5000}, () => median(pairs.map(() => pairs[rand(pairs.length)]))).sort((a,b)=>a-b);
    comparisons[control] = {pairedMedianRatio: median(pairs), pairedMedianBootstrap95: [bootstrap[125],bootstrap[4875]]};
  }
  const med = (v,k) => median(rows.filter(r=>r.variant===v).map(r=>r[k]/iterations));
  const summary = {name, lifecycle, iterations, observations: rows.length, comparisons,
    measurements: Object.fromEntries(variants.map(v => [v, {
      ns: med(v,'elapsed_ns'), alloc_calls: med(v,'alloc_calls'), allocated_bytes: med(v,'allocated_bytes'),
    }]))};
  summaries.push(summary); console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here, `${stem}-summary.json`), JSON.stringify({...metadata, finished: new Date().toISOString(), summaries,
  limits: 'Identity rows compare additional decisions with baseline Unknown; sign versus reuse has matching outcomes. CPU and allocation binaries differ; requested bytes are not peak live memory. TLS cache has bounded weak retention per thread, not a whole-query work bound. Fixed process preconditioning and independent nested-sine atoms at depths 1/8/32/128; not all graph sharing patterns. Host/session bootstrap intervals only.'}, null, 2)+'\n', {flag:'wx'});
