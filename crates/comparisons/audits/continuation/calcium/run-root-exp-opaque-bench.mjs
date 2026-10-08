import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const [mode] = process.argv.slice(2);
const phase = 'query';
assert(['query', 'numeric'].includes(phase) && ['cpu', 'alloc'].includes(mode));
const stem = `root-exp-opaque-${mode}`;
const variants = ['baseline', 'sign'];
const binaries = Object.fromEntries(variants.map(v => [v, resolve(workspace,
  `.audit-builds/calcium-${v === 'baseline' ? 'baseline' : 'prototype'}/release/calcium-root-exp-opaque-${v}-bench`)]));
const metadata = { phase, mode, cpu: 6, started: new Date().toISOString(), binaries: {} };
for (const [v, path] of Object.entries(binaries)) metadata.binaries[v] = {
  path, bytes: statSync(path).size, sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
  size: execFileSync('size', [path], { encoding: 'utf8' }).trim(),
};
const cases = ['identity-1','identity-8','identity-32','identity-128',
  'unresolved-1','unresolved-8','unresolved-32','unresolved-128'];
const lives = ['warm-pair','warm-difference'];
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
  const iterations = mode === 'alloc' ? 100 : Math.max(20, Math.min(100000, Math.ceil(1e7/ns)));
  const blocks = mode === 'cpu' ? 12 : 3;
  const rows = [];
  for (let block = 0; block < blocks; block++) {
    const order = block % 2 ? ['sign','baseline','baseline','sign'] : ['baseline','sign','sign','baseline'];
    for (const variant of order) {
      const r = { variant, block, ...run(variant, name, lifecycle, iterations) };
      assert.equal(r.case, name); assert.equal(r.lifecycle, lifecycle); assert.equal(r.iterations, iterations);
      assert(r.elapsed_ns > 0);
      if (mode === 'cpu') assert.equal(r.alloc_calls, 0);
      if (phase === 'query') {
        const expected = name.startsWith('identity') ? variant === 'sign' ? 0 : 2 : name.startsWith('unresolved') ? 2 : 1;
        assert.deepEqual(r.outcomes, [0,1,2].map(i => i === expected ? iterations : 0));
      }
      rows.push(r); appendFileSync(output, JSON.stringify(r)+'\n');
    }
  }
  const pairs = Array.from({length: blocks}, (_, b) => {
    const get = v => median(rows.filter(r => r.block === b && r.variant === v).map(r => r.elapsed_ns));
    return get('sign')/get('baseline');
  });
  const bootstrap = Array.from({length: 5000}, () => median(pairs.map(() => pairs[rand(pairs.length)]))).sort((a,b)=>a-b);
  const med = (v,k) => median(rows.filter(r=>r.variant===v).map(r=>r[k]/iterations));
  const summary = {name, lifecycle, iterations, observations: rows.length,
    baseline_ns: med('baseline','elapsed_ns'), sign_ns: med('sign','elapsed_ns'),
    pairedMedianRatio: median(pairs), pairedMedianBootstrap95: [bootstrap[125],bootstrap[4875]],
    baseline_alloc_calls: med('baseline','alloc_calls'), sign_alloc_calls: med('sign','alloc_calls'),
    baseline_alloc_bytes: med('baseline','allocated_bytes'), sign_alloc_bytes: med('sign','allocated_bytes')};
  summaries.push(summary); console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here, `${stem}-summary.json`), JSON.stringify({...metadata, finished: new Date().toISOString(), summaries,
  limits: 'Identity rows compare additional decisions with baseline Unknown, not equal work. Unresolved controls require Unknown in both variants. CPU and allocation binaries differ; requested bytes are cumulative, not peak heap. Fixed process preconditioning; host/session bootstrap intervals only. Opaque nested-sine depths are1,8,32,128; both variants use the same independently reconstructed graphs. Collector visit caps do not bound all opaque descendants. Not a whole-program resource bound.'}, null, 2)+'\n', {flag:'wx'});
