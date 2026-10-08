import { readFileSync, writeFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft=process.argv.includes('--draft-monic-costs');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},binaries:{},gates:[],reads:[],
  status:'Monic costs and polynomial source checkpoint; no production transfer; full ecosystem incomplete.',
  limits:'Monic candidate remains the immutable checkpoint17 source. No new lifecycle/serde/concurrent or downstream/application-size qualification and no focused in-crate regression tests. CPU intervals are per-group host/corpus estimates, not multiplicity-adjusted or universal bounds. Allocation/peak counts are requested Rust bytes, not native heap, RSS or allocator overhead. Native scalar recurrences share the donor scalar backend; no archived execution or branch instrumentation.'};
const files=['bind-monic-costs.mjs','verify-monic-costs.mjs','capture.mjs','run-monic-costs.mjs',
  'run-monic-churn.mjs','monic-cost-corpus.rs','monic-cpu.rs','monic-allocation.rs',
  'flint-polynomial-compose-probe.c','flint-polynomial-series-probe.c','monic-fractionfree-probe.rs',
  'monic-cost-read-selection.json','polynomial-closure-experiment.json','monic-cost-binaries.json',
  'monic-cost-cpu-summary.json','monic-cost-allocation-summary.json','monic-cost-churn-summary.json',
  'results/monic-cost-cpu.jsonl','results/monic-cost-allocation.jsonl','results/monic-cost-churn.jsonl'];
for(const pkg of ['monic-cost-baseline','monic-cost-trial','monic-fractionfree-probe'])
  for(const file of ['Cargo.toml','Cargo.lock']) files.push(`${pkg}/${file}`);
const gates=['monic-cost-baseline-build','monic-cost-trial-build','monic-cost-cpu-run',
  'monic-cost-allocation-run','monic-cost-churn-run','monic-compose-compile','monic-compose-native',
  'monic-compose-memcheck','monic-fractionfree-debug','monic-fractionfree-release',
  'monic-fractionfree-memcheck','monic-candidate-clippy','monic-candidate-wasm'];
for(const tag of gates){ const g=json(`results/${tag}.json`); assert.equal(g.code,0,tag);assert.equal(g.signal,null);
  manifest.gates.push(tag);for(const suffix of ['json','stdout','stderr'])files.push(`results/${tag}.${suffix}`); }
for(const p of files)manifest.files[p]=sha(p);
const coverage=json('coverage.json');
manifest.reads=json('monic-cost-read-selection.json').map(e=> {
  const v=coverage.find(c=>c.repo===e.repo&&c.path===e.path);assert(v);return v;
});
assert.equal(manifest.reads.length,46);
for(const b of Object.values(json('monic-cost-binaries.json'))){ assert.equal(sha(b.path),b.sha256); manifest.binaries[b.path]=b.sha256; }
const binaryRoot='/tmp/calcium-monic-costs.U6fpvm';
for(const profile of ['debug','release']){
  const source=`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/${profile}/calcium-monic-fractionfree-probe`;
  const path=draft?source:`${binaryRoot}/fractionfree-${profile}`;
  if(!draft)copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  assert.equal(sha(path),sha(source));manifest.binaries[path]=sha(path);
}
const native=`${binaryRoot}/flint-polynomial-compose-probe`;manifest.binaries[native]=sha(native);
if(!draft)writeFileSync(resolve(here,'monic-cost-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'monic costs binding',draft,files:files.length,gates:gates.length,
  readFiles:manifest.reads.length,readLines:manifest.reads.reduce((n,v)=>n+v.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),
  binaries:Object.keys(manifest.binaries).length,binaryBytes:Object.keys(manifest.binaries).reduce((n,p)=>n+statSync(p).size,0)}));
export {manifest};
