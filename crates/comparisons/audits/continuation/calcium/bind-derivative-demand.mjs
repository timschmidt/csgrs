import { readFileSync, writeFileSync, statSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources, sha, json } from './derivative-demand-sources.mjs';
import { costs } from './check-derivative-costs.mjs';
import { effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const draft=process.argv.includes('--draft-derivative-demand');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},gates:[],reads:[],binaries:{},
  candidateSources:sources(),costs:costs(),coverageAtBinding:effectiveSummary(),
  status:'No new production/donor change. Demand-bounded derivative v1 remains isolated, promising but not fully qualified or retained. Three exact regression tests pass in baseline/candidate debug/release. Paired public CPU/allocation and focused memory checks complete; broader feature/consumer/state/size qualification remains. LLL source support advanced, simple native/memory controls pass, prior field-relation abort unresolved. Ecosystem goal remains open.',
  limits:'80 warm/prechecked groups at rational parameter 1/3, including fresh curve construction from shared prebuilt controls. All requested rational derivatives preserved. Faster high-order workloads do not establish universal speedup: one low-order control has an interval above one. Allocation requests/bytes lower in 16 groups, peak/live deltas equal in all; 14 groups still retain bytes after eight measured queries and are not a steady-state or leak diagnosis. Public Memchecks have zero errors/losses but reachable caches. No endpoint-helper CPU benchmark, arbitrary-state/fuzz/full CI/WASM execution or representative stripped-example size qualification. LLL controls use simple closed-form full-rank integer bases, not field-relation reproductions, arbitrary inputs or proof of directed-conversion soundness. No donor assertion bypass or crash-evidence deletion.'};
const files=['bind-derivative-demand.mjs','verify-derivative-demand.mjs','check-derivative-costs.mjs',
  'derivative-demand-sources.mjs','prepare-derivative-demand.mjs','derivative-demand-origin.json',
  'record-derivative-lll-reads.mjs','derivative-lll-read-selection.json','flint-lll-controls.c',
  'derivative-demand-harness-initial.rs','derivative-cost-corpus.rs','derivative-cost-cpu.rs',
  'derivative-cost-allocation.rs','run-derivative-costs.mjs','derivative-cost-binaries.json',
  'derivative-cost-cpu-summary.json','derivative-cost-allocation-summary.json','spectral-experiment.json','capture.mjs',
  'results/derivative-cost-cpu.jsonl','results/derivative-cost-allocation.jsonl'];
for(const v of ['baseline','candidate'])for(const f of ['Cargo.toml','Cargo.lock'])files.push('derivative-cost-'+v+'/'+f);
const specs=[
 ['derivative-demand-candidate-focused-debug',101],
 ['derivative-demand-candidate-focused-debug-v2',0],['derivative-demand-baseline-focused-debug',0],
 ['derivative-demand-baseline-focused-release',0],['derivative-demand-candidate-focused-release',0],
 ['derivative-cost-baseline-build',0],['derivative-cost-candidate-build',0],
 ['derivative-demand-candidate-fmt',0],['derivative-public-baseline-memcheck',0],['derivative-public-candidate-memcheck',0],
 ['lll-controls-compile',0],['lll-controls-native',0],['lll-controls-memcheck',0],
 ['derivative-cost-cpu',0],['derivative-cost-allocation',0]];
for(const[tag,code]of specs) {
  const g=json('results/'+tag+'.json');assert.equal(g.code,code);assert.equal(g.signal,null);
  manifest.gates.push({tag,code,signal:null});for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
}
for(const p of files)manifest.files[p]=sha(p);
const selection=json('derivative-lll-read-selection.json'),coverage=json('coverage.json');
manifest.reads=selection.map(s=>{const e=coverage.find(e=>e.repo===s.repo&&e.path===s.path);assert(e);return e;});
const retained=json('retained-monic.json');
for(const[p,h]of Object.entries(retained.liveSources))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
manifest.hyperReadRanges={
 'hypercurve/src/rational_bezier_general.rs':[[1,80],[2400,2510],[5000,5215],[9460,9565],[10810,10925]],
 'hypercurve/src/curve.rs':[[125,175]],
 'hypercurve/Cargo.toml':[[1,readFileSync(resolve(workspace,'hypercurve/Cargo.toml'),'utf8').trimEnd().split('\n').length]],
 'hyperreal/src/rational/convert.rs':[[1,105]]
};
for(const b of Object.values(json('derivative-cost-binaries.json')))manifest.binaries[b.path]={sha256:b.sha256,bytes:b.bytes};
const p='/tmp/calcium-derivative-costs.FA5c2M/lll-controls';manifest.binaries[p]={sha256:sha(p),bytes:statSync(p).size};
manifest.nativeLibrary=json('spectral-experiment.json').nativeLibrary;assert.equal(sha(manifest.nativeLibrary.path),manifest.nativeLibrary.sha256);
if(!draft)writeFileSync(resolve(here,'derivative-demand-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'derivative demand binding',draft,files:files.length,gates:specs.length,reads:manifest.reads.length,
  sourcesPerVariant:Object.fromEntries(Object.entries(manifest.candidateSources).map(([v,s])=>[v,Object.keys(s).length])),
  binaries:Object.keys(manifest.binaries).length,binaryBytes:Object.values(manifest.binaries).reduce((n,b)=>n+b.bytes,0)}));
export {manifest};
