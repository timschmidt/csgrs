import { readFileSync,writeFileSync,statSync } from 'node:fs';
import { dirname,resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources,sha,json } from './derivative-demand-sources.mjs';
import { qualificationCosts } from './check-derivative-qualification-costs.mjs';
import { effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const draft=process.argv.includes('--draft-derivative-qualification');
export const specifications=[
 ['derivative-candidate-consumer-all-debug',0],['derivative-candidate-clippy-all',0],
 ['derivative-candidate-wasm',101],['derivative-baseline-wasm-all',101],
 ['derivative-candidate-wasm-library',0],['derivative-baseline-wasm-library',0],
 ['derivative-wasm-dependency-tree',0],['derivative-retention-staircase',0],
 ...['baseline','candidate'].flatMap(v=>[
  ['derivative-state-'+v+'-debug',0],['derivative-state-'+v+'-release',0],['derivative-state-'+v+'-memcheck',97],
  ['derivative-endpoint-'+v+'-build',101],['derivative-endpoint-'+v+'-build-v2',0],
  ['derivative-endpoint-'+v+'-memcheck',0]]),
 ['derivative-endpoint-cpu',0],['derivative-endpoint-allocation',0],['derivative-app-size',0],
 ...['baseline','candidate'].flatMap(v=>[['derivative-app-'+v+'-build',0],
  ...['basic','arrangement'].flatMap(e=>['strip','size','run'].map(k=>['derivative-app-'+v+'-'+k+'-'+e,0]))]),
 ...(!draft?[['derivative-retained-focused-debug',0],['derivative-retained-focused-release',0],['derivative-retained-fmt',0]]:[])
];
const files=['bind-derivative-qualification.mjs','verify-derivative-qualification.mjs',
 'check-derivative-qualification-costs.mjs','derivative-demand-experiment.json',
 'run-derivative-retention.mjs','derivative-retention-staircase.json','results/derivative-retention-staircase.jsonl',
 'derivative-state-probe.rs','derivative-endpoint-corpus.rs','derivative-endpoint-corpus-initial.rs',
 'derivative-endpoint-cpu.rs','derivative-endpoint-allocation.rs','run-derivative-endpoint.mjs',
 'derivative-endpoint-binaries.json','derivative-endpoint-cpu-summary.json','derivative-endpoint-allocation-summary.json',
 'results/derivative-endpoint-cpu.jsonl','results/derivative-endpoint-allocation.jsonl',
 'measure-derivative-app-size.mjs','derivative-app-size-summary.json',
 'record-derivative-qualification-reads.mjs','derivative-qualification-read-selection.json'];
for(const v of ['baseline','candidate'])for(const p of ['state','endpoint'])for(const f of ['Cargo.toml','Cargo.lock'])
 files.push('derivative-'+p+'-'+v+'/'+f);
for(const[tag,code]of specifications) {
 const g=json('results/'+tag+'.json');assert.equal(g.code,code,tag);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
}
const candidateSources=sources(),coverage=json('coverage.json'),retained=json('retained-monic.json');
const manifest={schema:1,recorded:new Date().toISOString(),draft,files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:specifications.map(([tag,code])=>({tag,code,signal:null})),candidateSources,
 costs:qualificationCosts(),coverageAtBinding:effectiveSummary(),
 reads:json('derivative-qualification-read-selection.json').map(s=>coverage.find(e=>s.repo===e.repo&&s.path===e.path)),
 hyperReadRanges:{
  'hypercurve/benches/bezier_tangent_order.rs':[[1,213]],
  'hypercurve/Cargo.toml':[[1,readFileSync(resolve(workspace,'hypercurve/Cargo.toml'),'utf8').trimEnd().split('\n').length]],
  'hyperreal/AGENTS.md':[[1,readFileSync(resolve(workspace,'hyperreal/AGENTS.md'),'utf8').trimEnd().split('\n').length]],
  'hyperreal/src/rational/arithmetic/representation.rs':[[100,215],[280,330]],
  'hyperreal/src/rational/arithmetic/ops.rs':[[733,1015]],
  'hypercurve/src/bezier_arrangement.rs':[[1,170],[222,345],[1064,1128],[3010,3145],[3220,3345]],
  'hypercurve/src/bezier_split.rs':[[940,1045]],
  'hypercurve/tests/hypercurve_bezier_arrangement.rs':[[1,120]]
 },
 binaries:{},
 production:{paths:['hypercurve/src/rational_bezier_general.rs','hypercurve/src/derivative_demand_tests.rs'],
  priorSnapshot:'polynomial-monic-qualified-trial',qualifiedSnapshot:'derivative-demand-candidate',
  priorSourceFiles:954,qualifiedSourceFiles:955,algorithmAddedLines:6,algorithmRemovedLines:4,testLines:170},
 status:draft?'Qualified isolated derivative v1; production retention pending final source check and focused live gates.':
  'Retained demand-bounded polynomial Horner derivative work in Hypercurve, with all requested polynomial outputs and rational quotient orders preserved. Previous scalar and Hypersolve improvements unchanged. Full ecosystem audit remains open.',
 limits:'Workload-specific improvement, not universal speedup: the earlier pi-scaled degree-one/order-zero retained control has a slowdown interval. Representative stripped examples grow 224/208 bytes. All-feature WASM compilation fails identically in the optional comparative-benchmark getrandom dependency; ordinary library features compile, no WASM execution. Threaded Memchecks retain one identical 48-byte possible loss in Rust thread initialization plus reachable caches; not clean all-freed gates. Endpoint Memchecks have no errors/losses. Empirical retention plateaus through 512 queries are not arbitrary-program memory bounds. No scalar/backend replacement, donor assertion bypass or resolution of the prior matrix-function assertion. No full release consumer suite, full CI, arbitrary-state fuzzing or whole-application size qualification.'
};
for(const p of Object.keys(retained.liveSources))assert.equal(sha(retained.frozenSnapshot+'/'+p),retained.liveSources[p]);
for(const b of Object.values(json('derivative-endpoint-binaries.json')))manifest.binaries[b.path]={sha256:b.sha256,bytes:b.bytes};
const apps=json('derivative-app-size-summary.json');
for(const b of [...apps.artifacts.flatMap(a=>a.files),...apps.state])manifest.binaries[b.path]={sha256:b.sha256,bytes:b.bytes};
for(const[p,b]of Object.entries(manifest.binaries)){assert.equal(sha(p),b.sha256);assert.equal(statSync(p).size,b.bytes);}
const live=draft?retained.liveSources:candidateSources.candidate;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
if(!draft)writeFileSync(resolve(here,'derivative-qualification-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'derivative qualification binding',draft,files:files.length,gates:specifications.length,
 reads:manifest.reads.length,sourceFilesPerVariant:955,binaries:Object.keys(manifest.binaries).length,
 binaryBytes:Object.values(manifest.binaries).reduce((n,b)=>n+b.bytes,0)}));
export {manifest};
