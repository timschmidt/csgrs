import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json,sources,here} from './complex-product-v2-sources.mjs';
import {checkComplexProduct} from './check-complex-product-v2.mjs';
const previous=json('complex-product-experiment.json'),sourceMap=sources();
const gates=['build-candidate','freeze','check-candidate','memcheck-candidate','trace-build-candidate',
 'allocation','trace-freeze','trace-candidate','toolchain','machine','cpu','output-check'].map(s=>'complex-product-v2-'+s);
const names=['prepare-complex-product-v2.mjs','complex-product-v2-origin.json','complex-product-v2-sources.mjs',
 'freeze-complex-product-v2.mjs','run-complex-product-v2-costs.mjs','check-complex-product-v2.mjs',
 'complex-product-v2-app-candidate/Cargo.toml','complex-product-v2-app-candidate/Cargo.lock',
 'complex-product-v2-binaries.json','complex-product-v2-trace-binaries.json',
 'complex-product-v2-cpu-summary.json','complex-product-v2-allocation-summary.json',
 'results/complex-product-v2-cpu.jsonl','results/complex-product-v2-allocation.jsonl',
 'complex-product-v2-disposition.json','bind-complex-product-v2.mjs','verify-complex-product-v2.mjs',
 'complex-product-corpus.rs','complex-product-cpu.rs','complex-product-allocation.rs','complex-product-check.rs','complex-product-trace.rs',
 'complex-product-allocation-summary.json'];
for(const kind of ['check','memcheck','trace'])for(const ext of ['json','stdout','stderr'])names.push('results/complex-product-'+kind+'-baseline.'+ext);
for(const tag of gates)for(const ext of ['json','stdout','stderr'])names.push('results/'+tag+'.'+ext);
assert.equal(names.length,68);assert.equal(new Set(names).size,68);
for(const p of names)if(previous.files[p])assert.equal(sha(p),previous.files[p],p);
const frozen=json('complex-product-v2-binaries.json'),traces=json('complex-product-v2-trace-binaries.json');
assert.deepEqual(sourceMap,frozen.sourceMap);
const binaries=[...Object.values(frozen.binaries),...Object.values(traces)];
let newBinaryBytes=0,reusedBinaryBytes=0;
for(const b of binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);
 if(b.path.startsWith(frozen.root+'/'))newBinaryBytes+=b.bytes;else reusedBinaryBytes+=b.bytes;}
const summary=json('complex-product-v2-cpu-summary.json').summaries,oldAllocation=json('complex-product-allocation-summary.json').summaries;
const allocation=json('complex-product-v2-allocation-summary.json').summaries;
const selected=s=>s.bits>=256&&!['unbalanced','mixed-scale','zero'].includes(s.family)&&!(s.route==='lattice'&&s.lifecycle==='reused');
const counts=ss=>({groups:ss.length,minRatio:Math.min(...ss.map(s=>s.pairedMedianRatio)),maxRatio:Math.max(...ss.map(s=>s.pairedMedianRatio)),
 below:ss.filter(s=>s.pairedMedianBootstrap95[1]<1).length,above:ss.filter(s=>s.pairedMedianBootstrap95[0]>1).length,
 overlap:ss.filter(s=>s.pairedMedianBootstrap95[0]<=1&&s.pairedMedianBootstrap95[1]>=1).length});
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(names.map(p=>[p,sha(p)])),sourceMap,
 liveSources:previous.liveSources,coverageAtBinding:previous.coverageAtBinding,newDonorLines:0,
 gates:gates.map(tag=>{const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);return{tag,code:g.code,signal:g.signal};}),
 reusedBaselineGates:['complex-product-check-baseline','complex-product-memcheck-baseline','complex-product-trace-baseline'],
 checks:checkComplexProduct(),selectedCosts:counts(summary.filter(selected)),bypassCosts:counts(summary.filter(s=>!selected(s))),
 selectedPeakLowerThanV1:allocation.filter((s,i)=>selected(s)&&s.measurements.candidate[3].max<oldAllocation[i].measurements.candidate[3].min).length,
 selectedCumulativeDemandMatchesV1:allocation.filter((s,i)=>selected(s)&&JSON.stringify(s.measurements.candidate.slice(0,2))===JSON.stringify(oldAllocation[i].measurements.candidate.slice(0,2))).length,
 binaryFiles:8,newBinaryFiles:4,newBinaryBytes,reusedBinaryBytes,disposition:json('complex-product-v2-disposition.json'),
 hyperReadAdditions:[{path:'hyperlattice/src/complex.rs',sha256:previous.liveSources['hyperlattice/src/complex.rs'],ranges:[[126,269],[326,555]],
  note:'Completes the gaps in the checkpoint-36 read record. All four ownership multiplication/division wrappers delegate to common components; powers and checked-zero boundaries use those helpers. Source reading does not qualify their unmeasured workloads.'}],
 limits:'Same finite release oracle and benchmark corpus as v1; old baseline numerical/memory/trace evidence and four exact binaries are reused, while all CPU/allocation baseline measurements are fresh. No new full crate/debug/state-history/concurrency/internal-unreduced-storage/quotient timing/application-size/WASM/other-architecture qualification. Paired per-group bootstrap intervals are not multiplicity-adjusted. Source copy and dedicated executable sizes do not bound build-cache growth or RSS. Earlier failed gates remain preserved.',
 production:'No live or donor source change. All 955 live hashes remain at the four retained continuation transfers. No cleanup, deletion, commit, push or external report.',
};
writeFileSync(resolve(here,'complex-product-v2-experiment.json'),JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:names.length,gates:gates.length,newBinaryBytes,reusedBinaryBytes,checks:m.checks,
 selectedCosts:m.selectedCosts,bypassCosts:m.bypassCosts,selectedPeakLowerThanV1:m.selectedPeakLowerThanV1,disposition:m.disposition}));
