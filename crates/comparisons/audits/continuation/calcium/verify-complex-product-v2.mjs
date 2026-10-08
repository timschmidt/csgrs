import './verify-complex-product.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json,sources,here} from './complex-product-v2-sources.mjs';
import {checkComplexProduct} from './check-complex-product-v2.mjs';
const m=json('complex-product-v2-experiment.json'),previous=json('complex-product-experiment.json');
assert.equal(Object.keys(m.files).length,68);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMap,sources());assert.deepEqual(m.liveSources,previous.liveSources);
assert.deepEqual(m.sourceMap.baseline,m.liveSources);assert.equal(Object.keys(m.liveSources).length,955);
assert.deepEqual(m.coverageAtBinding,previous.coverageAtBinding);assert.equal(m.newDonorLines,0);
const p='hyperreal/src/rational/arithmetic/aggregate_products.rs';
const code=readFileSync(resolve(here,'complex-product-v2-candidate',p),'utf8');
assert.match(code,/#\[inline\(never\)\]\n    fn complex_product_common_scale/);
assert.match(code,/word_parts\.iter\(\)\.all\(Option::is_none\)/);assert.match(code,/left\[0\]\.numerator\.bits\(\) >= 256/);
assert.match(code,/widths\.iter\(\)\.any\(\|&bits\| bits < 256\)/);assert.match(code,/let cross = \{\n            let ab/);
assert.equal(m.gates.length,12);assert.equal(new Set(m.gates.map(g=>g.tag)).size,12);
for(const g of m.gates){const r=json('results/'+g.tag+'.json');assert.equal(r.tag,g.tag);assert.equal(r.code,0);assert.equal(r.signal,null);assert.equal(g.code,r.code);assert.equal(g.signal,r.signal);assert(Date.parse(r.finished)>=Date.parse(r.started));}
assert.deepEqual(m.reusedBaselineGates,['complex-product-check-baseline','complex-product-memcheck-baseline','complex-product-trace-baseline']);
for(const tag of m.reusedBaselineGates)for(const ext of ['json','stdout','stderr'])assert.equal(m.files['results/'+tag+'.'+ext],previous.files['results/'+tag+'.'+ext]);
for(const file of ['complex-product-corpus.rs','complex-product-cpu.rs','complex-product-allocation.rs','complex-product-check.rs','complex-product-trace.rs'])assert.equal(m.files[file],previous.files[file]);
const frozen=json('complex-product-v2-binaries.json'),old=json('complex-product-binaries.json'),traces=json('complex-product-v2-trace-binaries.json');
for(const kind of ['cpu','allocation','check'])assert.deepEqual(frozen.binaries['baseline-'+kind],old.binaries['baseline-'+kind]);
assert.deepEqual(traces.baseline,json('complex-product-trace-binaries.json').baseline);
const binaries=[...Object.values(frozen.binaries),...Object.values(traces)];assert.equal(binaries.length,8);
let fresh=0,reused=0,count=0;for(const b of binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);if(b.path.startsWith(frozen.root+'/')){fresh+=b.bytes;count++;}else reused+=b.bytes;}
assert.equal(m.binaryFiles,8);assert.equal(count,4);assert.equal(m.newBinaryFiles,4);assert.equal(fresh,m.newBinaryBytes);assert.equal(reused,m.reusedBinaryBytes);
assert.deepEqual(m.checks,checkComplexProduct());assert.deepEqual(m.checks,json('results/complex-product-v2-output-check.stdout'));
assert.equal(m.checks.candidateSelections,1800);assert.equal(m.selectedCosts.groups,60);assert.equal(m.bypassCosts.groups,132);
assert.equal(m.selectedPeakLowerThanV1,36);assert.equal(m.selectedCumulativeDemandMatchesV1,60);
const selected=s=>s.bits>=256&&!['unbalanced','mixed-scale','zero'].includes(s.family)&&!(s.route==='lattice'&&s.lifecycle==='reused');
const summaries=json('complex-product-v2-cpu-summary.json').summaries;
const counts=ss=>({groups:ss.length,minRatio:Math.min(...ss.map(s=>s.pairedMedianRatio)),maxRatio:Math.max(...ss.map(s=>s.pairedMedianRatio)),
 below:ss.filter(s=>s.pairedMedianBootstrap95[1]<1).length,above:ss.filter(s=>s.pairedMedianBootstrap95[0]>1).length,
 overlap:ss.filter(s=>s.pairedMedianBootstrap95[0]<=1&&s.pairedMedianBootstrap95[1]>=1).length});
assert.deepEqual(m.selectedCosts,counts(summaries.filter(selected)));assert.deepEqual(m.bypassCosts,counts(summaries.filter(s=>!selected(s))));
const alloc=json('complex-product-v2-allocation-summary.json').summaries,oldAlloc=json('complex-product-allocation-summary.json').summaries;
assert.equal(m.selectedPeakLowerThanV1,alloc.filter((s,i)=>selected(s)&&s.measurements.candidate[3].max<oldAlloc[i].measurements.candidate[3].min).length);
assert.equal(m.selectedCumulativeDemandMatchesV1,alloc.filter((s,i)=>selected(s)&&JSON.stringify(s.measurements.candidate.slice(0,2))===JSON.stringify(oldAlloc[i].measurements.candidate.slice(0,2))).length);
for(const r of m.hyperReadAdditions){assert.equal(sha(resolve(here,'../../../..',r.path)),r.sha256);assert.equal(r.sha256,m.liveSources[r.path]);const s=readFileSync(resolve(here,'../../../..',r.path),'utf8');const n=s.split('\n').length-Number(s.endsWith('\n'));for(const[a,b]of r.ranges)assert(a>=1&&b>=a&&b<=n);}
assert.equal(sha('results/complex-product-v2-toolchain.stdout'),previous.files['results/complex-product-toolchain.stdout']);
assert.deepEqual(m.disposition,json('complex-product-v2-disposition.json'));
console.log(JSON.stringify({checkpoint:'Cold rational complex-product v2: crossover, dispatch and temporary lifetime',files:68,gates:12,
 liveFiles:955,newDonorLines:0,newBinaryFiles:4,newBinaryBytes:fresh,reusedBinaryBytes:reused,checks:m.checks,
 selectedCosts:m.selectedCosts,bypassCosts:m.bypassCosts,selectedPeakLowerThanV1:m.selectedPeakLowerThanV1,
 disposition:m.disposition,limits:m.limits,production:m.production}));
