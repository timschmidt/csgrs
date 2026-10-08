import {writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {signFilterSources,sha,json} from './sign-filter-mask-sources.mjs';
import {checkSignFilterMask} from './check-sign-filter-mask.mjs';
const tags=['full-debug','app-build','full-release','trace-build','trace','cpu','allocation','output-check'].map(t=>'sign-filter-mask-'+t);
const files=['setup-sign-filter-mask.mjs','sign-filter-mask-sources.mjs','prepare-sign-filter-mask-costs.mjs',
 'sign-filter-app-mask/Cargo.toml','sign-filter-app-mask/Cargo.lock','run-sign-filter-mask-costs.mjs',
 'check-sign-filter-mask.mjs','bind-sign-filter-mask.mjs','verify-sign-filter-mask.mjs',
 'sign-filter-mask-binaries.json','sign-filter-mask-cpu-summary.json','sign-filter-mask-allocation-summary.json',
 'results/sign-filter-mask-cpu.jsonl','results/sign-filter-mask-allocation.jsonl','sign-filter-experiment.json',
 'record-high-product-reads.mjs','high-product-read-selection.json'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0,tag);assert.equal(g.signal,null);
 for(const e of ['json','stdout','stderr'])files.push('results/'+tag+'.'+e);}
const sourceMaps=signFilterSources(),previous=json('sign-filter-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,'live '+p);
const binaries=json('sign-filter-mask-binaries.json');
binaries['baseline-trace']=previous.binaries['baseline-trace'];
const path='/tmp/calcium-sign-filter-mask.ctu9Hl/candidate-trace';
binaries['candidate-trace']={path,sha256:sha(path),bytes:statSync(path).size};
for(const b of Object.values(binaries)){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
const coverage=json('coverage.json'),readRecords=json('high-product-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path);assert(r);assert.deepEqual(r.ranges,s.newRanges);return r;
});
assert.equal(readRecords.length,14);
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),sourceMaps,liveSources:live,binaries,readRecords,checks:checkSignFilterMask(),
 production:'Unchanged: all 955 retained live scalar/consumer source and support hashes match checkpoint 31. Both constant-state sign-filter prototypes remain isolated and unselected; the four earlier continuation transfers remain retained.',
 status:'The bit mask preserves the finite correctness/Unknown/trace corpus and saves an allocation, with smaller nontrace benchmark artifacts, but matched CPU results remain mixed. No exactness/completeness gain justifies its sampled timing regressions under the requested priorities. Do not retain this implementation.',
 limits:'364 identical tests per profile, including 299593 exhaustive short sequences, 1280 long cases and 32 private traces; 45 public traces agree with the frozen baseline. These are finite checks, not arbitrary expressions, serialization/thread/abort histories, cold scalars or full downstream CI. The old independent 24-ring rational shoelace/Machin sign oracle is reused. CPU measurements comprise 2304 batches/8346816 queries in 48 groups, 12 ABBA blocks per group on CPU 6; separate allocations comprise 288 batches/4608 queries. Pilots and warmups are not timed-query totals. Bootstrap intervals are per group and not multiplicity-adjusted; bypass controls also vary. Allocation measurements are Rust requests/bytes/live/peak, not RSS or a general retention bound. Unstripped driver size deltas are artifact-specific, not representative application size or a proven codegen cause. No new sanitizer, WASM or concurrent qualification is claimed. High-product coverage adds 12 complete files and two partial ranges (2196 lines); donor tests were read, not newly executed, and no independent high-product numerical qualification is claimed. Supporting assembly, FFT and remaining header/docs are still open. Earlier numerical and memory failures remain preserved.',
 followup:'Stop this sign-filter transfer after two unselected implementations. Qualify ordinary bounded high-product inputs against an independent GMP full-product oracle, reconciling the documented n+2 guard-ulp contract with weaker donor tests, then continue called high-product/ARF/generic matrix/field support and every remaining original reference. No unsafe size/alias or prior assertion reproduction.'};
writeFileSync('sign-filter-mask-experiment.json',JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'bit-mask sign filter binding',files:files.length,gates:tags.length,
 sourceMaps:Object.fromEntries(Object.entries(sourceMaps).map(([v,m])=>[v,Object.keys(m).length])),liveFiles:Object.keys(live).length,
 reads:readRecords.length,readLines:2196,binaries:Object.keys(binaries).length,
 newSnapshotBytes:Object.entries(binaries).filter(([k])=>k.startsWith('candidate-')).reduce((n,[,b])=>n+b.bytes,0),
 status:manifest.status,checks:{tests:manifest.checks.tests.perProfile,cpu:manifest.checks.cpu,allocation:manifest.checks.allocation}}));
