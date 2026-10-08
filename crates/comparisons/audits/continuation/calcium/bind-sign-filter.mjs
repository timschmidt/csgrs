import {writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {signFilterSources,sha,json} from './sign-filter-sources.mjs';
import {checkSignFilter} from './check-sign-filter.mjs';
const tags=['sign-filter-candidate-focused-debug','sign-filter-candidate-full-debug','sign-filter-baseline-full-debug',
 'sign-filter-candidate-full-release','sign-filter-baseline-full-release','sign-filter-baseline-app-build',
 'sign-filter-candidate-app-build','sign-filter-baseline-trace-build','sign-filter-candidate-trace-build',
 'sign-filter-baseline-trace','sign-filter-candidate-trace','sign-filter-ring-oracle','sign-filter-cpu',
 'sign-filter-allocation','sign-filter-output-check'];
const files=['setup-sign-filter.mjs','sign-filter-origin.json','sign-filter-sources.mjs','sign_filter_summary_tests.rs',
 'sign-filter-corpus.rs','sign-filter-cpu.rs','sign-filter-allocation.rs','run-sign-filter-costs.mjs',
 'sign-filter-ring-oracle.mjs','sign-filter-binaries.json','sign-filter-cpu-summary.json','sign-filter-allocation-summary.json',
 'check-sign-filter.mjs','bind-sign-filter.mjs','verify-sign-filter.mjs','nfloat-complex-experiment.json',
 'sign-filter-app-baseline/Cargo.toml','sign-filter-app-baseline/Cargo.lock','sign-filter-app-candidate/Cargo.toml',
 'sign-filter-app-candidate/Cargo.lock','results/sign-filter-cpu.jsonl','results/sign-filter-allocation.jsonl'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0,tag);assert.equal(g.signal,null);
 for(const e of ['json','stdout','stderr'])files.push('results/'+tag+'.'+e);}
const sourceMaps=signFilterSources(),live=json('sign-filter-origin.json').sourceHashes;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,'live '+p);
const binaries=json('sign-filter-binaries.json');
for(const v of ['baseline','candidate']) {
 const path='/tmp/calcium-sign-filter.TCGoGs/'+v+'-trace';binaries[v+'-trace']={path,sha256:sha(path),bytes:statSync(path).size};
}
for(const b of Object.values(binaries)){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),sourceMaps,liveSources:live,binaries,checks:checkSignFilter(),
 production:'Unchanged: all 955 live source/support hashes match checkpoint 30. The two-sign representative-array prototype is isolated and unselected; four earlier continuation changes remain retained.',
 status:'The exact constant-state summary is semantically successful on exhaustive/long/trace and matched public tests, but this implementation is not retained: one saved allocation is accompanied by mixed CPU results and larger benchmark executables. The broader idea remains open.',
 limits:'No new donor line coverage this turn; prior 806 complete/13 partial/98501-line continuation scope remains. 299593 short sequences through length six and 1280 long cases are finite tests, not all possible expressions; same small branch and Unknown evaluation order are additionally source-checked. Thirty-two private trace cases and 45 public queries agree, but process-cold, arbitrary serialization/abort/thread histories, downstream full-stack consumers, WASM, broad CI and memory sanitizers are not newly qualified. CPU campaign has 2304 measured batches across 48 groups with 12 ABBA blocks/group, plus pilots; allocation campaign is separate, 288 batches of 16 queries. Sixteen prewarm queries and cloned prebuilt rings do not measure cold scalar construction. All outcomes/certainty/stages match and signs have an independent rational shoelace/Machin-series oracle. Per-group bootstrap intervals are not multiplicity-adjusted. Runtime ratios range from about 0.946 to 1.080, not a universal slowdown/speedup claim. Memory counts are Rust requested/live/peak bytes, not RSS, stacks, native allocator overhead or a general retention bound. Unstripped benchmark executables, not representative application builds, grow 2536/2552 bytes; section deltas are observations, not a demonstrated codegen cause or general Hyper binary-size regression.',
 followup:'Preserve this first prototype and all measurements. Try an independently isolated bit-mask summary to reduce per-term bookkeeping while retaining the exact same short-circuit/trace contract; compare it against this baseline and controls before any production change. If no worthwhile implementation survives, record the idea as rejected and continue remaining high-product/ARF/generic matrix/field support and the original ecosystem inventory.'};
writeFileSync('sign-filter-experiment.json',JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'two-sign filter trial binding',files:files.length,gates:tags.length,
 sourceMaps:Object.fromEntries(Object.entries(sourceMaps).map(([v,m])=>[v,Object.keys(m).length])),
 liveFiles:Object.keys(live).length,binaries:Object.keys(binaries).length,binaryBytes:Object.values(binaries).reduce((s,b)=>s+b.bytes,0),
 status:manifest.status,checks:{tests:manifest.checks.tests.perProfile,cpu:manifest.checks.cpu,allocation:manifest.checks.allocation}}));
