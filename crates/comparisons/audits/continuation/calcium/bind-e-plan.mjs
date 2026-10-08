import {writeFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-sources.mjs';
import {checkEPlan} from './check-e-plan.mjs';
import {checkEPlanRegressions} from './check-e-plan-regressions.mjs';
const sourceMap=sources(),previous=json('mag-series-experiment.json'),frozen=json('e-plan-binaries.json');
assert.deepEqual(frozen.sourceMap,sourceMap);const files={...sourceMap.files};
for(const p of ['e-plan-binaries.json','check-e-plan.mjs','check-e-plan-regressions.mjs','run-e-plan-checks.mjs',
 'run-e-plan-regressions.mjs','bind-e-plan.mjs','verify-e-plan.mjs','mag-series-experiment.json','capture.mjs',
 'results/e-plan-check.initial-build.rs','e-plan-cpu-summary.json','e-plan-allocation-summary.json',
 'results/e-plan-cpu.jsonl','results/e-plan-allocation.jsonl'])files[p]=sha(p);
const tags=['build-baseline','build-baseline-fixed','build-candidate','freeze',
 ...['baseline','candidate'].flatMap(v=>['plans','numeric','state'].map(k=>'native-'+v+'-'+k)),
 ...['baseline','candidate'].flatMap(v=>['plans','numeric'].map(k=>'memcheck-'+v+'-'+k)),
 'cpu','allocation','output-check',...['baseline','candidate'].flatMap(v=>['debug','release'].map(p=>'tests-'+v+'-'+p))];
const gates=tags.map(t=>{
 const tag='e-plan-'+t,g=json('results/'+tag+'.json');assert.equal(g.code,t==='build-baseline'?101:0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr']){const p='results/'+tag+'.'+ext;files[p]=sha(p);}return{tag,code:g.code,signal:g.signal};
});assert.equal(gates.length,21);
const binaryBytes=Object.values(frozen.binaries).reduce((n,b)=>{
 assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);return n+b.bytes;
},0);assert.equal(binaryBytes,11204552);
const m={schema:1,recorded:new Date().toISOString(),files,gates,sourceMap,binaries:frozen.binaries,binaryBytes,
 newDonorLines:0,readRecords:[],coverageAtBinding:previous.coverageAtBinding,
 hyperReadRanges:{'hyperreal/AGENTS.md':[[1,14]],'hyperreal/.github/workflows/ci.yml':[[1,50]],'hyperreal/Cargo.toml':[[1,84]],
  'hyperreal/src/computable/approximation/constants.rs':[[1,225]],'hyperreal/src/computable/approximation.rs':[[1,62]],
  'hyperreal/src/computable/node/tests.rs':[[1540,1710]],'hyperreal/src/computable/node/primitive_constructors.rs':[[275,460]],
  'hyperreal/src/computable/node/structural_analysis.rs':[[680,793]],'hyperreal/src/computable/node/approximation_queries.rs':[[60,123]],
  'hyperreal/src/computable/node/algebra.rs':[[1906,1926]]},
 checks:checkEPlan(),regressions:checkEPlanRegressions(),
 status:'PROGRESS. Planning-only e candidate isolated and promising, not selected for production. No new donor-source credit or ecosystem completion.',
 production:'No production/donor edit, fifth retained transfer, cleanup, deletion, commit, push or external report. All955 live hashes still match the retained derivative checkpoint. Candidate copies only180 Hyperreal files (5312930 bytes); six frozen executables total11204552 bytes in /tmp/calcium-e-plan.mOIsBs; existing build cache reused.',
 findings:'A normalized lower factorial mantissa preserves the stopping certificate while eliminating growing BigInt term planning. Exact word multiplication and downward truncation cannot stop early; a proof bounds operations/termination over the pre-existing supported precision arithmetic. Exact GMP and independent BigInt agree on4151 requests per variant with no extra sampled terms; every20367 candidate-loop lower invariant steps is checked against GMP. Directed MPFR and independent rational BigInt enclosures validate300 kernel/public/state outputs per variant. Source body, binary splitting, rounding and shared cache are unchanged outside the eight-net-line planner edit. First build failed only in audit Float conversion/unused import; failed source and gate are preserved and corrected before the successful builds.',
 comparison:'CPU71 groups/3408 observations: paired median ratios0.003568..1.084187,37 per-group intervals below one,3 above,31 overlap. All10 planner and10 direct-kernel intervals improve, but cached and unchanged pi controls have mixed timings. Fresh public65536-bit median3.4048715ms to1.2938615ms; allocation requests11558 to4643, requested bytes32693112 to1628872, peak103248 bytes unchanged. Allocation requests/bytes fall in28 groups, unchanged43, none rise; live unchanged71; peak falls7, unchanged64. Six audit binaries include instrumentation/oracles: CPU/allocation files shrink1304/1472 bytes, check grows944; not representative application size.',
 limits:'Bounded precision corpus, sampled histories and native x86-64 only. Numerical oracle is not a finite-prefix-only comparison: exact BigInt series includes a complete geometric remainder. Thread checks have no focused thread Memcheck in this checkpoint. Four sequential Memchecks have zero errors/lost/suppressed blocks but544 runtime bytes or35656 runtime/cache/MPFR bytes remain reachable. Cumulative Memcheck allocations include setup/oracles and are not peak/RSS measures. All-feature855-test membership matches in both profiles; default-only, downstream, persistent in-crate regressions, Clippy/fuzz/WASM/other-target and representative size qualification remain. Candidate uses u128 and may need target-specific cost qualification. Tiny single-query/cached timings and unchanged pi controls show noise/layout sensitivity; per-group intervals are not multiplicity-adjusted. No universal speed/size gain, arbitrary precision/history proof, i32::MIN repair, new completeness gain or donor backend adoption. All prior failures and unfinished requirements remain.',
 followup:'Continue qualification of the isolated e planner: add durable regressions, check default-feature/downstream/target behavior and representative sizes before retention. Resolve measured control tradeoffs without discarding unfavorable evidence. Continue unread ARF/scalar/MPFR high helpers, generic field/matrix/algebraic and every original reference; reconcile full inventory and older open transfers.'};
writeFileSync('e-plan-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:Object.keys(files).length,gates:gates.length,binaryBytes,readLines:0,status:m.status}));
