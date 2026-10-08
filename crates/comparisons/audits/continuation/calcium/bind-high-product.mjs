import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkHighProductControls} from './check-high-product-controls.mjs';
const specs=[['high-product-compile',0],['high-product-native',1],['high-product-memcheck',1],
 ['high-product-linked-libraries',0],['high-product-bigint-oracle',0],['high-product-output-check',0],['high-product-inventory',0]];
const files=['flint-high-product-controls.c','high-product-bigint-oracle.mjs','check-high-product-controls.mjs',
 'record-high-product-qualification-reads.mjs','high-product-qualification-read-selection.json',
 'bind-high-product.mjs','verify-high-product.mjs','sign-filter-mask-experiment.json','capture.mjs'];
for(const[tag,code]of specs){const g=json('results/'+tag+'.json');assert.equal(g.code,code,tag);assert.equal(g.signal,null);
 for(const e of ['json','stdout','stderr'])files.push('results/'+tag+'.'+e);}
const live=json('sign-filter-mask-experiment.json').liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,'live '+p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('high-product-qualification-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const paths=[...readFileSync('results/high-product-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(paths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,json('nfloat-complex-experiment.json').libraries);
const binaryPath='/tmp/calcium-high-product.03dkHN/high-product-controls';
const config=resolve('../../../../exact-real-references/flint/src/flint-config.h');
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:specs.map(([tag,code])=>({tag,code,signal:null})),liveSources:live,libraries,configurationFiles:{[config]:sha(config)},
 binary:{path:binaryPath,bytes:statSync(binaryPath).size,sha256:sha(binaryPath)},readRecords,coverageAtBinding:effectiveSummary(),
 initialBinding:{path:'high-product-initial-binding.json',sha256:sha('high-product-initial-binding.json'),
  correction:'Before verification, correct the agent-instruction file range from 1..18 to its actual 1..14 lines. Original draft preserved; numerical evidence and donor/source coverage are unchanged.'},
 hyperReadRanges:{'hyperreal/AGENTS.md':[[1,14]],'hyperreal/src/computable/approximation/arithmetic_kernels.rs':[[1,264]],
  'hyperreal/src/computable/node/scale.rs':[[1,20]],'hyperreal/src/computable/node/tests.rs':[[2035,2105],[2420,2495]]},
 checks:checkHighProductControls(),
 production:'No new production or donor edits. All 955 retained live source/support hashes match checkpoint 32; four prior continuation transfers remain retained.',
 status:'Independent GMP checks expose guard-bound discrepancies in a valid finite corpus. Twelve public-square outputs exceed the documented n+2 integer guard-ulp bound, and twelve internal naive/recursive multiplication outputs exceed the same test bound. Twenty additional rows fail only the stronger full-residual interpretation. All 44 are reconstructed independently with JavaScript BigInt. No raw high-product transfer selected.',
 limits:'53,248 outputs from 9,216 input cases, 144 positive lengths 1..2049, 32 deterministic patterns and two normalisation input forms; seven routes with internal naive/recursive limited to 128 limbs and normalised routes restricted to top-bit-set inputs. Disjoint storage and seeded outputs only. No invalid lengths, alias overlaps, extreme indices, 32-bit, arm, FFT-small, concurrency or higher-level nfloat accuracy qualification. The generated build configuration is 64-bit ADX with FFT-small disabled. GMP full multiplication is independent of FLINT truncated kernels; JavaScript reconstructs 3059 selected output rows including every recorded discrepancy, not the entire GMP corpus. The 24/44 counts repeat routes and input forms, not independent defects. All outputs are one-sided and satisfy the looser scaled 2n bound in this corpus; 832 public full-fallback controls are exact. Native/Memcheck numerical stdout is identical. Memory reports zero errors/live blocks and all 628419 allocations freed; 891786227 cumulative bytes include the oracle and are not peak RSS or donor-only cost. Both numerical runs exit 1 and remain failed gates. No speed, allocation or binary improvement claimed: no Hyper candidate was implemented or benchmarked. Seven complete files and two header extensions add 1708 source lines; called hardcoded/normalised/arm assembly, rounding helpers and the full ecosystem remain open.',
 finding:'The triangular high-product algorithm omits low halves on diagonal n-2 plus lower full partial products. Their exact sum explains the observed deficits and can exceed n+2 guard ulps; the script checks the formula and a conservative bound for its selected cases. Public square examples have integer deficits 30 versus 25 at n=23 and 124 versus 89 at n=87. This is a numerical contract discrepancy, not an observed incorrect Hyper or nfloat result. The 20 additional fractional-only rows depend on interpreting the stated ulp bound relative to the full product rather than its integer truncation.',
 followup:'Continue hardcoded/normalised/arm high-product support and ARF rounding/temp-storage contracts, with independent rounded-product controls where relevant. Examine demand-sized product and bounded scratch-reuse ideas only with a certified error/carry design and matched Hyper workloads. Finish remaining generic matrix/field support and every original reference; no full-inventory completion claim.'};
writeFileSync('high-product-experiment.json',JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'independent high-product qualification binding',files:files.length,gates:specs.length,
 liveFiles:Object.keys(live).length,readRecords:readRecords.length,readLines:1708,coverage:manifest.coverageAtBinding,
 binaryBytes:manifest.binary.bytes,summary:manifest.checks.summary,status:manifest.status}));
