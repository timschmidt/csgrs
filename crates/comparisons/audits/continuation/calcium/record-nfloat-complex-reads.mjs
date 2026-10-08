import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha} from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url));
const coverage=JSON.parse(readFileSync(resolve(here,'coverage.json')));
const inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'))).sources.find(s=>s.repo==='flint');
const specs=[
 ['src/nfloat/complex.c',2068,'Full packed-pair complex implementation and context/method registration. ACB import discards radii and export encloses only the encoded approximate midpoint. Ordinary approximate, not directed complex arithmetic. Scalar-axis shortcuts; 1..4-limb specialised square/product with guard limbs; standard four-product below 12 limbs, aligned three-product Karatsuba at 12+; square Karatsuba at 20+; exponent-gap fallbacks. Reciprocal/division use norm and conjugation, principal sqrt/rsqrt select cancellation-avoiding formulas by real-part sign. Absolute comparison uses exponent separation, padded double norm filter, then exact ARF squares and rounded sum sign. Known finite packed values make that sign comparison a narrower problem than arbitrary computable-real equality. Methods declare approximate/nonexact algebra rather than exact field laws; many transcendental/mixed operations use generic defaults. Whole aliases need numerical qualification; no arbitrary overlapping storage, nonfinite or exponent-limit assumption. Hyperlattice already has reuse-sensitive exact-rational three-product dispatch, cold fused kernels, shared inverse and checked Unknown-zero gates. No precision threshold or midpoint-as-enclosure transfer.'],
 ['src/nfloat/profile/p-complex_mat_mul.c',422,'Whole complex matrix profiler. Rational-derived moderate random components, fixed 100ms sequential timers, classical/fixed/block threshold searches, rectangular capability not profiled (square cases), active public-versus-classical sweep stops after slow baseline. Data-dependent success and timing only; no exact-error, paired order, allocation, peak memory or binary qualification. Reuses output buffer but does not establish equal numerical work. No Hyper cutoff adopted.'],
 ['src/nfloat/profile/p-vs_acf.c',174,'Whole ACF/nfloat complex vector profiler. 64..4096 bits and lengths 10/100, same seeded construction recipe but backend-rounded inputs, sequential 100ms add/mul/square/scalar/addmul/sum/product/dot timing. Repeated addmul mutates prior vector output and accumulates over timer repetitions; iteration counts and values can differ between backends. No accuracy verification or state-equated benchmark. Context cleanup omitted, but source observation alone does not establish allocated context leakage.'],
 ['src/nfloat/profile/p-vs_arf.c',160,'Whole ARF/nfloat real vector profiler. Same sequential timing and mutating addmul qualifications as complex comparator, positive rational-derived magnitudes with random signs and pi scalar. Fixed precision is not equal exact-real demand; no independent numerical oracle, randomized paired order or storage/size measurements.'],
 ['src/nfloat/test/t-complex_mat_mul.c',76,'Whole approximate complex matrix test. Weighted 1..4-limb versus all-limb random precision, max-norm tolerance 2^(-prec+2), reorder/fixed/block/public paths, low-precision-heavy repetition counts and public dimension up to 120. Fixed routine tested twice at different size budgets. Reference/helper internals must be read separately; registration does not prove each sampled path succeeded or arbitrary alias correctness.'],
 ['src/nfloat/test/t-nfloat_complex.c',76,'Whole scalar complex test. All word precisions, ACB reference at prec+64, tolerance 2^(-prec+3), comparisons/arithmetic/conjugate/re/im/root operations and dots at tolerance 2^(-prec+4). Low precisions get 10000 repetitions, higher scalar precisions only one, higher dots 100. Extra tol1 allocated but unused. Generic helpers determine status and alias semantics; approximate closeness is not exact enclosure or arbitrary equality completeness.']
];
const entries=specs.map(([path,lines,note])=>{
 assert(!coverage.some(e=>e.repo==='flint'&&e.path===path),path);
 const f=inventory.files.find(f=>f.path===path);assert.equal(f.lines,lines);
 assert.equal(sha(resolve(here,'../../../../exact-real-references/flint',path)),f.sha256);
 return {repo:'flint',path,ranges:[[1,lines]],note};
});
console.log('*** Begin Patch\n*** Update File: '+here+'/coverage.json\n@@\n [\n'+
 entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')+'\n*** Add File: '+here+'/nfloat-complex-read-selection.json\n'+
 JSON.stringify(entries.map(e=>({repo:e.repo,path:e.path,newRanges:e.ranges})),null,2).split('\n').map(s=>'+'+s).join('\n')+'\n*** End Patch');
