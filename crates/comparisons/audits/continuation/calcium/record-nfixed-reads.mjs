import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha} from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url));
const coverage=JSON.parse(readFileSync(resolve(here,'coverage.json')));
const inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'))).sources.find(s=>s.repo==='flint');
const specs=[
 ['src/nfloat/nfixed.c',1626,'Full fixed-point implementation. Sign/magnitude vector add/sub without overflow handling; specialised 2..4-limb signed high-product accumulators, 5..8-limb positive/negative sums, generic high products, classical multiplication and extra-limb comparison helper, Waksman row/column corrections, Strassen windows/two temporary matrices/odd borders, precision/parity cutoffs, corresponding intermediate and ULP bound routines, complex three-product bound. Duplicate dispatch and bound paths differ in automatic cutoff parity, and S4=A22-A21+A12-A11 is bounded as 3A although it can equal 4A. Bounded exact controls validate one intermediate-bound discrepancy without approaching overflow; 24 repeated configurations are not independent bugs. 52 cutoff-bound discrepancies are algorithm/bound inconsistencies, not observed incorrect final products. Partial-product carry text and positive-length raw-dot assumptions remain qualifications; no overflow, zero-length raw-dot, assertion or invalid-alias reproduction.'],
 ['src/nfloat/test/t-nfixed_dot.c',159,'Whole test. Lengths 1..10, 2..8 limbs, random sign/limbs scaled down ten top bits. Reference sums use the same approximate high-product routine, with a (2*nlimbs-1)*len ULP tolerance, not an independent exact product. No negative/noncontiguous stride cases or empty-dot contract established.'],
 ['src/nfloat/test/t-nfixed_mat_mul.c',111,'Whole automatic-matrix test. Dimensions 1..20 and 2..12 limbs, top padding selected by the same bound helper under test, compared against an extra-limb approximate classical route plus heuristic 1.01 ULP allowance. Automatic Strassen thresholds above tested dimensions are not reached.'],
 ['src/nfloat/test/t-nfixed_mat_mul_classical.c',111,'Whole classical test. Dimensions 1..20, 2..12 limbs, input scale from its own bound helper, sign/magnitude difference against extra-limb approximate classical output. Neither exact independent oracle nor observed intermediate-range validation.'],
 ['src/nfloat/test/t-nfixed_mat_mul_strassen.c',114,'Whole Strassen test. Explicit random cutoffs 0..5, dimensions 1..20, padding from the same bound helper; despite the classical_precise_error variable, reference call is ordinary classical multiplication. Automatic negative-cutoff parity mismatch and true largest intermediate are not checked.'],
 ['src/nfloat/test/t-nfixed_mat_mul_waksman.c',111,'Whole Waksman test. Dimensions 1..20, 2..12 limbs, own-helper padding, extra-limb approximate classical comparison and tolerant integer difference. Tests final differences, not all intermediate bounds.'],
 ['src/nfloat/profile/p-nfixed_mat_mul.c',268,'Whole fixed-point profiler. Fixed ten-bit top scaling, 100ms CPU timing, sequential classical/Waksman comparisons, even/odd Strassen threshold tuning requiring two successive wins and active fixed threshold profile. No paired order randomisation, independent correctness gate, allocation/size evidence or fixed accuracy matching; thresholds are workload/platform calibration, not transferable Hyper constants.'],
 ['src/nfloat/test/t-mat_mul.c',120,'Whole real matrix test. Approximate max-norm/positive-entry comparisons for fixed/block/public routes, small low-precision dimensions and reduced high-precision counts; rare public dimensions up to 120. Status-aware approximation tests differ from exact entrywise oracle checks.'],
 ['src/nfloat/test/t-add_sub_n.c',96,'Whole low-level add/sub test. One-to-four-limb specialised routes compared with generic routes using identical default rounding, random exponent gaps and extra near-gap/all-ones cases. Proves matching implementation behavior for samples, not independent exact enclosure.'],
 ['src/nfloat/test/t-addmul_submul.c',91,'Whole vector multiply-accumulate test. All word precisions with short lengths 0..3, low-precision-heavy sample budget; specialised routines compared with scalar/generic routines sharing arithmetic and exact structural output equality. No directed or independent numerical oracle qualification.'],
 ['src/nfloat/test/main.c',47,'Whole test translation-unit inventory: twelve included and registered functions, including real/complex arithmetic, specialised vectors, fixed-point and matrix tests. Reading registration does not credit unread included complex tests.'],
 ['src/nfloat/inlines.c',14,'Whole inline-definition translation unit: defines NFLOAT_INLINES_C and includes the previously read public header. No additional algorithm.'],
 ['src/nfloat/profile/p-mat_mul.c',418,'Whole real matrix profiler. Rational-derived random approximate inputs, sequential 100ms classical/fixed/block comparisons and cutoff searches, explicit matrix dimension/precision sweeps and active public-versus-classical profile. No correctness/accuracy equality, randomized paired order, memory or binary qualification. Native precision-dependent dispatch is relevant but its raw timings cannot establish Hyper exact-work gains.']
];
const entries=specs.map(([path,lines,note])=>{
 assert(!coverage.some(e=>e.repo==='flint'&&e.path===path),path);
 const f=inventory.files.find(f=>f.path===path);assert.equal(f.lines,lines);
 assert.equal(sha(resolve(here,'../../../../exact-real-references/flint',path)),f.sha256);
 return {repo:'flint',path,ranges:[[1,lines]],note};
});
console.log('*** Begin Patch\n*** Update File: '+here+'/coverage.json\n@@\n [\n'+
 entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')+'\n*** Add File: '+here+'/nfixed-read-selection.json\n'+
 JSON.stringify(entries.map(e=>({repo:e.repo,path:e.path,newRanges:e.ranges})),null,2).split('\n').map(s=>'+'+s).join('\n')+'\n*** End Patch');
