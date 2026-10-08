// Record only ranges actually read; declarations do not credit their callees.
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const coverage=json('coverage.json'),inventory=json('inventory.json').sources.find(s=>s.repo==='flint');
const full=[
 ['src/mpn_extras/mulhigh.c',274,'Full implementation, including generated tuning table. Mulders split computes the high block, two recursively truncated cross products, carry propagation and two high-half diagonal corrections. Temporary 2n-limb storage, full-product and FFT-middle fallbacks, dispatch and optional one-bit normalisation are all read. A returned guard limb does not make the result the exact truncation. Positive lengths and nonaliasing fallback requirements matter; assembly/FFT callees are not credited. No tuning table adopted as a Hyper threshold.'],
 ['src/mpn_extras/mulhigh_basecase.c',481,'Full architecture tables and portable basecase, including all generated/unrolled routines. x86 ADX table through 13 with assembly through nine and C extensions, armv8 through eight, portable table through 16. Upper triangle and boundary high halves retain a guard limb; carry accounting is not a general exact-truncation proof. Declarations and dispatch entries do not credit assembly implementations. A redundant second return in one extension is harmless dead source, not an observed numerical defect.'],
 ['src/mpn_extras/mulhigh_naive.c',54,'Full portable reference: full products for one/two limbs, three-word diagonal accumulation for three and upper-triangle accumulation with boundary high-half corrections otherwise. Used by donor sibling-consistency tests; it is not an independent full-product oracle.'],
 ['src/mpn_extras/mulhigh_recursive.c',87,'Full alternate recursive high product: half splits, architecture best-size window, cross products, carry words and boundary diagonal corrections, temporary storage. Positive finite operand lengths only; no alias or unsafe boundary qualification inferred.'],
 ['src/mpn_extras/sqrhigh.c',230,'Full square implementation and generated split table. Squared high block plus doubled truncated cross product and doubled boundary correction; temporary storage, full-square/FFT fallback and optional one-bit normalisation. Guard precision and error accounting, not exact field arithmetic. Architecture-tuned cutoffs remain donor-specific.'],
 ['src/mpn_extras/sqrhigh_basecase.c',88,'Full architecture tables and portable one/two-limb implementations. Larger portable basecase delegates to header helper. Tables and external declarations are not source coverage of assembly bodies.'],
 ['src/mpn_extras/test/t-mulhigh_normalised.c',90,'Full test: random normalised inputs at lengths 1..64; checks high bit and exact equality to the unnormalised sibling result or its one-bit shift. Useful consistency, but no independent product bound or exhaustive shift/carry coverage.'],
 ['src/mpn_extras/test/t-mulhigh_n.c',159,'Full test: basecase sizes compared to naive sibling; larger sizes compared to full FLINT multiplication with nonpositive error and a 2n-ulp guard-limb lower bound. Larger lengths and full-product fallback sampled infrequently. The 2n test bound is weaker than the documented n+2 bound for n>2. Code read, not a newly executed donor suite.'],
 ['src/mpn_extras/test/t-mulhigh_n_recursive.c',61,'Full test: random lengths 1..100 compare every output/guard limb with the naive sibling. Does not independently establish the documented error bound or alternate architecture correctness.'],
 ['src/mpn_extras/test/t-mulhigh_n_tab.c',61,'Full table test: random supported table lengths compare all output/guard limbs to the naive sibling. No assembly body coverage or independent full-product bound follows from registration.'],
 ['src/mpn_extras/test/t-sqrhigh_normalised.c',93,'Full normalised-square test: lengths 1..64, top-bit-set inputs; verifies output normalisation and equality to shifted/unshifted sibling. No independent exact-square oracle.'],
 ['src/mpn_extras/test/t-sqrhigh.c',150,'Full square test: basecase lengths compared to naive multiplication of equal inputs; larger sizes use full FLINT square and a 2n-ulp guard-limb lower bound. Random larger/fallback sampling and sibling checks do not certify all dispatches or documented n+2 accuracy.']
];
const entries=full.map(([path,noteLines,note])=>({repo:'flint',path,ranges:[[1,noteLines]],note}));
entries.push(
 {repo:'flint',path:'src/mpn_extras.h',ranges:[[610,680],[980,1180]],note:'Architecture widths, native basecase restrictions, tuning cutoffs, high/low/middle declarations, public dispatch and normalised wrappers read. Public small table paths explicitly permit aliases, but fallback contracts differ; do not assume a blanket alias guarantee. Scratch-capable high/low wrappers still allocate in some Mulders paths (source TODO). x86 square restriction comments differ from the documentation; this pass makes no invalid-size experiment. Unread header ranges, assembly and generic/FFT callees remain open.'},
 {repo:'flint',path:'doc/source/mpn_extras.rst',ranges:[[110,205]],note:'Subtraction/Toom lead-in and high/low-product contracts read. Rough, precise and exact high products differ because omitted low products can carry. Precise high product returns n high limbs plus a guard, with stated error at most n+2 guard ulps and no overestimate. Exact truncation may inspect/correct the low part; fast approximate output alone cannot replace a certified Hyper enclosure. Recursive rough terminology and native basecase restrictions need reconciliation against implementation, not assumed as newly proved contracts.'}
);
for(const e of entries){
 assert(!coverage.some(x=>x.repo===e.repo&&x.path===e.path),e.path);
 const f=inventory.files.find(x=>x.path===e.path);assert(f?.text,e.path);
 assert.equal(sha(resolve('../../../../exact-real-references/flint',e.path)),f.sha256,e.path);
 for(const[a,b]of e.ranges)assert(a>=1&&b>=a&&b<=f.lines,e.path);
 if(full.some(x=>x[0]===e.path))assert.equal(e.ranges[0][1],f.lines,e.path);
}
assert.equal(entries.reduce((n,e)=>n+e.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),2196);
console.log('*** Begin Patch\n*** Update File: '+resolve('coverage.json')+'\n@@\n [\n'+entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')+
 '\n*** Add File: '+resolve('high-product-read-selection.json')+'\n'+JSON.stringify(entries.map(e=>({repo:e.repo,path:e.path,newRanges:e.ranges})),null,2).split('\n').map(l=>'+'+l).join('\n')+'\n*** End Patch');
