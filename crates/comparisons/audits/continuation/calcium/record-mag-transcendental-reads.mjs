import {resolve} from 'node:path';
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const notes={
 'src/mag/atan.c':'Full upper/lower atan: reciprocal reduction, top limbs of external Arb atan table, residual degree-five correction and binary64 padding; tiny and large argument bounds. Native bounded transitions and aliases pass directed MPFR. The external table implementation/source is a separate dependency, not credited by reading this reference.',
 'src/mag/bernoulli_div_fac_ui.c':'Full 16-entry even-index table and odd-index cases; large even n uses a padded inverse-two-pi power intended to absorb zeta(n). All constants read, but no new independent Bernoulli qualification; do not infer a proof merely from the omitted zeta factor or the comment about slack.',
 'src/mag/bin_uiui.c':'Full zero/symmetry/tiny-k handling, n<256 factorial-table route, and larger entropy-style binpow upper bounds with exp(k) alternative. Source-only this pass; no complete integer-domain or exact-binomial oracle campaign.',
 'src/mag/binpow_uiui.c':'Full upper (1+1/m)^n estimate: undefined m=0 maps to infinity, n<m uses exp(n/m)<=1+x+x^2, otherwise upper base and power. Source-only; no unbounded word-exponent cost claim.',
 'src/mag/const_pi.c':'Full adjacent 30-bit pi bounds. Both constants independently compared to directed MPFR at512/768 bits; no arbitrary-precision pi algorithm transfer.',
 'src/mag/cosh.c':'Full upper/lower exp/expinv composition and special cases. Matching addition direction and exact division by two preserve polarity. Finite transitions, seeded output and whole alias controls pass; promoted/infinite cases remain source-only.',
 'src/mag/d_log.c':'All 32 log entries and 48 reciprocal entries read, together with near-one series, binary reduction, residual polynomial and sign-dependent padding. The source claims a conservative ten-ulp error and all-FENV/x87 padding by inspection; the new 680-positive-double corpus per precision qualifies default native/Memcheck mode only, not that universal claim.',
 'src/mag/exp.c':'All inverse-factorial entries and upper/lower tiny/moderate/huge branches read. Moderate reduction perturbs ln2 oppositely before subtracting and pads the polynomial. The raw helper has a bounded-input precondition and is never called directly. Finite exponents through26 exercise the public huge path; arbitrary fmpz exponent and saturation paths remain source-only. Coarse large-input bounds are not tight-quality promises.',
 'src/mag/exp_tail.c':'Full N=0/special handling, N>=2x geometric upper estimate 2*x^N/N!, else full exponential. Independent positive-series oracle sums N..N+256 and bounds every remaining term using x/(N+258); x<=4 and N<=257 only. This improves qualification over a finite lower sum, without changing the donor or importing a new Hyper representation.',
 'src/mag/expinv.c':'Full exp(-x) pair with reversed huge-exp/inversion polarity and tiny-argument shortcuts. Directed MPFR references negate the exact input and evaluate exp, with default-mode finite public aliases. No extreme promoted-exponent or nonfinite closure claim.',
 'src/mag/expm1.c':'Full cancellation-avoiding tiny bounds and moderate exp-minus-one mantissa adjustment; large inputs keep an exp bound. Finite transitions at negative16, negative15 and exponent6 are qualified, with quality deliberately not asserted for very large exponentials.',
 'src/mag/fac_ui.c':'Read every one of the256 factorial and256 reciprocal-factorial table pairs, then both Stirling/log/power-of-two ceiling fallbacks. All579 lines are read; table-generation proofs and independent combinatorial numerical checks remain open. The later branches are deliberately coarse upper bounds, not exact factorial representations.',
 'src/mag/geom_series.c':'Full upper geometric tail x^n/(1-x), opposite lower bound on the denominator, zero and divergence conventions. Source-only in this checkpoint; no finite oracle run or all-n completeness claim.',
 'src/mag/get_d.c':'Full conversion: ordinary exact30-bit dyadic scale, conservative tiny saturation at2^-1000 and large infinity. This is an upper bound, not nearest conversion. Used indirectly by moderate hyperbolic code, but no new comprehensive conversion campaign.',
 'src/mag/get_d_log2_approx.c':'Full explicitly approximate planner log2: exponent fallback outside a small exponent window, special/promoted clamps. It is not a certified bound and is excluded from bound-oracle assertions and any proposed proof-level transfer.',
 'src/mag/get_fmpq.c':'Full shallow ARF view to rational conversion. Only bounded magnitudes are appropriate; the remaining ARF conversion implementation remains partially read. No giant rational expansion or direct conversion qualification in this pass.',
 'src/mag/get_fmpz.c':'Full ceil/floor integer adapters through shallow ARF view. Moderate public exp-huge calls exercise bounded values indirectly, not every integer conversion or unbounded storage case.',
 'src/mag/hurwitz_zeta_uiui.c':'Full s<=1/a=0 infinity and upper first-term-plus-integral estimate for positive integer parameters. Source-only; no new independent zeta corpus.',
 'src/mag/inlines.c':'Full header-inline emission unit, macro and include. All mag.h implementation lines were credited in previous checkpoints; rereading them adds no duplicate credit.',
 'src/mag/io.c':'Full print, dump/load string/file wrappers and ARF bridge validation read. Early file-output failure cleanup and parser validation deserve separate review; no malformed-input, write-failure, memory-safety reproduction or donor patch attempted. Valid IO roundtrips are also not newly qualified here.',
 'src/mag/log.c':'Full positive-log/negative-log upper/lower pairs: clamping at1, ordinary double path, exp1000/negative970 scaled paths, promoted exponent ln2 bounds and reciprocal fallback. Direct MPFR uses sign-reversed rounding for negative-log. Finite crossover results pass; promoted and infinite paths remain source-only.',
 'src/mag/log1p.c':'Full tiny x upper bound, inflated1+x double path and large scale decomposition. Source-only promoted fallback. The new corpus checks transitions at exponentnegative10 and1000 with direct MPFR log1p, not a rounded1+x reference that loses tiny input.',
 'src/mag/log_ui.c':'Full integer wrapper: zero infinity, one zero, otherwise log1p of upper n-1. Source-only; its semantic zero convention is not that of mag_log(max(1,x)).',
 'src/mag/polylog_tail.c':'Full x^N log(N)^d/N^sigma initial bound, ratio inflation for negative sigma and log powers, and infinity when the ratio cannot be bounded below1. Floor of a lower NlogN controls the logarithmic ratio. Bounded nonnegative parameters and finite integer conversion are prerequisites for any future tests; no risky extreme conversion or new numerical qualification.',
 'src/mag/randtest.c':'Full normalized random and special magnitude generators; zero/inf selection and random fmpz exponents. Source-only. Reading donor random tests does not mean executing them or matching their full generated domain.',
 'src/mag/root.c':'Full n=0/1/2/4 cases and scaled exp(log1p(x)/n) upper construction. The added1 trades tightness for a simple conservative bound. Eight positive degrees through31 pass exact-input directed MPFR and supplemental quality at512/768 bits. Hyper already has demand-sized integer power enclosures, so no replacement selected.',
 'src/mag/set_d.c':'Full absolute binary64 upper/lower conversion, special conventions, normalization and padded macros. Moderate conversion paths are exercised through transcendental outputs; direct subnormal/large/NaN conversion qualification is not claimed.',
 'src/mag/set_d_2exp_fmpz.c':'Full scaled double constructors, sign/special conventions and inline-versus-general exponent updates. Source-only; no promoted-exponent state or extreme raw boundary calls.',
 'src/mag/sinh.c':'All polynomial coefficients and triple-angle reconstruction read. Tiny upper uses expm1; lower uses x. Large lower uses padded exp_lower, upper expinv and upper subtraction, so bound polarity cannot be judged by subtraction name alone. Sampled transitions and large public inputs pass direct MPFR; compensating slack is not a universal proof.',
 'src/mag/impl.h':'Complete21-line private declarations for the exp helper and huge upper/lower routes. Declarations do not relax the helper input precondition.',
 'src/double_extras.h':'Complete108-line header: ordinary Horner polynomial, floating constants, signed/random declarations, NaN predicate, and three binary-scaling helpers. Normal-range helpers carry explicit exponent/nonzero preconditions; only valid uses through public magnitude functions run. External libm, d_log2 and other declared bodies are not credited by this read.'
};
const tests={
 exp:'Upper/lower direct MPFR through ARF, random scaled magnitudes, relative quality widened for large x and in-place agreement.',
 expinv:'Both inverse-exponential directions and negative argument MPFR; broad upper quality and conditional lower tolerance, whole aliases.',
 expm1:'Direct expm1 MPFR reference, tiny/moderate scaling, conditional relative quality and whole alias.',
 log:'Both directions, clamping and large fmpz exponent reconstruction with ln2; nontrivial shared ARF/MPFR support.',
 neg_log:'Both directions and sign-reversed logarithm, clamping and arbitrary exponent decomposition; rounding names must be interpreted with ARF signed semantics.',
 log1p:'Direct log1p oracle avoids loss of tiny inputs in1+x, relative tolerance and whole alias.',
 atan:'Upper/lower MPFR atan, normalized bit checks, broad relative envelope and in-place agreement.',
 sinh:'Upper/lower MPFR sinh, scaled random magnitudes, relative checks and in-place agreement.',
 cosh:'Upper/lower MPFR cosh, scaled random magnitudes, relative checks and in-place agreement.',
 root:'Degrees0..29 and arbitrary exponent magnitudes, ARF root comparison and whole aliases; new finite positive-degree corpus does not equal this random domain.',
 d_log_lower_bound:'Complete custom random double generator, special-input cases,53-bit MPFR downward reference and relative error checks. NaN comparisons are not positive finite proof obligations.',
 d_log_upper_bound:'Complete duplicated random generator,53-bit upward MPFR then binary64 conversion, special cases and relative check. New qualification compares512/768-bit endpoints directly.',
 exp_tail:'Complete signed exponentiation helpers and donor test: only50 positive terms summed downward, so it checks a lower finite sum, not enclosure of the full infinite tail. New independent oracle adds a rigorous remainder bound.'
};
for(const[n,note]of Object.entries(tests))notes['src/mag/test/t-'+n+'.c']='Full donor test read. '+note+' Donor random suite not newly executed; new deterministic controls have their own explicit corpus.';
const prior=[...json('coverage.json'),...json('coverage-extensions.json')];
const inv=json('inventory.json').sources.find(s=>s.repo==='flint');
const records=Object.entries(notes).map(([path,note])=>{
 const f=inv.files.find(f=>f.path===path);assert(f?.text);assert(!prior.some(r=>r.repo==='flint'&&r.path===path),path);
 const absolute=resolve('../../../../exact-real-references/flint',path);assert.equal(sha(absolute),f.sha256,path);
 const s=readFileSync(absolute,'utf8'),lines=s.split('\n').length-Number(s.endsWith('\n'));assert.equal(lines,f.lines,path);
 return{repo:'flint',path,ranges:[[1,lines]],note};
});
assert.equal(records.length,44);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),4660);
const top=inv.files.filter(f=>/^src\/mag\/[^/]+\.c$/.test(f.path));
for(const f of top)assert([...prior,...records].some(r=>r.repo==='flint'&&r.path===f.path&&r.ranges.length===1&&r.ranges[0][0]===1&&r.ranges[0][1]===f.lines),f.path);
console.log('*** Begin Patch\n*** Update File: '+resolve('coverage.json')+'\n@@\n [\n'+records.map(r=>'+'+JSON.stringify(r)+',').join('\n')+
 '\n*** Add File: '+resolve('mag-transcendental-read-selection.json')+'\n'+JSON.stringify(records.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})),null,2).split('\n').map(s=>'+'+s).join('\n')+'\n*** End Patch');
