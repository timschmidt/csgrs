import {resolve} from 'node:path';
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const tests={
 'main.c':'All50 test includes and50 test registrations plus test-driver macro read. No Bernoulli-specific test registration appears. Completing this driver does not imply executing its random suite or qualifying every declared function.',
 't-add_2exp_fmpz.c':'Promoted exponent generation, positive ARF input,40-bit downward reference and relative upper allowance. Bound conversion and shared ARF backend can hide a small direction error; prior independent finite rational controls remain distinct.',
 't-bin_uiui.c':'Random n,k<10000 and exact fmpz binomial comparison, arbitrary prior output state and normalized bits. New native/JS recurrence checks use all k through n+1 for selected n, not the same random domain.',
 't-binpow_uiui.c':'Complete signed/unsigned binary-exponentiation helpers and128-bit rounded (1+1/m)^n reference. Native/JS new controls instead construct the entire rational from integer powers for bounded m,n.',
 't-cmp.c':'Random special/promoted magnitudes compared with exact ARF order. This is order of represented bounds, not decidability of arbitrary real equality.',
 't-cmp_2exp_si.c':'Magnitude-to-power-of-two comparison checked against ARF, including host-word exponent generation. New direct raw extreme-word tests are not run.',
 't-dump_file.c':'Valid self-generated single and sequential file roundtrips, tmpfile/flush/rewind/close and Windows exclusions. Only valid string roundtrips run in the new corpus; file-error and malformed-input paths are not reproduced.',
 't-dump_str.c':'Allocation/free smoke loop and valid self-generated string roundtrip equality with seeded output. New bounded finite roundtrips require exact value preservation, not merely absence of a crash.',
 't-fac_ui.c':'n<2000 factorial upper-bound comparison against fmpz factorial via ARF. New exact GMP recurrence and independent JS sweep0..4096 include every table entry and selected fallback range.',
 't-fast_add_2exp_si.c':'Inline exponent assumptions, exact ARF reference, upper relative allowance and normalized bits. Fast API contracts remain narrower than general magnitude arithmetic.',
 't-fast_addmul.c':'Full separate and three factor/output alias shapes,15-bit exponent range and finite ARF fused reference. New checkpoint41 does not rerun fast arithmetic; checkpoint39 finite controls remain scoped.',
 't-fast_mul.c':'Full separate, either-factor and identical-factor aliases with exact ARF product, normalized mantissas and relative upper allowance. It does not validate arbitrary raw partial overlap.',
 't-fast_mul_2exp_si.c':'Finite inline-exponent rescaling checked for exact ARF equality. Earlier bounded public fast controls remain qualification, not all possible host shifts.',
 't-geom_series.c':'Complete shared exponentiation helpers; test sums only50 terms downward and checks a whole output alias. New GMP/JS rational x^N/(1-x) oracle includes the entire convergent tail, including near-one mantissas.',
 't-get_d.c':'Magnitude-to-double exact equality in the ordinary range and upper direction outside it, special/promoted random generation. New bounded exports check documented saturation at2^-1000/infinity and direct exact rational comparison.',
 't-hurwitz_zeta_uiui.c':'Only50 positive terms checked by Arb containment. New directed MPFR zeta(s)-finite-prefix reference encloses the full Hurwitz tail for2<=s<=32,1<=a<=17; no arbitrary-word range claim.',
 't-mul_2exp_fmpz.c':'Exact ARF equality under random100-bit exponents and special magnitudes. New checkpoint41 does not newly exercise promoted exponent histories.',
 't-mul_2exp_si.c':'Exact ARF equality for a signed-word scaling exponent, special magnitude handling and normalized result. Source-only extension beyond earlier bounded scale corpus.',
 't-polylog_tail.c':'Arb containment of only100 terms, N and d below100, signed s aroundzero, special/indeterminate wrapper and whole alias. New directed oracle covers the complete tail with an eventual geometric remainder and separately counts valid but uninformative infinity.',
 't-pow_fmpz.c':'All duplicated exponentiation helpers, negative exponent polarity reversal, upper/lower signed200-bit random exponent checks and whole aliases read. New tests do not call zero negative powers or unbounded exponent loops.',
 't-pow_ui.c':'All duplicated helpers, upper/lower word exponent paths, NaN reference convention and supported whole aliases. A rounded sibling reference is not an exact-value theorem.',
 't-rfac_ui.c':'Reciprocal factorial upper bound against60-bit upward ARF division of exact fmpz factorial. New native/JS entire rational comparisons cover all table entries without an approximate reciprocal oracle.',
 't-rsqrt.c':'Upper reciprocal-root ARF reference, relative tolerance, special/promoted magnitudes and whole alias. Previous finite exact-squaring controls are independent of this root oracle.',
 't-rsqrt_lower.c':'Lower reciprocal-root reference and relative window with special inputs/aliases. Source read adds no all-libm, all-exponent or architecture qualification.',
 't-set_d.c':'Complete two-limb random double generator, signed/special and normal/subnormal scaling, upper/lower conversion and relative quality comparisons. New finite bit-pattern corpus excludes NaN/infinity inputs and independently checks exact rationals.',
 't-set_d_2exp_fmpz.c':'Complete duplicated double generator, arbitrary100-bit scale, sign/special cases and both bound directions via ARF. New finite scales are only-1024,-31,0,31,1024; promoted scale claims remain source-only.',
 't-set_ui.c':'Word upper constructor checked through ARF and relative allowance. New checkpoint41 does not replace prior constructor qualification with an upstream random-suite claim.',
 't-set_ui_lower.c':'Word lower constructor, ARF comparison and relative allowance. All test lines are read; host-word exhaustive enumeration is not performed.'
};
const prior=[...json('coverage.json'),...json('coverage-extensions.json')];
const inv=json('inventory.json').sources.find(s=>s.repo==='flint');
const full=Object.entries(tests).map(([file,note])=>['src/mag/test/'+file,'Full donor test/driver read. '+note+' Upstream random suite not newly run.']);
full.push(['src/arf/io.c','Complete251-line ARF string, print and file layer, hexadecimal mantissa/exponent dump, special tags, parser cleanup, shallow Arb formatter and sequential file imports. Valid bounded mag string roundtrips exercise this bridge. Source-only file-output error cleanup concerns and malformed/special parser inputs are not reproduced; printing/backend recursion is not inferred audited.']);
const fresh=full.map(([path,note])=>{
 const f=inv.files.find(f=>f.path===path);assert(f?.text);assert(!prior.some(r=>r.repo==='flint'&&r.path===path),path);
 return{repo:'flint',path,ranges:[[1,f.lines]],note};
});
const added=[{repo:'flint',path:'src/arf/get.c',ranges:[[1,439],[531,604]],
 note:'Completes604-line get.c while preserving prior440..530 credit. Read signed binary64 saturation and MPFR corner dispatch, exact rational/dyadic extraction, integer tie-to-even and directed rounding, limb extraction, fixed-scale shallow adapters, MPFR huge exponent normalization and signed-word fit checks. New magnitude exports use finite bounded rational and ceil/floor conversion paths only. Direct arbitrary-width ARF rounding, promoted exponent states and raw limb-helper boundaries remain unqualified; no unsafe reproduction.'}];
let lines=0;
for(const r of [...fresh,...added]){
 const f=inv.files.find(f=>f.path===r.path),p=resolve('../../../../exact-real-references/flint',r.path);
 assert.equal(sha(p),f.sha256,r.path);const s=readFileSync(p,'utf8');assert.equal(s.split('\n').length-Number(s.endsWith('\n')),f.lines);
 const old=prior.filter(x=>x.repo===r.repo&&x.path===r.path);
 for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);lines+=b-a+1;for(const o of old)for(const[c,d]of o.ranges)assert(b<c||a>d,r.path);}
}
assert.equal(fresh.length,29);assert.equal(lines,3478);
const mag=inv.files.filter(f=>f.path.startsWith('src/mag/'));assert.equal(mag.length,104);
for(const f of mag)assert([...prior,...fresh].some(r=>r.repo==='flint'&&r.path===f.path&&JSON.stringify(r.ranges)===JSON.stringify([[1,f.lines]])),f.path);
console.log('*** Begin Patch\n*** Update File: '+resolve('coverage.json')+'\n@@\n [\n'+fresh.map(r=>'+'+JSON.stringify(r)+',').join('\n')+
 '\n*** Update File: '+resolve('coverage-extensions.json')+'\n@@\n [\n'+added.map(r=>'+'+JSON.stringify(r)+',').join('\n')+
 '\n*** Add File: '+resolve('mag-series-read-selection.json')+'\n'+JSON.stringify([...fresh,...added].map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})),null,2).split('\n').map(s=>'+'+s).join('\n')+'\n*** End Patch');
