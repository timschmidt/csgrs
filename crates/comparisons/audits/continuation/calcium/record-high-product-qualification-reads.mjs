import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
const coverage=json('coverage.json'),extensions=json('coverage-extensions.json');
const inventory=json('inventory.json').sources.find(s=>s.repo==='flint');
const full=[
 ['src/arf/mul_rnd_down.c',260,'Full round-toward-zero multiplication and mpz multiplier. One/two-limb exact products and explicit masking, larger full-product-then-round path, demand-sensitive MPFR delegation with a source FIXME for cutoffs. Work precision and inexact return are part of the ARF contract; ordinary nfloat high products cannot replace them blindly. Hyper already plans operand precision from certified magnitudes, reuses child square approximations and exactly multiplies integer approximations before final scale. No threshold transfer.'],
 ['src/arf/mul_rnd_any.c',62,'Full other-rounding-mode path: special values, full integer multiplication into temporary storage, directed rounding and exponent/sign reconstruction. No direct raw approximate-high dispatch; rounding helper/callees remain separate audit work.'],
 ['src/arf/mul_via_mpfr.c',80,'Full MPFR bridge: borrowed read-only significand views with zeroed local exponents, output precision capped by full-product size, mpfr_sqr for identical objects, rounding-mode conversion, exponent reconstruction and zero-low-limb removal. This is properly rounded finite-precision arithmetic, not a full constructive-real tower. No arbitrary exponent/alias/thread qualification claimed by reading.'],
 ['src/arf/mul_tmp_cleanup.c',22,'Full thread-local multiplication scratch globals and cleanup reset. Cleanup frees the retained buffer and zeroes pointer/capacity; execution/thread-lifetime contracts require separate qualification.'],
 ['src/mpn_extras/x86_64/broadwell/mulhigh_basecase.asm',299,'Full ADX basecase including initial triangle, high-half boundary products, separate CF/OF addmul chains, rotating unrolled labels, carry finalisation and ABI register restoration. Numerical triangular-sum oracle is distinct from an instruction-level proof. Bounded positive supported sizes only; no invalid-size, overlap or large-index reproduction.'],
 ['src/mpn_extras/x86_64/broadwell/sqrhigh_basecase_odd.asm',405,'Full odd-size ADX square, initial off-diagonal triangle, unrolled addmul chains, doubling, exact diagonal additions, PIC/non-PIC jump tables and register restoration. Doubled boundary high halves omit low-product carry; sibling consistency alone does not prove the documented n+2 guard bound. Native finite witnesses are independently reconstructed; no all-input or instruction-level proof.'],
 ['src/mpn_extras/x86_64/broadwell/sqrhigh_basecase_even.asm',418,'Full even-size ADX square including high-only boundary diagonal, off-diagonal triangle/doubling, full high diagonal and both jump-table forms. Same qualification boundary as odd square. Both actual assembly files read, not merely table declarations; other hardcoded/normalised/arm kernels remain open.']
];
const newRecords=full.map(([path,lines,note])=>({repo:'flint',path,ranges:[[1,lines]],note}));
const addedRanges=[
 {repo:'flint',path:'src/mpn_extras.h',ranges:[[320,395]],note:'Additional read after frozen checkpoint 32: three-limb reverse-dot macros, high-half-only boundary sum, exact reverse add-dot, two-limb addmul and nearby arithmetic macros. The explicit omitted low products explain why n+2 is not a general bound for the triangular scheme; separate numerical reconstruction matches the sampled outputs.'},
 {repo:'flint',path:'src/arf.h',ranges:[[700,785]],note:'Additional actual read: multiplication dispatch/MPFR thresholds and scratch macros. Up to 40 limbs on the stack; up to 1000 in grow-only thread-local storage with registered cleanup; larger allocations are transient. These are conditional source caps (320/8000 bytes on this 64-bit build), not measured peak RSS or a general concurrent/reentrant retention guarantee. Partial inline negated-multiply lead-in also read; rest of header/callees remain open.'}
];
for(const e of [...newRecords,...addedRanges]){
 const f=inventory.files.find(f=>f.path===e.path);assert(f?.text);assert.equal(sha(resolve('../../../../exact-real-references/flint',e.path)),f.sha256);
 for(const[a,b]of e.ranges)assert(a>=1&&b>=a&&b<=f.lines);
 if(newRecords.includes(e)){assert(!coverage.some(x=>x.repo===e.repo&&x.path===e.path));assert.equal(e.ranges[0][1],f.lines);}
 else {
  const old=[...coverage,...extensions].filter(x=>x.repo===e.repo&&x.path===e.path);assert(old.length);
  for(const[a,b]of e.ranges)for(const x of old)for(const[c,d]of x.ranges)assert(b<c||a>d);
 }
}
assert.equal([...newRecords,...addedRanges].reduce((n,e)=>n+e.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),1708);
console.log('*** Begin Patch\n*** Update File: '+resolve('coverage.json')+'\n@@\n [\n'+newRecords.map(e=>'+'+JSON.stringify(e)+',').join('\n')+
 '\n*** Update File: '+resolve('coverage-extensions.json')+'\n@@\n [\n'+addedRanges.map(e=>'+'+JSON.stringify(e)+',').join('\n')+
 '\n*** Add File: '+resolve('high-product-qualification-read-selection.json')+'\n'+JSON.stringify([...newRecords,...addedRanges].map(e=>({repo:e.repo,path:e.path,newRanges:e.ranges})),null,2).split('\n').map(l=>'+'+l).join('\n')+'\n*** End Patch');
