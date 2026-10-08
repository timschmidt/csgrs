import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const text=readFileSync(resolve(here,'coverage.json'),'utf8'),coverage=JSON.parse(text);
const inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'),'utf8')).sources.find(s=>s.repo==='flint');
const notes={
 'src/fmpz_mat/is_reduced.c':'Full 2025 exact validator: doubles Arb precision from 64 to an input-bit/dimension cutoff, then exact rational Gram-Schmidt. Unknown caused by zero norm in final rational context means not reduced. Certified fact/status distinction retained; no wholesale replay or backend replacement in Hyper.',
 'src/fmpz_mat/is_reduced_gram.c':'Exact rational Gram-matrix Gram-Schmidt, exact import of binary64 delta/eta, size and Lovasz comparisons, per-failure cleanup, trivial dimensions accepted. Valid Gram input contract assumed; not a general arbitrary-matrix positivity checker.',
 'src/fmpz_lll/is_reduced.c':'Strips only initial zero basis rows via readonly window, then tries binary64 validator, historically named mpfr validator (now nfloat), and exact fallback. Gram route has analogous fallback without row stripping. A false fast result is inability to certify, not proof of non-reduction.',
 'src/d_mat/mul_classical.c':'Transposes B into contiguous dot rows, blocks by eight doubles, temporary output for whole alias, zero inner dimension resets output. Caller rounding mode affects accumulation; not an independently sufficient enclosure without input conversion bounds. Existing Hyper specialized aggregate kernels already exploit fixed layouts.',
 'src/d_mat/transpose.c':'Eight-element tiled transpose, shape check, temporary for square whole alias and entrywise swap. No arbitrary overlapping-window guarantee or demonstrated missing Hyper operation.',
 'src/fmpz_mat/get.c':'Float and transposed-float imports check magnitude against DBL_MAX and call fmpz_get_d; zero return certifies range, not exact representability or directed enclosure. Also full modular import with symmetry reuse read. Do not import approximate float as an exact Hyper fact.',
 'src/fmpz_mat/test/t-get_d_mat_transpose.c':'Random dimensions below ten but entries only small nonnegative 3j+7k, exact cast comparison and cleanup. Does not test inexact large integers, negative entries or overflow status.',
 'src/fmpz_lll/context_init_default.c':'Default delta .99, eta .51, integer basis and approximate Gram strategy. Stores configuration only; not validation/proof.',
 'src/fmpz_lll/context_init.c':'Directly stores caller delta/eta/representation/Gram strategy; no range validation. Numerical controls use valid conventional parameters .75/.51.',
 'src/gr_mat/is_lll_reduced.c':'Generic row Gram-Schmidt, separate operation status and three-valued comparison, squared norms on diagonal, size/Lovasz checks and optional removal norms. Non-success forces Unknown. GR_SUCCESS and T_TRUE currently both zero but belong to different status domains. Called generic arithmetic remains supporting scope.',
 'src/fmpz/get.c':'All integer export methods read: exact small cast window +/-2^53, larger small limbs through flint_mpn_get_d and heap integers through GMP; MPFR explicit rounding, modular and signed/unsigned limb exports under documented capacity/sign preconditions. No new Hyper export transfer: existing exact dyadic/floating boundary must remain certified.',
 'src/mpn_extras/get_d.c':'Full limb-to-IEEE conversion implementation and historical comments: truncate toward zero independent of hardware mode, extract 53 bits with 32/64-bit and endian layouts, exponent overflow to infinity and denormal/underflow handling. Native architecture controls do not qualify all alternate layouts or generic comment claims; no source-level arithmetic-safety proof.',
 'src/fmpz_lll/test/t-lll_is_reduced.c':'Ten random NTRU/integer-relation/Ajtai/simultaneous-Diophantine cases, optional reduction first and both basis/Gram representations. Requires every positive floating/nfloat verdict to agree with exact checker, permits negative fast verdict on reduced basis. Explicit TODO for cases where fast methods fail; not an independent oracle or exhaustive rounding-state test.',
 'src/fmpz_lll/is_reduced_mpfr.c':'Full 881-line implementation: despite API name, uses nfloat contexts for nearest/floor/ceil at >=64 bits, generic matrix/vector operations, transposed Q/inverse-V vectorization, status-guarded norm contractions and interval residual/error matrices for QR/Cholesky. Delta/eta import exact at this precision, final exact assertion; no process FENV mutations. Input conversion and called nfloat/matrix-method proofs remain support scope. Neither this read nor simple controls identify the prior field-relation abort cause.'
};
const entries=[],selection=[];
for(const[path,note]of Object.entries(notes)) {
  assert(!coverage.some(e=>e.repo==='flint'&&e.path===path),path);
  const f=inventory.files.find(f=>f.path===path);assert(f?.text);
  const e={repo:'flint',path,ranges:[[1,f.lines]],note};entries.push(e);selection.push({repo:e.repo,path,newRanges:e.ranges});
}
assert.equal(entries.length,14);
const old=coverage.find(e=>e.repo==='flint'&&e.path==='src/fmpz_lll/is_reduced_d.c');assert.deepEqual(old.ranges,[[765,821]]);
const updated={...old,ranges:[[1,821]],note:old.note+' Completed 1..764: directed FE_DOWNWARD/UPWARD residual products, QR or Gram Cholesky estimate, inverse error contractions, size/Lovasz bounds, caller rounding restoration on exits. Float imports only check DBL_MAX range, not exact representability. Earlier assertion remains unresolved; no donor modification or assertion bypass. Companion exact/nfloat validators and supporting conversion/matrix code now read separately.'};
selection.push({repo:old.repo,path:old.path,newRanges:[[1,764]],previous:old});
const oldLine=text.split('\n').find(l=>l.includes('"src/fmpz_lll/is_reduced_d.c"'));assert(oldLine);
const plus=s=>s.split('\n').map(l=>'+'+l).join('\n');
console.log('*** Begin Patch\n*** Update File: '+here+'/coverage.json\n@@\n [\n'+
  entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')+'\n@@\n-'+oldLine+'\n+'+JSON.stringify(updated)+
  (oldLine.endsWith(',')?',':'')+'\n*** Add File: '+here+'/derivative-lll-read-selection.json\n'+
  plus(JSON.stringify(selection,null,2))+'\n*** End Patch');
