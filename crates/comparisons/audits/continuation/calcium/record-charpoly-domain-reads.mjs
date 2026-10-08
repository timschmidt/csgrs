// Emit a patch for actual completed reads; never infer coverage from a call graph.
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const text=readFileSync(resolve(here,'coverage.json'),'utf8'),coverage=JSON.parse(text);
const inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'),'utf8'));
const file=(repo,path)=>inventory.sources.find(s=>s.repo===repo).files.find(f=>f.path===path);
const additions=[],selection=[];
function add(repo,path,note,ranges) {
  const f=file(repo,path);assert(f?.text);ranges??=[[1,f.lines]];
  const old=coverage.find(e=>e.repo===repo&&e.path===path);assert(!old,`already credited ${repo}:${path}`);
  const e={repo,path,ranges,note};additions.push(e);selection.push({repo,path,newRanges:ranges});
}
const paired={
  'test/t-nonsingular_solve':'Default solve tests generic dimensions up to 3, otherwise rational up to 5; initialized generic X. Success checks AX=B and failure checks zero determinant, but each rejects only False, accepting Unknown. Unknown solve status accepted without a required decision. No RHS/output alias or independent solution oracle.',
  'test/t-nonsingular_solve_adjugate':'Independent adjugate-solve test read: generic only up to 3, rational through 5, seeded X, backend AX=B/determinant checks conditional on status. Unknown equalities and Unknown solve status accepted. No independent coefficient oracle or alias qualification.',
  'test/t-nonsingular_solve_fflu':'Independent fraction-free solve test read: same conditional AX=B and determinant assertions accepting Unknown, dimensions through 5 with generic values only through 3, initialized X. Does not prove universal successful solve or failure-output validity.',
  'test/t-nonsingular_solve_lu':'Independent LU solve test read: conditional same-backend product/determinant checks reject only False, Unknown status accepted. Small generic/rational dimensions and seeded output, no explicit whole RHS alias.',
  'test/t-solve_tril':'Lower triangular tests dimensions/RHS columns 0..14 with rational entries only (number-field TODO), known positive diagonal, unit mode replacing stored diagonal after RHS construction, optional whole RHS/output alias and seeded output. Backend matrix product constructs B; equality rejects only False. Classical/recursive coverage only through default dispatch.',
  'test/t-solve_triu':'Independent upper triangular test read: rational dimensions 0..14, positive diagonal, unit-mode ignored stored diagonal, optional whole RHS alias, seeded output. Same-backend RHS multiplication and Unknown-permissive equality; number-field TODO. No direct separate classical/recursive branch requirement.',
  charpoly_danilevsky:'Owned copy before inplace similarity elimination; archive implements pivot search, block split, row/column permutation, certified inversion and polynomial block multiplication, with immediate Unknown failure and unused T allocation. Current wrapper delegates to generic GR_CTX_CC_CA implementation. Fits output before status, so failure output is not a certificate. Hyper already has pivot-free determinant and algebraic-fiber Berkowitz support; no missing global capability inferred.',
  'test/t-charpoly':'Generic dimensions 0..4; requires output length n+1 and p(A) zero with T_TRUE (not Unknown-permissive here). Same-backend matrix-polynomial annihilation is not an independent full coefficient oracle: annihilators may be nonunique when the minimal polynomial has smaller degree.',
  'test/t-charpoly_danilevsky':'Rational dimensions 0..9, checks length and required-True annihilation only if algorithm reports success. No required success or independent characteristic coefficient oracle; leading monicity not separately asserted beyond length.',
  companion:'Checks dimensions from polynomial length, rejects special leading coefficient/inverse; writes superdiagonal ones and negative normalized coefficients in last row, with n=0 empty case. Known-one normalization echoes already retained Hyper monic fact preservation; no new unconditional leading-one inference.',
  'test/t-companion':'Generic nonempty polynomials through length 4 and seeded 100-bit rational matrices. On successful companion construction, compares charpoly multiplied by original lead against original, rejecting only False. Uses tested characteristic kernel rather than independent oracle; non-success allowed.',
  eigenvalues:'Thin default characteristic-polynomial then ca_poly_roots wrapper, clearing temporary. Root capabilities/unknown propagation depend on already audited polynomial roots; not an independent spectral implementation.',
  diagonalization:'Precomputed variant zeros D, computes right kernels of A-lambda I, returns Unknown on kernel failure and False when geometric nullity differs from supplied algebraic multiplicity; forms P from kernel columns. Trusts complete valid supplied roots; no broad alias promise inferred. Public wrapper checks square, owns root data, propagates unknown root construction.',
  'test/t-diagonalization':'Dimensions 0..3, generic only up to 2; rational otherwise. Conditional success then inverse P and conditional PDP^-1 comparison, rejecting only False. Non-diagonalizable and alias cases remain TODO; Unknown inverse/equality or diagonalization not rejected. No independent required-success spectrum oracle.'
};
for(const [name,note]of Object.entries(paired))for(const repo of ['calcium','flint'])
  add(repo,`${repo==='flint'?'src/':''}ca_mat/${name}.c`,`Complete independent ${repo} read: ${note}`);
add('flint','src/gr_mat/charpoly_danilevsky.c','Complete read: generic similarity elimination and block splitting, Unknown pivots/status failures abort without a coefficient certificate; all scalar statuses ORed. Strided dot computes into temporary then swaps because generic dot output cannot overlap later inputs. T vector and c scalar allocated/cleared but unused. Square and zero-ring wrapper handling. Underlying CA has no specialized vector-dot registration; no optimized joint dot inferred.');
add('flint','src/gr/ca.c','Complete 1,831-line current generic CA wrapper read: typed RR/CC/AA/QQbar/extended contexts, borrowed versus owned cleanup, checked coercions, exact/uncanonical/non-threadsafe flags, scalar arithmetic/special functions/conversions, polynomial roots, matrix forwarding and entire method table. RR asin/acos omit real-result checks; arg omits algebraic-domain checks; field flag reused for real-vector-space property incorrectly includes algebraic contexts. Native valid-input probes qualify these, with inherited negative-rational Arg phase defect independently witnessed. Floor/ceil of complex values intentionally use the real part. Approximate binary64 getter is not assumed correctly rounded. Generic vector/dot defaults remain active. No representation or ownership port proposed.');
add('flint','src/gr/test/t-ca.c','Complete wrapper test registration read: initializes RR, CC, AA, QQbar and invokes gr_test_ring with 100 iterations and flags 0, then clears. Called test framework was not read in full, so registration alone does not establish its domain/special-function coverage.');
add('flint','src/gr_vec.h','Complete header read, including repaired earlier truncated ranges: method-table vector/scalar wrappers, normalization, borrowed shallow-header memcpy, contiguous/reversed/strided dots and sum/product declarations. No ownership port or implementation coverage inferred from declarations.');
assert.equal(additions.length,32);
add('flint','doc/source/gr.rst','Only recorded documentation ranges read: mathematical typed-parent/GR_SUCCESS contract, DOMAIN versus UNABLE, failed output has no valid mathematical interpretation, property/predicate distinctions, complex integer-part conventions and arithmetic domains.',[[1,220],[530,600],[820,910]]);
add('flint','doc/source/gr_special.rst','Only elementary-function documentation 1..90 read; declarations do not provide a promotion exception to the generic typed-parent success contract.',[[1,90]]);
add('flint','doc/source/gr_domains.rst','Only recorded documentation ranges read: context mathematical-property truth, exact versus canonical predicates, RR/CC/AA/QQbar domains and extended contexts admitting special values.',[[1,92],[225,275]]);
const old=coverage.find(e=>e.repo==='flint'&&e.path==='src/gr_generic/generic.c');assert(old);
const newRanges=[[1980,2275],[2390,2582],[3034,3152]];
const merged=[...old.ranges,...newRanges].sort((a,b)=>a[0]-b[0]);
assert(merged.every(([a,b],i)=>a<=b&&(!i||a>merged[i-1][1])));
const updated={...old,ranges:merged,note:old.note+' Additional recorded ranges: vector/scalar binary-op macros, addmul/submul single temporary and status aggregation, contiguous/reversed/strided generic dot implementations and vector method registrations. Strided optimized-dot path gathers shallow headers only when a specialized contiguous dot exists; CA has none and falls through to a scalar loop. Output cannot alias later input elements. Range beginning 1980 is inside normalization, not full-function coverage.'};
selection.push({repo:old.repo,path:old.path,newRanges,previous:old});
assert.equal(selection.length,36);
const oldLine=text.split('\n').find(l=>l.includes('"src/gr_generic/generic.c"'));assert(oldLine);
const newLine=oldLine.slice(0,oldLine.indexOf('{'))+JSON.stringify(updated)+(oldLine.endsWith(',')?',':'');
const plus=s=>s.split('\n').map(l=>'+'+l).join('\n');
console.log(`*** Begin Patch\n*** Update File: ${here}/coverage.json\n@@\n [\n${additions.map(e=>'+'+JSON.stringify(e)+',').join('\n')}\n@@\n-${oldLine}\n+${newLine}\n*** Add File: ${here}/charpoly-domain-read-selection.json\n${plus(JSON.stringify(selection,null,2))}\n*** End Patch`);
