// Emit, but do not apply, coverage for completed individual reads in this pass.
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const coverage=json('coverage.json'), inventory=json('inventory.json');
const paired={
  mul:'Dimension checks, explicit zero-inner-dimension output, and temporary/swap for whole-input aliases. Rational dispatch for inner dimension>=3 borrows fmpq/fmpz headers, converts output to rational before backend work, writes headers back and frees only temporary header arrays; archived row tables become current stride storage. Common pointer-identical number fields selected for inner>=4 and both outer dimensions>=3. Specials rejected by field scan; no direct ownership port to immutable Rust scalars. Hyper already specializes rational/shared-scale multiplication.',
  ca_poly_evaluate:'Paterson-Stockmeyer block polynomial evaluation with m=floor(sqrt(length))+1, m+1 stored powers and Horner in A^m. Length0/1/2 use zero/scalar-identity/scaled-matrix shortcuts. Powers copied before output writes, supporting whole-matrix alias. Allocates and clears unused temporary t; repeated output-alias multiplication allocates its own temporary. Scalar-coefficient/entry alias not qualified. Workspace reuse and high-degree matrix evaluation are ideas, not demonstrated Hyper gaps or wins.',
  mul_classical:'Dimension check and explicit zero for empty inner product; each cell initializes first product then left-associated addition with one scalar temporary. Whole-input alias delegates through default multiplication into fresh storage, so an aliased classical call need not exercise the classical backend. Current uses flint_throw rather than archived abort.',
  addmul_ca:'Entrywise y+=a*x using one scalar temporary; whole matrix alias preserves per-entry operands when x is separate. Not a joint dot kernel; scalar x aliasing an output entry is not promised or tested. No unused Rust temporary or ownership transfer inferred.',
  submul_ca:'Independent complete read of entrywise y-=a*x with one temporary. Whole matrix alias is per-entry when x is separate; no blanket scalar-entry alias claim or Hyper transfer.',
  pow_ui_binexp:'Special empty/exponent0 identity, scalar dimension1, exponent1 copy and exponent2 square. Higher left-to-right binary powers use two owned matrix workspaces with swaps; original input is read until final output swap. Hyper already has repeated squaring and specialized borrowed small powers, so no missing power capability or performance improvement established.',
  trace:'Square check, empty trace zero, then A00 and remaining diagonal additions. Whole output scalar alias with a later diagonal entry is not qualified; no general scalar-entry alias guarantee assumed. Hyper immutable scalar/value outputs avoid this write-order interface.',
  'test/t-ca_poly_evaluate':'Tests additivity f(A)+g(A)=(f+g)(A), not multiplication despite variable fg. Dimensions0..2 and rational polynomials length<=10; one in-place g(A) path and seeded outputs. Equality rejects only False, accepting Unknown; no independent matrix-polynomial coefficient oracle.',
  'test/t-mul':'Associativity on generic dimensions0..3 and rational0..4, seeded outputs, no explicit alias cases. Unknown equality accepted. Generic dimensions never select default same-number-field dispatch requiring inner>=4, so the test does not cover that branch.',
  'test/t-mul_same_nf':'Direct same-number-field versus classical binary product, despite associativity comment; only non-rational NF selected. Dimensions0..4, common denominator scale from10-bit integer. Optional left whole alias only when all dimensions equal, no right/simultaneous alias or1000-bit denominator fallback controls. Current trial count one tenth archived; equality accepts Unknown.',
  add_ca:'Adds scalar times rectangular identity, not scalar to every entry. Whole matrix alias only updates min(rows,cols) diagonal; separate output copies off-diagonals. Assumes matching shapes and separate scalar; no entry-alias guarantee or port needed.',
  sub_ca:'Independent scalar-identity subtraction read. Same-matrix path changes only diagonal; separate path copies all off-diagonal entries. Mutability/shape and scalar-entry alias preconditions are not inferred away.',
  set_ca:'Writes scalar to each diagonal and exact zero elsewhere, including rectangular matrices. Caller-provided scalar must remain valid throughout writes; no scalar-entry alias qualification. Useful distinction between a scalar matrix and an all-constant matrix.',
  check_equal:'Shape mismatch immediately False; otherwise full entry scan remembers Unknown but any later False dominates. Empty matching shapes are True. Confirms three-valued aggregate scheduling already used by retained Hyper polynomial fact-first decisions; does not make all scalar equalities decidable.',
  check_is_zero:'Scans all entries, remembers Unknown, returns False on a later certified nonzero. Empty matrix is vacuously zero. Same-size witness dominance motivates rank idea, but combinatorial cost must be assessed separately from linear entry scans.',
  check_is_one:'Checks rectangular identity entrywise, with False dominating earlier Unknown; vacuously True for empty shapes and no square-shape restriction. This is a shape-specific identity predicate, not an unconditional scalar one test.',
  transpose:'Validates transposed dimensions; empty is no-op, exact whole alias is square and swaps only strict off-diagonal pairs; separate output copies each scalar. Current throws instead of archived abort. No overlapping-window alias or mutable-layout transfer assumed.',
  zero:'Overwrites every live matrix entry with exact scalar zero, unlike a status-only zero shortcut. Supports the earlier seeded zero-RREF output finding. Empty shape is a no-op; no missing production Hyper zeroing capability established.',
  one:'Writes exact one on the diagonal and zero elsewhere for all entries of rectangular as well as square matrices. No refinement/equality query needed to publish the constructive identity fact; existing Hyper constructors already do this.',
  ones:'Independent read of all-entries-one fill, distinct from scalar identity. Traverses row/column accessors, so archived row pointers and current stride storage share semantics. No algorithmic transfer warranted.'
};
const entries=[];
for(const[name,note]of Object.entries(paired))for(const repo of ['calcium','flint']) {
  const path=`${repo==='flint'?'src/':''}ca_mat/${name}.c`;
  assert(!coverage.some(e=>e.repo===repo&&e.path===path),`already credited ${repo}:${path}`);
  const f=inventory.sources.find(s=>s.repo===repo).files.find(f=>f.path===path);assert(f?.text);
  entries.push({repo,path,ranges:[[1,f.lines]],note:`Complete independent ${repo} read: ${note}`});
}
assert.equal(entries.length,40);
const lines=entries.map(e=>'+'+JSON.stringify(e)+',').join('\n');
const selection=JSON.stringify(entries.map(({repo,path})=>({repo,path})),null,2).split('\n').map(s=>'+'+s).join('\n');
console.log(`*** Begin Patch\n*** Update File: ${here}/coverage.json\n@@\n [\n${lines}\n*** Add File: ${here}/rank-cost-read-selection.json\n${selection}\n*** End Patch`);
