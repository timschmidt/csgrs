// Emit only reads actually completed. Preserve older checkpoint-bound partial
// records; an append-only extension ledger can add ranges without rewriting them.
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),text=readFileSync(resolve(here,'coverage.json'),'utf8');
const coverage=JSON.parse(text),inventory=JSON.parse(readFileSync(resolve(here,'inventory.json'),'utf8'));
const entries=[],selection=[];
function add(repo,path,note,ranges) {
  const f=inventory.sources.find(s=>s.repo===repo).files.find(f=>f.path===path);assert(f?.text);
  assert(!coverage.some(e=>e.repo===repo&&e.path===path),path);ranges??=[[1,f.lines]];
  entries.push({repo,path,ranges,note});selection.push({repo,path,newRanges:ranges});
}
const paired={
  jordan_blocks:'Checks square and block-size sum for construction, writes diagonal eigenvalues/superdiagonal ones. Decomposition obtains distinct eigenvalues/multiplicities, then ranks powers of A-lambda I and transposes rank differences as a Ferrers partition; multiplicity-one shortcut. No failed-rank output is a certificate. Current wrapper delegates to generic implementation. No mutable rank-state transfer to Hyper.',
  jordan_form:'Copies input if J or P aliases A, permits null P, obtains blocks then transformation and only writes J after success. Does not promise J/P simultaneous output alias. Current generic forwarding preserves these steps; read the implementation separately.',
  jordan_transformation:'Sage-derived chain construction: distinct eigenvalue nullspace shortcut, otherwise group block sizes, find vectors in ker(B^s) outside ker(B^(s-1)) plus existing chains using RREF, build powers/chains and reverse into P columns. Unknown residual stops candidate search; metadata validity assumed. Precomputed output alias remains TODO. Repeated powers/nullspaces/concatenations expose reuse ideas, not a cost-qualified Hyper transfer.',
  exp:'Archived Jordan decomposition plus certified inverse P, computes each eigenvalue exponential and finite factorial-scaled upper-triangular block, conjugates back. Current forwards to generic MAT_EXP. Source assumes scalar operations and does not provide a general arbitrary-real matrix-function certificate; evaluated structure can fail even though exp exists.',
  log:'Archive requires every eigenvalue provably nonzero, distinguishes False from Unknown, uses principal scalar log and signed inverse-power/k jets before conjugating back. Current wrapper maps GR_UNABLE before GR_DOMAIN. Matrix logarithm branch is explicitly defined by eigenvalue principal values, not arbitrary log(exp(A)) identity.',
  'test/t-jordan_blocks':'Rational known-block similarity family requires decomposition success, but block matching accepts Unknown equality. Generic/random family dimensions 0..5 (generic <=3) checks only if both decompositions succeed. Similarity is actually tested here; current counts are one tenth archived. Not independent spectrum arithmetic or guaranteed generic success.',
  'test/t-jordan_form':'Known-block family builds dense B=PAP^-1 but calls jordan_form on A, leaving that intended dense case unused. Success is conditional; inverse rejects False but not Unknown, followed by same-backend reconstruction equality accepting Unknown. Random dimensions 0..4 (generic <=2) include P/input alias, not J/input. No required-True full chain or block-structure oracle.',
  'test/t-exp':'Dimensions only 0..2, occasional generic inputs, optional exp whole alias. Conditional trace/determinant and similarity identities accept Unknown; log check is conditional exp(log(A)) roundtrip accepting Unknown, not direct coefficients or branch oracle. Current nominal iterations 10 versus archived 1000. High-order Jordan jets are not exercised by these dimensions.',
  dft:'Builds 2n powers of exp(2*pi*i/n), n=min(rows,cols), sign by type, modulo 2n lookup, then inverse or unitary scaling. Includes rectangular periodic extension and empty early return. Caching n instead of 2n powers is a donor memory idea only; Hyper has no demonstrated matching hot DFT matrix constructor needing this transfer.',
  'test/t-dft':'Both versions require True for forward/inverse and unitary inverse products in square dimensions 0..16. Fresh outputs, no rectangular coefficient oracle; product uses same backend. Stronger than merely overlap/Unknown acceptance, but not full constructor coverage.',
  hilbert:'Archive writes each reciprocal of i+j+1; current delegates to generic Hilbert construction. Empty dimensions naturally no-op. Generic implementation still recomputes equal antidiagonal values; no Hyper workload requiring a Hilbert API or qualified reuse gain established.',
  pascal:'Upper/lower/symmetric rectangular Pascal recurrences with explicit zeros/ones and empty handling. Archive treats any other triangular selector as symmetric; generic current accepts only -1,0,1. No behavior inferred for invalid selector inputs. Exact integer combinatorics already exist in Hyper basis conversion.',
  stirling:'Three descending-row recurrences: unsigned first kind, signed first kind and second kind, explicit zero tail/diagonal and empty dimensions. Archive uses row pointers; current generic entry/stride helpers and status aggregation. Current rejects selectors outside 0..2. No finite-word factorial substitution for Hyper exact coefficients.',
  add:'Elementwise scalar addition over first input dimensions, trusting valid matching shapes. Whole-input alias is scalar-local; overlapping windows are not inferred safe. No new representation or fused kernel.',
  sub:'Independent elementwise subtraction read, matching-shape preconditions, scalar-local whole alias. No status/refinement or new Hyper algorithm.',
  neg:'Independent scalar negation loop over source dimensions. Output already initialized/sized; no allocation schedule beyond scalar operations.',
  conj:'Validates matching shape then scalar conjugation per entry; current throws versus archive abort. Inherits scalar branch-cut semantics already audited; no numerical representation replacement.',
  conj_transpose:'Two passes: transpose then inplace elementwise conjugation. Whole square alias follows transpose semantics; no arbitrary overlap guarantee. Fusion is a donor idea only, not a demonstrated current real-only Hyper bottleneck.',
  set:'Skips exact self-copy and zero-column matrices, otherwise scalar copies across source dimensions. Assumes valid sized destination. No mutable C header ownership port to immutable shared Real.',
  set_fmpq_mat:'Copies exact rational entries over destination shape, no shape conversion or approximation. Different source/destination domains, not in-place ownership alias. Hyper already exact rational import.',
  set_fmpz_mat:'Independent exact integer matrix import loop over destination shape, initialized correctly sized objects assumed. No floating conversion; no missing Hyper scalar import capability.',
  transfer:'Same-context fast copy, otherwise ca_transfer per element reconstructing into destination context. Context-local extension identities cannot be shallow-moved across contexts. Hyper synchronized immutable expressions are not replaced by this ownership model.',
  randops:'Random row/column addition or subtraction between distinct indices, preserving rank; empty early return and same-index draws skipped. Test helper only, not a numerical oracle.',
  randtest:'Random per-matrix density 0..99, elementwise generic/rational generation or explicit zero. Sparse mix does not systematically cover storage histories, alias shapes or proof-Unknown placements.',
  inlines:'Defines the inlines-instantiation macro and includes ca_mat.h; no additional algorithm. Header implementation coverage is tracked separately.',
  get_fexpr:'Collects extensions across all entries, one shared variable/Where definition scope, builds Row/Matrix expressions, and clears temporaries. Sharing helps exported expression size, but it is not a serialized numeric-state roundtrip or a demonstrated gap in Hyper serialization.',
  print:'Diagnostic exact per-entry printing and compact requested-digit rendering with row punctuation, including empty shapes. No proof parser, no exact numeric roundtrip guarantee, no formatting performance claim.'
};
assert.equal(Object.keys(paired).length,27);
for(const [name,note]of Object.entries(paired))for(const repo of ['calcium','flint'])
  add(repo,`${repo==='flint'?'src/':''}ca_mat/${name}.c`,`Complete independent ${repo} read: ${note}`);
const generic={
  jordan_blocks:'Generic block constructor and rank-power Ferrers decomposition, square/sum validation and status propagation. Multiplicity storage is fmpz but bounded by matrix dimension in valid decomposition. Failure outputs not used by our probes; no failure-state memory reproduction.',
  jordan_form:'Full generic wrapper read, including input copy for J/A or P/A alias, nullable P and final J publication only after successful blocks/transformation. Does not assert simultaneous output alias.',
  jordan_transformation:'Full generic chain implementation, row-space difference RREF, shared temporary scalar/vector storage, exact status checks, matrix-ring generic powering helper, distinct-eigenvalue shortcut and generalized nullspace chains. Called matrix-ring implementation remains supporting scope, not credited by this wrapper read.',
  exp:'Exponential jet recursively divides exp(x) by 1..n-1; default MAT_EXP method dispatcher and Jordan wrapper. No matrix-specific approximation kernel or Hyper type-layout replacement.',
  log:'Log jet computes inverse powers by balanced index split then divides by order and negates even orders, reducing dependency depth relative to archive sequential powers. Scalar-expression/cache consequences need measurement before any Hyper transfer.',
  func_jordan:'Shared function evaluator owns P/Q/J, requires field or real precision, uses approximate distinct-eigenvalue diagonalization fallback only for precision contexts. Exact path obtains Jordan data/inverse; evaluates one maximal jet per distinct eigenvalue and copies prefixes to repeated blocks. Output failure status invalidates values. Read does not cover approximate diagonalization implementation.',
  hilbert:'Generic reciprocal construction with aggregated statuses and symmetry TODO; exact dimension loops. Read independently of the CA forwarding wrapper.',
  pascal:'Generic triangular/symmetric rectangular recurrence with zeros/ones, empty fast return and invalid selector GR_DOMAIN. Dynamic method dispatch per scalar, no fmpz specialization yet.',
  stirling:'All three generic row recurrences, descending order and explicit zero tails; status aggregation and valid-kind dispatch. Full independent read of wrapper and private helpers.'
};
for(const [name,note]of Object.entries(generic))add('flint',`src/gr_mat/${name}.c`,`Complete current generic read: ${note}`);
assert.equal(entries.length,63);
add('flint','doc/source/gr_mat.rst','Only 952..1008 read: diagonalization domain/status contract, nullable left/right outputs, precomputed eigenvalue requirements, Jordan declarations and normalized jet callback definition. Range starts/ends within broader documentation sections.',[[952,1008]]);
const old=coverage.find(e=>e.repo==='flint'&&e.path==='src/gr_generic/generic.c');assert(old);
const newRanges=[[3153,3169]];
const updated={...old,ranges:[...old.ranges,...newRanges].sort((a,b)=>a[0]-b[0]),note:old.note+' Read 3153..3169 additionally: default polynomial and matrix method registrations, including MAT_EXP/MAT_LOG pointing to the Jordan evaluators.'};
selection.push({repo:old.repo,path:old.path,newRanges,previous:old});
const oldLine=text.split('\n').find(l=>l.includes('"src/gr_generic/generic.c"'));assert(oldLine);
const newLine=oldLine.slice(0,oldLine.indexOf('{'))+JSON.stringify(updated)+(oldLine.endsWith(',')?',':'');
add('flint','src/fmpz_lll/is_reduced_d.c','Only 765..821 read after the instrumented matrix-function abort: final directed-rounding bounds, rounding-mode restoration, exact reduced-basis assertion and return. Earlier function/setup and recursively called reduced-basis checks remain unread; root cause and native reproducibility are not established by this tail.',[[765,821]]);
const supplement={repo:'flint',path:'doc/source/ca_mat.rst',ranges:[[515,624]],
  note:'Additional actual read after checkpoint 22: companion status, eigenvalue multiplicities/arbitrary order, Jordan block metadata and arbitrary failure outputs, nullable P, optional direct precomputed work, exp existence versus algorithmic failure and matrix log eigenvalue-principal branch. Earlier checkpoint binds the old partial entry exactly; preserve it and extend via this append-only ledger.'};
const plus=s=>s.split('\n').map(l=>'+'+l).join('\n');
console.log(`*** Begin Patch\n*** Update File: ${here}/coverage.json\n@@\n [\n${entries.map(e=>'+'+JSON.stringify(e)+',').join('\n')}\n@@\n-${oldLine}\n+${newLine}\n*** Add File: ${here}/spectral-read-selection.json\n${plus(JSON.stringify(selection,null,2))}\n*** Add File: ${here}/coverage-extensions.json\n${plus(JSON.stringify([supplement],null,2))}\n*** End Patch`);
