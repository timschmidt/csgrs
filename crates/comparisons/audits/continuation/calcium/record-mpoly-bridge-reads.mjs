import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const inv=json('inventory.json'),prior=effectiveCoverage(),records=[];
const extra=['src/fexpr/is_arithmetic_operation.c','src/fexpr/expanded_normal_form.c',
 'src/fexpr/arithmetic_nodes.c','src/fexpr/set_fmpz_mpoly.c'];
const notes={
 'fmpz_mpoly_q.h':'Entire archived API, pair representation, signed wrappers, bounded scalar content/GCD helper, successful-GCD prerequisite, exact-division dispatch and zero content one convention. Current header was already read at46.',
 'add.c':'All denominator cases, staged denominator-GCD and second cancellation restricted to that GCD, scalar-content routes, temporaries and whole-object alias routes. Mechanism already exists in Hyper integer rational addition.',
 'sub.c':'All signed difference cases and reversed scalar-denominator route with final negation. Same staged denominator/content cancellation and alias constraints as addition.',
 'add_fmpq.c':'Equal/unit denominator cases and content GCD reduction; residual content restricted to old GCD. Archived TODO for integer denominator specialization.',
 'sub_fmpq.c':'Signed rational-scalar counterpart; zero right operand first, negative result for zero left, content/GCD and alias temporaries.',
 'mul.c':'Opposite-side GCD pre-cancellation and equal-denominator shortcut under canonical inputs; archived scalar-denominator TODO is implemented in current FLINT. No new Hyper transfer demonstrated.',
 'mul_fmpq.c':'Scalar content pre-cancellation against opposite numerator/denominator, unit/equal denominator and zero shortcuts, two-content case.',
 'div.c':'Zero divisor checked before zero dividend; reciprocal multiplication, temporary for right alias and leading-denominator sign repair. Redundant second zero-divisor check is not a new mathematical feature.',
 'div_fmpq.c':'Nonzero rational divisor; reciprocal scalar multiplication with negative-sign normalization.',
 'div_fmpz.c':'Nonzero integer divisor; immediate signed-one temporary and sign-normalized reciprocal.',
 'inv.c':'Nonzero input requirement, copy only when distinct, pair swap and positive-leading-denominator sign repair.',
 'canonicalise.c':'Unit/zero/scalar shortcuts followed by exact polynomial GCD cancellation and denominator leading-sign convention.',
 'is_canonical.c':'Term canonicality, nonzero positive-leading denominator and coprime pair via same GCD backend; not an independent oracle.',
 'evaluate_acb.c':'Archived polynomial evaluator sums coefficient-weighted monomials with word exponents and per-variable powers; zero/constant shortcuts. Rational evaluator checks denominator enclosure first and returns indeterminate when it contains zero. No caching or Horner strategy; no archived runtime or large-exponent qualification.',
 'used_vars.c':'Constant/one-variable shortcuts, degree-based OR union and public zeroed masks. Private AND comment disagrees with OR implementation; same harmless mismatch persists current.',
 'print_pretty.c':'Denominator-aware direct printing; archived API has no generic string parser.',
 'randtest.c':'Bias toward unit/scalar denominator, replace generated zero denominator with one, canonicalize. Shared canonicality backend.',
 'init.c':'Initialize two polynomial owners and denominator one.',
 'clear.c':'Clear two owned polynomials.', 'swap.c':'Swap both polynomial owners.',
 'equal.c':'Exact structural canonical-pair equality in the formal fraction field, not total computable-real equality.',
 'neg.c':'Numerator negation and denominator copy including whole-object alias.',
 'set.c':'Self-copy guard and pair copying.', 'set_fmpq.c':'Import canonical rational pair without extra normalization.',
 'set_fmpz.c':'Integer numerator and unit denominator.', 'set_si.c':'Signed integer numerator and unit denominator.',
 'inlines.c':'Single translation-unit emission of inline definitions.',
 'add_fmpz.c':'Integer wrapper passes immediate denominator one to rational-scalar addition.',
 'sub_fmpz.c':'Integer wrapper passes immediate denominator one to rational-scalar subtraction.',
 'mul_fmpz.c':'Integer wrapper passes immediate denominator one to rational-scalar multiplication.'};
for(const s of inv.sources)for(const f of s.files){
 const selected=s.repo==='calcium'?f.path.startsWith('fmpz_mpoly_q/')||f.path==='fmpz_mpoly_q.h'||/fexpr\/(get|set)_fmpz_mpoly_q/.test(f.path):
  /src\/(gr\/(test\/t-)?fmpz_mpoly_q|fexpr\/(get|set)_fmpz_mpoly_q)/.test(f.path)||extra.includes(f.path);
 if(!selected)continue;
 const base=f.path.split('/').at(-1);let note;
 if(f.path.includes('fexpr/')){
  note=base==='get_fmpz_mpoly_q.c'?'Formal recursive interpreter for integer leaves, arithmetic and integer powers, exact terminal lookup and explicit conversion status. No algebraic independence proof. Both generations differ only in license/include scaffolding; malformed/failure-output paths are source-read but unexecuted.':
   base==='set_fmpz_mpoly_q.c'?'Unit denominator emits polynomial directly; otherwise expression Div of emitted numerator/denominator. Does not recover authored domain obligations.':
   base==='is_arithmetic_operation.c'?'Recognizes Pos/Neg/Add/Sub/Mul/Div heads in inline and general call layouts; separate from arity validation.':
   base==='expanded_normal_form.c'?'Collect/sort formal leaves, build rational function, emit normalized expression on success and preserve original expression on conversion failure. Flags reserved; all-integer fmpq shortcut is still TODO.':
   base==='arithmetic_nodes.c'?'Recursive traversal skips integer leaves, descends through arithmetic and integer powers, inserts all other expressions uniquely without semantic relations.':
   'Expanded polynomial-to-expression conversion, constant/zero and default-symbol cases, per-term factor lists and word-exponent powers; shared array-allocation TODO. No giant-variable or exponent qualification.';
 }else if(f.path.startsWith('src/gr/')){
  note=base==='t-fmpz_mpoly_q.c'?'Ten random 0/1/2-variable contexts and random monomial orders delegate to gr_test_ring with100 iterations and flags0. Source-read only; generic suite internals not newly read or run.':
   'Entire generic-ring adapter: shared polynomial context helpers, exact truth predicates, field metadata, arithmetic wrappers, domain versus inability statuses, signed powers, numerator/denominator projection, balanced-addition string hook and method registration. Disabled square-root/factor blocks are not active implementations. Shared helpers/default dispatch/parser internals remain open; no blanket thread-safety qualification.';
 }else if(f.path.includes('/test/')){
  note=['t-add.c','t-sub.c','t-mul.c','t-div.c'].includes(base)?
   'Complete random lex-only arithmetic test with shared polynomial/GCD canonical reference and one randomly selected whole-object alias. Nonzero divisor generation for division.':
   base==='t-inv.c'?'Random lex-only inverse/inverse and canonicality checks; random in-place second inversion, nonzero input.':
   base==='t-randtest.c'?'Random lex-only canonicality check sharing normalization backend.':
   'Complete lex-only scalar-wrapper test with shared generic arithmetic or polynomial canonicalization reference; no explicit in-place output case for the tested scalar wrapper.';
 }else note=notes[base];
 assert(note,f.path);assert(!prior.some(r=>r.repo===s.repo&&r.path===f.path));
 assert.equal(sha(resolve('../../../../exact-real-references',s.repo,f.path)),f.sha256);
 records.push({repo:s.repo,path:f.path,ranges:[[1,f.lines]],note:'Checkpoint47: '+note});
}
records.push({repo:'flint',path:'doc/source/fexpr.rst',ranges:[[490,585]],
 note:'Checkpoint47: exact formal-indeterminate conversion contract and warning that related leaves may cause implicit division by zero; equal variable/context lengths, conversion statuses, no-simplification constructors, normal-form pipeline and vector initialization.'});
records.push({repo:'flint',path:'src/gr.h',ranges:[[1038,1045],[1080,1095],[1210,1227],[1600,1609]],
 note:'Checkpoint47: generic context/element lifecycle, numerator/denominator and power method wrappers and rational-function context declaration. Only these52 lines, not remaining header/default dispatch.'});
for(const r of records){const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);
 assert(!prior.some(p=>p.repo===r.repo&&p.path===r.path));assert.equal(sha(resolve('../../../../exact-real-references',r.repo,r.path)),f.sha256);
 for(const[a,b]of r.ranges)assert(a>=1&&b>=a&&b<=f.lines);}
assert.equal(records.length,56);const lines=records.reduce((n,r)=>n+r.ranges.reduce((n,[a,b])=>n+b-a+1,0),0);assert.equal(lines,5029);
writeFileSync('mpoly-bridge-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
console.log(JSON.stringify({newCompleteFiles:54,newPartialFiles:2,newLines:lines,coverage:effectiveSummary()}));
