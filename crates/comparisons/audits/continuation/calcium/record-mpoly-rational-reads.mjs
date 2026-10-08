import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const inv=json('inventory.json'),prior=effectiveCoverage(),records=[];
const notes={
 'add.c':'Zero/equal/unit/scalar denominator dispatch; denominator GCD reduces cross products, then cancellation is restricted to the old GCD. Public operand aliases use temporaries where needed. Scalar-content GCDs avoid general multivariate GCD.',
 'sub.c':'Full signed counterpart of add, including negation when scalar-denominator operands are reversed, zero numerator, same denominators and alias-aware cross differences. Same canonical-input prerequisites.',
 'mul.c':'Zero/equal-denominator fast cases; pre-cancel numerator against opposite denominator, with scalar-content variants. No post-product full GCD is needed for canonical inputs.',
 'div.c':'Reject zero divisor before zero dividend; reciprocal multiplication, right-output-alias temporaries and positive leading-denominator normalization. Scalar negative divisors are sign-normalized before multiply.',
 'canonicalise.c':'Unit/zero/constant numerator or denominator shortcuts, otherwise exact polynomial GCD and division. Direct construction requires valid polynomial objects and nonzero denominator; no invalid-input probes.',
 'is_canonical.c':'Checks polynomial term representation, nonzero positive-leading denominator and GCD one. Uses the same GCD backend as normalization, so this alone is not an independent numerical oracle.',
 'evaluate_acb.c':'Evaluate denominator first, emit indeterminate if its enclosure contains zero, otherwise evaluate numerator and divide. Unit denominator bypasses denominator evaluation; ball semantics do not establish original authored-domain validity after formal cancellation.',
 'used_vars.c':'Zeroed public masks merge numerator/denominator degree flags, with constant and one-variable shortcuts. Private comment says AND although the operation is OR; no arithmetic defect follows.',
 'clear.c':'Clear both owned polynomial components.',
 'equal.c':'Structural pair equality is mathematical equality only under the canonical rational-function invariant.',
 'init.c':'Initialize both polynomials and denominator one, yielding canonical zero.',
 'inlines.c':'Emit header inline definitions in one translation unit.',
 'inv.c':'Nonzero requirement, public self alias through pair swap, then denominator leading-sign normalization.',
 'neg.c':'Negate numerator and copy denominator, including public whole-object in-place use.',
 'print_pretty.c':'Direct denominator-aware printing; string conversion/parser route through a fresh generic-ring context and optional variable names. Parsing failure contracts remain unexecuted; generic wrapper support still unread.',
 'randtest.c':'Bounded random numerator, bias toward unit or scalar denominator, replace zero denominator by one, then canonicalize.',
 'set.c':'Self-copy guard and scalar/rational imports; input rational canonicality is assumed, not repaired.',
 'swap.c':'Swap both polynomial components without rebuilding coefficients.'};
for(const s of inv.sources)for(const f of s.files) {
 const selected=s.repo==='calcium'?f.path==='doc/source/fmpz_mpoly_q.rst':
  f.path.startsWith('src/fmpz_mpoly_q/')||['src/fmpz_mpoly_q.h','doc/source/fmpz_mpoly_q.rst'].includes(f.path);
 if(!selected)continue;
 let note;
 if(f.path.endsWith('.rst'))note='Complete rational-function contract: canonical coprime polynomial pair with positive leading denominator, context ordering, scalar arithmetic/content and explicit formal-field semantics. Current manual additionally makes canonical-input assumptions explicit and documents generic string parsing. Archived counterpart is source documentation only; archived implementation remains unread.';
 else if(f.path.endsWith('.h'))note='Complete public API and inline properties, constants, signed wrappers, scalar-content early exit, GCD-success requirement and exact-division dispatch. Zero content is represented as one in the implementation. Formal-field equality is not partial computable-real equality or selected-root domain evidence.';
 else if(f.path.includes('/test/')) {
  const base=f.path.split('/').at(-1);
  note=base==='main.c'?'All15 test registrations and includes read.':base==='t-get_set_str.c'?
   'Valid generated string roundtrips under optional names; no malformed input or failure-output qualification.':base==='t-randtest.c'?
   'Random canonicality checks share the normalization GCD backend.':base==='t-inv.c'?
   'Inverse/inverse identity and canonicality, random public in-place choice, nonzero inputs.':
   'Read complete '+base+': finite random arithmetic with shared polynomial/GCD canonical reference or shared scalar wrapper; all contexts use ORD_LEX. General binary tests randomly choose one whole-object alias. Scalar-wrapper tests do not explicitly exercise their own in-place output branch.';
 } else note=notes[f.path.split('/').at(-1)];
 assert(note,f.path);assert(!prior.some(r=>r.repo===s.repo&&r.path===f.path));
 assert.equal(sha(resolve('../../../../exact-real-references',s.repo,f.path)),f.sha256);
 records.push({repo:s.repo,path:f.path,ranges:[[1,f.lines]],note:'Checkpoint46: '+note});
}
const support={repo:'flint',path:'src/fmpz_mpoly.h',ranges:[[75,90],[245,315]],
 note:'Checkpoint46 supporting public API excerpts: fixed variable-count/order context lifecycle, coefficient insertion and complete term coefficient/exponent extraction. Bounded canonical polynomial inputs only; underlying polynomial arithmetic/GCD and remaining header are not declared read.'};
assert(!prior.some(r=>r.repo===support.repo&&r.path===support.path));
assert.equal(sha(resolve('../../../../exact-real-references/flint',support.path)),inv.sources.find(s=>s.repo==='flint').files.find(f=>f.path===support.path).sha256);
records.push(support);assert.equal(records.length,38);
assert.equal(records.reduce((n,r)=>n+r.ranges.reduce((n,[a,b])=>n+b-a+1,0),0),3852);
writeFileSync('mpoly-rational-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
console.log(JSON.stringify({newCompleteFiles:37,newPartialFiles:1,newLines:3852,coverage:effectiveSummary()}));
