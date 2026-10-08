import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=b=>createHash('sha256').update(b).digest('hex');
const inventory=JSON.parse(readFileSync('inventory-v85.json'));
// Ranges below record actual displayed reads, not dependency reachability or
// execution. Full files were read in bounded, untruncated chunks.
const notes={
 'README.md':'Setup, tested historical Coq versions, extraction postprocessing and historical benchmark conditions; no current native verification inferred.',
 'extracted-examples/stack.yaml':'Pinned resolver and AERN2 package versions; existing AERN2 checkout is a separate source specimen.',
 'extracted-examples/package.yaml':'Optimized Haskell library and fractal executable; generated helpers exported without proof-level preconditions.',
 'formalization/Extract.v':'Abstract Real/K/M runtime substitutions, erased proof preconditions, countable selection, limit and trace mappings; explicit unrealized boolean eliminator.',
 'formalization/ExtractMB.v':'Explicitly marked broken fixed-precision/restart variant; wrong monad/error interpretation is not a transferable implementation.',
 'formalization/Base/Base.v':'Type/Prop boundary, propositional and functional extensionality, proof irrelevance and dependent-choice assumptions; no executable total equality implied.',
 'formalization/Base/Kleene.v':'Three-valued semidecision axioms and definedness precondition; countable OR is not an arbitrary exact-decision oracle.',
 'formalization/Base/Monad.v':'Functor, unit, multiplication and coherence laws; these are part of the semantic contract erased by extraction.',
 'formalization/Real/RealAxioms.v':'Ordered-field, nonzero inverse, strict semidecision, Archimedean and fast-Cauchy/limit axioms; dyadic precision and total Prop order distinguished.',
 'formalization/Real/RealAssumption.v':'Bundled real/monad assumptions, naming and notations; not a concrete proved runtime instance.',
 'formalization/Real/Real.v':'Aggregation of real dependencies; transitive imports do not receive read credit here.',
 'formalization/Real/RealLimit2.v':'All-branch fast Cauchy and shared-limit conditions, coherent selection, uniqueness and hprop elimination; arbitrary branch histories do not suffice.',
 'formalization/Analysis/Minmax.v':'Overlapping strict predicates approximate a common maximum; dual minimum and order laws. Suspicious committed intro/real T_lt_plus_lt text at 205-206 remains an uncompiled source anomaly.',
 'formalization/Analysis/Magnitude.v':'Positive-domain coarse exponent between powers separated by two bits; scale/reciprocal cases and overlapping choice, not correctly rounded log2.',
 'formalization/Analysis/Testsearch.v':'Search requires an existing witness excluding Q; returned first selected P is not necessarily the least true P when predicates overlap.',
 'formalization/Analysis/Rounding.v':'Coarse integer bracketing and multivalued error-less-than-one rounding; dyadic approximation and all-choice Cauchy proof, not exact floor or ties-to-even.',
 'formalization/Makefile':'coq_makefile build and tests target; no compile was run and tests directory is absent from inventory.',
 'formalization/_CoqProject':'Actual project membership differs from whole inventory; omitted analytic/legacy files remain in audit scope.',
 'extracted-examples/src/Max.hs':'Entire extracted max uses limit of overlapping operand choices; boilerplate and imports included in reading.',
 'extracted-examples/src/Magnitude.hs':'Entire extracted positive magnitude algorithm and integer helper boilerplate; formal domain erased, no invalid-input runtime bug claimed.',
 'extracted-examples/src/Sqrt.hs':'Entire extracted Newton/scaling/nonnegative-limit algorithm and helpers. Only log2 call uses succ n, excluding zero for valid natural n; formal sqrt proof not read here.',
};
const records=Object.entries(notes).map(([path,note])=>{
 const f=inventory.files.find(f=>f.path===path);assert(f&&f.kind==='text');
 assert.equal(sha(readFileSync(inventory.root+'/'+path)),f.sha256);
 return{path,sha256:f.sha256,totalLines:f.lines,ranges:[[1,f.lines]],complete:true,note};
});
const f=inventory.files.find(f=>f.path==='formalization/Base/MultivalueMonad.v');
assert.equal(sha(readFileSync(inventory.root+'/'+f.path)),f.sha256);
records.push({path:f.path,sha256:f.sha256,totalLines:f.lines,ranges:[[1,200]],complete:false,
 note:'Monad definitions, trace-lift relation, strong choice/continuity axioms and initial picture lemmas only; lines 201-1077 remain unread.'});
records.sort((a,b)=>a.path.localeCompare(b.path));
const result={checkpoint:85,repository:inventory.repository,commit:inventory.commit,inventorySha256:sha(readFileSync('inventory-v85.json')),
 scope:'New coq-aern read credit only; no native Coq/Haskell execution and no credit for transitive dependencies or generated/binary assets.',records,
 totals:{complete:records.filter(x=>x.complete).length,partial:records.filter(x=>!x.complete).length,
 lines:records.reduce((n,r)=>n+r.ranges.reduce((s,[a,b])=>s+b-a+1,0),0)}};
if(process.argv.includes('--record'))writeFileSync('read-records-v85.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,JSON.parse(readFileSync('read-records-v85.json')));
console.log(JSON.stringify({status:process.argv.includes('--record')?'read-records-published':'read-records-verified',...result.totals}));
