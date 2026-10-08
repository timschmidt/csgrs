import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const inventory=JSON.parse(readFileSync('../inventory-v85.json')),previous=JSON.parse(readFileSync('../read-records-v85.json'));
assert.deepEqual(previous.totals,{complete:21,partial:1,lines:4666});
const notes={
 'formalization/Base/MultivalueMonad.v':'Completes 201-1077: picture semantics, path/fiber coherence, countable lifting, guarded choice, double semidecision, some/all and dependent membership. Exact observable equivalence and hprop elimination need the class assumptions read in 85; not a generic Unknown-to-choice conversion.',
 'formalization/Real/RealLimit0.v':'Classical weak-Cauchy subsequence and order-completeness proofs use countable choice and Prop-level excluded middle. No executable arbitrary-set supremum procedure follows from these proofs.',
 'formalization/Real/RealLimit1.v':'Classical fast limit and uniqueness, then strict/non-strict predicate limits and multivalued elimination with unique existence. Precision-indexed approximants require a common exact result.',
 'formalization/Analysis/Sqrt.v':'Complete real and complex roots. Normalized Newton error <=2^(-2^s), floor-based signed scale and near-zero overlap. csqrt_solutions and csqrt_small are Admitted at 628/710 and used by general complex-root construction. This is a proof gap, not an observed wrong answer.',
 'formalization/Analysis/Complex.v':'Two-coordinate Euclidean complex representation, constructors, ring arithmetic and algebraic laws. No independent native complex scalar implementation or branch convention inferred.',
 'formalization/Analysis/MultiLimit.v':'Consecutive displacement <=2^(-n-1) proves fast Cauchy; closed predicates and coherent M_paths preserve membership, with optional Q state invariant. Nearness alone cannot guarantee convergence or limit membership.',
 'formalization/Analysis/RealSubsets.v':'Open/closed and sequentially closed predicates, approximation witnesses, two-bit slack in closedness proof. Nonzero/positive sets are not closed under arbitrary limits.',
 'formalization/Real/RealRing.v':'Ring theory registration and linear recursive natural power, with raw CRLF retained. Power remains a runtime DAG/cost hypothesis, not a benchmarked change.',
 'extracted-examples/src/CSqrt.hs':'Entire generated module, integer/vector helpers and proof-erasure boilerplate. Recomputed foldl prefixes and coordinatewise limits; zero/root Bool state commits to a root once found. Generic vector shapes and integer naturals retain erased-precondition hazards; no invalid-input defect is claimed.',
};
const records=Object.entries(notes).map(([path,note])=>{
 const f=inventory.files.find(x=>x.path===path);assert(f?.kind==='text');assert.equal(sha(inventory.root+'/'+path),f.sha256);
 const prior=previous.records.find(x=>x.path===path);if(prior)assert.deepEqual(prior.ranges,[[1,200]]);
 return{path,sha256:f.sha256,totalLines:f.lines,ranges:[[prior?201:1,f.lines]],completeAfter:true,previousRanges:prior?.ranges??[],note};
});
const cumulative=previous.records.map(r=>({...r}));
for(const r of records){const old=cumulative.find(x=>x.path===r.path);
 if(old){old.ranges=[[1,r.totalLines]];old.complete=true;old.note+=' Checkpoint 86 completes remaining lines; see delta record.';}
 else cumulative.push({path:r.path,sha256:r.sha256,totalLines:r.totalLines,ranges:[[1,r.totalLines]],complete:true,note:r.note});
}
cumulative.sort((a,b)=>a.path.localeCompare(b.path));
const newLines=records.reduce((n,r)=>n+r.ranges.reduce((s,[a,b])=>s+b-a+1,0),0);
const result={checkpoint:86,commit:inventory.commit,inventorySha256:sha('../inventory-v85.json'),previousReadRecordsSha256:sha('../read-records-v85.json'),records,cumulative,
 delta:{newComplete:records.filter(r=>r.previousRanges.length===0).length,completedPartial:records.filter(r=>r.previousRanges.length>0).length,newLines},
 totals:{complete:cumulative.filter(r=>r.complete).length,partial:cumulative.filter(r=>!r.complete).length,lines:cumulative.reduce((s,r)=>s+r.ranges.reduce((n,[a,b])=>n+b-a+1,0),0)}};
assert.deepEqual(result.delta,{newComplete:8,completedPartial:1,newLines:4454});assert.deepEqual(result.totals,{complete:30,partial:0,lines:9120});
if(process.argv.includes('--record'))writeFileSync('read-records-v86.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,JSON.parse(readFileSync('read-records-v86.json')));
console.log(JSON.stringify({status:process.argv.includes('--record')?'published':'verified',delta:result.delta,totals:result.totals}));
