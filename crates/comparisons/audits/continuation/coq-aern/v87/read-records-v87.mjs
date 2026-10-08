import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const inv=JSON.parse(readFileSync('../inventory-v85.json')),old=JSON.parse(readFileSync('../v86/read-records-v86.json'));
assert.deepEqual(old.totals,{complete:30,partial:0,lines:9120});
const notes={
 'formalization/Real/RealMetric.v':'Complete scalar absolute value, distance, sign/triangle/scaling, convexity and interval-distance equivalence. Continuous abs uses overlapping selection and two-bit slack; classical sign case analysis proves properties, not an executable exact sign oracle.',
 'formalization/Analysis/Euclidean.v':'Complete dimension-indexed vectors and maximum-coordinate metric, coordinatewise fast limits, uniqueness, closed predicates and coherent multivalued paths with state. This is not the Euclidean-length metric; dimensions/shapes are proof guarded before extraction.',
 'extracted-examples/bench/bench.hs':'Complete hand/extracted/native benchmark dispatcher, helpers and commented complex cases. Hand sqrt2 uses 2^-n, unlike proved/extracted 2^(-2n-1), permitting an uncertified zero approximation. Actual Haskell wrong output not yet reproduced. All complex dispatch cases are commented out.',
 'extracted-examples/bench/runBench.sh':'Complete runner. Enabled csqrt3/csqrt5 names have no active Haskell dispatcher. Repeated fixed parameters overwrite logs; failures remove their log; 0.00 CPU readings become 0.01; target accuracy column is blank. No native timing qualification inferred from historical CSVs.',
 'extracted-examples/bench/package.yaml':'Complete dependency/optimization/threaded-runtime declaration and commented Criterion setup. -N chooses available capabilities; measurements would need controlled threading and independent accuracy checks.',
 'extracted-examples/bench/coq-aern-extracted-bench.cabal':'Complete generated executable manifest agrees with YAML, uses AERN >=0.2.15.1 and all-capabilities threaded RTS. Not proof of a successful installed dependency graph.',
};
const records=Object.entries(notes).map(([path,note])=>{
 const f=inv.files.find(x=>x.path===path);assert.equal(f.kind,'text');assert(!old.cumulative.some(r=>r.path===path));
 assert.equal(sha(inv.root+'/'+path),f.sha256);
 return{path,sha256:f.sha256,totalLines:f.lines,ranges:[[1,f.lines]],complete:true,note};
});
const cumulative=[...old.cumulative,...records].sort((a,b)=>a.path.localeCompare(b.path));
const delta={newComplete:records.length,newLines:records.reduce((n,r)=>n+r.totalLines,0)};
const totals={complete:cumulative.length,partial:0,lines:old.totals.lines+delta.newLines};
assert.deepEqual(delta,{newComplete:6,newLines:2864});assert.deepEqual(totals,{complete:36,partial:0,lines:11984});
const result={checkpoint:87,commit:inv.commit,inventorySha256:sha('../inventory-v85.json'),previousReadRecordsSha256:sha('../v86/read-records-v86.json'),records,cumulative,delta,totals};
if(process.argv.includes('--record'))writeFileSync('read-records-v87.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,JSON.parse(readFileSync('read-records-v87.json')));
console.log(JSON.stringify({status:process.argv.includes('--record')?'published':'verified',delta,totals}));
