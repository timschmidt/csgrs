import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {isDeepStrictEqual} from 'node:util';
import {sha} from './point-qualified-sources.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
import {config,variants,modes,groups,groupKey,frame,checkBindings,validateRecord,observe,lines} from './point-history-protocol.mjs';
const binding=checkBindings(),started=new Date().toISOString();fieldSelfTest();
const path='results/point-history-qualification.jsonl',failuresPath='results/point-history-qualification-failures.jsonl';
writeFileSync(path,'',{flag:'wx'});writeFileSync(failuresPath,'',{flag:'wx'});
const checks={},statuses={},old=Object.fromEntries(variants.map(v=>[v,new Map(lines('results/point-extended-public-'+v+'.stdout')
 .filter(r=>r.type==='query').map(r=>[[r.case,r.policy,r.history].join(':'),r]))]));
let observations=0,improved=0,unchanged=0;const preconditioningDifferences=[];
for(const group of groups()){
 const records={};
 for(const variant of variants)for(const mode of modes){
  const iterations=config.qualificationIterations[mode];
  const row=await observe(binding,variant,mode,group,iterations,failuresPath);
  appendFileSync(path,JSON.stringify(row)+'\n');observations++;
  validateRecord(row,{...group,variant,mode,iterations});
  const value=frame(row),result=validateExtendedObservation(value);
  assert.deepEqual(result.failures,[],JSON.stringify({variant,mode,group,failures:result.failures}));
  for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
  const key=[variant,mode,row.actual.status].join(':');statuses[key]=(statuses[key]??0)+1;
  if(mode==='cpu')records[variant]=value;else assert.deepEqual(value,records[variant],'instrumented full result');
 }
 const a=records.baseline,b=records.candidate;
 assert.deepEqual(a.left,b.left);assert.deepEqual(a.right,b.right);
 if(a.report.status==='InvalidTransformedEvidence'){
  assert.equal(a.report.message,'a collapsed refinement interval requires an exact witness');
  assert.equal(b.report.status,'Transformed');assert.deepEqual({...b,report:a.report},a);improved++;
 }else{assert.deepEqual(b,a);unchanged++;}
 for(const v of variants){
  const prior=old[v].get([group.case,group.policy,group.history].join(':'));
  if(!isDeepStrictEqual(prior,records[v])){
   // This is a different lifecycle (fresh process + nine preconditioning calls),
   // so record genuine full-representation changes, never silently drop them.
   preconditioningDifferences.push({variant:v,...group,before:prior.report.status,after:records[v].report.status,
    inputChanged:JSON.stringify([prior.left,prior.right])!==JSON.stringify([records[v].left,records[v].right]),
    reportChanged:JSON.stringify(prior.report)!==JSON.stringify(records[v].report)});
  }
 }
 if(group.lifecycle==='fresh')console.log(JSON.stringify({qualified:groupKey(group),observations}));
}
assert.equal(observations,3072);assert.equal(improved+unchanged,768);checkBindings();
const result={status:'pass',started,finished:new Date().toISOString(),config,binariesSha256:sha('point-history-binaries.json'),
 observations,groups:768,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),statuses,improved,unchanged,
 preconditioningDifferences,rowsSha256:sha(path),failuresSha256:sha(failuresPath),
 limits:'Independent exact checks apply to the final actual complete result in each observation. CPU uses one measured iteration; allocation uses eight, with a polynomial-length checksum for intermediate results. Nine preconditioning calls, process-local caches, and explicit constructed/coarse/deep/round-trip input histories. The held input records are checked unchanged; fresh input graphs are rebuilt and dropped inside each batch iteration. Final returned report remains live at the snapshot. Not an independent proof of every intermediate iteration, universal performance, RSS, or WASM.'};
writeFileSync('point-history-qualification.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({status:result.status,observations,groups:768,totalChecks:result.totalChecks,improved,unchanged,preconditioningDifferences:preconditioningDifferences.length}));
