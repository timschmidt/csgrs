import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {demandBindings} from './point-demand-bindings.mjs';
import {config,modes,groups,groupKey,frame,lines,validateRecord,observe} from './point-history-protocol.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
const binding=demandBindings();assert.equal(json('results/point-demand-public-check.json').code,0);
const expected=new Map(lines('results/point-history-qualification.jsonl').filter(r=>r.variant==='candidate'&&r.mode==='cpu').map(r=>[groupKey(r),frame(r)]));
assert.equal(expected.size,768);
const path='results/point-demand-history.jsonl',failurePath='results/point-demand-history-failures.jsonl';
writeFileSync(path,'',{flag:'wx'});writeFileSync(failurePath,'',{flag:'wx'});
const started=new Date().toISOString(),checks={},statuses={};let observations=0;
for(const group of groups())for(const mode of modes){
 const iterations=config.qualificationIterations[mode];
 const row=await observe(binding,'demand',mode,group,iterations,failurePath);appendFileSync(path,JSON.stringify(row)+'\n');observations++;
 validateRecord(row,{variant:'demand',mode,...group,iterations});assert.deepEqual(frame(row),expected.get(groupKey(row)));
 const result=validateExtendedObservation(frame(row));assert.deepEqual(result.failures,[]);
 for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
 statuses[row.actual.status]=(statuses[row.actual.status]??0)+1;
 if(mode==='allocation'&&group.lifecycle==='fresh')console.log(JSON.stringify({group:groupKey(group),observations}));
}
demandBindings();const totalChecks=Object.values(checks).reduce((a,b)=>a+b,0);assert.equal(totalChecks,44992);assert.equal(observations,1536);
const result={status:'pass',started,finished:new Date().toISOString(),config,observations,groups:768,checks,totalChecks,statuses,
 binariesSha256:sha('point-demand-binaries.json'),rowsSha256:sha(path),failuresSha256:sha(failurePath),
 referenceSha256:sha('results/point-history-qualification.jsonl'),
 limits:'Final actual full report independently checked and identical to the eager guarded variant in every group/mode. Nine preconditioning calls and retained/fresh input histories match checkpoint 56. Intermediate iterations have only a polynomial-length checksum. Final returned report is retained at the allocation snapshot; not cold process constants, RSS, WASM or a full consumer qualification.'};
writeFileSync('point-demand-history.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify({status:'pass',observations,totalChecks,groups:768}));
