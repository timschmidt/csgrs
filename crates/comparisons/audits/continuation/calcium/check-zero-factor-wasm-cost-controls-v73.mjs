import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {validateRow,validateRows,readRows,key} from './zero-factor-wasm-cost-protocol-v73.mjs';
import {json,sha} from './zero-factor-wasm-cost-sources-v73.mjs';
const changes=[['signed-coefficient',r=>{r.expected.root.polynomial[0]='-2/1';}],
 ['endpoint',r=>r.expected.root.lower='0/1'],['status',r=>r.expected.status='Undecided'],
 ['checksum',r=>r.checksum++],['host-checksum',r=>r.host_checksum++],['iterations',r=>r.iterations++],
 ['zero-duration',r=>r.elapsed_ns=0],['backward-clock',r=>r.finished='1970-01-01T00:00:00.000Z'],
 ['fractional-memory',r=>r.memoryAfterFinish+=0.5],['decreasing-memory',r=>r.memoryAfterBatch=0],
 ['instrumentation',r=>r.requests=0],['qualification-only-duration',r=>r.qualification_batch_ns=r.elapsed_ns],
 ['source-changed',r=>r.sources_unchanged=false],['result-changed',r=>r.final_matches=false],
 ['wrong-policy',r=>r.policy=1],['wrong-variant',r=>r.variant='candidate'],
 ['wrong-instance',r=>r.instanceMode=r.instanceMode==='fresh'?'persistent':'fresh'],['wrong-sequence',r=>r.sequence++]];
const results=[],streamResults=[];
for(const instanceMode of ['persistent','fresh']){
 const path='results/zero-factor-wasm-cost-cpu-0-'+instanceMode+'-baseline-v73.stdout',planPath='zero-factor-wasm-cost-cpu-'+instanceMode+'-0-plan-v73.json',plan=json(planPath),rows=readRows(path);
 const index=plan.findIndex(g=>g.case===23&&g.policy===0&&g.lifecycle==='retained');assert(index>=0);
 const group=plan[index],original=rows[index];assert.equal(key(original),key(group));assert.equal(original.expected.root.polynomial[0],'2/1');
 for(const [name,alter]of [['valid',()=>{}],...changes]){
  const row=structuredClone(original);alter(row);let error=null;
  try{validateRow(row,group,'baseline',instanceMode,index);}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error===null,name==='valid');results.push({instanceMode,name,row,group,index,accepted:error===null,error});
 }
 // Two-row authored excerpts test membership/terminal rejection without copying
 // sixteen complete 456-row logs. Completion counts are adjusted explicitly;
 // these fixtures are not represented as separately executed short campaigns.
 const shortPlan=plan.slice(0,2),shortPlanPath='zero-factor-wasm-cost-control-'+instanceMode+'-plan-v73.json';
 writeFileSync(shortPlanPath,JSON.stringify(shortPlan)+'\n',{flag:'wx'});
 const shortRows=[...rows.slice(0,2),{...rows.at(-1),groups:2,gcs:0,planSha256:sha(shortPlanPath)}];
 for(const name of ['valid','drop-row','duplicate-row','swapped-rows','missing-terminal','wrong-terminal-count','wrong-terminal-gc','wrong-terminal-runtime']){
  const altered=structuredClone(shortRows);
  if(name==='drop-row')altered.splice(0,1);
  if(name==='duplicate-row')altered.splice(0,0,structuredClone(altered[0]));
  if(name==='swapped-rows')[altered[0],altered[1]]=[altered[1],altered[0]];
  if(name==='missing-terminal')altered.pop();
  if(name==='wrong-terminal-count')altered.at(-1).groups++;
  if(name==='wrong-terminal-gc')altered.at(-1).gcs++;
  if(name==='wrong-terminal-runtime')altered.at(-1).node='wrong';
  const fixture='zero-factor-wasm-cost-control-'+instanceMode+'-'+name+'-v73.jsonl';
  writeFileSync(fixture,altered.map(JSON.stringify).join('\n')+'\n',{flag:'wx'});
  let error=null;try{validateRows(fixture,shortPlan,'baseline',instanceMode);}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error===null,name==='valid');streamResults.push({instanceMode,name,path:fixture,sha256:sha(fixture),plan:shortPlanPath,
   sourceCapture:path,sourceCaptureSha256:sha(path),authoredExcerpt:true,accepted:error===null,error});
 }
}
const result={checkpoint:73,status:'wasm-cost-controls-pass',recordControls:{accepted:2,rejected:36},streamControls:{accepted:2,rejected:14},
 scripts:Object.fromEntries(['check-zero-factor-wasm-cost-controls-v73.mjs','zero-factor-wasm-cost-protocol-v73.mjs'].map(p=>[p,sha(p)])),results,streamResults};
writeFileSync('zero-factor-wasm-cost-controls-v73.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:73,status:result.status,recordControls:result.recordControls,streamControls:result.streamControls}));
