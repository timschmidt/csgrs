import {readFileSync,writeFileSync,appendFileSync} from 'node:fs';
import {once} from 'node:events';
import assert from 'node:assert/strict';
import {wasmSources,sha,json} from './zero-factor-wasm-sources-v72.mjs';
import {compileModules,rationalInstance,historyObservation,commonRecord} from './zero-factor-wasm-runner-v72.mjs';
import {fullCases,fullReference} from './zero-factor-wasm-input-v72.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {reference} from './zero-factor-cost-protocol-v71.mjs';
import {checkZeroFactor} from './check-zero-factor-v70.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
const failure='results/zero-factor-wasm-failures-v72.jsonl';writeFileSync(failure,'',{flag:'wx'});
const source=wasmSources();assert.deepEqual(source,json('zero-factor-wasm-origin-v72.json').source);
const build=json('results/zero-factor-wasm-build-v72.json');assert.equal(build.code,0);assert.equal(build.signal,null);
const files=['qualify-zero-factor-wasm-v72.mjs','zero-factor-wasm-runner-v72.mjs'];
const origin={checkpoint:72,recorded:new Date().toISOString(),source,binariesSha256:sha('zero-factor-wasm-binaries-v72.json'),
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),gcEvery:8,limits:'Qualification only; batch elapsed values are not paired benchmarks.'};
writeFileSync('zero-factor-wasm-qualification-origin-v72.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
let phase='compile',context=null,count=0,gcs=0;
const emit=async row=>{if(!process.stdout.write(JSON.stringify(row)+'\n'))await once(process.stdout,'drain');};
const gc=()=>{if(++count%8===0){global.gc();gcs++;}};
try{
 const runtime=await compileModules();assert.equal(checkZeroFactor().status,'pass');fieldSelfTest();
 const full=fullCases(),input=readFileSync('zero-factor-wasm-input-v72.json');assert.deepEqual(JSON.parse(input),full);
 const statuses={},counts={full:0,cost:0,history:0};let extendedChecks=0;
 phase='full-rational';let instances=Object.fromEntries(['baseline','candidate'].map(v=>[v,rationalInstance(runtime.modules[v+':rational'],input,full.length)]));
 for(const c of full)for(let policy=0;policy<2;policy++)for(const variant of ['baseline','candidate']){
  const group={case:c.id,policy,lifecycle:'retained'};context={phase,variant,...group,iterations:1};
  const row=instances[variant].observe(group,1);commonRecord(row,group,1);assert.deepEqual(row.expected,fullReference(c,policy,variant));
  const key=variant+':'+row.expected.status;statuses[key]=(statuses[key]??0)+1;counts.full++;
  await emit({...row,variant,corpus:'full',instanceMode:'persistent'});gc();
 }
 instances=null;global.gc();gcs++;
 phase='cost-fresh';const costs=costCases(),costInput=readFileSync('zero-factor-cost-input-v71.json');assert.deepEqual(JSON.parse(costInput),costs);
 for(const c of costs)for(let policy=0;policy<2;policy++)for(const lifecycle of ['retained','fresh'])for(const iterations of [1,8])for(const variant of ['baseline','candidate']){
  const group={case:c.id,policy,lifecycle};context={phase,variant,...group,iterations};
  let instance=rationalInstance(runtime.modules[variant+':rational'],costInput,costs.length);
  const row=instance.observe(group,iterations);instance=null;
  commonRecord(row,group,iterations);assert.equal(row.sequence,0);assert.deepEqual(row.expected,reference(c,policy,variant));counts.cost++;
  await emit({...row,variant,corpus:'cost',instanceMode:'fresh'});gc();
 }
 phase='history-fresh';const native=jsonLines('results/power-rebased-baseline-extended-v68.stdout').filter(r=>r.type==='query');assert.equal(native.length,384);
 assert.equal(sha('results/power-rebased-baseline-extended-v68.stdout'),sha('results/zero-factor-extended-run-v70.stdout'));
 for(let which=0;which<48;which++)for(let policy=0;policy<2;policy++)for(let history=0;history<4;history++)for(const lifecycle of ['retained','fresh'])for(const iterations of [1,8])for(const variant of ['baseline','candidate']){
  const group={case:which,policy,history,lifecycle};context={phase,variant,...group,iterations};
  const row=historyObservation(runtime.modules[variant+':history'],group,iterations);commonRecord(row,group,iterations);
  const expected=native[(which*2+policy)*4+history];assert.deepEqual(row.left,expected.left);assert.deepEqual(row.right,expected.right);assert.deepEqual(row.actual,expected.report);
  const checked=validateExtendedObservation({type:'query',case:which,policy,history,left:row.left,right:row.right,report:row.actual});
  assert.deepEqual(checked.failures,[]);extendedChecks+=Object.values(checked.checks).reduce((a,b)=>a+b,0);counts.history++;
  await emit({...row,variant,corpus:'history',instanceMode:'fresh'});gc();
 }
 assert.deepEqual(counts,{full:33384,cost:1824,history:3072});assert.equal(extendedChecks,89984);
 assert.deepEqual(wasmSources(),source);for(const[p,h]of Object.entries(origin.files))assert.equal(sha(p),h,p);
 const {modules,...metadata}=runtime;
 const result={checkpoint:72,status:'wasm-full-record-qualified',counts,statuses,extendedChecks,runtime:metadata,gcs,observations:count,
  sourceOriginSha256:sha('zero-factor-wasm-qualification-origin-v72.json'),finished:new Date().toISOString(),
  limits:'Persistent rational corpus plus fresh cost/history instances. Qualification timings are not a cost campaign; memory samples are linear-memory sizes, not allocation/peak/RSS. No live edit or retention.'};
 writeFileSync('zero-factor-wasm-qualification-v72.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});await emit({terminal:true,...result});
}catch(error){appendFileSync(failure,JSON.stringify({checkpoint:72,phase,context,failed:new Date().toISOString(),error:String(error),stack:error.stack})+'\n');throw error;}
function jsonLines(p){return readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);}
