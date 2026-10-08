import {createReadStream,readFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {commonRecord} from './zero-factor-wasm-runner-v72.mjs';
import {fullCases,fullReference} from './zero-factor-wasm-input-v72.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {reference} from './zero-factor-cost-protocol-v71.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
import {checkZeroFactor} from './check-zero-factor-v70.mjs';
import {sha,json} from './zero-factor-wasm-sources-v72.mjs';
export function* observationPlan(){
 for(let which=0;which<8346;which++)for(let policy=0;policy<2;policy++)for(const variant of ['baseline','candidate'])
  yield {corpus:'full',variant,case:which,policy,lifecycle:'retained',iterations:1};
 for(let which=0;which<114;which++)for(let policy=0;policy<2;policy++)for(const lifecycle of ['retained','fresh'])for(const iterations of [1,8])for(const variant of ['baseline','candidate'])
  yield {corpus:'cost',variant,case:which,policy,lifecycle,iterations};
 for(let which=0;which<48;which++)for(let policy=0;policy<2;policy++)for(let history=0;history<4;history++)for(const lifecycle of ['retained','fresh'])for(const iterations of [1,8])for(const variant of ['baseline','candidate'])
  yield {corpus:'history',variant,case:which,policy,history,lifecycle,iterations};
}
export function recordValidator(){
 const full=fullCases(),cost=costCases(),history=readFileSync('results/power-rebased-baseline-extended-v68.stdout','utf8').trimEnd().split('\n').map(JSON.parse).filter(r=>r.type==='query');
 assert.deepEqual(json('zero-factor-wasm-input-v72.json'),full);assert.deepEqual(json('zero-factor-cost-input-v71.json'),cost);
 assert.equal(history.length,384);assert.equal(sha('results/power-rebased-baseline-extended-v68.stdout'),sha('results/zero-factor-extended-run-v70.stdout'));
 fieldSelfTest();
 return (row,spec)=>{
  for(const[k,v]of Object.entries(spec))assert.equal(row[k],v,k);
  const {corpus,variant,iterations,...group}=spec;commonRecord(row,group,iterations);
  assert.equal(row.instanceMode,corpus==='full'?'persistent':'fresh');
  assert.equal(row.kind,corpus==='history'?'history':'rational');
  for(const name of ['initialMemory','memoryAfterPrepare','memoryAfterBatch','memoryAfterFinish'])
   assert(Number.isSafeInteger(row[name])&&row[name]>0&&row[name]%65536===0,name);
  if(corpus!=='history'){
   for(const name of ['initializedMemory','memoryBeforePrepare'])assert(Number.isSafeInteger(row[name])&&row[name]>0&&row[name]%65536===0,name);
   assert.equal(row.sequence,corpus==='full'?group.case*2+group.policy:0);
   const expected=corpus==='full'?fullReference(full[group.case],group.policy,variant):reference(cost[group.case],group.policy,variant);
   assert.deepEqual(row.expected,expected);return 0;
  }
  const native=history[(group.case*2+group.policy)*4+group.history];
  assert.deepEqual(row.left,native.left);assert.deepEqual(row.right,native.right);assert.deepEqual(row.actual,native.report);
  const checked=validateExtendedObservation({type:'query',...group,left:row.left,right:row.right,report:row.actual});assert.deepEqual(checked.failures,[]);
  return Object.values(checked.checks).reduce((a,b)=>a+b,0);
 };
}
export async function recordEvidence(){
 assert.equal(checkZeroFactor().status,'pass');
 const validate=recordValidator(),plan=observationPlan(),result=json('zero-factor-wasm-qualification-v72.json'),gate=json('results/zero-factor-wasm-qualification-v72.json');
 const counts={full:0,cost:0,history:0},statuses={},memory={},previousMemory={};
 let observations=0,extendedChecks=0,terminal=false,previousTime=Date.parse(gate.started),newAnswers=0,equalAnswers=0,baseline=null;
 const lines=createInterface({input:createReadStream('results/zero-factor-wasm-qualification-v72.stdout'),crlfDelay:Infinity});
 for await(const line of lines){
  assert(line.length>0);assert(!terminal);const row=JSON.parse(line),next=plan.next();
  if(next.done){assert.deepEqual(row,{terminal:true,...result});terminal=true;continue;}
  extendedChecks+=validate(row,next.value);observations++;counts[row.corpus]++;
  assert(Date.parse(row.started)>=previousTime);previousTime=Date.parse(row.finished);
  if(row.corpus==='full'){
   const key=row.variant+':'+row.expected.status;statuses[key]=(statuses[key]??0)+1;
   const before=previousMemory[row.variant];
   if(before){assert.equal(row.initialMemory,before.initialMemory);assert.equal(row.initializedMemory,before.initializedMemory);assert(row.memoryBeforePrepare>=before.memoryAfterFinish);}
   previousMemory[row.variant]=row;
   if(row.variant==='baseline')baseline=row;
   else{assert(baseline&&baseline.case===row.case&&baseline.policy===row.policy);
    if(JSON.stringify(baseline.expected)===JSON.stringify(row.expected))equalAnswers++;
    else{assert(['Undecided','UnsupportedDegree'].includes(baseline.expected.status));assert.equal(row.expected.status,'Transformed');newAnswers++;}
    baseline=null;
   }
  }
  const m=memory[row.corpus+':'+row.variant]??={observations:0,fields:{}};m.observations++;
  for(const name of ['initialMemory','initializedMemory','memoryBeforePrepare','memoryAfterPrepare','memoryAfterBatch','memoryAfterFinish'])if(name in row){
   const f=m.fields[name]??={min:row[name],max:row[name]};f.min=Math.min(f.min,row[name]);f.max=Math.max(f.max,row[name]);
  }
 }
 assert(terminal);assert.equal(baseline,null);assert.equal(observations,38280);assert.equal(extendedChecks,89984);
 assert.deepEqual(counts,result.counts);assert.deepEqual(statuses,result.statuses);assert.equal(result.observations,observations);
 assert.equal(result.extendedChecks,extendedChecks);assert.equal(result.gcs,Math.floor(observations/8)+1);
 assert.equal(newAnswers,184);assert.equal(equalAnswers,16508);
 assert(Date.parse(result.finished)>=previousTime&&Date.parse(gate.finished)>=Date.parse(result.finished));
 assert.equal(readFileSync('results/zero-factor-wasm-failures-v72.jsonl').length,0);
 assert.equal(result.sourceOriginSha256,sha('zero-factor-wasm-qualification-origin-v72.json'));
 return {checkpoint:72,status:'wasm-record-replay-pass',observations,counts,extendedChecks,statuses,newAnswers,equalAnswers,
  memory,limits:'All saved full records and authored order replayed; memory values are WASM linear-memory capacity, not allocation, peak or RSS. No timing inference.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(await recordEvidence()));
