import {readFileSync,writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {checkedPlan,config,paths,flags,variants,orderFor,groupKey,sha,envelope,validateCounters} from './point-attribution-runtime-v65.mjs';
import {nativeExpected,frame,validateWasmRecord} from './point-wasm-protocol.mjs';
const pass=Number(process.argv[2]);assert(Number.isInteger(pass)&&pass>=0&&pass<4);
const registered=checkedPlan(),plan=registered.plan,setting=config.passes[pass],out=paths(pass),expected=nativeExpected();
assert.equal(process.version,'v22.22.2');assert.equal(typeof process.threadCpuUsage,'function');
assert.equal(typeof global.gc,'function');
assert.deepEqual(process.execArgv,[...flags,...(setting==='single'?['--single-threaded-gc']:[])]);
const affinity=readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1];assert.equal(affinity,'6');
for(const p of [out.raw,out.controls,out.failures])writeFileSync(p,'',{flag:'wx'});
const started=new Date().toISOString(),modules={},metadata=[];
for(const b of plan.binaries){
 const module=await WebAssembly.compile(readFileSync(b.path));assert.deepEqual(WebAssembly.Module.imports(module),[]);
 assert.deepEqual(WebAssembly.Module.exports(module).map(e=>e.name).sort(),['memory','history_prepare','history_batch','history_finish','history_output_ptr'].sort());
 modules[b.variant]=module;metadata.push(b);
}
let observations=0,gcs=0,qualification=0,measured=0;
function controls(end){for(let index=0;index<config.emptyControlsPerEnd;index++){
 const row={pass,setting,end,index,...envelope(()=>0)};
 appendFileSync(out.controls,JSON.stringify(row)+'\n');validateCounters(row,true);assert.equal(row.value,0);
}}
function observe(variant,group,iterations,phase,block,position){
 const begin=new Date().toISOString();let stage='instantiate',raw;
 try{
  const e=new WebAssembly.Instance(modules[variant]).exports;assert(e.memory instanceof WebAssembly.Memory);
  const initialMemory=e.memory.buffer.byteLength;stage='prepare';
  e.history_prepare(group.case,group.policy,group.history,group.lifecycle==='fresh'?1:0);
  const memoryAfterPrepare=e.memory.buffer.byteLength;stage='batch';
  // Same import-free history_batch and held final result as checkpoint 59.
  // CPU/resource readings enclose the inner wall clock; their overhead is
  // measured separately, never subtracted or described as batch-only CPU.
  const timing=envelope(()=>e.history_batch(iterations));
  const memoryAfterBatch=e.memory.buffer.byteLength;stage='finish';
  const length=e.history_finish(),pointer=e.history_output_ptr();
  assert(length>0&&length<32*1024*1024&&pointer>0&&pointer+length<=e.memory.buffer.byteLength);
  raw=Buffer.from(new Uint8Array(e.memory.buffer,pointer,length)).toString('utf8');stage='decode';
  const row={...JSON.parse(raw),variant,platform:'wasm',mode:'cpu',pass,setting,phase,block,position,
   observation:observations,gcsBefore:gcs,started:begin,finished:new Date().toISOString(),
   ...timing,host_checksum:timing.value,initialMemory,memoryAfterPrepare,memoryAfterBatch,
   memoryAfterFinish:e.memory.buffer.byteLength,outputBytes:length};
  appendFileSync(out.raw,JSON.stringify(row)+'\n');stage='validate';
  validateCounters(row);validateWasmRecord(row,{...group,variant,iterations});
  assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(group)));
  observations++;if(phase==='qualification')qualification++;else measured++;
  if(observations%config.gcEvery===0){global.gc();gcs++;}
 }catch(error){appendFileSync(out.failures,JSON.stringify({pass,setting,variant,group,iterations,phase,block,position,
  started:begin,finished:new Date().toISOString(),stage,error:String(error),stack:error.stack,raw})+'\n');throw error;}
}
controls('before');
for(const group of plan.groups)for(const variant of variants)for(const iterations of config.qualificationIterations)
 observe(variant,{case:group.case,policy:group.policy,history:group.history,lifecycle:group.lifecycle},iterations,'qualification',-1,-1);
for(let block=0;block<config.blocks;block++){
 for(const g of plan.groups){const{iterations,...group}=g;
  for(const[position,variant]of orderFor('cpu',block).entries())observe(variant,group,iterations,'measurement',block,position);
 }
 if(block%6===5)console.log(JSON.stringify({pass,setting,blocks:block+1,measured,finished:new Date().toISOString()}));
}
controls('after');assert.equal(qualification,48);assert.equal(measured,1728);assert.equal(gcs,222);
assert.deepEqual(checkedPlan(),registered);
const result={pass,setting,started,finished:new Date().toISOString(),status:'pass',qualification,measured,observations,gcs,
 planSha256:sha('point-attribution-revised-plan-v65.json'),files:Object.fromEntries([out.raw,out.controls,out.failures].map(p=>[p,sha(p)])),
 runtime:{node:process.version,v8:process.versions.v8,execArgv:process.execArgv,affinity,binaries:metadata},config};
writeFileSync(out.summary,JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({status:'pass',pass,setting,qualification,measured,gcs,finished:result.finished}));
