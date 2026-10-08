import {readFileSync,statSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {coldBindings} from './point-cold-protocol.mjs';
import {sha,json} from './point-demand-sources.mjs';
import {config as nativeConfig,variants,orderFor,summarizer} from './point-demand-cost-protocol.mjs';
import {groups,groupKey,frame,lines,iterationsFor} from './point-history-protocol.mjs';
export {variants,groups,groupKey,frame,iterationsFor,orderFor,summarizer};
export const flags=['--expose-gc','--no-liftoff','--no-wasm-tier-up','--no-wasm-lazy-compilation'];
export const config={...nativeConfig,platform:'wasm32-unknown-unknown',flags,qualificationBatchSizes:[1,8],
 gcEvery:8,clock:'process.hrtime.bigint around one history_batch call',instance:'fresh per observation'};
export function wasmBindings(){
 coldBindings();const o=json('point-wasm-origin.json'),b=json('point-wasm-binaries.json');
 assert.equal(o.predecessorSha256,sha('point-cold-manifest.json'));assert.equal(o.sourceBindingSha256,sha('point-demand-source-binding.json'));
 assert.equal(b.originSha256,sha('point-wasm-origin.json'));assert.equal(b.root,o.root);assert.equal(b.artifacts.length,3);
 for(const[p,h]of Object.entries(b.harnessSources))assert.equal(sha(p),h,p);
 const native=readFileSync('point-history-work.rs','utf8');
 assert.equal(readFileSync('point-wasm-work.rs','utf8'),native.slice(native.indexOf('struct HistoryWork {'),native.indexOf('fn benchmark_main(')).trimEnd()+'\n');
 for(const v of variants){const matches=b.artifacts.filter(a=>a.variant===v);assert.equal(matches.length,1);const a=matches[0];
  assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);
  for(const gate of ['lock','build','clippy']){const g=json('results/point-wasm-'+v+'-'+gate+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);}
 }
 return b;
}
export function nativeExpected(){
 const expected=new Map();
 for(const row of lines('results/point-history-qualification.jsonl').filter(r=>r.mode==='cpu'))
  expected.set((row.variant==='candidate'?'eager':'baseline')+':'+groupKey(row),frame(row));
 for(const row of lines('results/point-demand-history.jsonl').filter(r=>r.mode==='cpu'))expected.set('demand:'+groupKey(row),frame(row));
 assert.equal(expected.size,2304);return expected;
}
export function validateWasmRecord(row,expected){
 for(const[k,v]of Object.entries(expected))assert.equal(row[k],v,k);
 assert.equal(row.platform,'wasm');assert.equal(row.mode,'cpu');
 assert(Number.isSafeInteger(row.elapsed_ns)&&row.elapsed_ns>0);assert(Number.isSafeInteger(row.iterations)&&row.iterations>0);
 assert.equal(row.final_matches_preconditioned,true);assert.equal(row.input_records_unchanged,true);
 assert.equal(row.checksum,row.iterations*(row.actual.root?.polynomial.length??1));assert.equal(row.host_checksum,row.checksum);
 assert(row.initialMemory>0&&row.memoryAfterPrepare>=row.initialMemory&&row.memoryAfterBatch>=row.memoryAfterPrepare&&row.memoryAfterFinish>=row.memoryAfterBatch);
 assert(Number.isSafeInteger(row.outputBytes)&&row.outputBytes>0);assert(!('requests'in row));
}
export async function wasmRunner(binding,failurePath){
 for(const flag of flags)assert(process.execArgv.includes(flag),flag);assert.equal(typeof global.gc,'function');
 const affinity=readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1];assert.equal(affinity,'6');
 const modules={},metadata=[];let count=0,gcs=0;
 for(const a of binding.artifacts){
  const module=await WebAssembly.compile(readFileSync(a.path));const imports=WebAssembly.Module.imports(module),exports=WebAssembly.Module.exports(module);
  assert.deepEqual(imports,[]);assert.deepEqual(exports.map(e=>e.name).sort(),['memory','history_prepare','history_batch','history_finish','history_output_ptr'].sort());
  modules[a.variant]=module;metadata.push({...a,imports,exports});
 }
 const compiled=new Date().toISOString();
 function observe(variant,group,iterations){
  const started=new Date().toISOString();let phase='instantiate',raw;
  try{
   const e=new WebAssembly.Instance(modules[variant]).exports;assert(e.memory instanceof WebAssembly.Memory);
   const initialMemory=e.memory.buffer.byteLength;phase='prepare';
   e.history_prepare(group.case,group.policy,group.history,group.lifecycle==='fresh'?1:0);
   const memoryAfterPrepare=e.memory.buffer.byteLength;phase='batch';
   const begin=process.hrtime.bigint();
   const checksum=e.history_batch(iterations);
   const elapsed=process.hrtime.bigint()-begin;
   // The final report remains alive. No output work, instance creation, GC or
   // full-value checking occurs between the two host clock reads.
   const memoryAfterBatch=e.memory.buffer.byteLength;phase='finish';
   const length=e.history_finish(),pointer=e.history_output_ptr();
   assert(length>0&&length<32*1024*1024&&pointer>0&&pointer+length<=e.memory.buffer.byteLength);
   raw=Buffer.from(new Uint8Array(e.memory.buffer,pointer,length)).toString('utf8');phase='decode';
   return{...JSON.parse(raw),variant,platform:'wasm',mode:'cpu',started,finished:new Date().toISOString(),elapsed_ns:Number(elapsed),
    host_checksum:checksum,initialMemory,memoryAfterPrepare,memoryAfterBatch,memoryAfterFinish:e.memory.buffer.byteLength,outputBytes:length};
  }catch(error){appendFileSync(failurePath,JSON.stringify({variant,group,iterations,started,finished:new Date().toISOString(),phase,
   error:String(error),stack:error.stack,raw})+'\n');throw error;}
 }
 return{observe,completed(){count++;if(count%config.gcEvery===0){global.gc();gcs++;}},
  runtime(){return{node:process.version,v8:process.versions.v8,execArgv:process.execArgv,affinity,modules:metadata,compiled,
   observations:count,gcs,gcEvery:config.gcEvery,limits:'Optimizing V8 compiler selected before module compilation; import-free fresh instance per observation. Host boundary and uncontended state bookkeeping are timed. No module compile, instance/setup/JSON/verification/host GC is timed. Linear memory samples are not per-query allocation/peak/RSS metrics.'};}};
}
