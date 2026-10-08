import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './zero-factor-wasm-sources-v72.mjs';
export const flags=['--expose-gc','--no-liftoff','--no-wasm-tier-up','--no-wasm-lazy-compilation'];
export async function compileModules(){
 for(const f of flags)assert(process.execArgv.includes(f));assert.equal(typeof global.gc,'function');
 assert.equal(readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.+)$/m)?.[1],'2');
 const binary=json('zero-factor-wasm-binaries-v72.json'),modules={},metadata=[];
 for(const a of binary.artifacts){
  assert.equal(sha(a.path),a.sha256);const module=await WebAssembly.compile(readFileSync(a.path));
  const imports=WebAssembly.Module.imports(module),exports=WebAssembly.Module.exports(module);
  assert.deepEqual(imports,[]);
  const names=a.kind==='rational'?['memory','zf_input','zf_init','zf_prepare','zf_batch','zf_finish','zf_output_ptr']:
   ['memory','history_prepare','history_batch','history_finish','history_output_ptr'];
  assert.deepEqual(exports.map(e=>e.name).sort(),names.sort());
  modules[a.variant+':'+a.kind]=module;metadata.push({...a,imports,exports});
 }
 return{modules,metadata,compiled:new Date().toISOString(),node:process.version,v8:process.versions.v8,flags:process.execArgv,affinity:'2'};
}
function decode(e,length,pointer){
 assert(Number.isSafeInteger(length)&&length>0&&length<32*1024*1024);
 assert(Number.isSafeInteger(pointer)&&pointer>0&&pointer+length<=e.memory.buffer.byteLength);
 return JSON.parse(Buffer.from(new Uint8Array(e.memory.buffer,pointer,length)).toString('utf8'));
}
export function rationalInstance(module,input,cases){
 const e=new WebAssembly.Instance(module).exports;assert(e.memory instanceof WebAssembly.Memory);
 const initialMemory=e.memory.buffer.byteLength,pointer=e.zf_input(input.length);
 assert(pointer>0&&pointer+input.length<=e.memory.buffer.byteLength);
 new Uint8Array(e.memory.buffer,pointer,input.length).set(input);assert.equal(e.zf_init(),cases);
 const initializedMemory=e.memory.buffer.byteLength;let sequence=0;
 return{exports:e,observe(group,iterations){
  const started=new Date().toISOString(),memoryBeforePrepare=e.memory.buffer.byteLength;
  e.zf_prepare(group.case,group.policy,group.lifecycle==='fresh'?1:0);
  const memoryAfterPrepare=e.memory.buffer.byteLength,begin=process.hrtime.bigint(),checksum=e.zf_batch(iterations),elapsed=process.hrtime.bigint()-begin;
  const memoryAfterBatch=e.memory.buffer.byteLength,length=e.zf_finish(),pointer=e.zf_output_ptr(),row=decode(e,length,pointer);
  return{...row,platform:'wasm',kind:'rational',mode:'qualification',sequence:sequence++,started,finished:new Date().toISOString(),
   qualification_batch_ns:Number(elapsed),host_checksum:checksum,initialMemory,initializedMemory,memoryBeforePrepare,memoryAfterPrepare,
   memoryAfterBatch,memoryAfterFinish:e.memory.buffer.byteLength,outputBytes:length};
 }};
}
export function historyObservation(module,group,iterations){
 const started=new Date().toISOString(),e=new WebAssembly.Instance(module).exports;assert(e.memory instanceof WebAssembly.Memory);
 const initialMemory=e.memory.buffer.byteLength;
 e.history_prepare(group.case,group.policy,group.history,group.lifecycle==='fresh'?1:0);
 const memoryAfterPrepare=e.memory.buffer.byteLength,begin=process.hrtime.bigint(),checksum=e.history_batch(iterations),elapsed=process.hrtime.bigint()-begin;
 const memoryAfterBatch=e.memory.buffer.byteLength,length=e.history_finish(),pointer=e.history_output_ptr(),row=decode(e,length,pointer);
 return{...row,platform:'wasm',kind:'history',mode:'qualification',started,finished:new Date().toISOString(),qualification_batch_ns:Number(elapsed),
  host_checksum:checksum,initialMemory,memoryAfterPrepare,memoryAfterBatch,memoryAfterFinish:e.memory.buffer.byteLength,outputBytes:length};
}
export function commonRecord(row,group,iterations){
 for(const[k,v]of Object.entries(group))assert.equal(row[k],v);assert.equal(row.iterations,iterations);
 assert.equal(row.platform,'wasm');assert.equal(row.mode,'qualification');
 assert(Number.isSafeInteger(row.qualification_batch_ns)&&row.qualification_batch_ns>0);
 assert(Date.parse(row.finished)>=Date.parse(row.started));
 assert.equal(row.host_checksum,row.checksum);assert.equal(row.checksum,iterations*((row.expected??row.actual).root?.polynomial.length??1));
 assert(row.initialMemory>0&&row.memoryAfterPrepare>=row.initialMemory&&row.memoryAfterBatch>=row.memoryAfterPrepare&&row.memoryAfterFinish>=row.memoryAfterBatch);
 assert(!('requests'in row));assert(Number.isSafeInteger(row.outputBytes)&&row.outputBytes>0);
 if(row.kind==='rational'){
  assert.equal(row.final_matches,true);assert.equal(row.sources_unchanged,true);
  assert(row.initializedMemory>=row.initialMemory&&row.memoryBeforePrepare>=row.initializedMemory&&row.memoryAfterPrepare>=row.memoryBeforePrepare);
 }else{assert.equal(row.kind,'history');assert.equal(row.final_matches_preconditioned,true);assert.equal(row.input_records_unchanged,true);}
}
