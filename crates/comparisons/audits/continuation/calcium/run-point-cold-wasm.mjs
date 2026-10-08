import {readFileSync,openSync,closeSync} from 'node:fs';
import assert from 'node:assert/strict';
import {variants,config,groups,groupKey,argsFor,bytesSha,writeAll,parseGroup,coldBindings,metadataPath} from './point-cold-protocol.mjs';
const variant=process.argv[2];assert(variants.includes(variant));assert.equal(typeof global.gc,'function');
const b=coldBindings(),a=b.artifacts.find(a=>a.variant===variant&&a.platform==='wasm');
const module=await WebAssembly.compile(readFileSync(a.path));
const imports=WebAssembly.Module.imports(module),exports=WebAssembly.Module.exports(module);assert.deepEqual(imports,[]);
const fd=openSync(metadataPath('wasm',variant),'wx');let count=0,outputBytes=0,minMemory=Infinity,maxMemory=0,gcs=0;
function collect(g){
 const started=new Date().toISOString(),instance=new WebAssembly.Instance(module),e=instance.exports;
 assert(e.memory instanceof WebAssembly.Memory);assert.equal(typeof e.cold_collect,'function');assert.equal(typeof e.cold_output_ptr,'function');
 const initialMemory=e.memory.buffer.byteLength,args=argsFor(g),length=e.cold_collect(...args),pointer=e.cold_output_ptr();
 assert(Number.isSafeInteger(length)&&length>0&&length<32*1024*1024);
 assert(Number.isSafeInteger(pointer)&&pointer>0&&pointer+length<=e.memory.buffer.byteLength);
 const out=Buffer.from(new Uint8Array(e.memory.buffer,pointer,length));writeAll(1,out);
 const memory=e.memory.buffer.byteLength;
 const meta={group:groupKey(g),args,started,finished:new Date().toISOString(),
  outputBytes:out.length,sha256:bytesSha(out),initialMemory,memoryAfterCollection:memory};
 writeAll(fd,Buffer.from(JSON.stringify(meta)+'\n'));parseGroup(out,g);return{length,memory};
}
try{
 for(const g of groups()){
  const result=collect(g);count++;outputBytes+=result.length;
  minMemory=Math.min(minMemory,result.memory);maxMemory=Math.max(maxMemory,result.memory);
  if(count%config.wasmGcEvery===0){global.gc();gcs++;}
 }
}finally{closeSync(fd);}
assert.equal(count,config.groups);assert.deepEqual(coldBindings(),b);
writeAll(2,Buffer.from(JSON.stringify({variant,platform:'wasm',node:process.version,v8:process.versions.v8,
 imports,exports,moduleBytes:a.bytes,groups:count,freshInstances:count,queries:count*9,outputBytes,
 minMemoryAfterCollection:minMemory,maxMemoryAfterCollection:maxMemory,gcs,gcEvery:config.wasmGcEvery,
 binarySha256:a.sha256,
 limits:'One compiled module and Node process per variant, fresh import-free instance/linear memory per group. Nine fully recorded queries, no unrecorded query warmup. Constructors and initial histories are setup. Linear memory after collection is not per-query peak, RSS, allocator demand or a CPU measurement.'})+'\n'));
