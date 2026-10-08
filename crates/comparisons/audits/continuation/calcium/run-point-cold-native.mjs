import {spawnSync} from 'node:child_process';
import {openSync,closeSync} from 'node:fs';
import assert from 'node:assert/strict';
import {variants,config,groups,groupKey,argsFor,bytesSha,writeAll,parseGroup,coldBindings,metadataPath} from './point-cold-protocol.mjs';
const variant=process.argv[2];assert(variants.includes(variant));
const b=coldBindings(),a=b.artifacts.find(a=>a.variant===variant&&a.platform==='native');
const fd=openSync(metadataPath('native',variant),'wx');let count=0,outputBytes=0;
try{
 for(const g of groups()){
  const args=argsFor(g).map(String),started=new Date().toISOString();
  const result=spawnSync(a.path,args,{maxBuffer:32*1024*1024,timeout:120000});
  const out=result.stdout??Buffer.alloc(0),err=result.stderr??Buffer.alloc(0);
  writeAll(1,out);writeAll(2,err);outputBytes+=out.length;
  writeAll(fd,Buffer.from(JSON.stringify({group:groupKey(g),args,started,finished:new Date().toISOString(),
   pid:result.pid,code:result.status,signal:result.signal,error:result.error?String(result.error):null,
   outputBytes:out.length,sha256:bytesSha(out),stderrBytes:err.length})+'\n'));
  assert(!result.error);assert.equal(result.status,0);assert.equal(result.signal,null);assert.equal(err.length,0);
  parseGroup(out,g);count++;
 }
}finally{closeSync(fd);}
assert.equal(count,config.groups);assert.deepEqual(coldBindings(),b);
writeAll(2,Buffer.from(JSON.stringify({variant,platform:'native',node:process.version,v8:process.versions.v8,
 groups:count,freshProcesses:count,queries:count*9,outputBytes,binarySha256:a.sha256,
 limits:'One fresh child process per group. Nine fully recorded queries, no unrecorded query warmup. Constructors and selected initial refinement/serialization are setup. No timing or allocator claim.'})+'\n'));
