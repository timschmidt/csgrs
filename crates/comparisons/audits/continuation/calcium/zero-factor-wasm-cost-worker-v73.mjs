import {readFileSync} from 'node:fs';
import {once} from 'node:events';
import assert from 'node:assert/strict';
import {rationalInstance} from './zero-factor-wasm-runner-v72.mjs';
import {config,flags,validateRow} from './zero-factor-wasm-cost-protocol-v73.mjs';
import {sha,json} from './zero-factor-wasm-sources-v72.mjs';
const [variant,instanceMode,planPath]=process.argv.slice(2);assert(['baseline','candidate'].includes(variant));assert(config.instanceModes.includes(instanceMode));
assert.deepEqual(process.execArgv,flags);assert.equal(typeof global.gc,'function');
const affinity=readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.+)$/m)?.[1];assert.equal(affinity,'2');
const binary=json('zero-factor-wasm-binaries-v72.json').artifacts.find(a=>a.variant===variant&&a.kind==='rational');assert(binary);assert.equal(sha(binary.path),binary.sha256);
const module=await WebAssembly.compile(readFileSync(binary.path));assert.deepEqual(WebAssembly.Module.imports(module),[]);
assert.deepEqual(WebAssembly.Module.exports(module).map(x=>x.name).sort(),['memory','zf_input','zf_init','zf_prepare','zf_batch','zf_finish','zf_output_ptr'].sort());
const input=readFileSync('zero-factor-cost-input-v71.json'),cases=JSON.parse(input).length;assert.equal(cases,114);
const plan=json(planPath);assert(plan.length>0);let persistent=instanceMode==='persistent'?rationalInstance(module,input,cases):null,gcs=0;
const emit=async row=>{if(!process.stdout.write(JSON.stringify(row)+'\n'))await once(process.stdout,'drain');};
for(const [i,group]of plan.entries()){
 let instance=persistent??rationalInstance(module,input,cases);
 const {qualification_batch_ns,...observed}=instance.observe(group,group.iterations);instance=null;
 const row={...observed,mode:'cpu',elapsed_ns:qualification_batch_ns,variant,instanceMode};
 validateRow(row,group,variant,instanceMode,i);await emit(row);
 if((i+1)%config.gcEvery===0){global.gc();gcs++;}
}
persistent=null;
await emit({terminal:true,checkpoint:73,variant,instanceMode,groups:plan.length,gcs,moduleSha256:sha(binary.path),inputSha256:sha('zero-factor-cost-input-v71.json'),
 planSha256:sha(planPath),node:process.version,v8:process.versions.v8,flags:process.execArgv,affinity,finished:new Date().toISOString()});
