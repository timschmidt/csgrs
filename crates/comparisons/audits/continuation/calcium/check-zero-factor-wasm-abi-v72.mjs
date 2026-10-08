import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {wasmSources,sha,json} from './zero-factor-wasm-sources-v72.mjs';
import {compileModules,rationalInstance,historyObservation,commonRecord} from './zero-factor-wasm-runner-v72.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {reference} from './zero-factor-cost-protocol-v71.mjs';
const source=wasmSources(),scriptSha256=sha('check-zero-factor-wasm-abi-v72.mjs');
assert.deepEqual(source,json('zero-factor-wasm-origin-v72.json').source);
writeFileSync('zero-factor-wasm-abi-origin-v72.json',JSON.stringify({checkpoint:72,source,scriptSha256,
 binariesSha256:sha('zero-factor-wasm-binaries-v72.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
const runtime=await compileModules(),c=costCases()[0],input=Buffer.from(JSON.stringify([c])),results=[];
function put(e,bytes){const p=e.zf_input(bytes.length);assert(p>0&&p+bytes.length<=e.memory.buffer.byteLength);
 new Uint8Array(e.memory.buffer,p,bytes.length).set(bytes);}
const load=e=>{put(e,input);assert.equal(e.zf_init(),1);};
const prepare=e=>{load(e);e.zf_prepare(0,0,0);};
const batch=e=>{prepare(e);e.zf_batch(1);};
const finish=e=>{batch(e);e.zf_finish();};
const payload=value=>e=>put(e,Buffer.from(JSON.stringify(value)));
const modified=change=>{const x=structuredClone(c);change(x);return [x];};
const malformedPrepare=value=>e=>{payload(value)(e);assert.equal(e.zf_init(),1);};
const rational=[
 ['input-zero',()=>{},e=>e.zf_input(0)],
 ['input-over-bound',()=>{},e=>e.zf_input(128*1024*1024+1)],
 ['input-negative-wasm-u32',()=>{},e=>e.zf_input(-1)],
 ['init-without-input',()=>{},e=>e.zf_init()],
 ['duplicate-input',e=>put(e,input),e=>e.zf_input(1)],
 ['input-after-init',load,e=>e.zf_input(1)],
 ['duplicate-init',load,e=>e.zf_init()],
 ['malformed-json',e=>put(e,Buffer.from('{invalid')),e=>e.zf_init()],
 ['truncated-json',e=>put(e,input.subarray(0,input.length-1)),e=>e.zf_init()],
 ['empty-cases',payload([]),e=>e.zf_init()],
 ['noncontiguous-id',payload(modified(x=>x.id=1)),e=>e.zf_init()],
 ['prepare-before-init',()=>{},e=>e.zf_prepare(0,0,0)],
 ['prepare-bad-case',load,e=>e.zf_prepare(1,0,0)],
 ['prepare-bad-policy',load,e=>e.zf_prepare(0,2,0)],
 ['prepare-bad-lifecycle',load,e=>e.zf_prepare(0,0,2)],
 ['prepare-twice',prepare,e=>e.zf_prepare(0,0,0)],
 ['prepare-malformed-scalar',malformedPrepare(modified(x=>x.left.lower='not-rational')),e=>e.zf_prepare(0,0,0)],
 ['prepare-invalid-validation',malformedPrepare(modified(x=>x.left.validation='Unknown')),e=>e.zf_prepare(0,0,0)],
 ['prepare-unknown-operation',malformedPrepare(modified(x=>x.operation='Power')),e=>e.zf_prepare(0,0,0)],
 ['batch-before-init',()=>{},e=>e.zf_batch(1)],
 ['batch-before-prepare',load,e=>e.zf_batch(1)],
 ['batch-zero',prepare,e=>e.zf_batch(0)],
 ['batch-over-bound',prepare,e=>e.zf_batch(20001)],
 ['batch-twice',batch,e=>e.zf_batch(1)],
 ['finish-before-init',()=>{},e=>e.zf_finish()],
 ['finish-before-prepare',load,e=>e.zf_finish()],
 ['finish-before-batch',prepare,e=>e.zf_finish()],
 ['finish-twice',finish,e=>e.zf_finish()],
];
const hp=e=>e.history_prepare(0,0,0,0),hb=e=>{hp(e);e.history_batch(1);};
const history=[
 ['bad-case',()=>{},e=>e.history_prepare(48,0,0,0)],
 ['bad-policy',()=>{},e=>e.history_prepare(0,2,0,0)],
 ['bad-history',()=>{},e=>e.history_prepare(0,0,4,0)],
 ['bad-lifecycle',()=>{},e=>e.history_prepare(0,0,0,2)],
 ['prepare-twice',hp,hp],
 ['batch-before-prepare',()=>{},e=>e.history_batch(1)],
 ['batch-zero',hp,e=>e.history_batch(0)],
 ['batch-twice',hb,e=>e.history_batch(1)],
 ['finish-before-prepare',()=>{},e=>e.history_finish()],
 ['finish-before-batch',hp,e=>e.history_finish()],
];
for(const variant of ['baseline','candidate']){
 for(const [kind,cases]of [['rational',rational],['history',history]])for(const [name,setup,action]of cases){
  let e=new WebAssembly.Instance(runtime.modules[variant+':'+kind]).exports;
  setup(e);let message;
  assert.throws(()=>action(e),error=>{assert(error instanceof WebAssembly.RuntimeError);message=error.message;return true;});
  results.push({variant,kind,name,status:'expected-trap',error:message});e=null;
  if(results.length%8===0)global.gc();
 }
 // The rational bridge intentionally resets work; a second complete query is legal.
 let instance=rationalInstance(runtime.modules[variant+':rational'],input,1);
 for(let policy=0;policy<2;policy++){
  const group={case:0,policy,lifecycle:policy?'fresh':'retained'},row=instance.observe(group,1);
  commonRecord(row,group,1);assert.equal(row.sequence,policy);assert.deepEqual(row.expected,reference(c,policy,variant));
 }
 instance=null;results.push({variant,kind:'rational',name:'two-complete-queries',status:'accepted'});
 // The reused history bridge does not reset state. Finish may be repeated, but prepare may not.
 let e=new WebAssembly.Instance(runtime.modules[variant+':history']).exports;hb(e);
 const n=e.history_finish(),p=e.history_output_ptr(),first=Buffer.from(new Uint8Array(e.memory.buffer,p,n));
 assert.equal(e.history_finish(),n);assert.deepEqual(Buffer.from(new Uint8Array(e.memory.buffer,e.history_output_ptr(),n)),first);
 const expected=historyObservation(runtime.modules[variant+':history'],{case:0,policy:0,history:0,lifecycle:'retained'},1);
 assert.deepEqual(JSON.parse(first).actual,expected.actual);
 assert.throws(()=>hp(e),WebAssembly.RuntimeError);e=null;
 results.push({variant,kind:'history',name:'repeat-finish-accepted-reprepare-traps',status:'accepted-with-expected-trap'});global.gc();
}
assert.equal(results.filter(r=>r.status==='expected-trap').length,76);assert.equal(results.length,80);
assert.deepEqual(wasmSources(),source);assert.equal(sha('check-zero-factor-wasm-abi-v72.mjs'),scriptSha256);
const {modules,...metadata}=runtime;
const result={checkpoint:72,status:'abi-controls-pass',negativeControls:76,positiveSequences:4,additionalExpectedTraps:2,
 originSha256:sha('zero-factor-wasm-abi-origin-v72.json'),runtime:metadata,results,
 limits:'Test-collector ABI only, not hostile-input hardening of a production API. Each negative control uses a disposable instance. History finish remains repeatable; it has no rational-bridge batch maximum.'};
writeFileSync('zero-factor-wasm-abi-v72.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
