import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {wasmSources,sha,json} from './zero-factor-wasm-sources-v72.mjs';
import {recordEvidence,recordValidator} from './check-zero-factor-wasm-records-v72.mjs';
import {flags} from './zero-factor-wasm-runner-v72.mjs';
export const buildTags=[...['baseline','candidate'].flatMap(v=>['rational','history'].flatMap(k=>['lock','metadata','build','clippy'].map(s=>'zero-factor-wasm-'+v+'-'+k+'-'+s+'-v72'))),'zero-factor-wasm-fmt-v72'];
export const gates=[...buildTags,'zero-factor-wasm-build-v72','zero-factor-wasm-qualification-v72','zero-factor-wasm-abi-v72','zero-factor-wasm-records-v72','zero-factor-wasm-corruptions-v72'];
export function gate(tag){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(g.elapsedSeconds>=0);return g;
}
export async function wasmEvidence(){
 const source=wasmSources(),origin=json('zero-factor-wasm-origin-v72.json'),binaries=json('zero-factor-wasm-binaries-v72.json');
 assert.deepEqual(origin.source,source);assert.equal(binaries.originSha256,sha('zero-factor-wasm-origin-v72.json'));
 assert.equal(binaries.root,origin.root);assert.equal(binaries.artifacts.length,4);
 const q=json('zero-factor-wasm-qualification-v72.json'),qo=json('zero-factor-wasm-qualification-origin-v72.json');
 assert.deepEqual(qo.source,source);assert.equal(qo.binariesSha256,sha('zero-factor-wasm-binaries-v72.json'));
 for(const[p,h]of Object.entries(qo.files))assert.equal(sha(p),h,p);assert.equal(qo.gcEvery,8);
 const abi=json('zero-factor-wasm-abi-v72.json'),ao=json('zero-factor-wasm-abi-origin-v72.json');
 assert.equal(ao.scriptSha256,sha('check-zero-factor-wasm-abi-v72.mjs'));assert.equal(ao.binariesSha256,sha('zero-factor-wasm-binaries-v72.json'));
 assert.deepEqual(ao.source,source);assert.equal(abi.originSha256,sha('zero-factor-wasm-abi-origin-v72.json'));
 assert.deepEqual(abi,json('results/zero-factor-wasm-abi-v72.stdout'));
 for(const r of [q.runtime,abi.runtime]){
  assert.deepEqual(r.flags,flags);assert.equal(r.affinity,'2');assert.equal(r.node,'v22.22.2');assert.equal(r.v8,'12.4.254.21-node.39');
  assert.equal(r.metadata.length,4);
 }
 let bytes=0;
 for(const [i,a]of binaries.artifacts.entries()){
  assert.deepEqual([a.variant,a.kind],[['baseline','rational'],['baseline','history'],['candidate','rational'],['candidate','history']][i]);
  assert.equal(a.path,origin.root+'/'+a.variant+'-'+a.kind+'.wasm');assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);bytes+=a.bytes;
  const module=await WebAssembly.compile(readFileSync(a.path)),imports=WebAssembly.Module.imports(module),exports=WebAssembly.Module.exports(module);
  assert.deepEqual(imports,[]);for(const r of [q.runtime,abi.runtime])assert.deepEqual(r.metadata[i],{...a,imports,exports});
 }
 assert.equal(bytes,5818881);
 for(const kind of ['rational','history']){
  const base=json('results/zero-factor-wasm-baseline-'+kind+'-metadata-v72.stdout');
  const candidate=JSON.parse(readFileSync('results/zero-factor-wasm-candidate-'+kind+'-metadata-v72.stdout','utf8')
   .replaceAll(resolve('zero-factor-wasm-candidate-'+kind+'-v72'),resolve('zero-factor-wasm-baseline-'+kind+'-v72'))
   .replaceAll(resolve('zero-factor-candidate-v70/hypersolve'),resolve('point-demand-candidate/hypersolve')));
  assert.deepEqual(candidate,base);assert.equal(base.packages.length,33);assert.equal(base.resolve.nodes.length,33);
 }
 const lock=sha('zero-factor-wasm-baseline-rational-v72/Cargo.lock');
 for(const v of ['baseline','candidate'])for(const k of ['rational','history'])assert.equal(sha('zero-factor-wasm-'+v+'-'+k+'-v72/Cargo.lock'),lock);
 for(const t of gates)gate(t);assert.equal(gates.length,22);assert.equal(new Set(gates).size,22);
 const build=gate('zero-factor-wasm-build-v72'),rows=readFileSync('results/zero-factor-wasm-build-v72.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,35);let previous=Date.parse(build.started);
 for(const [i,tag]of buildTags.entries()){
  const child=gate(tag),begin=rows[2*i];assert.deepEqual(rows[2*i+1],child);
  assert.deepEqual({...begin,pid:undefined},{tag,started:child.started,cwd:child.cwd,command:child.command,args:child.args,pid:undefined});
  assert(Number.isSafeInteger(begin.pid)&&begin.pid>0);assert(Date.parse(child.started)>=previous);previous=Date.parse(child.finished);
 }
 assert(Date.parse(build.finished)>=previous);assert.deepEqual(rows.at(-1).artifacts,binaries.artifacts);assert.equal(rows.at(-1).status,'wasm-builds-terminal');
 for(const [tag,script]of [['zero-factor-wasm-qualification-v72','qualify-zero-factor-wasm-v72.mjs'],['zero-factor-wasm-abi-v72','check-zero-factor-wasm-abi-v72.mjs']]){
  const g=gate(tag);assert.equal(g.command,'taskset');assert.deepEqual(g.args,['-c','2','node',...flags,script]);assert(Date.parse(g.started)>=Date.parse(build.finished));
 }
 for(const tag of ['zero-factor-wasm-build-v72','zero-factor-wasm-qualification-v72','zero-factor-wasm-abi-v72','zero-factor-wasm-records-v72','zero-factor-wasm-corruptions-v72'])
  assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
 const replay=await recordEvidence();assert.deepEqual(replay,json('results/zero-factor-wasm-records-v72.stdout'));
 assert.equal(abi.negativeControls,76);assert.equal(abi.positiveSequences,4);assert.equal(abi.additionalExpectedTraps,2);assert.equal(abi.results.length,80);
 assert.equal(new Set(abi.results.map(r=>[r.variant,r.kind,r.name].join(':'))).size,80);
 for(const v of ['baseline','candidate'])for(const k of ['rational','history']){
  const controls=abi.results.filter(r=>r.variant===v&&r.kind===k),negative=controls.filter(r=>r.status==='expected-trap');
  assert.equal(negative.length,k==='rational'?28:10);for(const n of negative)assert.equal(n.error,'unreachable');
  assert.equal(controls.length,negative.length+1);
 }
 const corrupt=json('zero-factor-wasm-corruptions-v72.json'),validate=recordValidator();
 assert.equal(corrupt.accepted,2);assert.equal(corrupt.rejected,21);assert.equal(corrupt.results.length,23);
 for(const[p,h]of Object.entries(corrupt.scripts))assert.equal(sha(p),h,p);
 for(const r of corrupt.results){let error;try{validate(r.row,r.spec);}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error,r.error);assert.equal(error===undefined,r.accepted);
 }
 assert.deepEqual(json('results/zero-factor-wasm-corruptions-v72.stdout'),{checkpoint:72,status:'record-corruptions-pass',accepted:2,rejected:21,sha256:sha('zero-factor-wasm-corruptions-v72.json')});
 assert.deepEqual(wasmSources(),source);
 return {checkpoint:72,status:'isolated-wasm-correctness-qualified',sourceOriginSha256:sha('zero-factor-wasm-origin-v72.json'),
  liveFiles:956,candidateFiles:176,retainedContinuationTransfers:6,newDonorLines:0,gates,
  dependencyGraph:{packagesPerApp:33,nodesPerApp:33,fullMetadataEqualAfterPaths:true,allLocksEqual:true},
  replay,abi:{negativeControls:76,positiveSequences:4,additionalExpectedTraps:2},recordControls:{accepted:2,rejected:21},
  runtime:{node:q.runtime.node,v8:q.runtime.v8,flags:q.runtime.flags,affinity:q.runtime.affinity,importFreeModules:4},
  captures:{build,qualification:gate('zero-factor-wasm-qualification-v72'),abi:gate('zero-factor-wasm-abi-v72')},
  storage:{moduleFiles:4,moduleBytes:bytes,moduleRoot:origin.root,rawObservationBytes:statSync('results/zero-factor-wasm-qualification-v72.stdout').size,noNewSolverCopy:true},
  limits:'No WASM cost inference, allocation/peak/RSS measurement, live adoption or donor coverage credit. Native checkpoint-71 artifacts are hash-checked, not statistically recomputed here. WASM matched costs, consumer/size gates and remaining full inventory stay open.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(await wasmEvidence()));
