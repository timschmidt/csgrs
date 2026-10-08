import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
export const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
export const json=p=>JSON.parse(readFileSync(p,'utf8'));
export const workspace=resolve('../../../..'),changed='hypersolve/src/algebraic_binary.rs',added='hypersolve/src/algebraic_binary/zero_factor_tests.rs';
export const originPath='zero-factor-retained-origin-v75.json';
export function retainedSources(){
 const o=json(originPath),previous=json('point-retained-origin-v67.json'),candidate=json('zero-factor-final-binding-v70.json'),consumer=json('zero-factor-consumer-binding-v74.json');
 assert.equal(o.protocolSha256,sha('zero-factor-retained-protocol-v75.md'));assert.equal(o.sourceScriptSha256,sha('zero-factor-retained-sources-v75.mjs'));
 assert.equal(o.previousSha256,sha('zero-factor-consumer-v74-manifest.json'));assert.deepEqual(o.before,previous.after);
 assert.equal(Object.keys(o.before).length,956);assert(!(added in o.before));assert.deepEqual(o.changed,[changed,added]);
 assert.deepEqual(o.candidate,candidate);assert.deepEqual(o.consumer,consumer);
 assert.equal(o.candidateSha256,sha('zero-factor-final-binding-v70.json'));assert.equal(o.consumerSha256,sha('zero-factor-consumer-binding-v74.json'));
 assert.deepEqual(o.after,{...o.before,[changed]:candidate.source[changed],[added]:candidate.source[added]});assert.equal(Object.keys(o.after).length,957);
 assert.notEqual(o.before[changed],o.after[changed]);
 for(const[p,h]of Object.entries(o.after))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
 for(const[p,h]of Object.entries(candidate.source))assert.equal(sha(candidate.root+'/'+p),h,'qualified solver '+p);
 for(const[p,h]of Object.entries(consumer.sources))assert.equal(sha(consumer.root+'/'+p),h,'qualified consumer '+p);
 for(const[p,h]of Object.entries(o.historicalFiles))assert.equal(sha(p),h,p);
 for(const p of [changed,added])assert.equal(sha(resolve(workspace,p)),sha(candidate.root+'/'+p));
 for(const p of ['hypersolve/src/resultant.rs','hypersolve/src/root_isolation.rs','hypersolve/src/root_isolation_monic_tests.rs'])assert.equal(o.after[p],o.before[p]);
 const g=json('results/zero-factor-consumer-verify-v74.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 return {checkpoint:75,originSha256:sha(originPath),changed:[changed,added],previousLiveFiles:956,liveFiles:957,unchangedPreviousFiles:955,
  qualifiedSolverFiles:176,qualifiedConsumerFiles:355,mainSha256:o.after[changed],testSha256:o.after[added],previousMainSha256:o.before[changed],
  historicalArtifacts:Object.keys(o.historicalFiles).length,previousFinished:g.finished,
  scope:'Exact promotion of the fully qualified zero-factor candidate; no other recorded live source/support change. Historical live flags describe older states. Source identity is separate from successful live regression/retention gates.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 if(process.argv.includes('--prepare')){
  const {consumerEvidence}=await import('./check-zero-factor-consumer-v74.mjs');
  const q=json('zero-factor-consumer-v74-manifest.json');assert.deepEqual(consumerEvidence(),q.evidence);
  const g=json('results/zero-factor-consumer-verify-v74.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  assert.equal(readFileSync('results/zero-factor-consumer-verify-v74.stderr').length,0);
  const previous=json('point-retained-origin-v67.json'),candidate=json('zero-factor-final-binding-v70.json'),consumer=json('zero-factor-consumer-binding-v74.json');
  const before=previous.after;for(const[p,h]of Object.entries(before))assert.equal(sha(resolve(workspace,p)),h,'before '+p);
  assert.equal(Object.keys(before).length,956);assert(!existsSync(resolve(workspace,added)));
  const historicalFiles={};
  for(const path of ['zero-factor-v70-manifest.json','zero-factor-cost-v71-manifest.json','zero-factor-wasm-v72-manifest.json',
   'zero-factor-wasm-cost-v73-manifest.json','zero-factor-consumer-v74-manifest.json']){
   const m=json(path);historicalFiles[path]=sha(path);
   for(const[p,h]of Object.entries(m.files)){assert.equal(sha(p),h,p);if(p in historicalFiles)assert.equal(historicalFiles[p],h,p);historicalFiles[p]=h;}
  }
  for(const p of ['point-retained-origin-v67.json','point-retained-v67-manifest.json','point-demand-source-binding.json',
   'zero-factor-final-binding-v70.json','zero-factor-consumer-binding-v74.json',
   ...['json','stdout','stderr'].map(ext=>'results/zero-factor-consumer-verify-v74.'+ext)])historicalFiles[p]=sha(p);
  const o={checkpoint:75,recorded:new Date().toISOString(),previousSha256:sha('zero-factor-consumer-v74-manifest.json'),
   protocolSha256:sha('zero-factor-retained-protocol-v75.md'),sourceScriptSha256:sha('zero-factor-retained-sources-v75.mjs'),
   before,after:{...before,[changed]:candidate.source[changed],[added]:candidate.source[added]},changed:[changed,added],candidate,consumer,
   candidateSha256:sha('zero-factor-final-binding-v70.json'),consumerSha256:sha('zero-factor-consumer-binding-v74.json'),historicalFiles,
   note:'Pre-adoption checks were completed with the original live state. This origin is a planned exact promotion, not a successful live retention gate.'};
  writeFileSync(originPath,JSON.stringify(o,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({checkpoint:75,status:'prepared-before-live-edit',changed:o.changed,before:o.before[changed],after:o.after[changed],
   addedSha256:o.after[added],historicalArtifacts:Object.keys(historicalFiles).length}));
 }else console.log(JSON.stringify(retainedSources()));
}
