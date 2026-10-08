import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
export {sha,json};
export const workspace=resolve('../../../..'),changed='hypersolve/src/algebraic_binary.rs';
export const originPath='point-retained-origin-v67.json';
export function retainedSources(){
 const o=json(originPath),q=json('point-consumer-v66-manifest.json'),p=json('point-qualified-origin.json'),
  candidate=json('point-demand-source-binding.json'),consumer=json('point-consumer-binding-v66.json');
 assert.equal(o.previousSha256,sha('point-consumer-v66-manifest.json'));
 assert.deepEqual(o.before,p.liveSources);assert.equal(Object.keys(o.before).length,956);
 assert.deepEqual(o.after,{...o.before,[changed]:candidate.sources[changed]});
 assert.deepEqual(o.candidate,candidate);assert.deepEqual(o.consumer,consumer);
 assert.equal(o.candidateSha256,sha('point-demand-source-binding.json'));assert.equal(o.consumerSha256,sha('point-consumer-binding-v66.json'));
 assert.equal(q.source.solverSourceSha256,o.candidateSha256);
 for(const[f,h]of Object.entries(o.before)){
  assert.equal(sha(p.baseline+'/'+f),h,'frozen pre-retention '+f);
  assert.equal(sha(resolve(workspace,f)),o.after[f],'current retained '+f);
 }
 for(const[f,h]of Object.entries(candidate.sources))assert.equal(sha(candidate.root+'/'+f),h,'qualified solver '+f);
 for(const[f,h]of Object.entries(consumer.sources))assert.equal(sha(consumer.root+'/'+f),h,'qualified consumer '+f);
 assert.notEqual(o.before[changed],o.after[changed]);
 assert.equal(sha(resolve(workspace,changed)),sha(candidate.root+'/'+changed));
 for(const[f,h]of Object.entries(o.historicalFiles))assert.equal(sha(f),h,f);
 const g=json('results/point-consumer-verify-v66.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 const out=readFileSync('results/point-consumer-verify-v66.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(out.length,1);assert.equal(out[0].checkpoint,66);assert.equal(readFileSync('results/point-consumer-verify-v66.stderr').length,0);
 return{originSha256:sha(originPath),changed:[changed],liveFiles:956,qualifiedSolverFiles:175,qualifiedConsumerFiles:355,
  sourceSha256:o.after[changed],previousSourceSha256:o.before[changed],previousFinished:g.finished,
  scope:'Exact promotion of the frozen qualified solver algorithm/tests. Earlier manifests and live checks describe historical source states; current live identity and new regression gates are separate. No earlier numerical or timing capture is relabelled as a live-path run.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 if(process.argv.includes('--prepare')){
  const q=json('point-consumer-v66-manifest.json');for(const[p,h]of Object.entries(q.files))assert.equal(sha(p),h,p);
  const {consumerEvidence}=await import('./check-point-consumer-v66.mjs');assert.deepEqual(consumerEvidence(),q.evidence);
  const p=json('point-qualified-origin.json'),candidate=json('point-demand-source-binding.json'),consumer=json('point-consumer-binding-v66.json');
  for(const[f,h]of Object.entries(p.liveSources))assert.equal(sha(resolve(workspace,f)),h,f);
  const g=json('results/point-consumer-verify-v66.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  const files=['point-consumer-v66-manifest.json',...Object.keys(q.files),...['json','stdout','stderr'].map(ext=>'results/point-consumer-verify-v66.'+ext)];
  const o={recorded:new Date().toISOString(),previousSha256:sha('point-consumer-v66-manifest.json'),
   before:p.liveSources,after:{...p.liveSources,[changed]:candidate.sources[changed]},candidate,consumer,
   candidateSha256:sha('point-demand-source-binding.json'),consumerSha256:sha('point-consumer-binding-v66.json'),
   historicalFiles:Object.fromEntries([...new Set(files)].map(f=>[f,sha(f)])),
   note:'Historical 66 verification was terminal and all pre-retention live identities matched before preparing this exact one-file promotion. This origin is not itself a successful retention or post-change regression gate.'};
  writeFileSync(originPath,JSON.stringify(o,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({prepared:true,changed,before:o.before[changed],after:o.after[changed],previousFinished:g.finished}));
 }else console.log(JSON.stringify(retainedSources()));
}
