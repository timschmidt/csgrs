import {readFileSync,writeFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,consumerSources} from './point-consumer-sources-v66.mjs';
import {sourceEvidence} from './reanalyse-early-statistics-v64.mjs';
import {consumerEvidence} from './check-point-consumer-v66.mjs';
const path='point-consumer-v66-manifest.json';
const tags=['point-consumer-metadata-baseline-v66','point-consumer-metadata-demand-v66','point-consumer-release-v66',
 'point-consumer-qualification-v66','point-consumer-fmt-v66','point-consumer-clippy-v66','point-consumer-wasm-v66',
 'point-consumer-app-build-v66',...['basic','arrangement'].flatMap(e=>['strip','baseline-run','eager-run','demand-run','size'].map(k=>`point-consumer-${e}-${k}-v66`)),
 'point-consumer-cold-size-v66','point-consumer-environment-v66','point-consumer-check-v66'];
function priorEvidence(){
 const previous=json('point-attribution-v65-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(sourceEvidence(),json('early-statistics-v64-manifest.json').sources);
 const g=json('results/point-attribution-verify.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 const records=readFileSync('results/point-attribution-verify.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(records.length,3);assert.equal(records.at(-1).checkpoint,65);
 assert.equal(readFileSync('results/point-attribution-verify.stderr').length,0);
 return{sha256:sha('point-attribution-v65-manifest.json'),artifacts:91,finished:g.finished,liveFiles:956};
}
function checks(){
 const evidence=consumerEvidence();assert.deepEqual(evidence,json('results/point-consumer-check-v66.stdout'));
 for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);}
 const rows=readFileSync('results/point-consumer-qualification-v66.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,33);assert.equal(rows.at(-1).checkpoint,66);assert.equal(rows.at(-1).dedicatedBytes,evidence.dedicatedBytes);
 assert.equal(readFileSync('results/point-consumer-qualification-v66.stderr').length,0);
 const before=readFileSync('e-plan-qualified-candidate/hypersolve/src/algebraic_binary.rs','utf8').split('#[cfg(test)]'),
  after=readFileSync('point-demand-candidate/hypersolve/src/algebraic_binary.rs','utf8').split('#[cfg(test)]');
 assert.equal(before.length,2);assert.equal(after.length,2);
 const lines=s=>s.split('\n').length-1;
 assert.deepEqual(after.map((s,i)=>lines(s)-lines(before[i])),[52,374]);
 return evidence;
}
const prior=priorEvidence(),source=consumerSources(),evidence=checks();assert.equal(tags.length,21);
if(process.argv.includes('--record')){
 const files=[...new Set(['point-consumer-v66-findings.md','point-consumer-sources-v66.mjs','qualify-point-consumer-v66.mjs',
  'check-point-consumer-v66.mjs','verify-point-consumer-v66.mjs','point-consumer-origin-v66.json','point-consumer-binding-v66.json',
  'point-consumer-app-origin-v66.json','point-consumer-apps-v66.json','point-qualified-capture.mjs','point-qualified-environment.mjs',
  'capture.mjs','point-attribution-v65-manifest.json','point-qualified-apps-summary.json','e-qualified-app-size-summary.json',
  'point-demand-source-binding.json','point-demand-manifest.json','point-cold-binaries.json','point-wasm-binaries.json',
  'point-qualified-origin.json','reanalyse-early-statistics-v64.mjs',
  ...['json','stdout','stderr'].map(ext=>'results/point-attribution-verify.'+ext),
  ...tags.flatMap(tag=>['json','stdout','stderr'].map(ext=>'results/'+tag+'.'+ext))])].sort();
 const m={checkpoint:66,status:'isolated-qualification-pass-selected-for-live-integration',recorded:new Date().toISOString(),prior,source,
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates:tags,evidence,newDonorLines:0,
  addedWorkspaceSourceBytes:23841357,newDedicatedFiles:4,newDedicatedBytes:evidence.dedicatedBytes,
  limits:'No live source changed yet. Earlier source identity and successful capture are checked, not a full historical-chain or statistical rerun. Consumer and size evidence is scoped; the full original audit remains open.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:66,status:'recorded',artifacts:files.length,gates:tags.length,dedicatedBytes:evidence.dedicatedBytes}));
}else{
 const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.prior,prior);assert.deepEqual(m.source,source);assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,tags);
 for(const a of evidence.applicationSizes.flatMap(g=>g.artifacts).filter(a=>a.variant==='demand'))for(const f of a.files)assert.equal(statSync(f.path).size,f.bytes);
 console.log(JSON.stringify({checkpoint:66,status:m.status,artifacts:Object.keys(m.files).length,gates:tags.length,consumer:evidence.consumer,
  applicationDeltas:evidence.applicationSizes.map(g=>({example:g.example,baseline:g.baselineDelta,eager:g.eagerDelta})),
  newDedicatedBytes:evidence.dedicatedBytes,liveFiles:956,copiedConsumerFiles:355,limits:m.limits}));
}
