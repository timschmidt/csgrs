import {readFileSync,writeFileSync,statSync,statfsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha,checkedPlan,paths} from './point-attribution-runtime-v65.mjs';
import {sourceEvidence} from './reanalyse-early-statistics-v64.mjs';
const manifestPath='point-attribution-v65-manifest.json';
function sources(){
 const p=json('early-statistics-v64-manifest.json');
 for(const[f,h]of Object.entries(p.files))assert.equal(sha(f),h,f);
 assert.deepEqual(sourceEvidence(),p.sources);
 const gate=json('results/early-statistics-verify.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
 const rows=readFileSync('results/early-statistics-verify.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,15);assert.equal(rows.at(-1).checkpoint,64);assert.equal(readFileSync('results/early-statistics-verify.stderr').length,0);
 checkedPlan();
 return{previousSha256:sha('early-statistics-v64-manifest.json'),previousArtifacts:Object.keys(p.files).length,
  previousFinished:gate.finished,liveFiles:p.sources.liveFiles,pointCandidateFiles:p.sources.pointCandidateFiles,
  planSha256:sha('point-attribution-revised-plan-v65.json'),classificationSha256:sha('statistical-matches-v65.json')};
}
const gates=['point-attribution-revised-0','point-attribution-revised-1','point-attribution-revised-2','point-attribution-revised-3',
 'point-attribution-analysis','point-attribution-check'];
function checkGates(){for(const tag of gates){
 const g=json(`results/${tag}.json`);assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);
 assert.equal(readFileSync(`results/${tag}.stderr`).length,0);assert(readFileSync(`results/${tag}.stdout`).length>0);
 assert(Date.parse(g.finished)>=Date.parse(g.started));
}}
if(process.argv.includes('--record')){
 const origin=sources(),a=json('point-attribution-analysis-v65.json'),p=checkedPlan().plan,c=json('statistical-matches-v65.json');
 const files=[...new Set(['point-attribution-v65-findings.md','point-attribution-analysis-v65.json',
  'classify-statistical-matches-v65.mjs','statistical-matches-v65.json','analyse-point-attribution-v65.mjs',
  'check-point-attribution-v65.mjs','verify-point-attribution-v65.mjs','point-attribution-revised-plan-v65.json',
  'early-statistics-v64-manifest.json','reanalyse-early-statistics-v64.mjs','prototype-statistics-v63-inventory-corrected.json',
  'prototype-statistics-inventory-v63.mjs','prototype-statistics-v63-inventory.json','prototype-statistics-v63-inventory.mjs',
  'point-demand-sources.mjs','point-demand-cost-protocol.mjs','point-history-protocol.mjs','point-cold-protocol.mjs',
  ...c.files.map(f=>f.path),...Object.keys(p.files),...Object.keys(p.initialAttempt),
  ...a.passes.flatMap(s=>Object.keys(s.files)),...['json','stdout','stderr'].map(ext=>`results/early-statistics-verify.${ext}`),
  ...gates.flatMap(tag=>['json','stdout','stderr'].map(ext=>`results/${tag}.${ext}`))])].sort();
 checkGates();const capacity=statfsSync('/tmp');
 const result={checkpoint:65,status:'progress',recorded:new Date().toISOString(),sources:origin,gates,
  files:Object.fromEntries(files.map(f=>[f,sha(f)])),workspaceBoundBytes:files.reduce((n,f)=>n+statSync(f).size,0),
  tmpCapacity:{availableBytes:Number(capacity.bavail)*Number(capacity.bsize),newDedicatedArtifacts:0},
  limits:a.limits+' No full historical 47-chain or 60–64 statistical recomputation; prior artifact/source identities are rechecked. Original full audit remains open.'};
 writeFileSync(manifestPath,JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:65,status:'recorded',artifacts:files.length,sources:origin,tmpCapacity:result.tmpCapacity}));
}else{
 const m=json(manifestPath);for(const[f,h]of Object.entries(m.files))assert.equal(sha(f),h,f);
 assert.deepEqual(sources(),m.sources);assert.deepEqual(m.gates,gates);checkGates();
 await import('./classify-statistical-matches-v65.mjs');
 await import('./check-point-attribution-v65.mjs');
 for(const[f,h]of Object.entries(m.files))assert.equal(sha(f),h,f);assert.deepEqual(sources(),m.sources);
 console.log(JSON.stringify({checkpoint:65,status:'verified-progress',artifacts:Object.keys(m.files).length,successfulGates:gates.length,
  sources:m.sources,measured:6912,qualification:192,emptyControls:1024,additionalEstimators:0,limits:m.limits}));
}
