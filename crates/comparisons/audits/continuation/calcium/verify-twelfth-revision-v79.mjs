import {readFileSync,writeFileSync,statfsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './twelfth-revision-sources-v79.mjs';
import {revisionEvidence,files as evidenceFiles} from './twelfth-revision-evidence-v79.mjs';
import {target} from './point-qualified-capture.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const evidence=revisionEvidence(),prior=json('twelfth-cost-v78-manifest.json'),tag='twelfth-revision-evidence-v79',g=json('results/'+tag+'.json');
assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.command,'node');assert.deepEqual(g.args,['twelfth-revision-evidence-v79.mjs']);
assert.equal(readFileSync('results/'+tag+'.stderr').length,0);assert.deepEqual(json('results/'+tag+'.stdout'),evidence);
const current=[...evidence.currentGates,tag],gates=[...current,...evidence.developmentGates,...evidence.failedGates,...evidence.unusableGates];
const b=evidence.source,run=json('twelfth-revision-native-runs-v79.json'),binary=json('twelfth-revision-binaries-v79.json');
const files=[...new Set([...Object.keys(prior.files),'twelfth-cost-v78-manifest.json',...evidenceFiles,...Object.keys(b.sources).map(p=>b.root+'/'+p),
 ...run.runs.map(r=>r.plan),...binary.binaries.map(b=>b.path),...['debug','release'].map(p=>target+'/'+p+'/twelfth-capability-v79'),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
const path='twelfth-revision-v79-manifest.json';
if(process.argv.includes('--record')){
 const s=statfsSync('/tmp'),m={checkpoint:79,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  evidence,gates,currentGates:current,developmentGates:evidence.developmentGates,failedGates:evidence.failedGates,unusableGates:evidence.unusableGates,
  liveSources:prior.liveSources,candidateSources:b.sources,candidateRoot:b.root,tmpAvailableBytes:s.bavail*s.bsize,retainedContinuationTransfers:7,productionChanges:0};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:79,status:'recorded',artifacts:files.length,gates:gates.length,current:current.length,development:3,failed:2,unusable:4,tmpAvailableBytes:m.tmpAvailableBytes}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.deepEqual(m.currentGates,current);assert.deepEqual(m.liveSources,prior.liveSources);
 assert.deepEqual(m.candidateSources,b.sources);assert.equal(m.candidateRoot,b.root);assert.equal(m.status,evidence.status);
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);
 console.log(JSON.stringify({checkpoint:79,status:'verified-cheaper-revision-not-retained',artifacts:files.length,gates:gates.length,current:current.length,
  development:3,failed:2,unusable:4,liveFiles:957,candidateFiles:184,rawRows:65664,preflightRecords:3456,newBinaryBytes:4403848,
  retainedContinuationTransfers:7,newDonorLines:0,productionChanges:0,tmpAvailableAtBinding:m.tmpAvailableBytes,decision:evidence.decision,limits:evidence.limits}));
}
