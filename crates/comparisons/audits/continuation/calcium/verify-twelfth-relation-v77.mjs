import {writeFileSync,readFileSync,statfsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './zero-factor-retained-sources-v75.mjs';
import {twelfthEvidence,evidenceFiles,finalGates,developmentGates,failedGates,emptyGates} from './check-twelfth-evidence-v77.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const evidence=twelfthEvidence(),b=json('twelfth-final-binding-v77.json'),previous=json('qqbar-trig-v76-manifest.json');
const gate='twelfth-evidence-v77',g=json('results/'+gate+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(g.command,'node');assert.deepEqual(g.args,['check-twelfth-evidence-v77.mjs']);
assert.deepEqual(json('results/'+gate+'.stdout'),evidence);assert.equal(readFileSync('results/'+gate+'.stderr').length,0);
const accepted=[...finalGates,gate],gates=[...developmentGates,...failedGates,...emptyGates,...accepted];
assert.equal(new Set(gates).size,gates.length);
const files=[...new Set([...Object.keys(previous.files),...evidenceFiles,...Object.keys(b.files),
 ...Object.keys(b.sources).map(p=>b.root+'/'+p),...evidence.executables.map(x=>x.path),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
const path='twelfth-relation-v77-manifest.json';
if(process.argv.includes('--record')){
 const fs=statfsSync('/tmp');
 const m={checkpoint:77,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  liveSources:previous.liveSources,candidateSources:b.sources,candidateRoot:b.root,evidence,gates,acceptedFinalGates:accepted,
  developmentGates,failedGates,unusableEmptyCompletionGates:emptyGates,
  tmpAvailableBytes:fs.bavail*fs.bsize,retainedContinuationTransfers:7,productionChanges:0};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:77,status:'recorded',artifacts:files.length,gates:gates.length,acceptedFinal:accepted.length,development:developmentGates.length,failed:failedGates.length,unusableEmpty:emptyGates.length,tmpAvailableBytes:m.tmpAvailableBytes}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.liveSources,previous.liveSources);assert.deepEqual(m.candidateSources,b.sources);assert.equal(m.candidateRoot,b.root);
 assert.deepEqual(m.gates,gates);assert.deepEqual(m.acceptedFinalGates,accepted);assert.deepEqual(m.developmentGates,developmentGates);
 assert.deepEqual(m.failedGates,failedGates);assert.deepEqual(m.unusableEmptyCompletionGates,emptyGates);
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);assert.equal(m.status,evidence.status);
 console.log(JSON.stringify({checkpoint:77,status:'verified-not-retained',artifacts:files.length,gates:gates.length,acceptedFinal:accepted.length,
  development:developmentGates.length,failed:failedGates.length,unusableEmpty:emptyGates.length,liveFiles:957,candidateFiles:183,
  improvedIdentities:32,improvedRepeats:192,publicRecordsPerProfile:1728,regressions:evidence.regressions.results.map(x=>({variant:x.variant,features:x.features,profile:x.profile,passed:x.passed})),
  retainedContinuationTransfers:7,productionChanges:0,newDonorReadLines:0,tmpAvailableAtBinding:m.tmpAvailableBytes,
  limits:evidence.limits,next:evidence.next}));
}
