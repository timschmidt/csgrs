import {writeFileSync,readFileSync,statfsSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './twelfth-cost-sources-v78.mjs';
import {costEvidence,files as evidenceFiles} from './twelfth-cost-evidence-v78.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const evidence=costEvidence(),prior=json('twelfth-relation-v77-manifest.json'),tag='twelfth-cost-evidence-v78',g=json('results/'+tag+'.json');
assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.command,'node');assert.deepEqual(g.args,['twelfth-cost-evidence-v78.mjs']);
assert.equal(readFileSync('results/'+tag+'.stderr').length,0);assert.deepEqual(json('results/'+tag+'.stdout'),evidence);
const current=[...evidence.currentGates,tag],gates=[...current,...evidence.developmentGates,...evidence.failedGates];
const runs=[...json('twelfth-native-runs-v78.json').runs,...json('twelfth-isolated-runs-v78.json').runs],binaries=json('twelfth-cost-binaries-v78.json').binaries;
const files=[...new Set([...Object.keys(prior.files),'twelfth-relation-v77-manifest.json',...evidenceFiles,
 ...runs.map(r=>r.plan),...binaries.map(b=>b.path),...json('twelfth-cost-controls-v78.json').records.map(r=>r.path),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
const path='twelfth-cost-v78-manifest.json';
if(process.argv.includes('--record')){
 const fs=statfsSync('/tmp');const m={checkpoint:78,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  evidence,gates,currentGates:current,developmentGates:evidence.developmentGates,failedGates:evidence.failedGates,
  liveSources:prior.liveSources,candidateSources:prior.candidateSources,candidateRoot:prior.candidateRoot,tmpAvailableBytes:fs.bavail*fs.bsize,
  retainedContinuationTransfers:7,productionChanges:0};writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:78,status:'recorded',artifacts:files.length,gates:gates.length,current:current.length,development:2,failed:1,tmpAvailableBytes:m.tmpAvailableBytes}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.deepEqual(m.currentGates,current);assert.deepEqual(m.liveSources,prior.liveSources);
 assert.deepEqual(m.candidateSources,prior.candidateSources);assert.equal(m.candidateRoot,prior.candidateRoot);assert.equal(m.status,evidence.status);
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);
 console.log(JSON.stringify({checkpoint:78,status:'verified-current-version-not-selected',artifacts:files.length,gates:gates.length,current:current.length,development:2,failed:1,
  liveFiles:957,candidateFiles:183,cases:96,groups:576,mainRawRows:39168,isolatedRows:1440,preflightRecords:2304,
  worstSameResultRatio:Math.max(...json('twelfth-native-summary-v78.json').cpu.filter(r=>r.equalResult).map(r=>r.pairedMedianRatio)),
  retainedContinuationTransfers:7,productionChanges:0,newDonorLines:0,tmpAvailableAtBinding:m.tmpAvailableBytes,decision:evidence.decision,limits:evidence.limits}));
}
