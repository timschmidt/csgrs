import {writeFileSync,readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {extractionEvidence,evidenceFiles} from './quadratic-extraction-evidence-v82.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const path='quadratic-extraction-v82-manifest.json',evidence=extractionEvidence(),o=json('quadratic-extraction-origin-v82.json'),prior=json('scalar-boundary-v81-manifest.json');
const tag='quadratic-extraction-evidence-v82',g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(g.command,'node');assert.deepEqual(g.args,['quadratic-extraction-evidence-v82.mjs']);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
assert.deepEqual(evidence,json('results/'+tag+'.stdout'));
const currentGates=[...evidence.successfulGates,tag],unusableGates=evidence.unusableGates,gates=[...currentGates,...unusableGates];
assert.equal(gates.length,19);assert.equal(currentGates.length,17);
const files=[...new Set([...evidenceFiles,...Object.keys(prior.files),...Object.keys(o.files),...Object.keys(o.libraries),
 ...Object.keys(o.configurationFiles),...Object.keys(evidence.libraries),...evidence.executables.map(b=>b.path),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
if(process.argv.includes('--record')){
 const m={checkpoint:82,status:'qualified-quadratic-extraction-source-and-capability',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),evidence,gates,currentGates,unusableGates,donorSources:o.donorSources,
  readRecords:o.records,coverage:evidence.coverageAtBinding,liveSources:prior.liveSources,retainedContinuationTransfers:7,productionChanges:0};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:82,status:'recorded',artifacts:files.length,gates:gates.length,current:17,unusable:2,
  liveFiles:957,readFiles:4,newLines:644,productionChanges:0}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,'qualified-quadratic-extraction-source-and-capability');assert.deepEqual(m.evidence,evidence);
 for(const[k,v]of Object.entries({gates,currentGates,unusableGates}))assert.deepEqual(m[k],v);
 assert.deepEqual(m.donorSources,o.donorSources);assert.deepEqual(m.readRecords,o.records);assert.deepEqual(m.coverage,evidence.coverageAtBinding);
 assert.deepEqual(m.liveSources,prior.liveSources);assert.deepEqual(retainedSources(),o.current);
 for(const b of evidence.executables){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);
 console.log(JSON.stringify({checkpoint:82,status:'verified-quadratic-extraction-source-and-capability',artifacts:files.length,gates:gates.length,
  current:17,unusable:2,liveFiles:957,readFiles:4,newLines:644,coverage:evidence.coverageAtBinding,
  mathematicalChecks:evidence.mathematical.checks,hyperRecordsPerProfile:evidence.hyper.records,
  retainedContinuationTransfers:7,productionChanges:0,executablesBytes:evidence.executables.reduce((n,b)=>n+b.bytes,0),
  limits:evidence.limits,next:evidence.next}));
}
