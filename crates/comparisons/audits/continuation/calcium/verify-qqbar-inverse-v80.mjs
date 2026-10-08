import {writeFileSync,readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {inverseFinalEvidence,evidenceFiles} from './qqbar-inverse-evidence-final-v80.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const path='qqbar-inverse-v80-manifest.json',evidence=inverseFinalEvidence(),o=json('qqbar-inverse-origin-v80.json'),prior=json('twelfth-revision-v79-manifest.json');
const tag='qqbar-inverse-evidence-final-v80',g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(g.command,'node');assert.deepEqual(g.args,['qqbar-inverse-evidence-final-v80.mjs']);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
assert.deepEqual(evidence,json('results/'+tag+'.stdout'));
const currentGates=[...evidence.successfulGates,tag],developmentGates=evidence.developmentGates,unusableGates=evidence.preservedUnusableGates;
const gates=[...currentGates,...developmentGates,...unusableGates];assert.equal(gates.length,25);assert.equal(currentGates.length,22);
const files=[...new Set([...evidenceFiles,...Object.keys(prior.files),...Object.keys(o.files),...Object.keys(o.libraries),
 ...Object.keys(o.configurationFiles),...Object.keys(evidence.libraries),...evidence.executables.map(b=>b.path),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
if(process.argv.includes('--record')){
 const manifest={checkpoint:80,status:'qualified-inverse-source-and-completeness-audit',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),evidence,gates,currentGates,developmentGates,unusableGates,
  donorSources:o.donorSources,readRecords:o.records,coverage:evidence.coverageAtBinding,liveSources:prior.liveSources,
  retainedContinuationTransfers:7,productionChanges:0};
 writeFileSync(path,JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:80,status:'recorded',artifacts:files.length,gates:gates.length,current:22,development:1,unusable:2,
  liveFiles:957,readFiles:19,newLines:1738,productionChanges:0}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,'qualified-inverse-source-and-completeness-audit');assert.deepEqual(m.evidence,evidence);
 for(const[k,v]of Object.entries({gates,currentGates,developmentGates,unusableGates}))assert.deepEqual(m[k],v);
 assert.deepEqual(m.donorSources,o.donorSources);assert.deepEqual(m.readRecords,o.records);assert.deepEqual(m.coverage,evidence.coverageAtBinding);
 assert.deepEqual(m.liveSources,prior.liveSources);assert.deepEqual(retainedSources(),o.current);
 for(const b of evidence.executables){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);
 console.log(JSON.stringify({checkpoint:80,status:'verified-inverse-source-and-completeness-audit',artifacts:files.length,gates:gates.length,
  current:22,development:1,unusable:2,liveFiles:957,readFiles:19,newLines:1738,coverage:evidence.coverageAtBinding,
  mathematicalChecks:evidence.mathematical.totalChecks,hyperRecordsPerProfile:evidence.hyper.records,
  counterexample:{denominator:3360,degree:384,recognition:0,exactPrincipalAngle:'1/3360'},retainedContinuationTransfers:7,
  productionChanges:0,executablesBytes:evidence.executables.reduce((n,b)=>n+b.bytes,0),limits:evidence.limits,next:evidence.next}));
}
