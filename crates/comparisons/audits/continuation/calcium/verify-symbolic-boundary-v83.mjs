import {writeFileSync,readFileSync,statSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {boundaryEvidence,evidenceFiles}from './symbolic-boundary-evidence-v83.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const path='symbolic-boundary-v83-manifest.json',evidence=boundaryEvidence(),o=json('symbolic-boundary-origin-v83.json'),prior=json('quadratic-extraction-v82-manifest.json');
const tag='symbolic-boundary-evidence-v83',g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(g.command,'node');assert.deepEqual(g.args,['symbolic-boundary-evidence-v83.mjs']);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
assert.deepEqual(evidence,json('results/'+tag+'.stdout'));
const gates=[...evidence.gates,{tag,classification:'success'}];assert.equal(gates.length,55);
const files=[...new Set([...evidenceFiles,...Object.keys(prior.files),...Object.keys(o.files),...Object.keys(o.libraries),...Object.keys(o.configurationFiles),
 ...Object.keys(evidence.libraries),...evidence.executables.map(b=>b.path),...gates.flatMap(g=>['json','stdout','stderr'].map(e=>'results/'+g.tag+'.'+e))])].sort();
if(process.argv.includes('--record')){
 const m={checkpoint:83,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  evidence,gates,donorSources:o.donorSources,readRecords:o.records,coverage:evidence.coverageAtBinding,liveSources:prior.liveSources,
  retainedContinuationTransfers:7,productionChanges:0};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:83,status:'recorded',artifacts:files.length,gates:gates.length,successful:43,diagnosticFailures:10,harnessFailures:2,
  readFiles:26,newLines:5728,liveFiles:957,productionChanges:0}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,evidence.status);assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);
 assert.deepEqual(m.donorSources,o.donorSources);assert.deepEqual(m.readRecords,o.records);assert.deepEqual(m.coverage,evidence.coverageAtBinding);
 assert.deepEqual(m.liveSources,prior.liveSources);assert.deepEqual(retainedSources(),o.current);
 for(const b of evidence.executables){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.productionChanges,0);
 console.log(JSON.stringify({checkpoint:83,status:'verified-source-and-classified-symbolic-defects',artifacts:files.length,gates:gates.length,
  successful:43,diagnosticFailures:10,harnessFailures:2,readFiles:26,newLines:5728,coverage:evidence.coverageAtBinding,liveFiles:957,
  symbolicFalseSuccessRows:136,complexMathematicalChecks:evidence.complex.checks,hyperRecordsPerProfile:evidence.hyper.records,
  normEquality:evidence.hyper.norms,retainedContinuationTransfers:7,productionChanges:0,executablesBytes:5971960,limits:evidence.limits,next:evidence.next}));
}
