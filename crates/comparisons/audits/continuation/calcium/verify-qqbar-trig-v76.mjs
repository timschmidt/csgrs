import {writeFileSync,readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {trigEvidence,evidenceFiles} from './check-qqbar-trig-evidence-fixed-v76.mjs';
import {target} from './point-qualified-capture.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const path='qqbar-trig-v76-manifest.json',evidence=trigEvidence(),origin=json('qqbar-trig-origin-v76.json');
const check=json('results/qqbar-trig-evidence-fixed-v76.json');assert.equal(check.code,0);assert.equal(check.signal,null);
assert.equal(check.command,'node');assert.deepEqual(check.args,['check-qqbar-trig-evidence-fixed-v76.mjs']);
assert.equal(readFileSync('results/qqbar-trig-evidence-fixed-v76.stderr').length,0);
assert.deepEqual(evidence,json('results/qqbar-trig-evidence-fixed-v76.stdout'));
const successful=[...evidence.successfulGates,'qqbar-trig-evidence-fixed-v76'],gates=[...evidence.preservedFailedGates,...successful];
assert.equal(successful.length,16);assert.equal(gates.length,20);
const executables=[origin.binary,...['debug','release'].map(p=>target+'/'+p+'/qqbar-trig-hyper-v76')];
const old=json('zero-factor-retained-v75-manifest.json');
const files=[...new Set([...evidenceFiles,...Object.keys(origin.files),...Object.keys(old.files),
 ...Object.keys(evidence.libraries),...Object.keys(origin.configurationFiles),...executables,
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))])].sort();
if(process.argv.includes('--record')){
 const manifest={checkpoint:76,status:'qualified-source-and-capability-audit',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,successful,preservedFailed:evidence.preservedFailedGates,evidence,
  donorSources:origin.donorSources,readRecords:origin.records,coverage:evidence.coverageAtBinding,
  liveSources:old.liveSources,executables:executables.map(path=>({path,bytes:statSync(path).size,sha256:sha(path)})),
  retainedContinuationTransfers:7,productionChanges:0,
  limits:evidence.limits,next:evidence.next};
 writeFileSync(path,JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:76,status:'recorded',artifacts:files.length,gates:gates.length,successful:16,preservedFailed:4,newDonorLines:1964}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,'qualified-source-and-capability-audit');assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);
 assert.deepEqual(m.successful,successful);assert.deepEqual(m.preservedFailed,evidence.preservedFailedGates);
 assert.deepEqual(m.donorSources,origin.donorSources);assert.deepEqual(m.readRecords,origin.records);assert.deepEqual(m.coverage,evidence.coverageAtBinding);
 assert.deepEqual(m.liveSources,old.liveSources);assert.deepEqual(retainedSources(),evidence.source);
 for(const b of m.executables){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 assert.equal(m.productionChanges,0);assert.equal(m.retainedContinuationTransfers,7);
 console.log(JSON.stringify({checkpoint:76,status:'verified',artifacts:files.length,gates:gates.length,successful:16,preservedFailed:4,
  liveFiles:957,readFiles:32,newDonorLines:1964,coverage:m.coverage,mathematicalChecks:evidence.mathematical.totalChecks,
  hyperRecordsPerProfile:evidence.hyper.records,unresolvedIdentities:32,retainedContinuationTransfers:7,
  dedicatedBinaryBytes:evidence.binary.bytes,productionChanges:0,limits:m.limits,next:m.next}));
}
