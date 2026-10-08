import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../../..'),refs=resolve(root,'../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const attempt=process.argv[2];assert(/^[a-z0-9-]+$/.test(attempt??''),'provide a new evidence attempt name');
const manifest=root+'/checkpoint-'+attempt+'.json';assert(!existsSync(manifest),'preserve prior checkpoint evidence');
const stages=[];
for(const script of ['analyze.mjs','public/analyze.mjs','../check-round-constants-02-04.mjs']){
 const source=resolve(root,script),started=new Date().toISOString();
 const r=spawnSync(process.execPath,[source],{encoding:'utf8',timeout:30000});
 const file=root+'/checkpoint-'+attempt+'-'+stages.length+'.log';
 writeFileSync(file,r.stdout??'');writeFileSync(file+'.stderr',r.stderr??'');
 stages.push({script,sourceSha256:hash(source),started,finished:new Date().toISOString(),status:r.status,signal:r.signal,error:r.error?.code??null,file,outputSha256:hash(file),stderrSha256:hash(file+'.stderr')});
 writeFileSync(manifest,JSON.stringify({status:'VALIDATING',stages},null,2)+'\n');
 assert.equal(r.status,0,r.stderr);assert.equal(r.error,undefined);assert.equal(r.signal,null);
}
const parse=p=>JSON.parse(readFileSync(root+'/'+p,'utf8'));
const isolated=parse('analysis.json'),caller=parse('public/analysis.json');
assert.equal(isolated.processes,48);assert.equal(caller.processes,108);
assert.equal(isolated.checksPerMode,1956);assert.equal(caller.publicChecksPerModePerAlgorithm,81);
assert(isolated.families.every(f=>f.controlWithinTenPercent));
assert(caller.families.every(f=>f.controlWithinTenPercent));
const snapshot=parse('snapshot.json');
const currentDrift=snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p,h])=>({path:p,expected:h,actual:hash(ws+'/'+p)}));
const inventory=readFileSync(refs+'/RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
const coverage=readFileSync(refs+'/RUFFINI_READ_COVERAGE.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
assert.equal(inventory.length,245);assert.equal(coverage.length,234);
let totalLines=0,readLines=0;
for(const[p,size,lineCount,h,status]of inventory){
 const bytes=readFileSync(refs+'/Ruffini/'+p),text=bytes.toString('utf8');
 assert.equal(bytes.length,+size);assert.equal(hash(refs+'/Ruffini/'+p),h);
 assert.equal(text.split('\n').length-(text.endsWith('\n')?1:0),+lineCount);
 totalLines+=+lineCount;
 if(status==='READ'){assert.deepEqual(coverage.find(r=>r[0]===p),[p,lineCount,h,'1-'+lineCount]);readLines+=+lineCount;}
 else assert.equal(status,'UNREAD');
}
assert.equal(totalLines,26234);assert.equal(readLines,17192);
const result={status:'PROGRESS; full ecosystem goal ACTIVE/OPEN',finished:new Date().toISOString(),validatorSha256:hash(import.meta.filename),stages,currentDrift,snapshotFiles:397,sourceCoverage:{files:245,physicalLines:totalLines,readFiles:234,readLines,unreadFiles:inventory.filter(r=>r[4]==='UNREAD').map(r=>r[0])},isolated:{processes:48,checksPerMode:1956,verifiedMeasuredCalls:isolated.verifiedCalls},public:{processes:108,checksPerModePerAlgorithm:81,verifiedMeasuredCalls:caller.verifiedCalls,families:caller.families},memory:isolated.allocationSummary,newProductionChanges:0,decision:'No production transfer retained at this checkpoint: exploratory public caller results do not establish a worthwhile end-to-end improvement. Helper gains and memory tradeoffs remain preserved for profile-driven follow-up.',limitations:caller.limitations};
writeFileSync(manifest,JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify({status:result.status,currentDrift,sourceCoverage:result.sourceCoverage,isolated:result.isolated,public:{...result.public,families:undefined},decision:result.decision,manifest},null,2));
