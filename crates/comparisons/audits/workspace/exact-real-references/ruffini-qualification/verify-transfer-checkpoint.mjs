import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
for(const[file,folder]of [['native-build-manifest.json','classes'],['shared-build-manifest.json','tests']]){
 const m=JSON.parse(readFileSync(root+'/'+file,'utf8'));
 for(const[p,h]of m.sources)assert.equal(hash(p),h,p);
 for(const[p,h]of m.classes)assert.equal(hash(build+'/'+folder+'/'+p),h,p);
 if(m.dependencies)for(const[n,h]of Object.entries(m.dependencies))assert.equal(hash(build+'/deps/'+n),h,n);
}
const stages=[];
const historicalLiveGuards=new Map([
 ['analyze-hyper.mjs',['hyper-source-snapshot.json','hyperreal/']],
 ['analyze-matrix.mjs',['hyper-matrix-source-snapshot.json','']],
 ['analyze-polynomial.mjs',['hyper-polynomial-source-snapshot.json','']],
]);
for(const script of ['inventory.mjs','analyze-boundary.mjs','analyze-shared.mjs','analyze-hyper.mjs','analyze-matrix.mjs','analyze-matrix-bench.mjs','analyze-matrix-bench-stable.mjs','analyze-polynomial.mjs','analyze-polynomial-bench.mjs','analyze-classgroup.mjs','analyze-elliptic-coordinates.mjs','check-read-matrices.mjs','check-remaining-matrices.mjs','check-read-round-constants.mjs','hyper-polynomial-pilot/analyze.mjs']){
 const r=spawnSync(process.execPath,[root+'/'+script],{encoding:'utf8',timeout:60000,maxBuffer:16*1024*1024});
 assert.equal(r.error,undefined,script);assert.equal(r.signal,null,script);
 let classification='PASS',liveSourceDrift=[];
 if(historicalLiveGuards.has(script)){
  const [file,prefix]=historicalLiveGuards.get(script),snapshot=JSON.parse(readFileSync(root+'/'+file,'utf8'));
  liveSourceDrift=snapshot.map(([reportedPath,expected])=>({path:prefix+reportedPath,reportedPath,expected,actual:hash(resolve(root,'../..',prefix+reportedPath))})).filter(p=>p.expected!==p.actual);
 }
 if(liveSourceDrift.length){
  assert.equal(r.status,1,script+' must reject changed live source');
  assert(r.stderr.includes('AssertionError [ERR_ASSERTION]: '+liveSourceDrift[0].reportedPath));
  assert(r.stderr.includes(liveSourceDrift[0].expected)&&r.stderr.includes(liveSourceDrift[0].actual));
  classification='SOURCE_DRIFT: historical Hyper qualification NOT revalidated';
 }else assert.equal(r.status,0,script+' '+r.stderr);
 stages.push({script,sourceSha256:hash(root+'/'+script),status:r.status,classification,liveSourceDrift,stdoutSha256:createHash('sha256').update(r.stdout).digest('hex'),stderr:r.stderr});
 console.log(classification+' '+script);
}
const rows=readFileSync(root+'/../RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
assert.equal(rows.length,245);const read=rows.filter(r=>r[4]==='READ');assert.equal(read.length,230);
assert.equal(read.reduce((s,r)=>s+ +r[2],0),15691);
const pilot=JSON.parse(readFileSync(root+'/hyper-polynomial-pilot/analysis.json','utf8'));
assert.equal(pilot.correctnessChecksPerMode,3245);assert.equal(pilot.coefficientExport.checksPerMode,4905);
assert.equal(pilot.stable.processes,60);assert.equal(pilot.stable.verifiedProducts,7453824);assert.equal(pilot.allocationProfiles,288);
assert(pilot.families.every(f=>f.identicalCodeControlWithinTenPercent));assert.equal(pilot.productionChanges,0);
const result={created:new Date().toISOString(),stages,passedStages:stages.filter(s=>s.status===0).length,sourceDriftStages:stages.filter(s=>s.status!==0).map(s=>s.script),readFiles:read.length,readLines:15691,unreadFiles:245-read.length,hyperSnapshotFiles:pilot.snapshotFiles,currentHyperDrift:pilot.currentDrift,coefficientExport:pilot.coefficientExport,newProductionChanges:0,oldHyperQualification:'not reclassified; original scalar/matrix/polynomial guards reject changed live source; exact drift and errors recorded; old frozen evidence preserved',scope:'Ruffini source/native evidence and isolated frozen Hyper transfer study only; source-drift stages are NOT passes',actualCallerAndSizeRetentionGates:'OPEN',fullRuffiniAudit:'OPEN',fullEcosystemGoal:'ACTIVE/OPEN'};
writeFileSync(root+'/transfer-checkpoint-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({...result,stages:stages.length},null,2));
