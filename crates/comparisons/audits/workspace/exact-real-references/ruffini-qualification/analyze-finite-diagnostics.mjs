import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './analyze-finite-boundaries.mjs';
const root=import.meta.dirname,ws=resolve(root,'../..'),hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(root+'/'+p,'utf8'));
const build=json('finite-diagnostics-build-manifest.json');assert.equal(hash(build.source),build.sourceSha256);
for(const[p,h]of build.classes)assert.equal(hash(ws+'/.audit-ruffini-build.LmZgYM/finite-diagnostics-v1/'+p),h,p);
for(const[p,h]of build.dependencies)assert.equal(hash(root+'/'+p),h,p);
const previous=json('finite-boundaries-v2-run-runs.json'),runs=json('finite-diagnostics-runs.json');assert.equal(runs.length,3);
const results=[];
for(const r of runs){
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);
 assert.equal(hash(root+'/FiniteDiagnostics.java'),r.sourceSha256);assert.equal(hash(root+'/run-finite-diagnostics.mjs'),r.runnerSha256);
 for(const[p,h]of r.dependencies)assert.equal(hash(root+'/'+p),h,p);
 assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/'+r.file+'.stderr'),r.stderrSha256);assert.equal(readFileSync(root+'/'+r.file+'.stderr','utf8'),'');
 if(r.mode==='build')continue;
 const expected=new Map();for(const p of previous.filter(x=>x.name.startsWith(r.mode+'-berlekamp-')))for(const row of readFileSync(root+'/'+p.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t')))if(row[0]==='RESULT')expected.set(row[1],row);
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));assert.deepEqual(rows.pop(),['SUMMARY','249']);
 const details=rows.filter(row=>row[0]==='DETAIL');assert.equal(details.length,249);assert.equal(new Set(details.map(r=>r[1])).size,249);
 let returned=0,exhausted=0,nulls=0;
 for(const[,id,type,message]of details){const e=expected.get(id);assert(e);
  if(type==='VALUE'){assert.equal(e[2],'PASS');assert.equal(message,e[3]);returned++;}
  else if(type==='IllegalArgumentException'){assert.equal(e[2],'UNRESOLVED');assert.equal(message,'Exceeded max number of iterations without finding a root for the given polynomial');exhausted++;}
  else{assert.equal(type,'NullPointerException');assert.equal(e[2],'EXCEPTION');assert(message.includes('Integer.intValue()'));nulls++;}
 }
 const frames=rows.filter(r=>r[0]==='FRAME');assert.equal(frames.length,6);assert(frames.every(r=>r[2]==='dk.jonaslindstrom.ruffini.integers.structures.Integers.negate(Integers.java:44)'));
 assert.deepEqual(rows.filter(r=>r[0]==='HIGH'),[['HIGH','StackOverflowError']]);const highFrames=rows.filter(r=>r[0]==='HIGHFRAME').map(r=>r[1]);assert.equal(highFrames.length,3);
 // The interpreter exhausts its stack during Integer boxing before the next
 // Power call. Do not mistake different top frames for different arithmetic.
 if(r.mode==='jit')assert(highFrames.every(s=>/^dk\.jonaslindstrom\.ruffini\.common\.algorithms\.Power\.apply\(Power.java:\d+\)$/.test(s)));
 else assert.deepEqual(highFrames,['java.base/java.lang.Number.<init>(Number.java:59)','java.base/java.lang.Integer.<init>(Integer.java:1029)','java.base/java.lang.Integer.valueOf(Integer.java:1006)']);
 results.push({mode:r.mode,returned,confirmedIterationExhaustions:exhausted,confirmedNullErrors:nulls,highSquareOverflowConfirmed:true,highFrames,outputSha256:r.sha256});
}
assert.deepEqual(results.map(({mode,outputSha256,highFrames,...rest})=>rest),Array(2).fill({returned:133,confirmedIterationExhaustions:110,confirmedNullErrors:6,highSquareOverflowConfirmed:true}));
const snapshot=json('square-root-pilot/snapshot.json');const currentDrift=snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p,h])=>({path:p,expected:h,actual:hash(ws+'/'+p)}));
const coverage=readFileSync(root+'/../RUFFINI_READ_COVERAGE.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));assert.equal(coverage.length,235);assert.equal(coverage.reduce((a,r)=>a+(+r[1]),0),17768);
for(const[p,n,h,range]of coverage){assert.equal(hash(root+'/../Ruffini/'+p),h,p);assert.equal(range,'1-'+n);}
const result={status:'PROGRESS; full ecosystem audit OPEN',results,currentHyperSnapshotFiles:snapshot.files.length,currentDrift,coverage:{files:235,physicalLines:17768,totalFiles:245,totalPhysicalLines:26234},sourceSha256:hash(import.meta.filename),numericalValidatorSha256:hash(root+'/analyze-finite-boundaries.mjs'),newProductionChanges:0};writeFileSync(root+'/finite-diagnostics-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
