import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../..'),build=ws+'/.audit-ruffini-build.LmZgYM';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const rows=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const manifest=JSON.parse(readFileSync(root+'/polynomial-build-manifest.json','utf8'));
for(const[p,h]of manifest.sources)assert.equal(hash(p),h,p);
for(const[p,h]of manifest.classes)assert.equal(hash(build+'/polynomial/'+p),h,p);
assert.equal(manifest.sourceCount,manifest.sources.length);
const expectedBoundaries={
 'sparse-equality-zero':['WRONG'],'sparse-equality-nonzero':['WRONG'],'trailing-zero-equality':['WRONG'],
 'constant-derivative-zero':['EXCEPTION','NoSuchElementException'],'empty-polynomial-zero':['EXCEPTION','NoSuchElementException'],
 'builder-snapshot':['WRONG'],'copy-snapshot':['WRONG'],'fastdivision-monomial':['WRONG'],
 'field-division-nonmonic-control':['PASS'],'karatsuba-field-division-nonmonic':['WRONG'],
 'batch-zero-1':['PASS'],'batch-sparse-1':['EXCEPTION','NullPointerException'],
 'batch-zero-2':['PASS'],'batch-sparse-2':['EXCEPTION','NullPointerException'],
 'batch-zero-3':['EXCEPTION','IllegalArgumentException'],'batch-sparse-3':['EXCEPTION','IllegalArgumentException'],
 'batch-zero-4':['PASS'],'batch-sparse-4':['EXCEPTION','NullPointerException'],
 'batch-zero-8':['PASS'],'batch-sparse-8':['EXCEPTION','NullPointerException'],
 'tree-interpolation-1':['PASS'],'lagrange-control-1':['PASS'],'tree-interpolation-2':['PASS'],'lagrange-control-2':['PASS'],
 'tree-interpolation-4':['EXCEPTION','NullPointerException'],'lagrange-control-4':['PASS'],
 'tree-interpolation-8':['EXCEPTION','NullPointerException'],'lagrange-control-8':['PASS'],
 'fft-cyclic-not-polynomial':['WRONG'],'fft-oversize-import':['WRONG'],'fft-zero-polynomial':['EXCEPTION','NoSuchElementException'],
 'multivariate-leading-coefficient':['WRONG'],'multivariate-snapshot':['WRONG'],'multivariate-equality-explicit-zero':['PASS'],
 'monomial-array-snapshot':['WRONG'],'recursive-add-alias':['WRONG'],
};
const runs=JSON.parse(readFileSync(root+'/polynomial-runs.json','utf8'));assert.equal(runs.length,28);assert.equal(new Set(runs.map(r=>r.mode+'/'+r.test)).size,28);
const counts={arithmetic:2630,boundaries:36,parser:17,permutations:7,fields:136,bigfields:132,gaussian:1,quadratic:1};
const summary={};
for(const r of runs) {
 assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/'+r.file+'.stderr'),r.stderrSha256);
 assert(['jit','interpreter'].includes(r.mode));assert(r.args.includes('-ea'));assert.equal(r.args.includes('-Xint'),r.mode==='interpreter');
 if(['trailing-division','multivariate-division'].includes(r.test)) {assert.equal(r.status,null);assert.equal(r.error,'ETIMEDOUT');assert.equal(r.signal,'SIGKILL');assert.equal(r.capMs,2000);assert.equal(readFileSync(root+'/'+r.file,'utf8'),'');continue;}
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);
 if(r.test.startsWith('junit-')) {const expected=r.test==='junit-polynomials'?7:1;assert(readFileSync(root+'/'+r.file,'utf8').includes('OK ('+expected+' test'+(expected===1?'':'s')+')'));continue;}
 const data=rows(r.file);assert.deepEqual(data.pop(),['SUMMARY',String(counts[r.test])]);
 const probes=data.filter(r=>r[0]!=='OBS');assert.equal(probes.length,counts[r.test]);assert.equal(new Set(probes.map(r=>r[1])).size,probes.length);
 const families={};
 for(const[status,id,error]of probes) {
  assert(['PASS','WRONG','EXCEPTION'].includes(status));
  const group=id.replace(/-[0-9].*$/,'')+'/'+status+(error?'/'+error:'');families[group]=(families[group]??0)+1;
  if(r.test==='arithmetic')assert.equal(status,'PASS');
  if(r.test==='boundaries')assert.deepEqual([status,...(error?[error]:[])],expectedBoundaries[id],id);
  if(r.test==='fields')assert.equal(status,id.startsWith('base-prime-')||id.endsWith('-1')?'PASS':'WRONG');
  if(r.test==='bigfields')assert.equal(status,id.startsWith('normalized-extension-')||id.endsWith('-1')?'PASS':'WRONG');
  if(['gaussian','quadratic'].includes(r.test))assert.deepEqual([status,error],['EXCEPTION','ArithmeticException']);
  if(r.test==='permutations')assert.equal(status,id==='uniform-2'?'PASS':'WRONG');
 }
 if(r.test==='parser') {
  const wanted=['PASS','PASS','PASS','PASS','PASS','PASS','WRONG','WRONG','WRONG','EXCEPTION','PASS','PASS'];
  for(let i=0;i<12;i++)assert.equal(probes.find(r=>r[1]==='parser-'+i)[0],wanted[i]);
  for(const id of ['parser-malformed-)','parser-malformed-1,2'])assert.deepEqual(probes.find(r=>r[1]===id),['EXCEPTION',id,'NullPointerException']);
  assert.equal(probes.find(r=>r[1]==='parser-malformed-(1')[0],'PASS');
  assert.equal(probes.find(r=>r[1]==='integer-polynomial-repeated-power')[0],'WRONG');
  assert.deepEqual(probes.find(r=>r[1]==='integer-polynomial-negative-implicit'),['EXCEPTION','integer-polynomial-negative-implicit','NumberFormatException']);
  const obs=data.filter(r=>r[0]==='OBS');assert.equal(obs.length,11);assert.equal(obs.find(r=>r[1]==='parser-f(2)+3')[2],'25.0');assert.equal(obs.find(r=>r[1]==='parser-f(2)*3')[2],'36.0');assert.equal(obs.find(r=>r[1]==='parser-2^3^2')[2],'64.0');
 }
 if(r.test==='permutations')assert.deepEqual(data.filter(r=>r[0]==='OBS'),[
  ['OBS','uniform-2','2','2','1','1'],['OBS','uniform-3','6','4','1','2'],['OBS','uniform-4','24','11','1','4'],
  ['OBS','uniform-5','120','40','1','7'],['OBS','uniform-6','720','162','1','17'],['OBS','uniform-7','5040','788','1','34'],
 ]);
 summary[r.mode]??={};summary[r.mode][r.test]=families;
}
assert.deepEqual(summary.jit,summary.interpreter);
for(const test of Object.keys(counts))assert.equal(runs.find(r=>r.mode==='jit'&&r.test===test).sha256,runs.find(r=>r.mode==='interpreter'&&r.test===test).sha256);
const progressManifest=JSON.parse(readFileSync(root+'/polynomial-progress-build-manifest.json','utf8'));assert.equal(hash(root+'/PolynomialProgress.java'),progressManifest.sourceSha256);for(const[p,h]of progressManifest.classes)assert.equal(hash(build+'/polynomial/'+p),h,p);
const progressRuns=JSON.parse(readFileSync(root+'/polynomial-progress-runs.json','utf8'));assert.equal(progressRuns.length,2);
for(const r of progressRuns){assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(root+'/'+r.file),r.sha256);assert.deepEqual(rows(r.file),[['PREFIX','univariate','1025','unchanged-1-plus-0x'],['PREFIX','multivariate','1025','cycle-1-2-4-3']]);}
const hyper=JSON.parse(readFileSync(root+'/hyper-polynomial-runs.json','utf8'));assert.equal(hyper.length,2);
const snapshot=JSON.parse(readFileSync(root+'/hyper-polynomial-source-snapshot.json','utf8'));for(const[p,h]of snapshot)assert.equal(hash(ws+'/'+p),h,p);
for(const r of hyper){assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(r.binary),r.binarySha256);assert.equal(hash(root+'/polynomial-hyper/controls.rs'),r.sourceSha256);assert.equal(hash(root+'/'+r.file),r.sha256);assert.deepEqual(rows(r.file),[['PASS','evaluation','420'],['PASS','exact-division','675'],['PASS','monomial-division','117'],['PASS','monic-gcd','49'],['PASS','bivariate-division','243'],['SUMMARY','1520']]);}
assert.equal(hyper[0].sha256,hyper[1].sha256);
const result={nativeProbesPerMode:Object.values(counts).reduce((a,b)=>a+b,0),native:summary.jit,originalJUnitPassPerMode:10,boundedTimeoutsPerMode:2,verifiedProgressPrefixesPerMode:2,verifiedPrefixObservationsPerPath:1025,hyperChecksPerMode:1520,hyperSnapshotFiles:snapshot.length,newProductionChanges:0,status:'qualified partial polynomial checkpoint'};
writeFileSync(root+'/polynomial-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
