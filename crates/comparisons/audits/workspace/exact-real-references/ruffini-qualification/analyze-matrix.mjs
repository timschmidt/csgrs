import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../..'),build=ws+'/.audit-ruffini-build.LmZgYM';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const rows=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const manifest=JSON.parse(readFileSync(root+'/matrix-build-manifest.json','utf8'));
for(const[p,h]of manifest.sources)assert.equal(hash(p),h,p);
for(const[p,h]of manifest.classes)assert.equal(hash(build+'/matrix/'+p),h,p);
const runs=JSON.parse(readFileSync(root+'/matrix-runs.json','utf8'));assert.equal(runs.length,8);
assert.equal(new Set(runs.map(r=>r.mode+'/'+r.test)).size,8);
const singletonBoundary={
 'complex-identity-inverse':['EXCEPTION','IllegalArgumentException'],
 'real-identity-inverse':['PASS'],
 'sparse-builder-snapshot':['WRONG'],
 'mutable-copy-is-independent':['PASS'],
 'vector-unequal-length':['WRONG'],
 'vector-shorter-right':['EXCEPTION','IndexOutOfBoundsException'],
 'complex-projection':['WRONG'],
 'qr-tall':['EXCEPTION','ArrayIndexOutOfBoundsException'],
 'ksubsets-list':['WRONG'],
 'ksubsets-array-control':['PASS'],
 'multidimensional-width-constructor':['WRONG'],
 'multidimensional-shape-control':['PASS'],
 'limb-carry':['WRONG'],
 'limb-multiply':['WRONG'],
 'int-min-norm':['WRONG'],
 'int-division-overflow':['WRONG'],
};
const summaries={};
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);
 const file=r.mode+'-matrix-'+r.test+'.log';assert.equal(hash(root+'/'+file),r.sha256);
 if(r.test==='junit-integers'){assert(readFileSync(root+'/'+file,'utf8').includes('OK (2 tests)'));continue;}
 const data=rows(file),summary=data.pop(),expected={arithmetic:1809,boundaries:599,transforms:69}[r.test];
 assert.deepEqual(summary,['SUMMARY',String(expected)]);assert.equal(data.length,expected);assert.equal(new Set(data.map(r=>r[1])).size,expected);
 const groups={};for(const [status,id,error]of data){assert(['PASS','WRONG','EXCEPTION'].includes(status));const key=id.replace(/-[0-9].*$/,'')+'/'+status+(error?'/'+error:'');groups[key]=(groups[key]??0)+1;}
 if(r.test==='arithmetic') {
  for(const row of data) {
   const[status,id,error]=row;
   if(id.startsWith('strassen-')) {const[,seed,nText,bText]=id.split('-'),n=+nText,b=+bText;const fails=(n&(n-1))!==0&&n>b&&b>1;assert.equal(status,fails?'EXCEPTION':'PASS');if(fails)assert.equal(error,'NullPointerException');}
   else assert.equal(status,id==='det-empty-lazy'?'WRONG':'PASS');
  }
 } else if(r.test==='transforms') for(const row of data)assert.equal(row[0],'PASS');
 else {
  const seenSingletons=new Set();
  for(const [status,id,error]of data) {
   if(id.startsWith('congruence-'))assert.equal(status,id.endsWith('-0')?'PASS':'WRONG');
   else if(id.startsWith('modsqrt-')) {assert.equal(status,id.startsWith('modsqrt-17-')?'EXCEPTION':'PASS');if(status==='EXCEPTION')assert.equal(error,'IllegalArgumentException');}
   else if(id.startsWith('matrix-equals-')){const f=id.split('-');assert.equal(status,f[2]===f[3]?'PASS':'EXCEPTION');if(status==='EXCEPTION')assert(['IndexOutOfBoundsException','ArrayIndexOutOfBoundsException'].includes(error));}
   else if(id.startsWith('collapse-columns-')){const f=id.split('-');assert.equal(status,f[2]===f[3]?'PASS':+f[2]<+f[3]?'WRONG':'EXCEPTION');if(status==='EXCEPTION')assert(['IndexOutOfBoundsException','ArrayIndexOutOfBoundsException'].includes(error));}
   else {assert(Object.hasOwn(singletonBoundary,id),id);assert.deepEqual([status,...(error?[error]:[])],singletonBoundary[id],id);seenSingletons.add(id);}
  }
  assert.equal(seenSingletons.size,Object.keys(singletonBoundary).length);
 }
 summaries[r.mode]??={};summaries[r.mode][r.test]=groups;
}
assert.deepEqual(summaries.jit,summaries.interpreter);
for(const test of ['arithmetic','boundaries','transforms'])assert.equal(runs.find(r=>r.mode==='jit'&&r.test===test).sha256,runs.find(r=>r.mode==='interpreter'&&r.test===test).sha256);
const hyper=JSON.parse(readFileSync(root+'/hyper-matrix-runs.json','utf8'));assert.equal(hyper.length,2);
const snapshot=JSON.parse(readFileSync(root+'/hyper-matrix-source-snapshot.json','utf8'));for(const[p,h]of snapshot)assert.equal(hash(ws+'/'+p),h,p);
for(const r of hyper) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(r.binary),r.binarySha256);assert.equal(hash(root+'/matrix-hyper/controls.rs'),r.sourceSha256);assert.equal(hash(root+'/'+r.file),r.sha256);
 const data=rows(r.file);assert.deepEqual(data.pop(),['SUMMARY','420']);assert.equal(data.length,252);for(const row of data)assert.equal(row[0],'PASS');
}
assert.equal(hyper[0].sha256,hyper[1].sha256);
const result={native:summaries.jit,nativeProbesPerMode:2477,originalIntegerJUnitPassPerMode:2,hyperChecksPerMode:420,hyperSnapshotFiles:snapshot.length,newProductionChanges:0,status:'qualified partial matrix checkpoint'};
writeFileSync(root+'/matrix-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
