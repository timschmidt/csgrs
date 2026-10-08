import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const m=JSON.parse(readFileSync(root+'/classgroup-build-manifest.json','utf8'));
for(const[p,h]of m.sources)assert.equal(hash(p),h,p);for(const[p,h]of m.classes)assert.equal(hash(build+'/classgroup/'+p),h,p);
assert.equal(m.originalTestBuild.status,1);assert.equal(m.originalTestBuild.error,null);assert(readFileSync(root+'/classgroup-original-test-build.log','utf8').includes('package dk.jonaslindstrom.ruffini.finitefields.quadraticform does not exist'));
const runs=JSON.parse(readFileSync(root+'/classgroup-runs.json','utf8'));assert.equal(runs.length,2);assert.equal(runs[0].sha256,runs[1].sha256);
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(root+'/'+r.file),r.sha256);
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));assert.deepEqual(rows.pop(),['SUMMARY','146']);assert.equal(rows.length,146);assert.equal(new Set(rows.map(r=>r[1])).size,146);
 for(let n=1;n<=128;n++){const valid=[0,1].includes(((-n%4)+4)%4);assert.deepEqual(rows[n-1],[valid?'PASS':'WRONG','discriminant-'+n+'-'+(valid?'valid':'invalid')]);}
 for(const[status,id]of rows.slice(128))assert.equal(status,['equal-one-cache','equal-hash-contract'].includes(id)?'WRONG':'PASS');
 assert.equal(rows.filter(r=>r[0]==='PASS').length,80);assert.equal(rows.filter(r=>r[0]==='WRONG').length,66);
}
const result={probesPerMode:146,passPerMode:80,wrongPerMode:66,validDiscriminantControls:64,invalidDiscriminantsAccepted:64,cacheSensitiveEquality:true,equalHashFailure:true,coefficientGroupControlsPass:true,originalTests:'direct compilation fails on obsolete package imports; no unchanged JUnit run claimed',newProductionChanges:0};
writeFileSync(root+'/classgroup-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
