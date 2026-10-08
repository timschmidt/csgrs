import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM/elliptic-coordinates');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const buildRoot=resolve(build,'..');
for(const[file,folder]of [['native-build-manifest.json','classes'],['matrix-build-manifest.json','matrix'],['polynomial-build-manifest.json','polynomial']]){
 const dependency=JSON.parse(readFileSync(root+'/'+file,'utf8'));
 for(const[p,h]of dependency.sources)assert.equal(hash(p),h,p);
 for(const[p,h]of dependency.classes)assert.equal(hash(buildRoot+'/'+folder+'/'+p),h,p);
 if(dependency.dependencies)for(const[name,h]of Object.entries(dependency.dependencies))assert.equal(hash(buildRoot+'/deps/'+name),h,name);
}
const manifest=JSON.parse(readFileSync(root+'/elliptic-coordinate-build-manifest.json','utf8'));
assert.equal(manifest.sources.length,8);assert.equal(manifest.release,16);
for(const[p,h]of manifest.sources)assert.equal(hash(p),h,p);
for(const[p,h]of manifest.classes)assert.equal(hash(build+'/'+p),h,p);
const runs=JSON.parse(readFileSync(root+'/elliptic-coordinate-runs.json','utf8'));assert.equal(runs.length,2);
let summary;
for(const r of runs){
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert(r.args.includes('-ea'));assert.equal(r.args.includes('-Xint'),r.mode==='interpreter');
 assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/'+r.file+'.stderr'),r.stderrSha256);assert.equal(readFileSync(root+'/'+r.file+'.stderr','utf8'),'');
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
 const tail=rows.pop();assert.equal(tail[0],'SUMMARY');assert.equal(rows.length,+tail[1]);assert.equal(new Set(rows.map(r=>r[1])).size,rows.length);
 const families={};for(const[status,id,error]of rows){
  assert(['PASS','WRONG','EXCEPTION'].includes(status));const family=id.replace(/-[0-9].*$/,'');
  families[family]??={};const key=status+(error?'/'+error:'');families[family][key]=(families[family][key]??0)+1;
  if(['affine-add','projective-conversion','projective-doubling','montgomery-add'].includes(family))assert.equal(status,'PASS');
  if(family==='projective-invalid-equality')assert.equal(status,'WRONG');
  if(family==='affine-negate'||family==='montgomery-negate')assert.equal(status,id.endsWith('-0')?'EXCEPTION':'PASS');
 }
 const result={probesPerMode:rows.length,families};if(summary)assert.deepEqual(result,summary);else summary=result;
}
assert.equal(runs[0].sha256,runs[1].sha256);
assert.deepEqual(summary,{probesPerMode:1672,families:{
 'affine-negate':{'EXCEPTION/NullPointerException':3,PASS:25},
 'projective-conversion':{PASS:206},'jacobian-conversion':{PASS:54,WRONG:152},
 'projective-doubling':{PASS:206},'affine-add':{PASS:302},'projective-add':{PASS:228,WRONG:74},
 'projective-invalid-equality':{WRONG:3},'montgomery-negate':{'EXCEPTION/NullPointerException':3,PASS:29},
 'montgomery-add':{PASS:384},'montgomery-invariant':{WRONG:3},
}});
const result={...summary,unchangedDonorSourceFiles:7,originalJUnitRun:false,scope:'local small-field coordinate and group-law controls; not whole-module or cryptographic qualification',newProductionChanges:0};
writeFileSync(root+'/elliptic-coordinate-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
