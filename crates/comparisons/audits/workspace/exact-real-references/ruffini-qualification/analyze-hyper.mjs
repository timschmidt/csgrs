import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const rows=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const abs=n=>n<0n?-n:n;
for(const [p,h] of JSON.parse(readFileSync(root+'/hyper-source-snapshot.json','utf8')))assert.equal(hash(ws+'/hyperreal/'+p),h,p);
const corpus=rows('rational-corpus.tsv').slice(1);
const runs=JSON.parse(readFileSync(root+'/hyper-runs.json','utf8'));assert.equal(runs.length,2);
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);
 assert.equal(hash(root+'/hyper_boundary.rs'),r.sourceSha256);assert.equal(hash(r.binary),r.binarySha256);assert.equal(hash(root+'/hyper-'+r.mode+'.tsv'),r.outputSha256);
 const data=rows('hyper-'+r.mode+'.tsv');assert.deepEqual(data.pop(),['SUMMARY','1575','584']);
 assert.equal(data.filter(r=>r[0]==='PASS').length,560); // 12 integer rows each check three identities.
 corpus.forEach((f,i)=>{
  const [kind,id,bits,a]=data[i];assert.equal(kind,'CORPUS');assert.equal(id,f[0]);assert.equal(bits,f[2]);
  const an=BigInt(f[3]),ad=BigInt(f[4]),bn=BigInt(f[5]),bd=BigInt(f[6]);let n,d;
  if(f[1]==='add'){n=an*bd+bn*ad;d=ad*bd;}else if(f[1]==='mul'||f[1]==='public-mul'){n=an*bn;d=ad*bd;}else if(f[1]==='neg'){n=-an;d=ad;}else if(f[1]==='inv'){n=ad;d=an;}else throw Error('unknown op');
  assert(abs(BigInt(a)*d-(n<<BigInt(bits)))<=d,id);
 });
 assert.equal(data.length,1575+560);
}
assert.equal(runs[0].outputSha256,runs[1].outputSha256);
const summary={modes:2,rationalCasesPerMode:1575,additionalChecksPerMode:584,directedMpfrIrrationalChecksPerMode:480,sourceFiles:JSON.parse(readFileSync(root+'/hyper-source-snapshot.json','utf8')).length,status:'PASS',newProductionChanges:0};
writeFileSync(root+'/hyper-analysis.json',JSON.stringify(summary,null,2)+'\n');console.log(JSON.stringify(summary,null,2));
