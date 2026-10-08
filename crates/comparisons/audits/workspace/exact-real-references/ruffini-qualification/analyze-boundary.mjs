import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname;
const rows=name=>readFileSync(root+'/'+name,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const sha=name=>createHash('sha256').update(readFileSync(root+'/'+name)).digest('hex');
const abs=x=>x<0n?-x:x;
const corpus=rows('rational-corpus.tsv').slice(1);
assert.equal(corpus.length,1575);
assert.equal(sha('rational-corpus.tsv'),'f66f1b004d3bb133b8b3e1dc93ba1ef6cef8f7c5b57a5485bebc3c44f7762b0f');
const runs=JSON.parse(readFileSync(root+'/boundary-runs.json','utf8')); assert.equal(runs.length,14);
for(const r of runs) {
 assert.equal(sha(r.mode+'-'+r.test+'.tsv'),r.stdoutSha256);
 if(r.test==='negative-reciprocal') {assert.equal(r.error,'ETIMEDOUT');assert.equal(r.signal,'SIGKILL');assert.deepEqual(rows(r.mode+'-'+r.test+'.tsv'),[['ENTER','public-negative-reciprocal']]);}
 else {assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);}
}
const all={};
for(const mode of ['jit','interpreter']) {
 const out=rows(mode+'-corpus.tsv');assert.equal(out.length,corpus.length);
 const groups={},failures=[];
 corpus.forEach((f,i)=>{
  const [id,op,bitsText]=f,bits=BigInt(bitsText),an=BigInt(f[3]),ad=BigInt(f[4]),bn=BigInt(f[5]),bd=BigInt(f[6]);
  assert.equal(out[i][0],'CORPUS');assert.equal(out[i][1],id);assert.equal(out[i][2],bitsText);
  let n,d;
  if(op==='add') {n=an*bd+bn*ad;d=ad*bd;}
  else if(op==='mul'||op==='public-mul') {n=an*bn;d=ad*bd;}
  else if(op==='neg') {n=-an;d=ad;}
  else if(op==='inv') {n=ad;d=an;assert(d>0n);}
  else throw Error('unknown corpus op');
  const got=BigInt(out[i][3]),error=abs(got*d-(n<<bits)),pass=error<=d;
  groups[op]??={pass:0,fail:0};groups[op][pass?'pass':'fail']++;
  if(!pass) failures.push({id,op,bits:bitsText,an:f[3],ad:f[4],bn:f[5],bd:f[6],got:got.toString(),scaledErrorNumerator:error.toString(),scaledErrorDenominator:d.toString()});
 });
 const cache=rows(mode+'-cache.tsv');assert.equal(cache.length,300);
 for(const [kind,id,p,n,d,a] of cache) {assert.equal(kind,'APPROX');assert(abs(BigInt(a)*BigInt(d)-(BigInt(n)<<BigInt(p)))<BigInt(d),id);}
 const eq=rows(mode+'-equality.tsv');assert.equal(eq.length,3);
 assert.deepEqual(eq[0],['EQUALITY','nontransitive','true','true','false','true']);
 assert.equal(eq[1][0],'HASH');assert.equal(eq[1][2],'true');assert.notEqual(eq[1][3],eq[1][4]);assert.equal(eq[1][5],'2');
 assert.deepEqual(eq[2],['CACHE','call-count','1','2','2','12']);
 // One eager call at 16, then newly evaluated indices 17..4112 inclusive.
 assert.deepEqual(rows(mode+'-signed-search.tsv'),[['BOUNDED','signed-search','4097','4112']]);
 const ints=rows(mode+'-integer-boundaries.tsv');assert.equal(ints.length,25);
 let integerPass=0,integerWrong=0,stack=0;
 for(const r of ints) {
  if(r[0]==='STACK') {stack++;continue;}
  if(r[0]==='CONTROL') {assert.equal(r[2],'1.0');integerPass++;continue;}
  assert.equal(r[0],'SCALE');if(Number(r[3])===3*Number(r[2])) integerPass++;else integerWrong++;
 }
 assert.equal(stack,7);assert.equal(integerWrong,5);assert.equal(integerPass,13);
 const format=rows(mode+'-format.tsv');assert.equal(format.length,98);
 let decimalPass=0,decimalFail=0,importPass=0;
 const decimalFailures=[];
 for(const f of format) {
  if(f[0]==='DECIMAL') {
   const [whole,frac='']=f[4].split('.'),scale=10n**BigInt(frac.length),value=BigInt(whole+frac);
   const error=abs(value-BigInt(f[3])*scale),pass=(error<<BigInt(f[2]))<=scale;
   if(pass) decimalPass++;else {decimalFail++;decimalFailures.push({id:f[1],bits:f[2],integer:f[3],got:f[4],errorNumerator:error.toString(),errorDenominator:scale.toString()});}
  } else {
   assert.equal(f[0],'IMPORT');let n=BigInt(f[3]),d=1n;const scale=Number(f[4]);if(scale>=0)d=10n**BigInt(scale);else n*=10n**BigInt(-scale);
   assert(abs(BigInt(f[5])*d-(n<<BigInt(f[2])))<=d,f[1]);importPass++;
  }
 }
 all[mode]={groups,failures,cachePass:cache.length,integerPass,integerWrong,stack,decimalPass,decimalFail,decimalFailures,importPass,equalityContractFailures:2};
}
assert.deepEqual(all.jit,all.interpreter);
writeFileSync(root+'/boundary-analysis.json',JSON.stringify(all,null,2)+'\n');
console.log(JSON.stringify({...all.jit,failures:all.jit.failures.slice(0,6),decimalFailures:all.jit.decimalFailures.slice(0,6)},null,2));
