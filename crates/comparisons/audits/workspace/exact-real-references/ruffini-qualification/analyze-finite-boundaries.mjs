import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex'),json=p=>JSON.parse(readFileSync(root+'/'+p,'utf8'));
for(const[file,folder]of [['native-build-manifest.json','classes'],['matrix-build-manifest.json','matrix'],['polynomial-build-manifest.json','polynomial'],['finite-boundaries-build-manifest.json','finite-boundaries-v1'],['finite-boundaries-v2-build-manifest.json','finite-boundaries-v2']]){
 const m=json(file);for(const[p,h]of m.sources??[[m.source,m.sourceSha256]])assert.equal(hash(p),h,p);
 for(const[p,h]of m.classes)assert.equal(hash(build+'/'+folder+'/'+p),h,p);
 if(Array.isArray(m.dependencies))for(const[p,h]of m.dependencies)assert.equal(hash(root+'/'+p),h,p);
 else if(m.dependencies)for(const[p,h]of Object.entries(m.dependencies))assert.equal(hash(build+'/deps/'+p),h,p);
}
function validate(r,source,runner){
 assert.equal(hash(root+'/'+source),r.sourceSha256);assert.equal(hash(root+'/'+runner),r.runnerSha256);
 for(const[p,h]of r.dependencies??[])assert.equal(hash(root+'/'+p),h,p);
 const path=r.file.startsWith('/')?r.file:root+'/'+r.file;assert.equal(hash(path),r.sha256);assert.equal(hash(path+'.stderr'),r.stderrSha256);
}
const failedBuild=json('finite-boundaries-build01-runs.json');assert.equal(failedBuild.length,1);validate(failedBuild[0],'FiniteBoundaries.java','run-finite-boundaries.mjs');assert.equal(failedBuild[0].error,'EPERM');
const host=json('finite-boundaries-host-build.json');validate(host,'FiniteBoundaries.java','build-finite-boundaries-host.mjs');assert.equal(host.status,0);assert.equal(host.error,null);
for(const[p,h]of host.preexistingUnqualifiedClasses)assert.equal(hash(build+'/finite-boundaries-v1/'+p),h,p);
const draft=json('finite-boundaries-check01-runs.json');assert.equal(draft.length,1);validate(draft[0],'FiniteBoundaries.java','run-finite-boundaries.mjs');assert.equal(draft[0].status,1);assert(readFileSync(root+'/'+draft[0].file+'.stderr','utf8').includes('Modulus must be greater than 3'));
const builds=json('finite-boundaries-v2-build-runs.json');assert.equal(builds.length,1);validate(builds[0],'FiniteBoundariesV2.java','run-finite-boundaries-v2.mjs');assert.equal(builds[0].status,0);assert.equal(builds[0].error,null);
const mod=(a,p)=>((a%p)+p)%p;
const binary=[[0,1],[1,1,1],[1,0,1,1],[1,1,0,0,1],[1,0,1,0,0,1],[1,1,0,0,0,0,1],[1,1,0,0,0,0,0,1],[1,1,0,1,1,0,0,0,1]];
const fields=new Map();
function field(p,m){
 const key=p+'/'+JSON.stringify(m);if(fields.has(key))return fields.get(key);
 const n=m.length-1,q=p**n,P=BigInt(p);assert.equal(m[n],1);
 function coefficients(a){const out=[];for(let i=0;i<n;i++){out.push(BigInt(a%p));a=Math.floor(a/p);}assert.equal(a,0);return out;}
 function reduce(a){a=a.map(x=>mod(x,P));for(let d=a.length-1;d>=n;d--){const c=a[d];for(let i=0;i<=n;i++)a[d-n+i]=mod(a[d-n+i]-c*BigInt(m[i]),P);}let v=0;for(let i=n-1;i>=0;i--)v=v*p+Number(a[i]??0n);return v;}
 // A single BigInt product with carry-free coefficient packing, independent
 // of the Java oracle's coefficient-by-coefficient multiplication loops.
 function multiply(a,b){const pack=x=>coefficients(x).reduce((v,c,i)=>v+(c<<BigInt(16*i)),0n);const v=pack(a)*pack(b);const c=Array.from({length:2*n-1},(_,i)=>(v>>BigInt(16*i))&65535n);return reduce(c);}
 function power(a,e){let v=1;while(e){if(e%2)v=multiply(v,a);a=multiply(a,a);e=Math.floor(e/2);}return v;}
 function raw(text){if(text==='empty')return 0;const out=[];for(const term of text.split(',')){const[i,c]=term.split(':');assert(/^\d+$/.test(i)&&/^-?\d+$/.test(c));assert(+i<=128);assert.equal(out[+i],undefined);out[+i]=BigInt(c);}return reduce(Array.from({length:Math.max(n,out.length)},(_,i)=>out[i]??0n));}
 const f={q,n,multiply,power,raw,reduce};fields.set(key,f);
 for(let a=1;a<q;a++)assert.equal(multiply(a,power(a,q-2)),1,'independent finite-field unit certificate '+key+'/'+a);
 return f;
}
const prime=p=>{if(p<2n)return false;if(p%2n===0n)return p===2n;for(let d=3n;d<=p/d;d+=2n)if(p%d===0n)return false;return true;};
const expected=new Map();const add=(id,data)=>{assert(!expected.has(id));expected.set(id,data);};
for(const spec of [[2,1,1,1],[2,1,0,1,1],[3,2,2,1],[5,2,0,1],[7,1,0,1]]){
 const[p,...m]=spec,f=field(p,m);for(let scale=1;scale<p;scale++)for(let a=0;a<f.q;a++)for(const algorithm of p>3?['int','big','normalized']:['int'])add('inverse/'+p+'/'+JSON.stringify(m)+'/'+scale+'/'+a+'/'+algorithm,{family:'inverse',f,p,a,algorithm,scale,expected:a?f.power(a,f.q-2):-1,storage:a===0?'zero':a<p**(f.n-1)?'trailing-zero':'canonical'});
}
for(let n=1;n<=8;n++){const f=field(2,binary[n-1]);for(let a=0;a<f.q;a++){const r=f.power(a,2**(n-1));assert.equal(f.multiply(r,r),a);add('binary/'+n+'/'+a,{family:'binary',f,a,expected:r});}}
for(const p of [2,3])add('unsupported-constructor/'+p,{family:'unsupported-constructor',expected:'modulus-greater-than-3'});
for(const p of [5,7,11,13,17,29,41,97]){assert(prime(BigInt(p)));const roots=new Set(Array.from({length:p},(_,x)=>x*x%p));for(let a=0;a<p;a++)for(const shift of [-1,0,1])add('big-small/'+p+'/'+(a+shift*p),{family:'big-small',p,a,square:roots.has(a),expected:roots.has(a)?'SQUARE':'NONSQUARE'});}
for(const p of [3,5,7,13,17,41,97]){const roots=new Set(Array.from({length:p},(_,x)=>x*x%p)),f=field(p,[0,1]);for(const seed of [17,42,149])for(let a=0;a<p;a++)add('odd/'+p+'/'+seed+'/'+a,{family:'odd',p,f,a,square:roots.has(a),expected:roots.has(a)?'SQUARE':'NONSQUARE'});}
for(const p of [3,5,7])for(const seed of [17,42,149])for(let b=0;b<p;b++)for(let c=0;c<p;c++){const roots=Array.from({length:p},(_,x)=>x).filter(x=>(x*x+b*x+c)%p===0);add('berlekamp/'+p+'/'+seed+'/'+b+'/'+c,{family:'berlekamp',p,b,c,roots,expected:JSON.stringify(roots)});}
const runs=json('finite-boundaries-v2-run-runs.json');assert.equal(runs.length,34);
const highPrimes=[];for(const r of runs.filter(r=>r.name.startsWith('jit-high-'))){
 const line=readFileSync(root+'/'+r.file,'utf8').split('\n')[0].split('\t');assert.equal(line[0],'PRIME');const s=+line[1],p=BigInt(line[2]),q=+line[3];assert.equal(line[4],'exhaustive-trial-division');assert.equal(p,(BigInt(q)<<BigInt(s))+1n);assert(q>0&&q<100&&q%2===1);assert(prime(p));highPrimes.push({s,p:p.toString(),oddPart:q});
 for(const a of [2,3,5,17,31,65537])add('high/'+s+'/'+p+'/'+a,{family:'high',p,input:BigInt(a)**2n%p,expected:(BigInt(a)**2n%p).toString()});
}
assert.deepEqual(highPrimes.map(r=>r.s),[30,31,32,35]);assert.equal(expected.size,3206);
const results={};const counter=(target,key,status)=>{(target[key]??={})[status]=((target[key]??{})[status]??0)+1;};
for(const mode of ['jit','interpreter']){
 const seen=new Set(),families={},inverseGroups={},berlekampGroups={};
 for(const r of runs.filter(r=>r.name.startsWith(mode+'-'))){
  validate(r,'FiniteBoundariesV2.java','run-finite-boundaries-v2.mjs');assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(readFileSync(root+'/'+r.file+'.stderr','utf8'),'');
  assert(r.args.includes('-ea')&&r.args.includes('-Xmx256m'));assert.equal(r.args.includes('-Xint'),mode==='interpreter');
  const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t')),summary=rows.pop();assert.equal(summary[0],'SUMMARY');let count=0,start=null;
  for(const row of rows){
   if(row[0]==='PRIME')continue;
   if(row[0]==='START'){assert.equal(start,null);start=row[1];continue;}
   const[tag,id,status,actual,wanted]=row;assert.equal(tag,'RESULT');assert.equal(id,start);start=null;assert(!seen.has(id));seen.add(id);count++;
   const e=expected.get(id);assert(e,id);assert.equal(wanted,String(e.expected));
   assert(['PASS','WRONG','EXCEPTION','DOMAIN_REJECT','CONSTRUCTOR_UNSUPPORTED','REJECTED_SQUARE','UNRESOLVED','HARNESS_BUDGET'].includes(status));
   if(e.family==='inverse'||e.family==='binary'){
    if(status==='PASS'||status==='WRONG')assert.equal(status,e.f.raw(actual)===e.expected?'PASS':'WRONG');
    else if(status==='DOMAIN_REJECT')assert.equal(e.a,0);else assert.equal(status,'EXCEPTION');
    if(e.family==='inverse')counter(inverseGroups,e.algorithm+'/'+e.storage+'/'+(e.scale===1?'monic':'nonmonic'),status+(status==='EXCEPTION'?'/'+actual:''));
   }else if(e.family==='unsupported-constructor'){assert.equal(status,'CONSTRUCTOR_UNSUPPORTED');assert.equal(actual,'IllegalArgumentException');}
   else if(e.family==='big-small'||e.family==='odd'){
    const exception=e.family==='big-small'?'NotASquareException':'IllegalArgumentException';
    if(actual===exception)assert.equal(status,e.square?'REJECTED_SQUARE':'PASS');
    else if(status!=='EXCEPTION'){const a=e.family==='big-small'?BigInt(actual):BigInt(e.f.raw(actual));assert.equal(status,mod(a*a,BigInt(e.p))===BigInt(e.a)?'PASS':'WRONG');}
   }else if(e.family==='berlekamp'){
    if(status==='PASS'||status==='WRONG')assert.equal(status,e.roots.includes(Number(mod(BigInt(actual),BigInt(e.p))))?'PASS':'WRONG');
    counter(berlekampGroups,e.roots.length?'has-root':'no-root',status+(status==='EXCEPTION'?'/'+actual:''));
   }else if(e.family==='high'){
    if(status==='PASS'||status==='WRONG')assert.equal(status,mod(BigInt(actual)**2n,e.p)===e.input?'PASS':'WRONG');
    else assert.equal(status,'EXCEPTION');
   }
   counter(families,e.family,status+(status==='EXCEPTION'?'/'+actual:''));
  }
  assert.equal(start,null);assert.equal(count,+summary[1]);
  const other=runs.find(x=>x.name===r.name.replace(mode+'-',(mode==='jit'?'interpreter':'jit')+'-'));assert.equal(r.sha256,other.sha256);
 }
 assert.equal(seen.size,expected.size);results[mode]={probes:seen.size,families,inverseGroups,berlekampGroups};
}
assert.deepEqual(results.jit,results.interpreter);
const result={...results.jit,processes:34,identicalModeOutputs:true,highPrimes,sourceSha256:hash(import.meta.filename),independentOracle:'BigInt carry-free packed multiplication, polynomial reduction and q-2 unit checks; independent exact trial-division primality; direct square/residual replay',scope:'finite-field arithmetic only; unsupported constructors, trailing-zero storage and finite randomized budgets distinguished; no cryptographic, Maven lifecycle, performance or Hyper-equivalence claim',newProductionChanges:0};
writeFileSync(root+'/finite-boundaries-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
