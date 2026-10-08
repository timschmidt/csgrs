import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {constant,add,mul,scale,factors,recipes,decode,valueCheck} from './mpoly-bridge-math.mjs';
const read=p=>readFileSync(p,'utf8'),log=(tag,ext)=>read('results/mpoly-bridge-'+tag+'.'+ext);
function power(p,e,n){let r=constant(1n,n);for(let i=0;i<e;i++)r=mul(r,p);return r;}
function fractionPower([a,b],e,n){return e<0?[power(b,-e,n),power(a,-e,n)]:[power(a,e,n),power(b,e,n)];}
export function checkMpolyBridge() {
 const c=read('flint-mpoly-rational-controls-v2.c'),js=read('check-mpoly-rational.mjs');
 assert.equal(read('mpoly-bridge-helpers.h'),c.slice(0,c.indexOf('static void context(')));
 assert.equal(read('mpoly-bridge-math.mjs'),js.slice(0,js.indexOf('export function checkMpolyRational()'))+
  'export {constant,add,mul,scale,factors,recipes,operation,decode,valueCheck,same};\n');
 const native=log('native','stdout');assert.equal(native,log('memcheck','stdout'));
 const rows=native.trimEnd().split('\n').map(JSON.parse),summary=rows.pop();
 let cursor=0,values=0,aliases=0,certificates=0,powerRows=0,partRows=0,roundtripRows=0,authoredRows=0;
 function check(kind,metadata,target,n,ord,fs,count){
  const row=rows[cursor++];assert.equal(row.kind,kind);
  for(const[k,v]of Object.entries(metadata))assert.deepEqual(row[k],v);
  const result=count===1?[row.value]:row.values;assert.equal(result.length,count);
  for(const v of result)assert.deepEqual(v,result[0]);
  valueCheck(result[0],target,n,ord,fs);values+=count;certificates++;return result[0];
 }
 for(let ni=0;ni<3;ni++)for(let ord=0;ord<3;ord++){
  const n=[1,2,4][ni],ctx=ni*3+ord,fs=factors(n),input=recipes(n,fs),canonical=[];
  for(let i=0;i<20;i++){
   const v=check('input',{ctx,i},input[i],n,ord,fs,1);
   canonical.push([decode(v[0],n,ord),decode(v[1],n,ord)]);
  }
  for(let i=0;i<20;i++){
   check('roundtrip',{ctx,i},input[i],n,ord,fs,2);roundtripRows++;
   for(const e of[-3,-1,0,1,2,3]){
    if(i===0&&e<0)continue;
    check('power',{ctx,i,e},fractionPower(input[i],e,n),n,ord,fs,e<0?4:6);
    aliases+=e<0?1:2;powerRows++;
   }
   for(let part=0;part<2;part++){
    check('part',{ctx,i,part},[canonical[i][part],constant(1n,n)],n,ord,fs,2);
    aliases++;partRows++;
   }
  }
  const [A,B]=fs,one=constant(1n,n),zero=constant(0n,n);
  // Independent simplified mathematical targets for the authored constructor
  // recipes, not values obtained from another FLINT evaluator or point samples.
  const targets=[[one,one],[add(A,B),one],[one,one],[scale(add(A,B),-1n),one],
   [A,B],[add(scale(A,2n),B),one],[mul(mul(A,A),B),one],
   [power(B,3,n),power(A,3,n)],[zero,one],[zero,one]];
  for(let i=0;i<10;i++){check('authored',{ctx,i},targets[i],n,ord,fs,2);authoredRows++;}
  for(let i=0;i<20;i++)check('preserved',{ctx,i},input[i],n,ord,fs,1);
 }
 assert.equal(cursor,rows.length);
 assert.deepEqual(summary,{summary:true,contexts:9,rows:2052,values:7308,aliases:2142,failures:0});
 assert.equal(values,summary.values);assert.equal(aliases,summary.aliases);
 for(const tag of['compile','native','linked-libraries','memcheck']){
  const g=JSON.parse(log(tag,'json'));assert.equal(g.code,0);assert.equal(g.signal,null);
  assert(Date.parse(g.finished)>=Date.parse(g.started));if(tag!=='memcheck')assert.equal(log(tag,'stderr'),'');
 }
 const mem=log('memcheck','stderr');
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);assert.match(mem,/All heap blocks were freed/);
 const usage=mem.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/).slice(1).map(s=>Number(s.replaceAll(',','')));
 assert.equal(usage[0],usage[1]);
 return{summary,independentPrimaryPolynomialCertificates:certificates,powerRows,partRows,roundtripRows,authoredRows,
  identicalNumericalBytes:Buffer.byteLength(native),memory:{errors:0,suppressed:0,liveBytes:0,liveBlocks:0,
   allocations:usage[0],frees:usage[1],cumulativeBytesIncludingChecks:usage[2]},
  limits:'Current FLINT native64 only. Full coefficientwise identities and known primitive-linear-factor canonicality certificates; no point-sampling oracle. Counts include precision-free repeated APIs, all three orders and aliases. Valid finite formal expressions and powers -3,-1,0,1,2,3 only; no selected-root/nonzero-domain proof, malformed expressions, failure-output guarantees, huge exponents, arbitrary multivariate GCD proof, general generic-ring suite, archived runtime, performance/peak/RSS or Hyper speed claim. Memcheck includes setup and checks; old failures remain preserved.'};
}
if(process.argv.includes('--mpoly-bridge-summary'))console.log(JSON.stringify(checkMpolyBridge()));
