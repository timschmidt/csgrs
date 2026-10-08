import {readFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import {q,cmp,add,neg}from './symbolic-boundary-oracle-v83.mjs';
import {scalar,integer,fzero,fadd,fmul,fsub,fsign,root2}from './point-extended-field.mjs';
import {normalExpected}from './check-qqbar-remainder-fixed-v84.mjs';
export function longExpected(length){
 const x=fmul(scalar(q(1,2)),root2());let power=integer(1),sum=fzero();
 for(let i=0;i<length;i++){sum=fadd(sum,fmul(integer((5*i+length)%7-3),power));power=fmul(power,x);}return sum;
}
function bound(raw,x){
 assert(Array.isArray(raw)&&raw.length===2);const b=raw.map(a=>{
  assert(Array.isArray(a)&&a.length===2&&a.every(s=>typeof s==='string'&&/^-?\d+$/.test(s)&&s.length<12000));
  const r=q(BigInt(a[0]),BigInt(a[1]));assert.deepEqual(a,r.map(String));return r;
 });
 assert(cmp(b[0],b[1])<=0&&cmp(add(b[1],neg(b[0])),q(1n,1n<<96n))<=0);
 assert(fsign(fsub(x,scalar(b[0])))>=0&&fsign(fsub(scalar(b[1]),x))>=0);
}
export function checkHyperRows(rows){
 assert.equal(rows.length,117);assert.deepEqual(rows.at(-1),{terminal:true,rows:116});let index=0,intervals=0;const proofs={Equal:0,Unknown:0},unknown=[];
 for(const exponent of [0,30,63,64,80,256])for(const base of [-1,0,1])for(const terms of [1,2])for(const precision of [-128,-256]){
  const r=rows[index++];assert.deepEqual(Object.keys(r).sort(),['family','exponent','base','terms','precision','value','exact'].sort());
  for(const[k,v]of Object.entries({family:'monomial',exponent,base,terms,precision}))assert.equal(r[k],v,k);
  const result=(base===0?0:exponent===0?base:1)+terms-1;assert.deepEqual(r.exact,[String(result),'1']);bound(r.value,integer(result));intervals++;
 }
 for(const family of ['normal-mpoly','long-poly'])for(const input of family==='normal-mpoly'?[0,1,2,3,4,5,6,7,8,9]:[0,1,2,7,8,9,63,64,65,127,128,129])for(const precision of [-128,-256]){
  const key=family==='normal-mpoly'?'seed':'length',names=family==='normal-mpoly'?['direct','nested']:['actual','horner'],r=rows[index++];
  assert.deepEqual(Object.keys(r).sort(),['family',key,'precision',...names,'equality','certificate'].sort());
  assert.equal(r.family,family);assert.equal(r[key],input);assert.equal(r.precision,precision);
  const expected=family==='normal-mpoly'?normalExpected(input):longExpected(input);for(const name of names){bound(r[name],expected);intervals++;}
  assert(r.equality===true||r.equality===null);const decision=r.equality?'Equal':'Unknown';
  assert(r.certificate.startsWith(decision+' {'));proofs[decision]++;
  if(!r.equality)unknown.push({family,[key]:input,precision});
 }
 assert.equal(index,116);return{records:117,intervals,proofs,unknown};
}
export function hyperEvidence(){
 const raw=readFileSync('results/qqbar-remainder-hyper-debug-fixed-v84.stdout');
 for(const tag of ['qqbar-remainder-hyper-release-fixed-v84','qqbar-remainder-hyper-debug-v84','qqbar-remainder-hyper-release-v84'])
  assert(raw.equals(readFileSync('results/'+tag+'.stdout')),'numeric streams and full certificates must match '+tag);
 assert(raw.length>0&&raw.at(-1)===10);const rows=raw.toString().trimEnd().split('\n').map(JSON.parse),result=checkHyperRows(rows);
 let corruptions=0;for(const mutate of [r=>r.pop(),r=>r[1]=r[0],r=>r.at(-1).rows--,r=>r[0].exact=['0','1'],r=>r[0].value=[['0','1'],['0','1']],r=>r[0].precision=0,
  r=>r[72].direct=[['0','1'],['0','1']],r=>r[72].nested=[['0','1'],['0','1']],r=>r[72].equality=false,r=>r[72].certificate='Unknown { forged }',
  r=>r[110].actual=[['-1','1'],['1','1']],r=>r[110].certificate='Equal { forged }']){
  const bad=structuredClone(rows);mutate(bad);assert.throws(()=>checkHyperRows(bad));corruptions++;
 }
 return{checkpoint:84,status:'independent-hyper-power-and-polynomial-values-qualified',...result,bytes:raw.length,corruptions,
  scope:'Exact rational field oracle for both evaluation forms; full debug/release output and certificate equality. Certificates are not independently replayed proof objects. Default Hyper features only; no full-stack regression, WASM, benchmark or product-size qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperEvidence()));
