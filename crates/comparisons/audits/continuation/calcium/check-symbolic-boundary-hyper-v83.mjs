import {readFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import {cases,piValue,radianIntervals,q,cmp,add,neg,selfTest}from './symbolic-boundary-oracle-v83.mjs';
import {scalar,fsub,fsign}from './point-extended-field.mjs';
const scales=[q(-7,3),q(0),q(1),q(5,2)];
const inputs=()=>cases().filter(c=>c.initial!==-11).map(({initial,...c})=>c).sort((a,b)=>a.op-b.op);
function interval(raw){
 assert(Array.isArray(raw)&&raw.length===2);
 const b=raw.map(r=>{assert(Array.isArray(r)&&r.length===2&&r.every(v=>typeof v==='string'&&/^-?\d+$/.test(v)));
  const v=q(BigInt(r[0]),BigInt(r[1]));assert.deepEqual(r,v.map(String));return v;});
 assert(cmp(b[0],b[1])<=0);assert(cmp(add(b[1],neg(b[0])),q(1n,1n<<96n))<=0);return b;
}
function fieldBounds(raw,a){const b=interval(raw);assert(fsign(fsub(a,scalar(b[0])))>=0&&fsign(fsub(scalar(b[1]),a))>=0);return 3;}
function rationalBounds(raw,truth){const b=interval(raw);assert(cmp(b[0],truth[0])<=0&&cmp(b[1],truth[1])>=0);return 3;}
function checkAngle(r,c,precision){
 for(const[k,v]of Object.entries(c))assert.equal(r[k],v,k);assert.equal(r.precision,precision);
 const p=c.family==='large-angle'?BigInt(c.sign)*((1n<<BigInt(c.exponent))+(c.den===2?1n:0n)):BigInt(c.kind===0?0:c.sign),
  den=BigInt(c.family==='large-angle'?c.den:c.kind===2?6:1),radian=c.family==='pi-factor'&&!c.hasPi&&c.kind!==0,
  truth=radian?radianIntervals(c.op,q(p,den)):piValue(c.op,p,den);
 const fields=[...Object.keys(c),'precision',...(truth?['real','imag']:['error'])].sort();assert.deepEqual(Object.keys(r).sort(),fields);
 if(!truth){assert(['NotANumber','DivideByZero'].includes(r.error));return{checks:1,outcome:'pole'};}
 return{checks:radian?rationalBounds(r.real,truth.real)+rationalBounds(r.imag,truth.imag):fieldBounds(r.real,truth.re)+fieldBounds(r.imag,truth.im),outcome:radian?'radian':'pi'};
}
function checkNorm(r,scale,k,precision){
 assert.deepEqual(Object.keys(r).sort(),['family','scale','k','precision','norm','abs','normEquality','certificate'].sort());
 assert.equal(r.family,'complex-norm');assert.equal(r.scale,scale);assert.equal(r.k,k);assert.equal(r.precision,precision);
 const s=scales[scale],norm=q(s[0]*s[0],s[1]*s[1]),abs=q(s[0]<0n?-s[0]:s[0],s[1]);
 const checks=rationalBounds(r.norm,[norm,norm])+rationalBounds(r.abs,[abs,abs]);
 assert(r.normEquality===true||r.normEquality===null,'known identity cannot be NotEqual');
 assert(r.certificate.startsWith(r.normEquality?'Equal {':'Unknown {'));
 return{checks:checks+1,outcome:r.normEquality?'Equal':'Unknown'};
}
export function checkHyperRecords(rows){
 assert.equal(rows.length,585);assert.deepEqual(rows.at(-1),{terminal:true,rows:584});
 let index=0,checks=0;const angles={},norms={};
 for(const c of inputs())for(const precision of [-128,-256]){const v=checkAngle(rows[index++],c,precision);checks+=v.checks;angles[v.outcome]=(angles[v.outcome]??0)+1;}
 for(let scale=0;scale<4;scale++)for(let k=0;k<24;k++)for(const precision of [-128,-256]){
  const v=checkNorm(rows[index++],scale,k,precision);checks+=v.checks;norms[v.outcome]=(norms[v.outcome]??0)+1;
 }
 assert.equal(index,584);return{inputs:196,complexInputs:96,records:585,checks,angles,norms};
}
export function hyperEvidence(){
 selfTest();const raw=readFileSync('results/symbolic-boundary-hyper-debug-v83.stdout');
 assert(raw.equals(readFileSync('results/symbolic-boundary-hyper-release-v83.stdout')));assert(raw.length>0&&raw.at(-1)===10);
 const rows=raw.toString().trimEnd().split('\n').map(JSON.parse),result=checkHyperRecords(rows);
 const idx=rows.findIndex(r=>r.family==='pi-factor'&&!r.hasPi&&r.kind===1&&r.op===0),base=rows[idx],c=inputs().find(c=>Object.entries(c).every(([k,v])=>base[k]===v));
 let corruptions=0;
 for(const mutate of [r=>r.real=[['0','1'],['0','1']],r=>r.imag=[['1','1'],['1','1']],r=>r.real.reverse(),r=>r.real=[['-1','1'],['1','1']],r=>r.hasPi=1,r=>r.precision=0,r=>r.error='NotANumber']){
  const bad=structuredClone(base);mutate(bad);assert.throws(()=>checkAngle(bad,c,base.precision));corruptions++;
 }
 for(const mutate of [r=>r.pop(),r=>r[1]=r[0],r=>r[r.length-1]={terminal:true,rows:583}]){
  const bad=rows.slice();mutate(bad);assert.throws(()=>checkHyperRecords(bad));corruptions++;
 }
 const norm=rows.find(r=>r.family==='complex-norm');
 for(const mutate of [r=>r.normEquality=false,r=>r.norm=[['0','1'],['0','1']],r=>r.abs=[['-7','3'],['-7','3']],r=>r.certificate='Equal { forged }']){
  const bad=structuredClone(norm);mutate(bad);
  if(bad.certificate==='Equal { forged }')bad.normEquality=null;
  assert.throws(()=>checkNorm(bad,norm.scale,norm.k,norm.precision));corruptions++;
 }
 return{checkpoint:83,status:'independently-checked-hyper-angle-and-norm-comparison',...result,bytes:raw.length,corruptions,
  limits:'Current public API, two policies, bounded corpus. Full debug/release certificates match but are not general proof-object verification. No serde ingestion test, benchmark, full-stack regression, all-Hyper-features test or new WASM qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperEvidence()));
