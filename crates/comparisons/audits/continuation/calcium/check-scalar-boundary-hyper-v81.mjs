import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {floatCases,binaryValue,roundingValue,integers,reciprocal,controls} from './scalar-boundary-oracle-v81.mjs';

function widenedBits(width,hex){
 if(width===64)return hex;
 const bits=BigInt('0x'+hex),sign=(bits>>31n)<<63n,exp=(bits>>23n)&255n,f=bits&0x7fffffn;let out;
 assert(exp!==255n);
 if(!exp&&!f)out=sign;
 else if(exp)out=sign|((exp+896n)<<52n)|(f<<29n);
 else{const top=BigInt(f.toString(2).length-1);out=sign|((top+874n)<<52n)|((f-(1n<<top))<<(52n-top));}
 return out.toString(16).padStart(16,'0');
}
function inputs(){
 const out=floatCases();
 for(let b=0;b<7;b++)for(let kind=0;kind<23;kind++)for(const phase of [0,1])out.push({family:'round',b,kind,phase});
 for(const op of [0,1])for(let k=0;k<24;k++)for(const shift of [-3,0,5])for(const scale of [1,3])for(const phase of [0,1])
  out.push({family:'inverse',op,k,shift,scale,phase,precision:phase?-256:-64});
 for(const op of [0,1])for(let k=0;k<5;k++)out.push({family:'inverse-control',op,k});
 assert.equal(out.length,19340);return out;
}
const cases=inputs();
function checkRow(r,c){
 for(const[k,v]of Object.entries(c))assert.deepEqual(r[k],v,'Ordered membership');const keys=Object.keys(c).slice(),outcomes=[];
 if(c.family==='float'){
  const b=binaryValue(c.width,c.bits);
  if(b.value){
   keys.push('num','den','exportBits');assert.equal(r.num,b.value[0].toString());assert.equal(r.den,b.value[1].toString());
   assert.equal(r.exportBits,widenedBits(c.width,c.bits));outcomes.push('float:finite');
  }else{keys.push('error');assert.equal(r.error,b.error);outcomes.push('float:'+r.error);}
 }else if(c.family==='round'){
  keys.push('floor','ceil','near');const e=integers(roundingValue(c));
  for(const part of ['floor','ceil']){
   if('value'in r[part]){assert.deepEqual(r[part],{value:e[part].toString()});outcomes.push(part+':Known');}
   else{assert.deepEqual(r[part],{error:'Exhausted'});outcomes.push(part+':Exhausted');}
  }
  assert(typeof r.near==='string'&&/^-?\d+$/.test(r.near));const n=BigInt(r.near);assert(n===e.floor||n===e.ceil);outcomes.push('near:adjacent');
 }else{
  const e=c.family==='inverse'?reciprocal(c):controls(c);
  if(!e||e.angle===null){keys.push('error');assert.equal(r.error,'NotANumber');outcomes.push(c.family+':DomainError');}
  else{
   keys.push('certificate');
   if(r.certificate.startsWith('Equal')){
    assert(['Equal { certificate: StructuralEquality }','Equal { certificate: DifferenceStructuralFacts }'].includes(r.certificate));outcomes.push(c.family+':Equal');
   }else{assert.equal(r.certificate,`Unknown { min_precision: ${c.precision??-256} }`);outcomes.push(c.family+':Unknown');}
  }
 }
 assert.deepEqual(Object.keys(r).sort(),keys.sort());return outcomes;
}
function checkRows(rows){
 assert.equal(rows.length,19341);assert.deepEqual(rows.at(-1),{rows:19340,terminal:true});const counts={},uncertain=[];
 for(let i=0;i<cases.length;i++){
  const c=cases[i],r=rows[i],outcomes=checkRow(r,c);for(const o of outcomes)counts[o]=(counts[o]??0)+1;
  if(outcomes.some(o=>o.endsWith('Exhausted')||o.endsWith('Unknown')))uncertain.push(c);
  if(c.family==='round'&&c.phase===1)for(const part of ['floor','ceil','near'])assert.deepEqual(r[part],rows[i-1][part],'Repeated rounding history');
 }
 return{status:'pass',records:rows.length,counts,uncertain};
}
export function hyperBoundaryEvidence(corruptions=true){
 const a=readFileSync('results/scalar-boundary-hyper-debug-fixed-v81.stdout'),b=readFileSync('results/scalar-boundary-hyper-release-fixed-v81.stdout');
 assert(a.equals(b),'Complete debug/release streams including errors and certificates');
 const rows=a.toString().trimEnd().split('\n').map(JSON.parse),result=checkRows(rows);let rejected=0;
 if(corruptions){
  const finite=rows.findIndex(r=>r.family==='float'&&r.bits==='0000000000000001'),bad=rows.findIndex(r=>r.error==='NotANumber'),
   zero=rows.findIndex(r=>r.family==='float'&&r.bits==='8000000000000000'),round=rows.findIndex(r=>r.family==='round'&&r.floor.value),
   inverse=rows.findIndex(r=>r.family==='inverse'&&r.certificate?.startsWith('Equal'));
  assert([finite,bad,zero,round,inverse].every(i=>i>=0));
  for(const[index,mutate]of [
   [finite,r=>{r.num='0';}],[finite,r=>{r.den='1';}],[finite,r=>{r.exportBits='0000000000000000';}],
   [bad,r=>{r.error='Infinity';}],[zero,r=>{r.exportBits='0000000000000000';}],
   [round,r=>{r.floor.value=(BigInt(r.floor.value)+1n).toString();}],
   [round,r=>{r.ceil={error:'NotANumber'};}],[round,r=>{r.near=(BigInt(r.near)+3n).toString();}],
   [inverse,r=>{r.certificate='NotEqual { certificate: StructuralFacts }';}],
   [inverse,r=>{r.certificate='Equal { certificate: Unverified }';}]
  ]){const copy=structuredClone(rows[index]);mutate(copy);assert.throws(()=>checkRow(copy,cases[index]));rejected++;}
  assert.throws(()=>checkRows(rows.slice(0,-1)));rejected++;const dup=rows.slice();dup[1]=rows[0];assert.throws(()=>checkRows(dup));rejected++;
 }
 return{...result,bytesPerProfile:a.length,fullStreamsMatch:true,corruptionControlsRejected:rejected,
  limits:'Public default-feature real scalar capability, not generic algebraic/complex imports, full-stack regression, WASM execution or benchmark. Inverses are explicit reciprocal compositions; Unknown/Exhausted stay distinct from incorrect results.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperBoundaryEvidence()));
