import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {q,primitive,binaryValue,complexBits,simpleComplex,reciprocal,controls,roundingValue,integers,nativeCases,endpoints,selfTest} from './scalar-boundary-oracle-v81.mjs';
import {gcd} from './qqbar-inverse-oracle-v80.mjs';
const cases=nativeCases();
function checkRow(r,c){
 for(const[k,v]of Object.entries(c))assert.deepEqual(r[k],v,'Ordered input identity');
 let e,keys=Object.keys(c).slice(),checks=0;
 if(c.family==='float'){
  const b=binaryValue(c.width,c.bits);keys.push('ok');assert.equal(r.ok,Number(b.value!==null));checks++;
  if(b.value)e=simpleComplex(b.value,q(0));
 }else if(c.family==='complex-float'){
  const a=binaryValue(64,complexBits[c.i]).value,b=binaryValue(64,complexBits[c.j]).value;
  keys.push('ok');assert.equal(r.ok,Number(a!==null&&b!==null));checks++;if(a&&b)e=simpleComplex(a,b);
 }else if(c.family==='inverse'||c.family==='inverse-control'){
  e=c.family==='inverse'?reciprocal(c):controls(c);
  if(c.family==='inverse'){keys.push('exists');assert.equal(r.exists,Number(e!==null));checks++;}
  if(e){
   keys.push('recognized');assert.equal(r.recognized,Number(e.angle!==null));checks++;
   if(e.angle){keys.push('p','q');assert(Number.isSafeInteger(r.p)&&Number.isSafeInteger(r.q)&&r.q>0);
    assert.equal(gcd(BigInt(r.p),BigInt(r.q)),1n);assert.deepEqual(q(r.p,r.q),e.angle);checks+=3;}
  }
 }else if(c.family==='round'){
  e=roundingValue(c);keys.push('floor','ceil','denominator','height','heightBits','numeratorPoly');
  const rounded=integers(e);assert.equal(r.floor,rounded.floor.toString());assert.equal(r.ceil,rounded.ceil.toString());checks+=2;
  const den=e.poly.at(-1),height=e.poly.reduce((n,x)=>{x=x<0n?-x:x;return n>x?n:x;},0n);
  assert.equal(r.denominator,den.toString());assert.equal(r.height,height.toString());assert.equal(r.heightBits,height.toString(2).length);checks+=3;
  const numerator=primitive(e.poly.map((x,i)=>q(x*den**BigInt(e.poly.length-1-i))));
  assert.deepEqual(r.numeratorPoly,numerator.map(String));checks++;
 }else if(c.family==='swap-integer'){
  e=simpleComplex(q(-3),q(0));keys.push('integer');assert.equal(r.integer,'-3');checks++;
 }else if(c.family==='phi')e={poly:[-1n,-1n,1n],real:{kind:'quadratic',a:q(1,2),b:q(1,2),d:5},imag:simpleComplex(q(0),q(0)).real};
 else{
  assert.equal(c.family,'rational-extraction');keys.push('num','den');assert.equal(r.num,'-7');assert.equal(r.den,'3');checks+=2;
 }
 if(e){
  keys.push('poly','real','imag');assert.deepEqual(r.poly,e.poly.map(String));checks++;
  for(const part of ['real','imag'])for(const[name,ok]of Object.entries(endpoints(e[part],r[part]))){assert(ok,part+'-'+name);checks++;}
 }
 assert.deepEqual(Object.keys(r).sort(),keys.sort(),'No access to failed outputs or unqualified fields');
 return{checks,valid:!!e};
}
function checkRows(rows){
 assert.equal(rows.length,19924);assert.deepEqual(rows.at(-1),{terminal:true,rows:19923});
 const counts={},statuses={};let totalChecks=0;
 for(let i=0;i<cases.length;i++){
  const c=cases[i],r=rows[i],result=checkRow(r,c);totalChecks+=result.checks;counts[c.family]=(counts[c.family]??0)+1;
  if('ok'in r){const key=c.family+':'+r.ok;statuses[key]=(statuses[key]??0)+1;}
  if('exists'in r){const key='inverse-exists:'+r.exists;statuses[key]=(statuses[key]??0)+1;}
  if('recognized'in r){const key=c.family+'-recognized:'+r.recognized;statuses[key]=(statuses[key]??0)+1;}
 }
 assert.deepEqual(counts,{float:18432,'complex-float':256,inverse:576,'inverse-control':12,round:644,'swap-integer':1,phi:1,'rational-extraction':1});
 return{status:'pass',records:rows.length,counts,statuses,totalChecks};
}
export function scalarBoundaryEvidence(path='results/scalar-boundary-native-v81.stdout',corruptions=true){
 const oracle=selfTest(),raw=readFileSync(path),rows=raw.toString().trimEnd().split('\n').map(JSON.parse),result=checkRows(rows);let rejected=0;
 if(corruptions){
  const finite=rows.findIndex(r=>r.family==='float'&&r.ok&&r.bits==='0000000000000001'),
   invalid=rows.findIndex(r=>r.family==='float'&&!r.ok),complex=rows.findIndex(r=>r.family==='complex-float'&&r.i===8&&r.j===8),
   inv=rows.findIndex(r=>r.family==='inverse'&&r.recognized&&r.p===1&&r.q===12),round=rows.findIndex(r=>r.family==='round'&&r.b===3&&r.kind===10&&r.imaginary===1),
   phi=rows.findIndex(r=>r.family==='phi');
  assert([finite,invalid,complex,inv,round,phi].every(i=>i>=0));
  const mutations=[
   [finite,r=>{r.bits='0000000000000002';}],[finite,r=>{r.poly[0]='-2';}],
   [finite,r=>{r.real=['0','0','0'];}],[invalid,r=>{r.ok=1;}],[invalid,r=>{r.poly=['0','1'];}],
   [complex,r=>{r.imag=['-1','-1','0'];}],[complex,r=>{r.poly[0]='-1';}],
   [inv,r=>{r.p+=2*r.q;}],[inv,r=>{r.p*=2;r.q*=2;}],[inv,r=>{r.recognized=0;}],
   [round,r=>{r.floor=(BigInt(r.floor)+1n).toString();}],[round,r=>{r.ceil=(BigInt(r.ceil)-1n).toString();}],
   [round,r=>{r.denominator='0';}],[round,r=>{r.heightBits++;}],[round,r=>{r.numeratorPoly[0]='0';}],
   [phi,r=>{r.real=['-1','0','0'];}]
  ];
  for(const[index,mutate]of mutations){const copy=structuredClone(rows[index]);mutate(copy);assert.throws(()=>checkRow(copy,cases[index]));rejected++;}
  assert.throws(()=>checkRows(rows.slice(0,-1)));rejected++;
  const duplicate=rows.slice();duplicate[1]=rows[0];assert.throws(()=>checkRows(duplicate));rejected++;
 }
 return{...result,bytes:raw.length,oracle,corruptionControlsRejected:rejected,
  limits:'Bounded deterministic input corpus, not exhaustive significands, arbitrary algebraic closure, upstream test execution, archive execution or performance qualification. Failed destinations are never interpreted.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(scalarBoundaryEvidence(process.argv[2])));
