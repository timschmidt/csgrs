import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {expected,q} from './qqbar-inverse-oracle-v80.mjs';

function inputs(){
 const out=[],push=x=>out.push(x);
 for(const op of ['asin','acos','atan','acot']){
  const d=op==='atan'||op==='acot'?24:12;
  for(let k=0;k<2*d;k++)for(const representation of ['stored','radical','huge-period'])for(const precision of [-64,-256])for(const control of ['identity','perturbed'])
   push({family:'angle',op,k,representation,precision,control});
 }
 for(const family of ['golden','cubic'])for(let which=0;which<(family==='golden'?4:6);which++)for(const op of ['asin','acos'])for(const precision of [-64,-256])for(const control of ['identity','perturbed'])
  push({family,which,op,precision,control});
 for(const op of ['asin','acos','atan','acot'])for(const sign of [-1,1])for(const precision of [-64,-256])push({family:'near',op,sign,precision,control:'near'});
 for(const sign of [-1,1])for(const representation of ['stored','huge-period'])for(const precision of [-64,-256])for(const control of ['identity','perturbed'])
  push({family:'3360',sign,representation,precision,control});
 for(const op of ['asin','acos'])for(const x of [-2,2])for(const precision of [-64,-256])push({family:'domain',op,x,precision});
 assert.equal(out.length,1848);return out;
}
const cases=inputs();
function check(rows){
 assert.equal(rows.length,1849);assert.deepEqual(rows.at(-1),{rows:1848,terminal:true});
 const counts={},groups={},unknown=[];
 for(let i=0;i<cases.length;i++){
  const c=cases[i],r=rows[i];for(const[k,v]of Object.entries(c))assert.deepEqual(r[k],v,'Ordered membership');
  let angle=null,pole=false;
  if(c.family==='3360')angle=q(c.sign,3360);
  else if(c.family==='near')angle=q(1,c.op==='asin'||c.op==='acos'?12:24);
  else if(c.family!=='domain'){const e=expected(c);pole=e===null;angle=e?.angle;}
  const keys=Object.keys(c).concat('outcome');
  if(c.family==='domain')assert.equal(r.outcome,'DomainError');
  else if(pole)assert.equal(r.outcome,'Pole');
  else{
   keys.push('p','q','certificate');assert(Number.isSafeInteger(r.p)&&Number.isSafeInteger(r.q)&&r.q>0);
   assert.deepEqual(q(r.p,r.q),angle,'Independent principal angle');
   assert(['Equal','NotEqual','Unknown'].includes(r.outcome));
   const unequal=c.control!=='identity';
   if(r.outcome==='Equal')assert(!unequal,'False equality');
   if(r.outcome==='NotEqual')assert(unequal,'False inequality');
   if(r.outcome==='Unknown'){
    assert.equal(r.certificate,`Unknown { min_precision: ${c.precision} }`);
    unknown.push(c);
   }else if(r.outcome==='Equal')assert.equal(r.certificate,'Equal { certificate: StructuralEquality }');
   else assert([
    'NotEqual { certificate: ExactRationalComparison }','NotEqual { certificate: DifferenceStructuralFacts }',
    'NotEqual { certificate: StructuralFacts }',`NotEqual { certificate: BoundedRefinement { min_precision: ${c.precision} } }`
   ].includes(r.certificate));
   if(c.family==='3360')assert.equal(r.outcome,unequal?'NotEqual':'Equal');
   if(c.control==='perturbed')assert.equal(r.outcome,'NotEqual');
   if(c.family==='near')assert.equal(r.outcome,c.precision===-64?'Unknown':'NotEqual');
  }
  assert.deepEqual(Object.keys(r).sort(),keys.sort());
  counts[r.outcome]=(counts[r.outcome]??0)+1;
  const key=[c.family,c.op??'',c.representation??'',c.control??'',c.precision,r.outcome].join(':');groups[key]=(groups[key]??0)+1;
 }
 assert.deepEqual(counts,{Equal:584,NotEqual:896,Unknown:312,Pole:48,DomainError:8});
 return{status:'pass',records:rows.length,counts,groups,unknown};
}
export function hyperInverseEvidence(controls=true){
 const debug=readFileSync('results/qqbar-inverse-hyper-debug-v80.stdout'),release=readFileSync('results/qqbar-inverse-hyper-release-v80.stdout');
 assert(debug.equals(release),'Every ordered row and full certificate must match debug/release');
 const rows=debug.toString().trimEnd().split('\n').map(JSON.parse),result=check(rows);let rejected=0;
 if(controls){
  const no=rows.findIndex(r=>r.outcome==='NotEqual'),unknown=rows.findIndex(r=>r.outcome==='Unknown'),pole=rows.findIndex(r=>r.outcome==='Pole');
  for(const mutate of [
   r=>{r[0].p+=2*r[0].q;},r=>{r[0].q=0;},r=>{r[no].outcome='Equal';},r=>{r[0].outcome='NotEqual';},
   r=>{r[unknown].certificate='NotEqual';},r=>{r[pole].outcome='Equal';},r=>{r[0].certificate='unverified';},
   r=>{r[1]=r[0];},r=>r.pop(),r=>{r.at(-1).rows--;},r=>{r[0].precision=-1;}
  ]){const copy=structuredClone(rows);mutate(copy);assert.throws(()=>check(copy));rejected++;}
 }
 return{...result,bytesPerProfile:debug.length,fullCertificatesMatch:true,corruptionControlsRejected:rejected,
  limits:'Default-feature public capability check, not whole-stack regression, generic algebraic import, complex logarithm, or cost comparison. acot is derived atan(1/x), with +pi/2 at zero. Repeated policies and representations are not independent defects.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperInverseEvidence()));
