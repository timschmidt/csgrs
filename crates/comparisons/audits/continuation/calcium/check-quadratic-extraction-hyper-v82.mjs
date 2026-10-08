import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {cases,expected,q,cmp,gcd,abs} from './quadratic-extraction-oracle-v82.mjs';
const inputs=cases().filter(c=>c.d>=0),sign=n=>n<0n?-1:n>0n?1:0;
function rational(r){
 assert(Array.isArray(r)&&r.length===2&&r.every(s=>typeof s==='string'&&/^-?\d+$/.test(s)&&s.length<4096));
 const[n,d]=r.map(BigInt);assert(d>0n&&gcd(n,d)===1n);return[n,d];
}
function valueSign(c){
 const e=expected(c);if(e.rational)return sign(e.rationalValue[0]);
 const a=e.a,b=e.b*e.s;if(!a)return sign(b);if(sign(a)===sign(b))return sign(a);
 return sign(a)*sign(a*a-b*b*e.d);
}
function checkRow(r,c,precision){
 assert.deepEqual(Object.keys(r).sort(),['id','precision','scale','rest','equality','equalityCertificate','square','squareCertificate','order','orderCertificate'].sort());
 assert.equal(r.id,c.id);assert.equal(r.precision,precision);
 const scale=rational(r.scale),rest=rational(r.rest),rad=BigInt(c.d)*BigInt(c.s)**2n;
 assert(scale[0]>=0n&&rest[0]>=0n);
 assert.equal(scale[0]**2n*rest[0],rad*scale[1]**2n*rest[1]);
 if(!rad){assert.deepEqual(scale,q(0));assert.deepEqual(rest,q(0));}
 const outcomes={},certificates={},unknown=`Unknown { min_precision: ${precision} }`,
  routes=['StructuralEquality','ExactRationalComparison','StructuralFacts','DifferenceStructuralFacts',`BoundedRefinement { min_precision: ${precision} }`];
 for(const k of ['equality','square']){
  assert(r[k]===true||r[k]===null);
  if(r[k]===null)assert.equal(r[k+'Certificate'],unknown);
  else assert(routes.some(route=>r[k+'Certificate']===`Equal { certificate: ${route} }`));
  outcomes[k]=r[k]===null?'Unknown':'Equal';certificates[k]=r[k+'Certificate'];
 }
 const expectedOrder=['Less','Equal','Greater'][valueSign(c)+1];
 if(r.order===null){assert.equal(r.orderCertificate,unknown);outcomes.order='Unknown';}
 else{assert.equal(r.order,expectedOrder);assert(routes.some(route=>r.orderCertificate===`Known { ordering: ${expectedOrder}, certificate: ${route} }`));outcomes.order=expectedOrder;}
 certificates.order=r.orderCertificate;
 return{outcomes,certificates,nonminimal:cmp(rest,q(c.d))!==0,rad};
}
function checkRecords(rows){
 assert.equal(inputs.length,467);assert.equal(rows.length,935);assert.deepEqual(rows.at(-1),{terminal:true,rows:934});
 let index=0;const counts={},certificates={},uncertain=[],nonminimal=[];
 for(const c of inputs)for(const precision of [-64,-256]){
  const r=rows[index++],out=checkRow(r,c,precision);
  for(const[k,v]of Object.entries(out.outcomes)){const key=k+':'+v;counts[key]=(counts[key]??0)+1;}
  for(const[k,v]of Object.entries(out.certificates)){const key=k+':'+v;certificates[key]=(certificates[key]??0)+1;}
  if(Object.values(out.outcomes).includes('Unknown'))uncertain.push({id:c.id,precision});
  if(out.nonminimal)nonminimal.push({id:c.id,precision});
  if(precision===-256)for(const key of ['scale','rest','equality','square','order'])assert.deepEqual(r[key],rows[index-2][key]);
 }
 return{inputs:467,records:935,counts,certificates,uncertain,nonminimal};
}
export function hyperQuadraticEvidence(){
 const raw=readFileSync('results/quadratic-extraction-hyper-debug-v82.stdout');
 assert(raw.equals(readFileSync('results/quadratic-extraction-hyper-release-v82.stdout')));assert(raw.length>0&&raw.at(-1)===10);
 const rows=raw.toString('utf8').trimEnd().split('\n').map(JSON.parse),out=checkRecords(rows);
 const index=rows.findIndex(r=>r.precision===-64&&r.rest[0]==='2'),c=inputs.find(c=>c.id===rows[index].id);assert(index>=0);
 let rejected=0;const reject=mutate=>{const r=structuredClone(rows[index]);mutate(r);assert.throws(()=>checkRow(r,c,-64));rejected++;};
 reject(r=>r.scale[0]=String(BigInt(r.scale[0])+1n));reject(r=>r.rest[0]='3');reject(r=>r.rest[1]='0');
 reject(r=>r.equality=false);reject(r=>r.square=false);reject(r=>r.equalityCertificate='Equal { certificate: Unchecked }');
 reject(r=>{r.equality=null;r.equalityCertificate='Unknown { min_precision: -256 }';});
 reject(r=>r.order=r.order==='Less'?'Greater':'Less');reject(r=>r.orderCertificate='Known { ordering: Equal, certificate: Unchecked }');
 reject(r=>r.id++);reject(r=>r.precision=-256);
 assert.throws(()=>checkRecords(rows.slice(0,-1)));rejected++;
 const duplicate=rows.slice();duplicate[1]=rows[0];assert.throws(()=>checkRecords(duplicate));rejected++;
 assert.equal(rejected,13);
 return{checkpoint:82,status:'independently-checked-hyper-quadratics',...out,bytesPerProfile:raw.length,fullStreamsMatch:true,corruptions:rejected,
  limits:'Bounded default-feature public scalar capability, not a complete algebraic-number API, full-stack regression, benchmark or WASM qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperQuadraticEvidence()));
