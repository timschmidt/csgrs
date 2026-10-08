import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {cases,expected,endpoints,q,gcd,selfTest} from './qqbar-inverse-oracle-v80.mjs';

const inputs=cases(),expectations=inputs.map(expected);
function checkRows(rows){
 assert.equal(rows.length,2163,'Full ordered corpus including terminal');
 const counts={cases:1081,rows:2162,values:0,poles:0,recognized:0,nonrecognized:0,near:0},checks={},failures=[];
 const check=(name,ok,r)=>{checks[name]=(checks[name]??0)+1;if(!ok)failures.push({check:name,id:r.id,phase:r.phase});};
 for(const c of inputs)for(const phase of [0,1]){
  const r=rows[2*c.id+phase],e=expectations[c.id];assert.equal(r.id,c.id);assert.equal(r.phase,phase);
  const keys=['id','phase','exists'];check('existence',r.exists===Number(e!==null),r);
  if(e){
   counts.values++;keys.push('poly','real','imag','recognized');
   check('minimal-polynomial',JSON.stringify(r.poly)===JSON.stringify(e.poly.map(String)),r);
   for(const part of ['real','imag'])for(const[name,ok]of Object.entries(endpoints(e[part],r[part])))check(part+'-'+name,ok,r);
   check('recognition',r.recognized===Number(e.angle!==null),r);
   if(e.angle){
    counts.recognized++;keys.push('p','q');
    assert(Number.isSafeInteger(r.p)&&Number.isSafeInteger(r.q));
    check('positive-denominator',r.q>0,r);check('reduced-fraction',gcd(BigInt(r.p),BigInt(r.q))===1n,r);
    check('principal-angle',r.q!==0&&JSON.stringify(q(r.p,r.q).map(String))===JSON.stringify(e.angle.map(String)),r);
   }else counts.nonrecognized++;
   if(e.proposal){keys.push('proposal');counts.near++;check('nearby-proposal',JSON.stringify(r.proposal)===JSON.stringify(e.proposal),r);}
  }else counts.poles++;
  assert.deepEqual(Object.keys(r),keys,'Do not read nonexistent or failed outputs');
 }
 assert.deepEqual(rows.at(-1),{terminal:true,cases:1081,rows:2162});
 assert.deepEqual(counts,{cases:1081,rows:2162,values:2114,poles:48,recognized:2042,nonrecognized:72,near:16});
 return{status:failures.length?'mathematical-fail':'pass',records:rows.length,counts,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),failures};
}
export function checkQqbarInverse(path,controls=true){
 const oracle=selfTest(),rows=readFileSync(path,'utf8').trimEnd().split('\n').map(JSON.parse),result=checkRows(rows);
 let rejected=0;
 if(controls&&result.status==='pass'){
  const value=rows.findIndex(r=>r.exists&&r.p===1&&r.q===12),pole=rows.findIndex(r=>!r.exists),
   no=rows.findIndex(r=>r.exists&&!r.recognized),near=rows.findIndex(r=>r.proposal);
  assert([value,pole,no,near].every(i=>i>=0));
  const mutations=[
   r=>{r[value].p+=2*r[value].q;},r=>{r[value].p*=-1;r[value].q*=-1;},
   r=>{r[value].p*=2;r[value].q*=2;},r=>{r[no].recognized=1;},r=>{r[no].p=0;r[no].q=1;},
   r=>{r[value].poly[0]='999';},r=>{r[value].real=['-2','-1','0'];},r=>{r[value].imag=['1','1','0'];},
   r=>{r[value].real=['-2','2','0'];},r=>{r[pole].exists=1;},r=>{r[near].proposal.overlap=0;},
   r=>{r[1]=r[0];},r=>{r[value].phase=7;},r=>r.pop(),r=>{r.at(-1).rows--;}
  ];
  for(const mutate of mutations){const copy=structuredClone(rows);mutate(copy);let failed=false;
   try{failed=checkRows(copy).status!=='pass';}catch{failed=true;}assert(failed,'Corruption not rejected');rejected++;}
 }
 return{...result,oracle,corruptionControlsRejected:rejected,
  limits:'Bounded exact algebraic oracle, both input components and principal fractions; source coverage is separate. Near values prove rejection of the proposed angle, not general transcendence. No performance claim.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const r=checkQqbarInverse(process.argv[2]);console.log(JSON.stringify(r));process.exitCode=r.status==='pass'?0:1;
}
