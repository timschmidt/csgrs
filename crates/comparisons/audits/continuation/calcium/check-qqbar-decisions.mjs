import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';

const cmp=(a,b)=>a<b?-1:a>b?1:0;
const coord=i=>[Math.floor(i/7)-3,i%7-3];
const rootOrder=(a,b)=>{
 const ar=a[1]===0,br=b[1]===0;
 if(ar!==br)return ar?-1:1;
 return -cmp(a[0],b[0])||cmp(Math.abs(a[1]),Math.abs(b[1]))||-cmp(Math.sign(a[1]),Math.sign(b[1]));
};
// A coordinate k denotes sign(k)*sqrt(abs(k)). Comparing a rational dyadic
// against it needs only signs and an exact square; no floating-point sqrt.
function compareDyadicToCoordinate(n,e,k){
 const ns=cmp(n,0n),ks=Math.sign(k);
 if(ns!==ks)return cmp(ns,ks);
 if(ns===0)return 0;
 let left=n*n,right=BigInt(Math.abs(k));
 const twice=2n*e;
 if(twice>=0n)left<<=twice;else right<<=-twice;
 return ns*cmp(left,right);
}
function intervalContains([lo,hi,exp],k){
 const l=BigInt(lo),h=BigInt(hi),e=BigInt(exp);
 assert(e>=-10000n&&e<=10000n);assert(l<=h);
 return compareDyadicToCoordinate(l,e,k)<=0&&compareDyadicToCoordinate(h,e,k)>=0;
}

export function checkQqbarDecisions(path){
 const data=readFileSync(path,'utf8');assert(data.endsWith('\n'));
 const rows=data.trimEnd().split('\n').map(s=>JSON.parse(s)),seen=new Set(),failures=[];
 const counts={unary:0,pair:0,enclosure:0,preserved:0,terminal:0},checks={};
 const check=(r,name,actual,expected)=>{
  checks[name]=(checks[name]??0)+1;
  if(actual!==expected)failures.push({type:r.type,phase:r.phase,i:r.i,j:r.j,prec:r.prec,name,actual,expected});
 };
 for(const r of rows){
  assert(r.type in counts);counts[r.type]++;
  if(r.type==='terminal'){
   assert.equal(r,rows.at(-1));assert.deepEqual(r,{type:'terminal',values:49,phases:2,pairs:4802,enclosures:294});
   continue;
  }
  assert([0,1].includes(r.phase));assert(Number.isInteger(r.i)&&r.i>=0&&r.i<49);
  const key=[r.type,r.phase,r.i,r.j??'',r.prec??''].join(':');assert(!seen.has(key));seen.add(key);
  const a=coord(r.i);
  if(r.type==='unary'){
   check(r,'signRe',r.re,Math.sign(a[0]));check(r,'signIm',r.im,Math.sign(a[1]));
   check(r,'complexSign',r.csgn,Math.sign(a[0])||Math.sign(a[1]));
   check(r,'copyEqual',r.copyEqual,1);check(r,'copyRootOrder',r.copyRootOrder,0);
   check(r,'copyHashEqual',r.copyHashEqual,1);
  }else if(r.type==='pair'){
   assert(Number.isInteger(r.j)&&r.j>=0&&r.j<49);const b=coord(r.j);
   check(r,'equal',r.equal,Number(r.i===r.j));
   check(r,'compareRe',r.re,cmp(a[0],b[0]));check(r,'compareIm',r.im,cmp(a[1],b[1]));
   check(r,'compareAbsRe',r.absRe,cmp(Math.abs(a[0]),Math.abs(b[0])));
   check(r,'compareAbsIm',r.absIm,cmp(Math.abs(a[1]),Math.abs(b[1])));
   check(r,'compareAbs',r.abs,cmp(Math.abs(a[0])+Math.abs(a[1]),Math.abs(b[0])+Math.abs(b[1])));
   check(r,'rootOrder',r.rootOrder,rootOrder(a,b));
  }else if(r.type==='enclosure'){
   assert([32,128,512].includes(r.prec));
   for(const [part,k,exact,acc]of [['real',a[0],r.exactRe,r.accRe],['imag',a[1],r.exactIm,r.accIm]]){
    check(r,'contains-'+part,intervalContains(r[part],k),true);
    check(r,'accuracy-'+part,acc>=r.prec-2,true);
    if(Math.abs(k)<=1)check(r,'exact-dyadic-'+part,exact,1);
   }
  }else{
   check(r,'preservedPolynomial',r.polynomial,1);check(r,'preservedEnclosure',r.enclosure,1);
  }
 }
 assert.deepEqual(counts,{unary:98,pair:4802,enclosure:294,preserved:98,terminal:1});
 const failedByCheck={};for(const f of failures)failedByCheck[f.name]=(failedByCheck[f.name]??0)+1;
 return{kind:'independent-signed-square-root-coordinate-oracle',status:failures.length?'mathematical-contract-fail':'pass',
  counts,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),failedByCheck,failures,
  limits:'49 degree-at-most-four valid values, two cache states, all ordered pairs, three enclosure precisions. Exact BigInt endpoint tests and independent order formulas. No general root isolation/LLL, archive execution, malformed input, benchmark or universal memory claim.'};
}
if(process.argv[2]){
 const result=checkQqbarDecisions(process.argv[2]);console.log(JSON.stringify(result));
 if(result.status!=='pass')process.exitCode=1;
}
