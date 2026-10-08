import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {q,add,neg,mul,cmp,gcd,exactDivide,evaluate,decodeEndpoints} from './qqbar-inverse-oracle-v80.mjs';

// Im((1+iX)^n) is zero precisely when n*atan(X) is an integer multiple of pi.
// This integer polynomial is constructed without FLINT, transcendental floats,
// its rational proposal, or the algebraic constructor's claimed denominator.
function tangentPolynomial(n){
 const p=Array(n).fill(0n);let choose=1n;
 for(let k=0;k<=n;k++){
  if(k%2)p[k]=k%4===1?choose:-choose;
  if(k<n){const numerator=choose*BigInt(n-k);assert.equal(numerator%BigInt(k+1),0n);choose=numerator/BigInt(k+1);}
 }
 assert.equal(choose,1n);assert.equal(p[1],BigInt(n));assert.equal(p.at(-1),-BigInt(n));return p;
}
function verify(r){
 assert.deepEqual(Object.keys(r),['denominator','degree','recognized','poly','real','imag','proposal']);
 assert.equal(r.denominator,3360);assert.equal(r.degree,384);assert.equal(r.recognized,0);
 assert.deepEqual(r.proposal,{p:1,q:3359,overlap:0});
 assert(Array.isArray(r.poly)&&r.poly.length===385&&r.poly.every(x=>typeof x==='string'&&/^-?\d+$/.test(x)));
 const p=r.poly.map(BigInt);assert(p.at(-1)>0n);assert.equal(p.reduce(gcd,0n),1n);
 const f=tangentPolynomial(3360),quotient=exactDivide(f,p);
 const [lo,hi]=decodeEndpoints(r.real),[il,ih]=decodeEndpoints(r.imag),lower=q(1,1200),upper=q(1,1000);
 assert(cmp(lo,hi)<0);assert(cmp(lo,lower)>0&&cmp(hi,upper)<0);
 assert(cmp(add(hi,neg(lo)),q(1n,1n<<96n))<=0);assert.equal(il[0],0n);assert.equal(ih[0],0n);
 assert(evaluate(p,lo)[0]*evaluate(p,hi)[0]<0n,'Exact sign-changing root bracket');
 // Let t=pi/3360. 3 < pi < 22/7 gives t > 1/1120 > lower.
 // tan(t) <= t/(1-t^2/2) < upper, using sin(t)<=t and cos(t)>=1-t^2/2.
 // tan(2t)>2t>1/560>upper. Thus the interval contains exactly the j=1
 // root of F, not 0, any negative root, or j>=2 (atan is increasing).
 const tLow=q(3,3360),tHigh=q(22,7*3360),den=add(q(1),neg(mul(q(1,2),mul(tHigh,tHigh))));
 const tanUpper=q(tHigh[0]*den[1],tHigh[1]*den[0]);
 assert(cmp(tLow,lower)>0);assert(cmp(den,q(0))>0);assert(cmp(tanUpper,upper)<0);assert(cmp(mul(q(2),tLow),upper)>0);
 // Independent first-accepted mediant: all earlier 1/k for k<=3358 have
 // larger error. Strict rational inequalities bracket the 1e-7 tolerance.
 const eps=q(1,10000000),actual=q(1,3360);
 assert(cmp(add(q(1,3359),neg(actual)),eps)<0);assert(cmp(add(q(1,3358),neg(actual)),eps)>0);
 return{status:'pass',denominator:3360,degree:384,polynomialTerms:p.length,tangentPolynomialTerms:f.length,
  quotientTerms:quotient.length,exactDivisionRemainder:'zero',rootBracket:['1/1200','1/1000'],
  principalAtanPi:'1/3360',observedRecognition:0,observedProposal:r.proposal,
  classification:'Documented recognition completeness failure; not an unsound accepted equality',
  independence:'Exact integer polynomial divisibility plus endpoint root signs and elementary rational bounds identify tan(pi/3360); no floating-point oracle or same-library round trip.'};
}
export function checkQqbarInverseCounterexample(path,controls=true){
 const rows=readFileSync(path,'utf8').trimEnd().split('\n').map(JSON.parse);assert.equal(rows.length,1);const r=rows[0],result=verify(r);
 let rejected=0;
 if(controls)for(const mutate of [
  x=>{x.denominator=3359;},x=>{x.degree=383;},x=>{x.recognized=1;},x=>{x.proposal.q=3360;},
  x=>{x.poly[0]=(BigInt(x.poly[0])+1n).toString();},x=>{x.real=['1','2','0'];},
  x=>{x.imag=['1','1','0'];},x=>{x.poly=x.poly.map(n=>(2n*BigInt(n)).toString());}
 ]){const copy=structuredClone(r);mutate(copy);assert.throws(()=>verify(copy));rejected++;}
 return{...result,corruptionControlsRejected:rejected,
  limits:'Executed current pinned FLINT only; matching archived source is read, not independently executed. Bounded probe is not a full recognition counterexample search or benchmark.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkQqbarInverseCounterexample(process.argv[2])));
