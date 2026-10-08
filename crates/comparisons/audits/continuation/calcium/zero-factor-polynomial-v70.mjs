// Independent rational Euclidean GCD plus polynomial-ring determinant.
// Preserve input leading signs: the production square-free step divides by a
// monic GCD, not by a positive-leading primitive replacement of the input.
import assert from 'node:assert/strict';
import {q,qc,parse} from './power-rational-oracle-v69.mjs';
import {oracle} from './power-sums-polynomial-oracle.mjs';
export {q,qc,parse};
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
const sub=(a,b)=>q(a[0]*b[1]-b[0]*a[1],a[1]*b[1]),mul=(a,b)=>q(a[0]*b[0],a[1]*b[1]),div=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const trim=p=>{while(p.length>1&&p.at(-1)[0]===0n)p.pop();return p;};
const zero=p=>p.every(c=>c[0]===0n),monic=p=>p.map(c=>div(c,p.at(-1)));
function divrem(a,b){
 assert(!zero(b));const r=a.map(c=>[...c]),out=Array.from({length:Math.max(1,a.length-b.length+1)},()=>q(0));
 while(!zero(r)&&r.length>=b.length){const k=r.length-b.length,c=div(r.at(-1),b.at(-1));out[k]=c;
  for(let j=0;j<b.length;j++)r[j+k]=sub(r[j+k],mul(c,b[j]));trim(r);}
 return [trim(out),r];
}
export function squareFreeSigned(p){
 let a=p,b=p.slice(1).map((c,i)=>mul(c,q(i+1)));
 while(!zero(b)){const r=divrem(a,b)[1];a=b;b=zero(r)?r:monic(r);}
 const [out,r]=divrem(p,monic(a));assert(zero(r));return out;
}
export function primitiveSigned(p){
 let d=1n;for(const c of p)d=d/gcd(d,c[1])*c[1];const v=p.map(c=>c[0]*(d/c[1])),g=v.reduce(gcd,0n);
 assert(g!==0n);return v.map(c=>c/g);
}
const cache=new Map();
export function expectedBinary(left,right,operation,deflate){
 let p=left.polynomial.map(parse),r=right.polynomial.map(parse);
 const nonzero=qc(parse(right.lower),q(0))>0||qc(parse(right.upper),q(0))<0;
 if(operation===3&&!nonzero)return {status:'DenominatorMayContainZero',polynomial:null};
 const oversized=(p.length-1)*(r.length-1)>9;
 if(oversized){p=squareFreeSigned(p);r=squareFreeSigned(r);}
 let k=0;if(deflate&&operation===3){while(k<r.length&&r[k][0]===0n)k++;if(k+1<r.length)r=r.slice(k);else k=0;}
 const degree=(p.length-1)*(r.length-1);
 if(degree===0||degree>9)return {status:'UnsupportedDegree',polynomial:null,degree};
 const lp=primitiveSigned(p),rp=primitiveSigned(r),key=JSON.stringify([lp.map(String),rp.map(String),operation]);
 if(!cache.has(key))cache.set(key,oracle(lp,rp,operation).poly);
 let polynomial=cache.get(key);
 if(polynomial&&k%2===1&&lp[0]!==0n&&((lp[0]<0n)!==((lp.length-1)%2===1)))polynomial=polynomial.map(n=>(-BigInt(n)).toString());
 return {status:polynomial===null?'Undecided':null,polynomial,degree,removed:k,oversized,
  left:lp.map(String),right:rp.map(String)};
}
export const determinantCount=()=>cache.size;
assert.deepEqual(squareFreeSigned([q(0),q(-5),q(10),q(-5)]).map(c=>c.join('/')),['0/1','5/1','-5/1']);
assert.deepEqual(primitiveSigned([q(0),q(5),q(-5)]),[0n,1n,-1n]);
