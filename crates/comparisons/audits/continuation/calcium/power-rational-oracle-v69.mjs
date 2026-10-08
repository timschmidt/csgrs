import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {oracle} from './power-sums-polynomial-oracle.mjs';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
function q(n,d=1n){n=BigInt(n);d=BigInt(d);assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];}
const qa=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]),qn=a=>[-a[0],a[1]];
const qs=(a,b)=>qa(a,qn(b)),qm=(a,b)=>q(a[0]*b[0],a[1]*b[1]),qd=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const qc=(a,b)=>{const n=a[0]*b[1]-b[0]*a[1];return n<0n?-1:n>0n?1:0;};
const parse=s=>{assert(/^-?\d+\/\d+$/.test(s));const [n,d]=s.split('/');const a=q(n,d);assert.equal(a.join('/'),s);return a;};
const trim=p=>{while(p.length>1&&p.at(-1)[0]===0n)p.pop();return p;};
const zero=p=>p.every(c=>c[0]===0n);
const monic=p=>zero(p)?[q(0)]:p.map(c=>qd(c,p.at(-1)));
const derivative=p=>p.length===1?[q(0)]:p.slice(1).map((c,i)=>qm(c,q(i+1)));
const evaluate=(p,x)=>p.reduceRight((v,c)=>qa(qm(v,x),c),q(0));
function divrem(a,b){assert(!zero(b));let r=a.map(c=>[...c]);const quotient=Array.from({length:Math.max(1,a.length-b.length+1)},()=>q(0));
 while(!zero(r)&&r.length>=b.length){const k=r.length-b.length, c=qd(r.at(-1),b.at(-1));quotient[k]=c;
  for(let j=0;j<b.length;j++)r[j+k]=qs(r[j+k],qm(c,b[j]));trim(r);}
 return[trim(quotient),r];
}
const sfCache=new Map();
const polyKey=p=>p.map(c=>c.join('/')).join(',');
function squareFree(p){const key=polyKey(monic(p));if(sfCache.has(key))return sfCache.get(key);
 let a=p,b=derivative(p);while(!zero(b)){const r=divrem(a,b)[1];a=b;b=monic(r);}
 const [s,r]=divrem(p,monic(a));assert(zero(r));const result=monic(s);sfCache.set(key,result);return result;
}
function integers(p){let d=1n;for(const c of p)d=d/gcd(d,c[1])*c[1];let out=p.map(c=>c[0]*(d/c[1]));
 let g=out.reduce(gcd,0n);assert(g!==0n);if(out.at(-1)<0n)g=-g;return out.map(c=>c/g);}
const sturmCache=new Map(),countCache=new Map();
function rootCount(p,lo,hi){assert(qc(lo,hi)<=0);p=squareFree(p);
 const key=polyKey(p),ck=key+';'+lo.join('/')+';'+hi.join('/');if(countCache.has(ck))return countCache.get(ck);
 if(!sturmCache.has(key)){const seq=[p,derivative(p)];while(!zero(seq.at(-1))&&seq.at(-1).length>1){
   const r=divrem(seq.at(-2),seq.at(-1))[1].map(qn);if(zero(r))break;seq.push(r);}
  sturmCache.set(key,seq.filter(s=>!zero(s)));}
 const variations=x=>{let old=0,n=0;for(const s of sturmCache.get(key)){const sign=qc(evaluate(s,x),q(0));if(sign){if(old&&old!==sign)n++;old=sign;}}return n;};
 const count=variations(lo)-variations(hi)+Number(evaluate(p,lo)[0]===0n);countCache.set(ck,count);return count;
}
function image(a,b,op){
 if(op===0)return[qa(a[0],b[0]),qa(a[1],b[1])];
 if(op===1)return[qs(a[0],b[1]),qs(a[1],b[0])];
 const values=a.flatMap(x=>b.map(y=>op===2?qm(x,y):qd(x,y))).sort(qc);return[values[0],values.at(-1)];
}
function root(r){return{p:r.polynomial.map(parse),lo:parse(r.lower),hi:parse(r.upper),exact:r.exact===null?null:parse(r.exact)};}
function selfTest(){
 const p=[q(-2),q(0),q(1)];assert.equal(rootCount(p,q(1),q(2)),1);assert.equal(rootCount(p,q(-2),q(2)),2);
 const repeated=[q(1),q(-2),q(1)];assert.equal(rootCount(repeated,q(1),q(1)),1);
 assert.equal(rootCount(repeated,q(0),q(1)),1);assert.equal(rootCount(repeated,q(1),q(2)),1);
 assert.equal(rootCount(repeated,q(2),q(3)),0);
}

selfTest();
export {parse,rootCount,image,root,q,qc,evaluate,integers};
