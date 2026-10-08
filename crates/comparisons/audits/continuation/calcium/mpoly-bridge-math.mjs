import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const read=p=>readFileSync(p,'utf8'),log=(tag,ext)=>read('results/mpoly-rational-'+tag+'.'+ext);
const hex=s=>s.startsWith('-')?-BigInt('0x'+s.slice(1)):BigInt('0x'+s);
const abs=x=>x<0n?-x:x;
const gcd=(a,b)=>{a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;};
const key=e=>e.join(','),exps=k=>k.split(',').map(Number);
const monomial=(c,e)=>new Map(c?[[key(e),c]]:[]);
const constant=(c,n)=>monomial(c,Array(n).fill(0));
function add(a,b,s=1n) {const r=new Map(a);for(const[k,v]of b){const c=(r.get(k)??0n)+s*v;if(c)r.set(k,c);else r.delete(k);}return r;}
function mul(a,b) {let r=new Map();for(const[k,v]of a)for(const[l,w]of b)
 r=add(r,monomial(v*w,exps(k).map((e,i)=>e+exps(l)[i])));return r;}
const scale=(a,c)=>new Map([...a].filter(()=>c!==0n).map(([k,v])=>[k,v*c]));
const content=p=>[...p.values()].reduce(gcd,0n);
function order(a,b,ord) {
 if(ord){const d=b.reduce((s,x)=>s+x,0)-a.reduce((s,x)=>s+x,0);if(d)return d;}
 if(ord===2){for(let i=a.length-1;i>=0;i--)if(a[i]!==b[i])return a[i]-b[i];}
 else for(let i=0;i<a.length;i++)if(a[i]!==b[i])return b[i]-a[i];
 return 0;
}
const sorted=(p,ord=0)=>[...p].sort(([a],[b])=>order(exps(a),exps(b),ord));
const same=(a,b)=>assert.deepEqual(sorted(a),sorted(b));
// Exact integer multivariate long division. All tested factors are primitive
// linear polynomials, hence irreducible over Q; Gauss's lemma ensures an exact
// rational divisibility witness would also have an integer quotient here.
function divide(a,b) {
 assert(b.size);const [bk,bc]=sorted(b)[0],be=exps(bk);let r=new Map(a),out=new Map(),steps=0;
 while(r.size) {
  assert(++steps<1000);const [rk,rc]=sorted(r)[0],re=exps(rk),e=re.map((x,i)=>x-be[i]);
  if(e.some(x=>x<0)||rc%bc!==0n)return null;
  const t=monomial(rc/bc,e);out=add(out,t);r=add(r,mul(t,b),-1n);
 }
 return out;
}
const isConstant=p=>[...p.keys()].every(k=>exps(k).every(x=>x===0));
function factors(n) {
 const e=Array(n).fill(0);e[0]=1;const D=monomial(1n,e),A=add(D,constant(1n,n));
 const f=Array(n).fill(0);f[n-1]=1;const B=add(monomial(1n,f),constant(2n,n));
 return[A,B,add(A,B),D,add(A,B,-1n)];
}
function recipes(n,fs) {
 const ns=[0n,1n,-1n,1n,1n,1n,1n,1n,1n,1n,1n,1n,1n,2n,3n,1n,1n,1n,1n,-2n];
 const ds=[1n,1n,1n,1n,1n,2n,3n,2n,3n,1n,1n,1n,1n,3n,2n,1n,1n,1n,1n,-4n];
 const nf=['','','','A','B','A','B','A','B','','','C','E','A','B','AD','A','A','C','C'];
 const df=['','','','','','','','B','A','A','B','AB','AB','B','A','BD','B','B','ABC','BC'];
 return ns.map((s,i)=>{let a=constant(s,n),b=constant(ds[i],n);
  for(const f of nf[i])a=mul(a,fs[f.charCodeAt(0)-65]);for(const f of df[i])b=mul(b,fs[f.charCodeAt(0)-65]);
  if(i===16||i===17){const c=(1n<<BigInt(i===16?65:129))+7n;a=scale(a,c);b=scale(b,c);}return[a,b];});
}
function operation([a,b],[c,d],op) {
 if(op===0||op===1)return[add(mul(a,d),mul(c,b),op===0?1n:-1n),mul(b,d)];
 if(op===2)return[mul(a,c),mul(b,d)];assert(c.size);return[mul(a,d),mul(b,c)];
}
function decode(terms,n,ord) {
 assert(Array.isArray(terms));const p=new Map();let previous;
 for(const[s,e]of terms) {assert.equal(e.length,n);assert(e.every(x=>Number.isInteger(x)&&x>=0&&x<=12));
  const c=hex(s);assert(c!==0n);assert(!p.has(key(e)));if(previous)assert(order(previous,e,ord)<0);previous=e;
  p.set(key(e),c);}
 return p;
}
function valueCheck(v,target,n,ord,fs) {
 assert.equal(v.length,7);const a=decode(v[0],n,ord),b=decode(v[1],n,ord);assert(b.size);
 assert(sorted(b,ord)[0][1]>0n);same(mul(a,target[1]),mul(target[0],b));
 const one=constant(1n,n);
 if(!a.size)same(b,one);
 else {
  assert.equal(gcd(content(a),content(b)),1n);
  let remaining=new Map(b);
  for(const factor of fs.filter(f=>!isConstant(f))) {
   let quotient;
   while((quotient=divide(remaining,factor))!==null) {
    assert.equal(divide(a,factor),null,'common nonconstant factor');remaining=quotient;
   }
  }
  assert(isConstant(remaining),'denominator factors outside proven corpus');
 }
 const oneDen=isConstant(b)&&[...b.values()][0]===1n;
 const oneNum=isConstant(a)&&a.size===1&&[...a.values()][0]===1n;
 assert.deepEqual(v[2],[Number(!a.size),Number(oneNum&&oneDen),Number(isConstant(a)&&oneDen),Number(isConstant(a)&&isConstant(b))]);
 const mask=p=>Array.from({length:n},(_,i)=>Number([...p.keys()].some(k=>exps(k)[i]!==0)));
 const am=mask(a),bm=mask(b);assert.deepEqual(v[3],am);assert.deepEqual(v[4],bm);assert.deepEqual(v[5],am.map((x,i)=>x|bm[i]));
 assert.deepEqual(v[6].map(hex),a.size?[content(a),content(b)]:[1n,1n]);
}
export {constant,add,mul,scale,factors,recipes,operation,decode,valueCheck,same};
