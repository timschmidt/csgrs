import assert from 'node:assert/strict';
import {q,scalar,integer,fzero,fadd,fneg,fsub,fdiv,fz,fsign,fkey,productRoots,fieldSelfTest} from './point-extended-field.mjs';
export {q};
export const abs=n=>n<0n?-n:n;
export function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
export const mod=(a,b)=>((a%b)+b)%b;
export const add=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]);
export const neg=a=>[-a[0],a[1]];
export const mul=(a,b)=>q(a[0]*b[0],a[1]*b[1]);
export const cmp=(a,b)=>{const d=a[0]*b[1]-b[0]*a[1];return d<0n?-1:d>0n?1:0;};
const radical=(a,b,c,d,den=1)=>[a,b,c,d].map(n=>q(n,den));
const quadrant=[integer(0),radical(0,-1,0,1,4),scalar(q(1,2)),radical(0,1,0,0,2),radical(0,0,1,0,2),radical(0,1,0,1,4),integer(1)];
export function sine(k){k=Number(mod(BigInt(k),24n));let negative=false;if(k>12){negative=true;k-=12;}if(k>6)k=12-k;return negative?fneg(quadrant[k]):quadrant[k];}
export const cosine=k=>sine(k+6);
export function primitive(poly){
 const den=poly.reduce((d,c)=>d/gcd(d,c[1])*c[1],1n),ints=poly.map(c=>c[0]*(den/c[1]));
 const content=ints.reduce(gcd,0n)*(ints.at(-1)<0n?-1n:1n);return ints.map(n=>n/content);
}
const minimalCache=new Map();
export function minimal(a){
 const key=fkey(a);if(minimalCache.has(key))return minimalCache.get(key);const images=new Map();
 for(let mask=0;mask<4;mask++){const v=a.map((v,i)=>(((i&mask)&1)^(((i&mask)>>1)&1))?neg(v):v);images.set(fkey(v),v);}
 const poly=productRoots([...images.values()]);assert(poly.every(c=>c.slice(1).every(v=>v[0]===0n)));
 const out=primitive(poly.map(c=>c[0]));minimalCache.set(key,out);return out;
}
export function exactDivide(a,b){
 assert(a.length>=b.length&&b.at(-1)!==0n);a=a.slice();const quotient=Array(a.length-b.length+1).fill(0n);
 for(let i=quotient.length-1;i>=0;i--){assert.equal(a[i+b.length-1]%b.at(-1),0n);const c=a[i+b.length-1]/b.at(-1);quotient[i]=c;
  for(let j=0;j<b.length;j++)a[i+j]-=c*b[j];}
 assert(a.every(c=>c===0n));return quotient;
}
const cycloCache=new Map();
function cyclotomic(n){
 assert(Number.isInteger(n)&&n>0&&n<=24);if(cycloCache.has(n))return cycloCache.get(n);
 let p=Array(n+1).fill(0n);p[0]=-1n;p[n]=1n;for(let d=1;d<n;d++)if(n%d===0)p=exactDivide(p,cyclotomic(d));
 cycloCache.set(n,p);return p;
}
export function cases(){
 const a=[],push=x=>a.push({id:a.length,...x});
 for(const op of ['asin','acos','atan','acot','log']){const d=op==='atan'||op==='acot'?24:12;
  for(let k=0;k<2*d;k++)for(const shift of [-3,0,5])for(const scale of [1,3])push({family:'angle',op,k,d,shift,scale});}
 for(let which=0;which<4;which++)for(const op of ['asin','acos'])push({family:'golden',which,op});
 for(let which=0;which<6;which++)for(const op of ['asin','acos'])push({family:'cubic',which,op});
 for(const op of ['asin','acos','atan','acot'])for(const sign of [-1,1])push({family:'near',op,sign});
 for(let which=0;which<9;which++)for(const op of ['asin','acos','atan','acot','log'])push({family:'control',which,op});
 assert.equal(a.length,1081);return a;
}
const field=a=>({kind:'field',a}),zero=()=>field(fzero());
const realField=(a,angle)=>({poly:minimal(a),real:field(a),imag:zero(),angle});
function angleValue(op,k){
 let a,angle,u;
 if(op==='asin'){a=sine(k);u=Number(mod(BigInt(k),24n));if(u>12)u-=24;if(u>6)u=12-u;if(u< -6)u=-12-u;angle=q(u,12);}
 else if(op==='acos'){a=cosine(k);u=Number(mod(BigInt(k),24n));if(u>12)u=24-u;angle=q(u,12);}
 else if(op==='atan'||op==='acot'){
  const s=sine(k),c=cosine(k);u=Number(mod(BigInt(k),24n));if(u>12)u-=24;
  if(op==='atan'){if(u===12)return null;a=fdiv(s,fadd(integer(1),c));}
  else{if(u===0)return null;a=u===12?fzero():fdiv(fadd(integer(1),c),s);}
  angle=q(u,24);
 }else{
  u=Number(mod(BigInt(k),24n));if(u>12)u-=24;angle=q(u,12);
  return{poly:cyclotomic(Number(24n/gcd(BigInt(k),24n))),real:field(cosine(k)),imag:field(sine(k)),angle};
 }
 return realField(a,angle);
}
const cubicIntervals=[[q(1,2),q(3,4)],[q(-1,4),q(-1,8)],[q(-1),q(-3,4)]];
export const evaluate=(poly,x)=>poly.reduceRight((a,c)=>add(mul(a,x),q(c)),q(0));
export function expected(c){
 if(c.family==='angle')return angleValue(c.op,c.k);
 if(c.family==='near'){
  const old=angleValue(c.op,1),a=fadd(old.real.a,scalar(q(c.sign,1n<<100n)));
  assert.equal(fsign(fsub(a,old.real.a)),c.sign);
  return{...realField(a,null),proposal:{p:1,q:c.op==='asin'||c.op==='acos'?12:24,overlap:1}};
 }
 if(c.family==='golden'){
  const sign=c.which>=2?-1:1,a=q((c.which%2?1:-1)*sign,4),b=q(sign,4),asin=q((c.which%2?3:1)*sign,10);
  return{poly:primitive([add(mul(a,a),neg(mul(q(5),mul(b,b)))),mul(q(-2),a),q(1)]),
   real:{kind:'quadratic',a,b,d:5},imag:zero(),angle:c.op==='asin'?asin:add(q(1,2),neg(asin))};
 }
 if(c.family==='cubic'){
  const sign=c.which>=3?-1:1,index=c.which%3,poly=sign===1?[-1n,-4n,4n,8n]:[1n,-4n,-4n,8n];
  const interval=sign===1?cubicIntervals[index]:cubicIntervals[index].map(neg).reverse(),asin=q([3,-1,-5][index]*sign,14);
  return{poly,real:{kind:'cubic',poly,interval},imag:zero(),angle:c.op==='asin'?asin:add(q(1,2),neg(asin))};
 }
 assert.equal(c.family,'control');let a,angle=null;
 if(c.which===7)return{poly:[1n,0n,1n],real:zero(),imag:field(integer(1)),angle:c.op==='log'?q(1,2):null};
 a=c.which===8?radical(0,1,0,0,4):scalar([q(0),q(1),q(-1),q(2),q(-2),q(1,2),q(1,3)][c.which]);
 if(c.which===0)angle=c.op==='log'?null:c.op==='asin'||c.op==='atan'?q(0):q(1,2);
 if(c.which===1)angle={asin:q(1,2),acos:q(0),atan:q(1,4),acot:q(1,4),log:q(0)}[c.op];
 if(c.which===2)angle={asin:q(-1,2),acos:q(1),atan:q(-1,4),acot:q(-1,4),log:q(1)}[c.op];
 if(c.which===5)angle=c.op==='asin'?q(1,6):c.op==='acos'?q(1,3):null;
 return realField(a,angle);
}
export function dyadic(n,e){assert(typeof n==='string'&&/^-?\d+$/.test(n));assert(typeof e==='string'&&/^-?\d+$/.test(e));
 n=BigInt(n);e=BigInt(e);assert(abs(e)<=8192n);return e<0n?q(n,1n<<-e):q(n<<e);}
export function decodeEndpoints(wire){assert(Array.isArray(wire)&&wire.length===3);return[dyadic(wire[0],wire[2]),dyadic(wire[1],wire[2])];}
function quadraticSign(a,b,d){const x=cmp(a,q(0)),y=cmp(b,q(0));if(!x)return y;if(!y||x===y)return x;return x*cmp(mul(a,a),mul(q(d),mul(b,b)));}
export function endpoints(value,wire){
 const [lo,hi]=decodeEndpoints(wire);let contains;
 if(value.kind==='field')contains=fsign(fsub(value.a,scalar(lo)))>=0&&fsign(fsub(value.a,scalar(hi)))<=0;
 else if(value.kind==='quadratic')contains=quadraticSign(add(value.a,neg(lo)),value.b,value.d)>=0&&quadraticSign(add(value.a,neg(hi)),value.b,value.d)<=0;
 else{assert.equal(value.kind,'cubic');contains=cmp(lo,value.interval[0])>=0&&cmp(hi,value.interval[1])<=0&&evaluate(value.poly,lo)[0]*evaluate(value.poly,hi)[0]<=0n;}
 return{ordered:cmp(lo,hi)<=0,contains,width:cmp(add(hi,neg(lo)),q(1n,1n<<96n))<=0};
}
export function selfTest(){
 fieldSelfTest();assert.deepEqual(minimal(sine(1)),[1n,0n,-16n,0n,16n]);
 assert.deepEqual(minimal(angleValue('atan',1).real.a),[1n,-8n,2n,8n,1n]);
 for(let k=0;k<24;k++){assert(cmp(angleValue('asin',k).angle,q(-1,2))>=0);assert(cmp(angleValue('asin',k).angle,q(1,2))<=0);}
 const p=[-1n,-4n,4n,8n];for(const[lo,hi]of cubicIntervals)assert(evaluate(p,lo)[0]*evaluate(p,hi)[0]<0n);
 for(const n of [-1,1])for(const d of [1,2,4,8])assert.notEqual(evaluate(p,q(n,d))[0],0n);
 assert.deepEqual([-1n,-2n,1n,1n].map((c,i)=>c*(2n**BigInt(i))),p);
 assert.throws(()=>exactDivide([1n,0n,1n],[-1n,1n]));
 return{status:'pass',field:'Q(sqrt(2),sqrt(3))',golden:'Q(sqrt(5))',cubic:'irreducible 8X^3+4X^2-4X-1 with three disjoint rational isolating intervals; cyclotomic trace identity',
  limits:'Independent bounded algebraic oracle; not a general inverse-transcendental decision procedure.'};
}
