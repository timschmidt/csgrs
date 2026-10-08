import assert from 'node:assert/strict';
import {q,add,neg,mul,cmp,primitive,sine,cosine}from './qqbar-inverse-oracle-v80.mjs';
import {scalar,integer,fzero,fadd,fneg,fsub,fmul,fdiv,fz,fkey,fsign,fieldSelfTest}from './point-extended-field.mjs';
import {endpoints81}from './scalar-boundary-endpoints-v81.mjs';
export {q,add,neg,mul,cmp};
const complex=(re,im=fzero())=>({re,im}),czero=()=>complex(fzero()),cone=()=>complex(integer(1)),
 cadd=(a,b)=>complex(fadd(a.re,b.re),fadd(a.im,b.im)),cneg=a=>complex(fneg(a.re),fneg(a.im)),
 cmul=(a,b)=>complex(fsub(fmul(a.re,b.re),fmul(a.im,b.im)),fadd(fmul(a.re,b.im),fmul(a.im,b.re))),
 key=a=>fkey(a.re)+';'+fkey(a.im);
export function complexMinimal(x){
 const images=new Map();
 for(let mask=0;mask<4;mask++)for(const imsign of [1,-1]){
  const conjugate=a=>a.map((v,i)=>(((i&mask)&1)^(((i&mask)>>1)&1))?neg(v):v),
   z=complex(conjugate(x.re),conjugate(imsign===1?x.im:fneg(x.im)));images.set(key(z),z);
 }
 let p=[cone()];for(const z of images.values()){
  const next=Array.from({length:p.length+1},czero);
  for(let i=0;i<p.length;i++){next[i]=cadd(next[i],cneg(cmul(p[i],z)));next[i+1]=cadd(next[i+1],p[i]);}p=next;
 }
 assert(p.every(c=>fz(c.im)&&c.re.slice(1).every(v=>v[0]===0n)));
 return primitive(p.map(c=>c.re[0]));
}
export function piValue(op,p,den){
 const k=Number(((p% (2n*den)+2n*den)%(2n*den))*12n/den),s=sine(k),c=cosine(k);
 if(op===0)return complex(s);if(op===1)return complex(c);
 if(op===2)return fz(c)?null:complex(fdiv(s,c));if(op===3)return fz(s)?null:complex(fdiv(c,s));
 if(op===4)return fz(c)?null:complex(fdiv(integer(1),c));if(op===5)return fz(s)?null:complex(fdiv(integer(1),s));
 assert.equal(op,6);return complex(c,s);
}
export function checkValue(r,x){
 assert.deepEqual(r.poly,complexMinimal(x).map(String));let checks=1;
 for(const [name,a]of [['real',x.re],['imag',x.im]]){
  const b=endpoints81({kind:'field',a},r[name]);assert(b.ordered&&b.contains&&b.width);checks+=3;
 }
 return checks;
}
export function cases(){
 const out=[];
 for(let op=0;op<7;op++)for(const exponent of [0,30,60,61,62,63,80,256])for(const sign of [-1,1])for(const initial of [7,-11])
  out.push({family:'large-angle',op,exponent,sign,initial,den:op===3||op===5?2:1});
 for(let op=0;op<7;op++)for(let kind=0;kind<3;kind++)for(const sign of [-1,1])for(const hasPi of [0,1])out.push({family:'pi-factor',op,kind,sign,hasPi});
 assert.equal(out.length,308);return out;
}
const divide=(a,b)=>{assert(b[0]);return q(a[0]*b[1],a[1]*b[0]);};
function trigInterval(x,sine){
 assert(cmp(x,q(-1))>=0&&cmp(x,q(1))<=0);const x2=mul(x,x);let term=sine?x:q(1),sum=q(0);
 for(let n=0;n<32;n++){
  sum=add(sum,term);term=divide(neg(mul(term,x2)),q(sine?(2*n+2)*(2*n+3):(2*n+1)*(2*n+2)));
 }
 // Alternating Taylor terms decrease at |x|<=1. The next term bounds the
 // remainder; return both endpoints in order, including negative x.
 const other=add(sum,term);return cmp(sum,other)<0?[sum,other]:[other,sum];
}
const intervalDivide=(a,b)=>{
 assert(cmp(b[0],q(0))*cmp(b[1],q(0))>0);const values=a.flatMap(x=>b.map(y=>divide(x,y))).sort(cmp);return[values[0],values.at(-1)];
};
export function radianIntervals(op,x){
 const s=trigInterval(x,true),c=trigInterval(x,false),zero=[q(0),q(0)],one=[q(1),q(1)];
 if(op===6)return{real:c,imag:s};
 return{real:op===0?s:op===1?c:op===2?intervalDivide(s,c):op===3?intervalDivide(c,s):op===4?intervalDivide(one,c):intervalDivide(one,s),imag:zero};
}
const outside=(a,b)=>fsign(fsub(a,scalar(b[0])))<0||fsign(fsub(a,scalar(b[1])))>0;
export function checkRow(r,c){
 for(const[k,v]of Object.entries(c))assert.equal(r[k],v,k);
 const p=c.family==='large-angle'?BigInt(c.sign)*((1n<<BigInt(c.exponent))+(c.den===2?1n:0n)):BigInt(c.kind===0?0:c.sign),
  den=BigInt(c.family==='large-angle'?c.den:c.kind===2?6:1),intended=piValue(c.op,p,den),tooLarge=c.family==='large-angle'&&c.exponent>=62;
 let observed=tooLarge?complex(integer(c.initial)):intended;assert.equal(r.ok,observed?1:0);
 assert.deepEqual(Object.keys(r).sort(),[...Object.keys(c),'ok',...(observed?['poly','real','imag']:[])].sort());
 if(!observed)return{checks:0,outcome:c.family==='pi-factor'&&!c.hasPi&&c.kind?'unsupported-radian-rejected':'pole-rejected'};
 let checks=checkValue(r,observed),outcome='correct';
 if(tooLarge){assert(intended);assert(key(observed)!==key(intended));outcome='false-success-stale-output';checks++;}
 else if(c.family==='pi-factor'&&!c.hasPi&&c.kind){
  const bounds=radianIntervals(c.op,q(p,den));assert(outside(observed.re,bounds.real)||outside(observed.im,bounds.imag));
  outcome='false-success-missing-pi';checks++;
 }
 return{checks,outcome};
}
export function selfTest(){
 fieldSelfTest();assert.deepEqual(complexMinimal(piValue(6,1n,6n)),[1n,0n,-1n,0n,1n]);
 assert.deepEqual(complexMinimal(complex(integer(3),integer(4))),[25n,-6n,1n]);
 const s=radianIntervals(0,q(1)).real,c=radianIntervals(1,q(1)).real;
 assert(cmp(s[0],q(5,6))>0&&cmp(s[1],q(1))<0);assert(cmp(c[0],q(1,2))>0&&cmp(c[1],q(13,24))<0);
 assert.throws(()=>radianIntervals(3,q(0)));
 return{status:'pass',field:'Q(sqrt(2),sqrt(3),i)',radianOracle:'32 alternating Taylor terms plus next-term exact rational remainder at |x|<=1; guarded rational interval quotients',
  scope:'Certifies inequalities against incorrect accepted outputs, not transcendence or a general symbolic simplifier.'};
}
