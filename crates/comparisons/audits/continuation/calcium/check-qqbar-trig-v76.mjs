// Independent algebraic values, minimal polynomials and exact endpoint signs.
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {q,scalar,integer,fzero,fadd,fneg,fsub,fmul,fdiv,finv,fsign,fz,fkey,productRoots,fieldSelfTest} from './point-extended-field.mjs';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
const mod=(a,n)=>((a%n)+n)%n;
const qa=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]);
const qn=a=>[-a[0],a[1]];
const cmp=(a,b)=>{const d=a[0]*b[1]-b[0]*a[1];return d<0n?-1:d>0n?1:0;};
const radical=(a,b,c,d,den=1)=>[a,b,c,d].map(n=>q(n,den));
const firstQuadrant=[integer(0),radical(0,-1,0,1,4),scalar(q(1,2)),radical(0,1,0,0,2),
 radical(0,0,1,0,2),radical(0,1,0,1,4),integer(1)];
function sine(k){k=Number(mod(BigInt(k),24n));let negative=false;if(k>12){negative=true;k-=12;}if(k>6)k=12-k;return negative?fneg(firstQuadrant[k]):firstQuadrant[k];}
const cosine=k=>sine(BigInt(k)+6n);
const minimalCache=new Map();
function minimal(a){
 const key=fkey(a);if(minimalCache.has(key))return minimalCache.get(key);
 const images=new Map();for(let mask=0;mask<4;mask++){
  const v=a.map((v,i)=>(((i&mask)&1)^(((i&mask)>>1)&1))?qn(v):v);images.set(fkey(v),v);
 }
 const poly=productRoots([...images.values()]);assert(poly.every(c=>c.slice(1).every(v=>v[0]===0n)));
 const rational=poly.map(c=>c[0]);const den=rational.reduce((d,c)=>d/gcd(d,c[1])*c[1],1n);
 const ints=rational.map(c=>c[0]*(den/c[1])),content=ints.reduce(gcd,0n)*(ints.at(-1)<0n?-1n:1n);
 const out=ints.map(n=>(n/content).toString());minimalCache.set(key,out);return out;
}
const cycloCache=new Map();
function exactDivide(a,b){
 a=a.slice();const quotient=Array(a.length-b.length+1).fill(0n);
 for(let i=quotient.length-1;i>=0;i--){assert(a[i+b.length-1]%b.at(-1)===0n);const c=a[i+b.length-1]/b.at(-1);quotient[i]=c;
  for(let j=0;j<b.length;j++)a[i+j]-=c*b[j];}
 assert(a.every(c=>c===0n));return quotient;
}
function cyclotomic(n){
 assert(Number.isInteger(n)&&n>0&&n<=24);if(cycloCache.has(n))return cycloCache.get(n);
 let poly=Array(n+1).fill(0n);poly[0]=-1n;poly[n]=1n;
 for(let d=1;d<n;d++)if(n%d===0)poly=exactDivide(poly,cyclotomic(d));
 cycloCache.set(n,poly);return poly;
}
const ops=['root','exp','sin','cos','tan','cot','sec','csc'];
function expected(r){
 const p=BigInt(r.p*r.scale),d=BigInt(r.q*r.scale),mult=r.op==='root'?24n:12n;
 assert(mult*p%d===0n);const k=mult*p/d,s=sine(k),c=cosine(k);
 if(r.op==='root'||r.op==='exp'){
  const denominator=r.op==='root'?d:2n*d,g=gcd(p,denominator),rq=denominator/g,rp=mod(p/g,rq);
  return {real:c,imag:s,poly:cyclotomic(Number(rq)).map(String),rp:Number(rp),rq:Number(rq)};
 }
 let real;
 if(r.op==='sin')real=s;else if(r.op==='cos')real=c;
 else if(r.op==='tan'){if(fz(c))return null;real=fdiv(s,c);}
 else if(r.op==='cot'){if(fz(s))return null;real=fdiv(c,s);}
 else if(r.op==='sec'){if(fz(c))return null;real=finv(c);}
 else if(r.op==='csc'){if(fz(s))return null;real=finv(s);}
 else throw Error('Unknown operation');
 return {real,imag:fzero(),poly:minimal(real)};
}
function dyadic(n,e){assert(typeof n==='string'&&/^-?\d+$/.test(n));assert(typeof e==='string'&&/^-?\d+$/.test(e));
 n=BigInt(n);e=BigInt(e);assert(abs(e)<=4096n);return e<0n?q(n,1n<<-e):q(n<<e);}
function endpoints(a,wire){
 assert(Array.isArray(wire)&&wire.length===3);const lo=dyadic(wire[0],wire[2]),hi=dyadic(wire[1],wire[2]);
 return {ordered:cmp(lo,hi)<=0,contains:fsign(fsub(a,scalar(lo)))>=0&&fsign(fsub(a,scalar(hi)))<=0,
  width:cmp(qa(hi,qn(lo)),q(1n,1n<<96n))<=0};
}
function expectedKeys(){
 const keys=[];for(const q of [1,2,3,4,6,12])for(let p=-2*q;p<=2*q;p++)for(const scale of [1,3])for(const op of ops)for(const phase of [0,1])keys.push(JSON.stringify(['angle',q,p,scale,op,phase]));
 for(let control=0;control<5;control++)for(const phase of [0,1])keys.push(JSON.stringify(['nonroot',control,phase]));
 keys.push(JSON.stringify(['terminal']));assert.equal(keys.length,3787);return keys;
}
function key(r){if(r.type==='angle')return JSON.stringify([r.type,r.q,r.p,r.scale,r.op,r.phase]);
 if(r.type==='nonroot')return JSON.stringify([r.type,r.control,r.phase]);assert.equal(r.type,'terminal');return JSON.stringify([r.type]);}
function checkRows(rows){
 const keys=expectedKeys(),counts={angles:0,values:0,poles:0,roots:0,nonroots:0},checks={},failures=[];
 assert.equal(rows.length,keys.length);const check=(name,ok,r)=>{checks[name]=(checks[name]??0)+1;if(!ok)failures.push({check:name,key:key(r)});};
 for(let i=0;i<rows.length;i++){
  const r=rows[i];assert.equal(key(r),keys[i],'Ordered complete corpus membership');
  if(r.type==='angle'){
   counts.angles++;const e=expected(r);check('pole-flag',r.ok===Number(e!==null),r);
   const fields=['type','q','p','scale','op','phase','ok'];
   if(e!==null){
    fields.push('poly','real','imag');counts.values++;
    check('minimal-polynomial',JSON.stringify(r.poly)===JSON.stringify(e.poly),r);
    for(const part of ['real','imag']){const b=endpoints(e[part],r[part]);for(const [k,ok] of Object.entries(b))check(part+'-'+k,ok,r);}
    if('rp' in e){fields.push('recognized','nullRecognized','rp','rq');counts.roots++;
     check('root-recognition',r.recognized===1&&r.nullRecognized===1&&r.rp===e.rp&&r.rq===e.rq,r);}
   }else counts.poles++;
   assert.deepEqual(Object.keys(r),fields);
  }else if(r.type==='nonroot'){
   counts.nonroots++;assert.deepEqual(Object.keys(r),['type','control','phase','recognized','nullRecognized']);
   check('nonroot-recognition',r.recognized===0&&r.nullRecognized===0,r);
  }else {assert.deepEqual(Object.keys(r),['type','rowsBeforeTerminal']);assert.equal(r.rowsBeforeTerminal,3786);}
 }
 assert.deepEqual(counts,{angles:3776,values:3408,poles:368,roots:944,nonroots:10});
 return {status:failures.length?'mathematical-fail':'pass',records:rows.length,counts,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),failures};
}
function selfTests(){
 fieldSelfTest();assert.deepEqual(minimal(sine(1)),['1','0','-16','0','16']);
 assert.deepEqual(minimal(fdiv(sine(1),cosine(1))),['1','-4','1']);
 assert.deepEqual(cyclotomic(24).map(String),['1','0','0','0','-1','0','0','0','1']);
 for(let k=0;k<24;k++)assert(fz(fsub(fadd(fmul(sine(k),sine(k)),fmul(cosine(k),cosine(k))),integer(1))));
 assert.throws(()=>exactDivide([1n,0n,1n],[-1n,1n]));
}
export function checkQqbarTrig(path,controls=true){
 selfTests();const rows=readFileSync(path,'utf8').trimEnd().split('\n').map(JSON.parse),result=checkRows(rows);
 let rejected=0;
 if(controls&&result.status==='pass'){
  const value=rows.findIndex(r=>r.type==='angle'&&r.op==='sin'&&r.ok===1&&r.q===12&&r.p===1),root=rows.findIndex(r=>r.type==='angle'&&r.op==='root');
  const pole=rows.findIndex(r=>r.type==='angle'&&!r.ok);
  const mutations=[
   r=>{r[value].poly[0]='999';},r=>{r[value].real=['-2','-1','0'];},r=>{r[value].imag=['1','1','0'];},
   r=>{r[root].rp+=1;},r=>{r[pole].ok=1;},r=>{r[value].phase=7;},r=>{r[1]=r[0];},r=>r.pop(),
   r=>{r.at(-1).rowsBeforeTerminal--;},r=>{r.find(x=>x.type==='nonroot').nullRecognized=1;}
  ];
  for(const mutate of mutations){const copy=structuredClone(rows);mutate(copy);let failed=false;
   try{failed=checkRows(copy).status!=='pass';}catch{failed=true;}assert(failed,'Corruption was not rejected');rejected++;}
 }
 return {...result,corruptionControlsRejected:rejected,limits:'Small in-contract pi/12 biquadratic corpus only; exact polynomial and both component containment checks, not arbitrary denominator closure, archived execution, branch coverage or performance qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const r=checkQqbarTrig(process.argv[2]);console.log(JSON.stringify(r));process.exitCode=r.status==='pass'?0:1;
}
