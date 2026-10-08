// Independent exact arithmetic in the four-dimensional field Q(sqrt(2),sqrt(3)).
// No FLINT, qqbar, floating-point root or interval-overlap oracle is used.
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';

const abs = n => n < 0n ? -n : n;
function gcd(a,b) { a=abs(a); b=abs(b); while(b) [a,b]=[b,a%b]; return a; }
const lcm = (a,b) => a/gcd(a,b)*b;
function q(n,d=1n) { n=BigInt(n); d=BigInt(d); assert(d!==0n); if(d<0n){n=-n;d=-d;} const g=gcd(n,d); return [n/g,d/g]; }
const qa=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]);
const qn=a=>[-a[0],a[1]], qm=(a,b)=>q(a[0]*b[0],a[1]*b[1]);
const qd=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const qc=(a,b)=>{const d=a[0]*b[1]-b[0]*a[1];return d<0n?-1:d>0n?1:0;};
const zero=()=>[q(0),q(0),q(0),q(0)];
const scalar=a=>[a,q(0),q(0),q(0)];
const one=()=>scalar(q(1));
const add=(a,b)=>a.map((v,i)=>qa(v,b[i]));
const neg=a=>a.map(qn), sub=(a,b)=>add(a,neg(b));
function mul(a,b) {
 const c=zero();
 for(let i=0;i<4;i++) for(let j=0;j<4;j++) {
  const repeated=i&j, factor=((repeated&1)?2:1)*((repeated&2)?3:1);
  c[i^j]=qa(c[i^j],qm(qm(a[i],b[j]),q(factor)));
 }
 return c;
}
const conjugate=(a,mask)=>a.map((v,i)=>(((i&mask)&1)^(((i&mask)>>1)&1))?qn(v):v);
function inv(a) {
 const other=mul(mul(conjugate(a,1),conjugate(a,2)),conjugate(a,3));
 const norm=mul(a,other);
 assert(norm.slice(1).every(v=>v[0]===0n)); assert(norm[0][0]!==0n);
 return other.map(v=>qd(v,norm[0]));
}
const div=(a,b)=>mul(a,inv(b));
function pow(a,n) { if(n<0)return pow(inv(a),-n); let v=one();for(let i=0;i<n;i++)v=mul(v,a);return v; }
const key=a=>a.map(v=>v.join('/')).join(',');
const equal=(a,b)=>key(a)===key(b);
const operations={add,sub,mul,div};
const recipes=[
 [0,0,0,0,1],[1,0,0,0,1],[-1,0,0,0,1],[1,0,0,0,2],
 [0,1,0,0,1],[0,-1,0,0,1],[0,0,1,0,1],[0,0,0,1,1],
 [1,1,0,0,1],[1,-1,0,0,1],[0,1,1,0,1],[0,1,-1,0,1],
 [1,1,1,1,1],[-1,2,-1,1,3],[2,-1,1,-2,5],[0,0,-1,0,1]
];
const values=recipes.map(r=>r.slice(0,4).map(n=>q(n,r[4])));
const affine=[[0,2,3],[-2,1,3],[3,-2,-5]];
const polys=[[],[q(1,3)],[q(-2),q(3,2)],[q(1),q(-2),q(0),q(1)],
 [q(1,3),q(-2,3),q(1),q(0),q(-1,3),q(2,3)]];
const evaluate=(a,p)=>p.reduceRight((v,c)=>add(mul(v,a),scalar(c)),zero());
function orbit(a) {
 const unique=new Map();for(let i=0;i<4;i++){const c=conjugate(a,i);unique.set(key(c),c);}return [...unique.values()];
}
function productOfLinearFactors(roots) {
 let p=[one()];
 for(const r of roots){const t=Array.from({length:p.length+1},zero);for(let i=0;i<p.length;i++){
  t[i]=sub(t[i],mul(p[i],r));t[i+1]=add(t[i+1],p[i]);}p=t;}
 assert(p.every(c=>c.slice(1).every(v=>v[0]===0n)),'Galois-invariant coefficients must be rational');
 const rational=p.map(c=>c[0]), den=rational.reduce((d,c)=>lcm(d,c[1]),1n);
 const integers=rational.map(c=>c[0]*(den/c[1]));
 const content=integers.reduce(gcd,0n)*(integers.at(-1)<0n?-1n:1n);
 return integers.map(n=>(n/content).toString());
}
// For this Galois field, distinct automorphic images are exactly the conjugates.
// Their product is minimal because 1,sqrt(2),sqrt(3),sqrt(6) are Q-independent.
const minimalCache=new Map();
function minimal(a){const k=key(a);if(!minimalCache.has(k))minimalCache.set(k,productOfLinearFactors(orbit(a)));return minimalCache.get(k);}
function sqrtFloor(n) {
 assert(n>=0n);if(n<2n)return n;let x=1n<<BigInt(Math.ceil(n.toString(2).length/2));
 for(;;){const y=(x+n/x)>>1n;if(y>=x)return x;x=y;}
}
const boundsCache=new Map();
function fieldBounds(a,bits) {
 const k=key(a)+':'+bits;if(boundsCache.has(k))return boundsCache.get(k);
 const scale=1n<<BigInt(bits), radicands=[1n,2n,3n,6n];let lo=q(0),hi=q(0);
 for(let i=0;i<4;i++){
  const target=radicands[i]*scale*scale, r=sqrtFloor(target);
  const low=q(r,scale), high=q(r*r===target?r:r+1n,scale);
  lo=qa(lo,qm(a[i],a[i][0]<0n?high:low));hi=qa(hi,qm(a[i],a[i][0]<0n?low:high));
 }
 const result=[lo,hi];boundsCache.set(k,result);return result;
}
function dyadic(n,e){n=BigInt(n);e=BigInt(e);assert(abs(e)<100000n);return e>=0n?q(n<<e):q(n,1n<<-e);}
function contains(a,lo,hi){
 for(let bits=64;bits<=4096;bits*=2){const [l,h]=fieldBounds(a,bits);
  if(qc(lo,l)<=0&&qc(h,hi)<=0)return true;
  if(qc(hi,l)<0||qc(h,lo)<0)return false;
 }
 throw Error('Independent endpoint separation budget exhausted');
}
function expected(r) {
 const a=values[r.i], p=r.parameter;
 if(operations[r.op])return operations[r.op](a,values[r.j]);
 switch(r.op){
  case 'input':return a;
  case 'inv':return inv(a);
  case 'pow':return pow(a,p);
  case 'scale':return a.map(v=>qm(v,p>=0?q(1n<<BigInt(p)):q(1n,1n<<BigInt(-p))));
  case 'affine':{const [c,b,d]=affine[p];return add(a.map(v=>qm(v,q(c,d))),scalar(q(b,d)));}
  case 'evaluate':return evaluate(a,polys[p]);
  default:throw Error('Unexpected operation '+r.op);
 }
}
function expectedKeys() {
 const wanted=new Set();
 const put=(phase,i,j,op,alias=0,parameter=0)=>wanted.add(JSON.stringify(['value',phase,i,j,op,alias,parameter]));
 for(let phase=0;phase<2;phase++)for(let i=0;i<16;i++){
  put(phase,i,-1,'input');
  for(let j=0;j<16;j++)for(const op of Object.keys(operations))if(op!=='div'||j!==0){
   for(let alias=0;alias<3;alias++)put(phase,i,j,op,alias);
   if(!phase)wanted.add(JSON.stringify(['composed',i,j,op]));
  }
  for(let alias=0;alias<2;alias++){
   if(i)put(phase,i,-1,'inv',alias);
   for(let n=-3;n<=4;n++)if(i||n>=0)put(phase,i,-1,'pow',alias,n);
   for(const n of [-3,0,3])put(phase,i,-1,'scale',alias,n);
   for(let p=0;p<3;p++)put(phase,i,-1,'affine',alias,p);
   for(let p=0;p<5;p++)put(phase,i,-1,'evaluate',alias,p);
  }
  for(let p=0;p<5;p++)for(let j=0;j<16;j++)wanted.add(JSON.stringify(['relation',phase,i,j,p]));
  wanted.add(JSON.stringify(['preserved',phase,i]));
 }
 wanted.add(JSON.stringify(['terminal']));return wanted;
}
function rowKey(r){switch(r.type){
 case 'value':return JSON.stringify([r.type,r.phase,r.i,r.j,r.op,r.alias,r.parameter]);
 case 'composed':return JSON.stringify([r.type,r.i,r.j,r.op]);
 case 'relation':return JSON.stringify([r.type,r.phase,r.i,r.j,r.parameter]);
 case 'preserved':return JSON.stringify([r.type,r.phase,r.i]);
 case 'terminal':return JSON.stringify([r.type]);default:throw Error('Unexpected record type');
}}
function selfTest(){
 assert.deepEqual(minimal(values[4]),['-2','0','1']);
 assert.deepEqual(minimal(values[10]),['1','0','-10','0','1']);
 assert.deepEqual(minimal(values[3]),['-1','2']);
 assert.deepEqual(minimal(values[0]),['0','1']);
 for(const a of values.slice(1))assert(equal(mul(a,inv(a)),one()));
 for(let n=0n;n<100n;n++){const r=sqrtFloor(n);assert(r*r<=n&&(r+1n)*(r+1n)>n);}
 assert(contains(values[4],q(7,5),q(3,2)));
 assert(!contains(values[4],q(-3,2),q(-7,5)));
}
export function checkQqbarArithmetic(path) {
 selfTest();const rows=readFileSync(path,'utf8').trim().split('\n').map(s=>JSON.parse(s));
 const wanted=expectedKeys(), failures=[], counts={}, checks={}, composedCache=new Map();
 const check=(name,ok,r,detail)=>{checks[name]=(checks[name]??0)+1;if(!ok)failures.push({check:name,key:rowKey(r),detail});};
 let positiveRelations=0,negativeRelations=0;
 for(const r of rows){const k=rowKey(r);assert(wanted.delete(k),'Duplicate or unexpected record '+k);counts[r.type]=(counts[r.type]??0)+1;
  if(r.type==='value'){
   const a=expected(r);check('minimal-polynomial',JSON.stringify(r.poly)===JSON.stringify(minimal(a)),r,{expected:minimal(a),actual:r.poly});
   const [l,h,e]=r.real, lo=dyadic(l,e),hi=dyadic(h,e);
   check('ordered-endpoints',qc(lo,hi)<=0,r);
   check('root-enclosure',contains(a,lo,hi),r);
   check('absolute-width-2^-96',qc(qa(hi,qn(lo)),q(1n,1n<<96n))<=0,r);
   check('exact-real-axis',r.imagZero===1,r);
  }else if(r.type==='composed'){
   const ckey=[r.i,r.j,r.op].join(':');
   if(!composedCache.has(ckey))composedCache.set(ckey,productOfLinearFactors(orbit(values[r.i]).flatMap(a=>orbit(values[r.j]).map(b=>operations[r.op](a,b)))));
   const p=composedCache.get(ckey);check('full-composed-polynomial',JSON.stringify(p)===JSON.stringify(r.poly),r,{expected:p,actual:r.poly});
  }else if(r.type==='relation'){
   const eq=equal(values[r.j],evaluate(values[r.i],polys[r.parameter]));
   if(eq)positiveRelations++;else negativeRelations++;
   check('polynomial-value-relation',r.equal===Number(eq),r,{expected:eq,actual:r.equal});
  }else if(r.type==='preserved'){
   check('readonly-polynomial',r.polynomial===1,r);check('readonly-enclosure',r.enclosure===1,r);
  }else{
   assert.equal(r.rowsBeforeTerminal,rows.length-1);assert.equal(r.values,16);assert.equal(r.phases,2);
  }
 }
 assert.equal(wanted.size,0,'Missing records');assert.equal(rows.at(-1).type,'terminal');
 return {status:failures.length?'mathematical-fail':'pass',records:rows.length,counts,checks,
  totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),positiveRelations,negativeRelations,failures,
  limits:'Small real biquadratic corpus only; full coefficients and selected roots, not branch coverage, general algebraic closure, archived execution, direct LLL qualification, or performance measurement.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=checkQqbarArithmetic(process.argv[2]);console.log(JSON.stringify(result));process.exitCode=result.failures.length?1:0;
}
