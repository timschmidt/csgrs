import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';

const gcd=(a,b)=>{a=a<0n?-a:a;while(b)[a,b]=[b,a%b];return a;};
const q=(n,d=1n)=>{assert(d>0n);const g=gcd(n,d);return[n/g,d/g];};
const add=([a,b],[c,d])=>q(a*d+c*b,b*d),neg=([a,b])=>[-a,b],sub=(a,b)=>add(a,neg(b));
const mul=([a,b],[c,d])=>q(a*c,b*d),cmp=([a,b],[c,d])=>a*d<c*b?-1:a*d>c*b?1:0;
const abs=a=>a[0]<0n?neg(a):a,bitLength=n=>n===0n?0:n.toString(2).length;
const power=e=>e<0?q(1n,1n<<BigInt(-e)):q(1n<<BigInt(e));
const serial=a=>a.map(String),max=(a,b)=>cmp(a,b)<0?b:a;
const sourceSteps=n=>bitLength(n+1n),tightSteps=n=>bitLength(n);
const newton=([a,b],[c,d])=>q(a*a*d+c*b*b,2n*a*b*d);
const containsRoot=(y,x,error,strict)=>{
 const lo=sub(y,error),hi=add(y,error);assert(hi[0]>0n);
 if(lo[0]>=0n)assert(strict?cmp(mul(lo,lo),x)<0:cmp(mul(lo,lo),x)<=0);
 assert(strict?cmp(mul(hi,hi),x)>0:cmp(mul(hi,hi),x)>=0);
};

export function model(){
 const digest=createHash('sha256');let checkedRecords=0;
 const record=x=>{digest.update(JSON.stringify(x)+'\n');checkedRecords++;};
 const domain=[...Array.from({length:29},(_,i)=>q(BigInt(i+4),16n)),q(1n,3n),q(2n,3n),q(5n,7n),q(7n,11n),add(q(1n,4n),power(-64)),sub(q(2n),power(-64))];
 let normalizedBounds=0,fastBounds=0,savedIterations=0,maxRationalBits=0;
 for(const x of domain){
  assert(cmp(x,q(1n,4n))>=0&&cmp(x,q(2n))<=0);
  const sequence=[q(1n)];
  for(let s=1;s<=9;s++)sequence.push(newton(sequence.at(-1),x));
  for(let s=0;s<sequence.length;s++){
   const y=sequence[s];assert(y[0]>0n);assert(cmp(y,q(1n,2n))>=0);
   if(s>0)assert(cmp(mul(y,y),x)>=0);
   containsRoot(y,x,power(-(2**s)),false);normalizedBounds++;
   maxRationalBits=Math.max(maxRationalBits,...y.map(bitLength));
   record({family:'exact-newton',x:serial(x),step:s,y:serial(y)});
  }
  for(let n=0;n<256;n++){
   const old=sourceSteps(BigInt(n)),tight=tightSteps(BigInt(n));
   containsRoot(sequence[old],x,power(-n),true);
   containsRoot(sequence[tight],x,power(-n),true);fastBounds+=2;
   savedIterations+=old-tight;
  }
 }
 let plannerChecks=0,plannerSavings=0;
 const checkPlanner=n=>{
  const old=sourceSteps(n),tight=tightSteps(n);assert((1n<<BigInt(tight))>n);
  if(tight>0)assert((1n<<BigInt(tight-1))<=n);
  assert(old>=tight&&old-tight<=1);assert((1n<<BigInt(old))>n);
  plannerChecks++;plannerSavings+=old-tight;record({family:'planner',n:String(n),old,tight});
 };
 for(let n=0;n<65536;n++)checkPlanner(BigInt(n));
 const bigPlannerInputs=new Set();
 for(const k of [16,17,31,32,63,64,127,128,255,256,511,512,1023,1024,2047,2048,4095,4096])for(const d of [-1n,0n,1n]){
  const n=(1n<<BigInt(k))+d;if(n>=65536n)bigPlannerInputs.add(String(n));
 }
 for(const n of bigPlannerInputs)checkPlanner(BigInt(n));
 // Every admissible coarse magnitude exponent, including odd negative z,
 // must normalize x into [1/4,2] with an exact inverse reconstruction.
 let scaleCases=0;
 for(let e=-64;e<=64;e++)for(const m of [q(1n),q(5n,4n),q(3n,2n),q(7n,4n)]){
  const x=mul(m,power(e));
  for(let z=e-3;z<=e+4;z++)if(cmp(power(z-2),x)<=0&&cmp(x,power(z))<=0){
   const half=Math.floor(z/2),y=mul(power(-2*half),x);
   assert(cmp(y,q(1n,4n))>=0&&cmp(y,q(2n))<=0);assert.equal(cmp(mul(power(2*half),y),x),0);scaleCases++;
   record({family:'scale',exponent:e,m:serial(m),z,half,y:serial(y)});
  }
 }
 let zeroPairs=0,zeroBranches=0;
 for(let n=0;n<=256;n++)for(let j=0;j<=32;j++){
  const eps=power(-n),x=mul(q(BigInt(j),16n),mul(eps,eps));
  const positive=x[0]>0n,small=cmp(x,mul(eps,eps))<0;assert(positive||small);
  if(small){assert(cmp(x,mul(eps,eps))<0);zeroBranches++;}zeroPairs++;
 }
 // Models of two admitted complex lemmas, not replacement formal proofs.
 const squares=new Map();let complexSmallPremises=0,complexSmallExcluded=0,complexSquareRoots=0;
 for(let a=-16;a<=16;a++)for(let b=-16;b<=16;b++){
  const u=q(BigInt(a),16n),v=q(BigInt(b),16n),re=sub(mul(u,u),mul(v,v)),im=mul(q(2n),mul(u,v));
  const key=JSON.stringify([serial(re),serial(im)]),roots=squares.get(key)??[];roots.push([a,b]);squares.set(key,roots);complexSquareRoots++;
  for(let n=0;n<=16;n++){
   const eps=power(-n),scaledU=mul(u,eps),scaledV=mul(v,eps),scaledRe=mul(re,mul(eps,eps)),scaledIm=mul(im,mul(eps,eps));
   if(cmp(max(abs(scaledRe),abs(scaledIm)),power(-2*n-1))<=0){assert(cmp(max(abs(scaledU),abs(scaledV)),eps)<=0);complexSmallPremises++;}
   else complexSmallExcluded++;
  }
 }
 for(const roots of squares.values()){
  assert(roots.length<=2);if(roots.length===2)assert.deepEqual(roots[0],roots[1].map(n=>n===0?0:-n));
 }
 // Countermodels show why multivalued limits need both coherence and closure.
 const alternating=[q(-1n),q(1n),q(-1n),q(1n)];
 assert(alternating.every(x=>x[0]===1n||x[0]===-1n));
 assert(cmp(abs(sub(alternating[1],alternating[2])),add(power(-1),power(-2)))>0);
 assert(cmp(power(-16),power(-15))<0);assert(power(-16)[0]>0n);assert.equal(q(0n)[0],0n);
 // Deliberately false numeric or scheduling claims must fail the oracle.
 const controls=[
  ()=>containsRoot(q(1n),q(1n,4n),power(-2),true),
  ()=>containsRoot(q(5n,8n),q(1n,4n),power(-3),true),
  ()=>assert((1n<<BigInt(tightSteps(255n)-1))>255n),
  ()=>assert(cmp(mul(power(-2*Math.trunc(-1/2)),power(-3)),q(1n,4n))>=0),
  ()=>assert(cmp(max(abs(q(2n)),abs(q(0n))),q(1n))<=0),
  ()=>assert(cmp(abs(sub(alternating[1],alternating[2])),add(power(-1),power(-2)))<=0),
 ];
 for(const fail of controls)assert.throws(fail);
 return{status:'verified-independent-exact-rational-model',normalizedInputs:domain.length,normalizedBounds,fastBounds,
 savedIdealNewtonIterations:savedIterations,maxRationalComponentBits:maxRationalBits,plannerChecks,plannerSavings,
 scaleCases,zeroPairs,admissibleZeroBranches:zeroBranches,complexSquareRoots,distinctComplexSquares:squares.size,
 complexSmallPremises,complexSmallExcluded,semanticCountermodels:2,corruptionControls:controls.length,checkedRecords,sha256:digest.digest('hex'),
 limits:'Mathematical model and deterministic operation counts only; not Coq/Haskell execution, a donor proof-hole repair, Hyper guard-bit proof or native performance/memory comparison.'};
}
console.log(JSON.stringify(model()));
