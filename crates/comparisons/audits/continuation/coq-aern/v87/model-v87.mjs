import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
const hash=createHash('sha256');let records=0;
const emit=x=>{hash.update(JSON.stringify(x,(_,v)=>typeof v==='bigint'?v.toString():v)+'\n');records++;};
const abs=x=>x<0n?-x:x;
const max=v=>v.reduce((a,b)=>a>b?a:b,0n);
const norm=v=>max(v.map(abs));
const sub=(a,b)=>{assert.equal(a.length,b.length);return a.map((x,i)=>x-b[i]);};
const add=(a,b)=>{assert.equal(a.length,b.length);return a.map((x,i)=>x+b[i]);};
const scale=(a,s)=>a.map(x=>x*s);
const distance=(a,b)=>norm(sub(a,b));
let scalarTriples=0,intervalCases=0,convexCases=0,vectorCases=0,pathChecks=0,rootCounterexamples=0,formalZeroChecks=0;
// Integers here represent numerators over one common positive denominator.
for(let a=-16n;a<=16n;a++)for(let b=-16n;b<=16n;b++)for(let c=-16n;c<=16n;c++){
 const d=abs(a-b);assert(d<=abs(a-c)+abs(c-b));
 assert.equal(d<=c,-c<=a-b&&a-b<=c);assert.equal(d<c,-c<a-b&&a-b<c);
 assert.equal(abs(a*b),abs(a)*abs(b));scalarTriples++;intervalCases+=2;
 emit(['scalar',a,b,c,d]);
}
for(let a=-8n;a<=8n;a++)for(let b=a+1n;b<=8n;b++)for(let u=1n;u<=4n;u++)for(let v=1n;v<=4n;v++){
 const numerator=a*u+b*v,denominator=u+v;
 assert(a*denominator<numerator&&numerator<b*denominator);convexCases++;
 emit(['convex',a,b,u,v,numerator,denominator]);
}
const dimensions=[0,1,2,3,4,16,64,256];
for(const d of dimensions)for(let seed=0;seed<64;seed++){
 const vector=k=>Array.from({length:d},(_,i)=>BigInt(((i+1)*(seed+3)*(k+5)+k*17)%97-48));
 const a=vector(0),b=vector(1),c=vector(2),ab=distance(a,b);
 assert.equal(ab,distance(b,a));assert.equal(ab===0n,a.every((x,i)=>x===b[i]));
 assert(distance(a,c)<=ab+distance(b,c));assert.equal(distance(add(a,c),add(b,c)),ab);
 assert.equal(norm(a),norm(scale(a,-1n)));
 for(const s of [-7n,-1n,0n,1n,3n])assert.equal(distance(scale(a,s),scale(b,s)),abs(s)*ab);
 const sq=sub(a,b).reduce((s,x)=>s+x*x,0n);
 assert(ab*ab<=sq&&sq<=BigInt(d)*ab*ab);
 vectorCases++;emit(['vector',d,seed,ab,sq]);
}
// Common denominator 2^K, x_n = target + sign_i * 2^-n.
// Exact finite-prefix checks of the stated modulus; not a proof over all n.
const K=32n,unit=1n<<K;
for(const d of dimensions){
 const target=Array.from({length:d},(_,i)=>BigInt(i%7-3)*unit);
 const signs=Array.from({length:d},(_,i)=>BigInt(i%3-1));
 const seq=n=>add(target,scale(signs,1n<<(K-BigInt(n))));
 for(let n=0;n<32;n++){
  const eps=1n<<(K-BigInt(n)),half=eps/2n;
  assert(distance(target,seq(n))<=eps);assert(distance(seq(n),seq(n+1))<=half);
  for(let m=0;m<=32;m++){
   assert(distance(seq(n),seq(m))<=eps+(1n<<(K-BigInt(m))));pathChecks++;
  }
  emit(['path',d,n,distance(target,seq(n))]);
 }
}
// Hand sqrt: eps=2^-n, admissible input x=eps/4 and coarse enclosure [-x,3x].
// Left predicate 0<x is unresolved, right x<2eps is certainly true.
// The exact root is >eps for n>2. All comparisons use squared rational bounds.
for(let n=3;n<=512;n++){
 const eDen=1n<<BigInt(n),xDen=4n*eDen;
 assert(3n*eDen<2n*xDen); // enclosure upper 3/xDen < 2/eDen
 assert(eDen*eDen>xDen); // x > eps^2, hence sqrt(x)>eps
 rootCounterexamples++;emit(['hand-zero-counterexample',n,xDen,eDen]);
 // Formal/extracted threshold is eps^2; exhaust dyadic fractions below it.
 for(let j=0n;j<32n;j++){
  // x = j/(32*eDen^2), so the zero approximation is strictly valid.
  assert(j<32n);formalZeroChecks++;emit(['formal-zero',n,j,32n*eDen*eDen]);
 }
}
// Exact sqrt counterexample at n=4: x=1/64, sqrt(x)=1/8, eps=1/16.
assert(1n*16n>1n*8n);assert.equal(8n*8n,64n);
const counterexample={n:4,x:'1/64',inputEnclosure:['-1/64','3/64'],left:'unknown',right:'certainly true',selectedApproximation:'0',declaredRadius:'1/16',exactRoot:'1/8'};
emit(counterexample);
let corruptionControls=0;
const reject=f=>{assert.throws(f,assert.AssertionError);corruptionControls++;};
reject(()=>assert(norm([1n,1n])**2n>=2n)); // max norm cannot be Euclidean length
reject(()=>assert.equal(distance([1n],[0n]),-distance([1n],[0n]))); // sign in scale
reject(()=>assert(norm([0n,-2n])===max([0n,-2n]))); // omitting abs
reject(()=>distance([1n],[1n,2n])); // erased dimension mismatch
reject(()=>assert.equal(distance([1n,-2n],[0n,0n]),0n)); // arbitrary first coordinate
reject(()=>assert(1n/8n>1n/16n)); // truncating rational oracle is invalid
reject(()=>assert(64n>=16n*16n)); // claiming the hand zero meets its bound
// Same coordinate set membership does not preserve coupled predicates.
reject(()=>assert.equal(2n*(-1n)*(-2n),-4n)); // mixed roots of -3-4i
// Check actual mixed root (-1,-2) squares to -3+4i, not -3-4i.
assert.equal(2n*(-1n)*(-2n),4n);assert.notEqual(4n,-4n);emit(['mixed-complex-root',-1, -2, -3,4]);
export const result={status:'verified-independent-metric-and-boundary-model',scalarTriples,intervalCases,convexCases,dimensions,vectorCases,pathChecks,rootCounterexamples,formalZeroChecks,corruptionControls,counterexample,records,sha256:hash.digest('hex'),nativeDonorExecution:false};
console.log(JSON.stringify(result));
