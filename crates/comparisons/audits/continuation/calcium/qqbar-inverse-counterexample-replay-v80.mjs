// A second exact proof replay. Homogeneous Horner avoids repeated gcd work on
// the degree-384 dyadic endpoints. The original rational-Horner proof and its
// full corruption run remain frozen and must agree with this independent path.
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {q,cmp,add,mul,neg,gcd,exactDivide,evaluate,decodeEndpoints} from './qqbar-inverse-oracle-v80.mjs';
const sign=n=>n<0n?-1:n>0n?1:0;
function homogeneous(p,[n,d]){
 assert(d>0n);let a=p.at(-1),power=1n;
 for(let i=p.length-2;i>=0;i--){power*=d;a=a*n+p[i]*power;}
 return[a,power];
}
export function counterexampleReplay(){
 let differential=0;
 for(let degree=0;degree<=20;degree++)for(const n of [-17,-3,0,1,19])for(const d of [1,2,17,131]){
  const p=Array.from({length:degree+1},(_,i)=>BigInt(((degree+3)*(i+7))%23-11)),x=q(n,d),a=homogeneous(p,x),b=evaluate(p,x);
  assert.equal(a[0]*b[1],b[0]*a[1]);differential++;
 }
 const lines=readFileSync('results/qqbar-inverse-counterexample-native-v80.stdout','utf8').trimEnd().split('\n');assert.equal(lines.length,1);
 const r=JSON.parse(lines[0]);assert.deepEqual(Object.keys(r),['denominator','degree','recognized','poly','real','imag','proposal']);
 assert.equal(r.denominator,3360);assert.equal(r.degree,384);assert.equal(r.recognized,0);assert.deepEqual(r.proposal,{p:1,q:3359,overlap:0});
 assert(Array.isArray(r.poly)&&r.poly.length===385&&r.poly.every(x=>typeof x==='string'&&/^-?\d+$/.test(x)));
 const p=r.poly.map(BigInt);assert.equal(p.reduce(gcd,0n),1n);assert(p.at(-1)>0n);
 const f=Array(3360).fill(0n);let choose=1n;
 for(let k=0;k<=3360;k++){
  if(k%2)f[k]=(k%4===1?1n:-1n)*choose;
  if(k<3360){const numerator=choose*BigInt(3360-k);assert.equal(numerator%BigInt(k+1),0n);choose=numerator/BigInt(k+1);}
 }
 assert.equal(choose,1n);assert.equal(f[1],3360n);assert.equal(f.at(-1),-3360n);
 const quotient=exactDivide(f,p);assert.equal(quotient.length,2976);
 const [lo,hi]=decodeEndpoints(r.real),[il,ih]=decodeEndpoints(r.imag);assert(cmp(lo,hi)<0);
 assert(cmp(lo,q(1,1200))>0&&cmp(hi,q(1,1000))<0);assert(cmp(add(hi,neg(lo)),q(1n,1n<<96n))<=0);
 assert.equal(il[0],0n);assert.equal(ih[0],0n);
 const signs=[sign(homogeneous(p,lo)[0]),sign(homogeneous(p,hi)[0])];assert.equal(signs[0]*signs[1],-1);
 const upperT=q(22,7*3360),den=add(q(1),neg(mul(q(1,2),mul(upperT,upperT))));
 assert(cmp(q(3,3360),q(1,1200))>0&&cmp(den,q(0))>0);
 assert(cmp(q(upperT[0]*den[1],upperT[1]*den[0]),q(1,1000))<0);assert(cmp(q(6,3360),q(1,1000))>0);
 assert(cmp(add(q(1,3359),neg(q(1,3360))),q(1,10000000))<0);
 assert(cmp(add(q(1,3358),neg(q(1,3360))),q(1,10000000))>0);
 const slow=JSON.parse(readFileSync('results/qqbar-inverse-counterexample-check-confirmed-v80.stdout','utf8'));
 assert.equal(slow.status,'pass');assert.equal(slow.corruptionControlsRejected,8);assert.equal(slow.denominator,3360);assert.equal(slow.degree,384);
 assert.equal(slow.polynomialTerms,p.length);assert.equal(slow.tangentPolynomialTerms,f.length);assert.equal(slow.quotientTerms,quotient.length);
 assert.equal(slow.exactDivisionRemainder,'zero');assert.deepEqual(slow.rootBracket,['1/1200','1/1000']);assert.equal(slow.principalAtanPi,'1/3360');
 assert.equal(slow.observedRecognition,0);assert.deepEqual(slow.observedProposal,r.proposal);
 return{status:'pass',primary:slow,homogeneousReplay:{differentialCases:differential,endpointSigns:signs,divisionRemainder:'zero'},
  note:'Primary captured proof uses normalized rational Horner; current replay uses fraction-free homogeneous Horner. Both prove the same exact input, not approximate agreement.'};
}
if(process.argv[1]?.endsWith('/qqbar-inverse-counterexample-replay-v80.mjs'))console.log(JSON.stringify(counterexampleReplay()));
