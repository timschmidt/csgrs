// Independent integer reconstruction of the triangular high-product sum.
// This does not call FLINT, GMP, the C checker, or its parsing routines.
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const B=1n<<64n,M=B-1n,T=B>>1n;
const wrap=x=>x&M;
function mix(x){x=wrap(x+0x9e3779b97f4a7c15n);x=wrap((x^(x>>30n))*0xbf58476d1ce4e5b9n);x=wrap((x^(x>>27n))*0x94d049bb133111ebn);return x^(x>>31n);}
function word(n,p,i){
 switch(p){
  case 0:return 0n;case 1:return i===0?1n:0n;case 2:return M;case 3:return i+1===n?T:0n;
  case 4:return i%2?0n:M;case 5:return 0xaaaaaaaaaaaaaaaan;case 6:return 0x5555555555555555n;
  case 7:return i<Math.floor(n/2)?M:0n;case 8:return i+1===n?M:1n;case 9:return i+1===n?T:M;
  case 10:return i===Math.floor(n/2)?T:0n;case 11:return i===0?M:T;
  case 12:return 1n<<BigInt((i*7+n)%64);case 13:return M^(1n<<BigInt((i*13+n)%64));
  case 14:return i+1===n?T-1n:M;case 15:return i+1===n?T+1n:0n;
  default:return mix(BigInt(n)*31337n+BigInt(p)*1000003n+BigInt(i));
 }
}
function operands(n,p,norm){
 const a=Array.from({length:n},(_,i)=>word(n,p,i));
 const b=a.map((v,i)=>p===8?v:p===9?word(n,p,n-1-i):p===10?(M^v):word(n,(p*7+3)%32,i));
 if(norm){a[n-1]|=T;b[n-1]|=T;}return[a,b];
}
const integer=a=>a.reduceRight((v,x)=>v*B+x,0n);
function triangle(a,b){
 const n=a.length;let sum=0n;
 for(let i=0;i<n;i++)for(let j=Math.max(0,n-2-i);j<n;j++){
  const product=a[i]*b[j],d=i+j-(n-1);
  sum+=d===-1?product/B:product*(B**BigInt(d));
 }
 return sum;
}
export function checkHighProductBigint(){
 const rows=readFileSync('results/high-product-native.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 let checked=0,integerFailures=0,fractionalFailures=0;const witnesses=[],cached=new Map();
 for(const r of rows){
  if(!r.op||r.n>128||!(r.pattern===13||[1,2,3,13,40,90].includes(r.n)))continue;
  const square=r.op.startsWith('square');
  if(square?r.n>90:!['mul-naive','mul-recursive'].includes(r.op)&&r.n>40)continue;
  const key=[r.n,r.pattern,r.normalised_input,square].join('/');let c=cached.get(key);
  if(!c){
   const[a,b]=operands(r.n,r.pattern,r.normalised_input),y=square?a:b;
   c={product:integer(a)*integer(y),approx:triangle(a,y),cut:B**BigInt(r.n-1)};cached.set(key,c);
   // Explicit omitted-product formula: low parts on diagonal n-2 plus
   // all full products below that diagonal, in the original product units.
   let omitted=0n;
   for(let i=0;i<r.n;i++)for(let j=0;j<r.n-1-i;j++){
    const p=a[i]*y[j],d=i+j;
    omitted+=(d===r.n-2?p%B:p)*(B**BigInt(d));
   }
   assert.equal(c.product-c.approx*c.cut,omitted);
   // For n>=3 the omitted tail is strictly below (2n-3) guard ulps.
   // This explains why an n+2 bound need not hold; it does not certify
   // other recursive/assembly/FFT algorithms outside the checked mapping.
   if(r.n>=3)assert(omitted<(2n*BigInt(r.n)-3n)*c.cut);
  }
  const scale=1n<<BigInt(r.shift),deficit=c.product*scale/c.cut-c.approx*scale;
  const residual=c.product*scale-c.approx*scale*c.cut;
  assert.equal(r.deficit,deficit.toString());assert.equal(r.exact,Number(deficit===0n));
  assert.equal(r.bound,Number((BigInt(r.n)+2n)*scale+scale-1n));
  const ok=deficit>=0n&&deficit<=BigInt(r.bound),fractional=residual<=(BigInt(r.n)+2n)*scale*c.cut;
  assert.equal(r.ok,Number(ok));assert.equal(r.fractional_ok,Number(fractional));
  checked++;integerFailures+=!ok;fractionalFailures+=!fractional;
  if(!ok&&r.normalised_input===1&&['square','mul-naive'].includes(r.op))
   witnesses.push({op:r.op,n:r.n,pattern:r.pattern,deficit:r.deficit,bound:r.bound});
 }
 return{checked,uniqueArithmeticCases:cached.size,integerFailures,fractionalFailures,witnesses,
  scope:'Pattern 13 across 1..128 limbs, plus all 32 patterns at 1/2/3/13/40/90; two normalisation input forms. Only naive/recursive up to 128, public multiply/scratch/normalised up to 40 and square/normalised up to 90 have the reconstructed triangular formula. Full GMP numerical corpus is broader; this is not proof of all assembly or recursive kernels.'};
}
if(process.argv.includes('--high-product-bigint-summary'))console.log(JSON.stringify(checkHighProductBigint()));
