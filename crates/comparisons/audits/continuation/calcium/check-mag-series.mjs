import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const abs=x=>x<0n?-x:x;
function gcd(a,b){a=abs(a);b=abs(b);while(b){const r=a%b;a=b;b=r;}return a;}
function rat(n,d=1n){const g=gcd(n,d);return[n/g,d/g];}
const cmp=([a,b],[c,d])=>a*d-c*b;
const sub=([a,b],[c,d])=>rat(a*d-c*b,b*d);
function pow([a,b],n){return[a**BigInt(n),b**BigInt(n)];}
function mag(r){
 assert.equal(r.length,3);const[m,e,inf]=r;
 if(inf){assert.deepEqual(r,[0,'-4611686018427387902',1]);return null;}
 assert.equal(inf,0);assert(Number.isSafeInteger(m)&&Number.isSafeInteger(e)&&e>=-100000&&e<=100000);
 if(m===0){assert.equal(e,0);return[0n,1n];}
 assert(m>=2**29&&m<2**30);const shift=e-30;
 return shift>=0?[BigInt(m)<<BigInt(shift),1n]:[BigInt(m),1n<<BigInt(-shift)];
}
function doubleQ(bits){
 const b=BigInt('0x'+bits),e=Number((b>>52n)&2047n),m=b&((1n<<52n)-1n);
 assert(e!==2047);const shift=e?e-1075:-1074,n=e?m+(1n<<52n):m;
 return shift>=0?[n<<BigInt(shift),1n]:[n,1n<<BigInt(-shift)];
}
export function checkMagSeries(){
 const read=p=>readFileSync('results/mag-series-'+p,'utf8'),native=read('native.stdout');
 assert.equal(native,read('memcheck.stdout'));assert.equal(read('native.stderr'),'');assert.equal(Buffer.byteLength(native),1308580);
 // Preserve the exact special exponent rather than rounding it as a JS Number.
 assert.equal(native.split('-4611686018427387902').length-1,796);
 const rows=native.replaceAll('-4611686018427387902','"-4611686018427387902"').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,7430);const summary=rows.pop();
 assert.deepEqual(summary,{kind:'summary',exactChecks:48890,directedChecks:3358,inputChecks:1441,aliasChecks:1240,
  roundtrips:402,infinitePolylogOutputs:796,failures:0});
 let index=0,exact=0,bounds=0,directed=0,inputs=0,aliases=0,infinite=0,binomials=0;
 const bound=(out,q,lower=false)=>{const value=mag(out);assert(value);assert(lower?cmp(value,q)<=0n:cmp(value,q)>=0n);exact++;bounds++;};
 const next=kind=>{const r=rows[index++];assert.equal(r.kind,kind);return r;};
 let factorial=1n;
 for(let n=0;n<=4096;n++){
  if(n)factorial*=BigInt(n);const r=next('fac');assert.equal(r.n,n);assert.equal(r.results.length,2);
  bound(r.results[0],[factorial,1n]);bound(r.results[1],[1n,factorial]);
 }
 for(const n of [...Array(258).keys(),511,512,513,1024]){
  const r=next('binomial');assert.equal(r.n,n);assert.equal(r.results.length,n+2);let b=1n;
  for(let k=0;k<=n+1;k++){
   if(k>n)b=0n;else if(k){b*=BigInt(n-k+1);assert.equal(b%BigInt(k),0n);b/=BigInt(k);}
   bound(r.results[k],[b,1n]);binomials++;
  }
 }
 const ms=[1,2,3,7,31,32,33,255,256,257,1024,1073741823,1073741824,1073741825];
 const ns=[0,1,2,3,7,31,32,33,127,255,256,257,512];
 for(const m of ms)for(const n of ns){const r=next('binpow');assert.equal(r.m,m);assert.equal(r.n,n);bound(r.result,[BigInt(m+1)**BigInt(n),BigInt(m)**BigInt(n)]);}
 // A separate defining recurrence: sum_(k=0)^n C(n+1,k) B_k=0
 // for n>=1. It uses the negative B1 convention, then absolute values.
 const bern=[[1n,1n]];factorial=1n;
 for(let n=0;n<=128;n++){
  if(n){factorial*=BigInt(n);let sum=[0n,1n],choose=1n;
   for(let k=0;k<n;k++){
    const[a,b]=bern[k];sum=rat(sum[0]*b+a*choose*sum[1],sum[1]*b);
    choose=choose*BigInt(n+1-k)/BigInt(k+1);
   }
   bern.push(rat(-sum[0],sum[1]*BigInt(n+1)));
  }
  const r=next('bernoulli');assert.equal(r.n,n);bound(r.result,[abs(bern[n][0]),bern[n][1]*factorial]);
 }
 assert.deepEqual(bern[1],[-1n,2n]);assert.deepEqual(bern[2],[1n,6n]);assert.deepEqual(bern[4],[-1n,30n]);
 const gm=[0,536870912,536870912,536870912,805306368,939524096,1073741822,1073741823],ge=[0,-59,-3,0,0,0,0,0];
 for(let i=0;i<8;i++)for(const n of [0,1,2,3,7,31,64,127,255,256,257]){
  const r=next('geom');assert.equal(r.i,i);assert.equal(r.n,n);assert.equal(r.results.length,2);
  const q=mag([gm[i],ge[i],0]),den=sub([1n,1n],q),[a,b]=pow(q,n),target=[a*den[1],b*den[0]];
  r.results.forEach(x=>bound(x,target));assert.deepEqual(r.results[0],r.results[1]);inputs++;aliases++;
 }
 const bitInputs=[];
 for(const b of [0n,1n,2n,3n,0xffffffffffffen,0xfffffffffffffn])for(const s of [0n,1n])bitInputs.push(b|(s<<63n));
 for(const e of [-1022,-1000,-970,-31,-30,-1,0,1,29,30,31,52,53,63,64,999,1000,1023])
  for(const f of [0n,1n,4194303n,4194304n,8388607n,8388608n,0xfffffffffffffn])for(const s of [0n,1n])
   bitInputs.push((BigInt(e+1023)<<52n)|f|(s<<63n));
 assert.equal(bitInputs.length,264);
 for(let id=0;id<264;id++){
  const r=next('double');assert.equal(r.id,id);assert.equal(r.bits,bitInputs[id].toString(16).padStart(16,'0'));assert.equal(r.results.length,5);
  const base=doubleQ(r.bits);
  for(const[si,scale]of [-1024,-31,0,31,1024].entries()){
   const out=r.results[si],target=scale>=0?[base[0]<<BigInt(scale),base[1]]:[base[0],base[1]<<BigInt(-scale)];
   assert.equal(out.length,scale===0?4:2);
   for(let j=0;j<out.length;j++)bound(out[j],target,j%2===1);
   if(scale===0){assert.deepEqual(out[0],out[2]);assert.deepEqual(out[1],out[3]);}
  }
 }
 const mm=[536870912,536870913,536870914,536870915,671088639,671088640,671088641,1073741821,1073741822,1073741823];
 const me=[-1075,-1001,-1000,-999,-31,-30,-29,-1,0,1,29,30,31,63,64,65,999,1000,1001,1024];
 for(let id=0;id<201;id++){
  const r=next('magnitude-conversion');assert.equal(r.id,id);
  const expected=id?[mm[(id-1)%10],me[Math.floor((id-1)/10)],0]:[0,0,0];assert.deepEqual(r.input,expected);assert.deepEqual(r.roundtrip,expected);
  const q=mag(expected),e=expected[1];assert.match(r.double,/^[0-9a-f]{16}$/);
  if(e>1000)assert.equal(r.double,'7ff0000000000000');
  else {const d=doubleQ(r.double);assert(cmp(d,q)>=0n);if(e>=-1000)assert.equal(cmp(d,q),0n);else assert.equal(cmp(d,[1n,1n<<1000n]),0n);}
  exact+=4;inputs++;
 }
 const histories=[];
 for(const p of [512,768]){
  const history=[];let id=0;
  for(let i=0;i<6;i++)for(const sigma of [-3,-1,0,1,3,8])for(const d of [0,1,2,4])for(const n of [2,3,8,31]){
   const r=next('polylog');assert.equal(r.precision,p);assert.equal(r.id,id++);assert.equal(r.i,i);assert.equal(r.sigma,sigma);assert.equal(r.d,d);assert.equal(r.n,n);
   assert.equal(r.results.length,2);assert.deepEqual(r.results[0],r.results[1]);inputs++;aliases++;
   for(const out of r.results){mag(out);directed++;infinite+=out[2];}
   if(i===0)assert.deepEqual(r.results[0],[0,0,0]);
   const{precision,...rest}=r;history.push(rest);
  }
  assert.equal(id,576);
  for(let s=2;s<=32;s++)for(let a=1;a<=17;a++){
   const r=next('hurwitz');assert.equal(r.precision,p);assert.equal(r.s,s);assert.equal(r.a,a);assert(mag(r.result));directed++;
   const{precision,...rest}=r;history.push(rest);
  }
  histories.push(history);
 }
 assert.deepEqual(histories[0],histories[1]);assert.equal(index,7429);
 assert.equal(exact,summary.exactChecks);assert.equal(directed,summary.directedChecks);assert.equal(inputs,summary.inputChecks);
 assert.equal(aliases,summary.aliasChecks);assert.equal(infinite,summary.infinitePolylogOutputs);assert.equal(bounds,48086);
 const mem=read('memcheck.stderr');assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/636,551 allocs, 636,551 frees, 73,880,835 bytes allocated/);assert.match(mem,/All heap blocks were freed/);
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 for(const t of ['native','memcheck']){const g=JSON.parse(read(t+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);}
 return{summary,groups:index,independentBigIntBoundChecks:bounds,independentBigIntDoubleExportChecks:201,
  nativeOnlyExactExportComparisons:603,factorialPairs:4097,binomialInputs:binomials,binpowInputs:182,bernoulliInputs:129,
  geometricInputs:88,doubleInputs:264,magnitudeConversionInputs:201,polylogInputsPerPrecision:576,
  finitePolylogInputsPerPrecision:377,infinitePolylogInputsPerPrecision:199,hurwitzInputsPerPrecision:527,
  equalAcrossPrecisions:true,identicalNumericalBytes:1308580,
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:636551,frees:636551,cumulativeBytes:73880835},
  limits:'Bounded valid public finite inputs under default FE_TONEAREST. GMP exact recurrence comparisons are independently rechecked using JS BigInt for48086 bounds; Bernoulli uses a different defining recurrence. Native603 exact fmpq/ceil/floor comparisons are not separately emitted for JS recomputation;201 double exports and roundtrip values are. Polylog and Hurwitz use512/768-bit directed MPFR, not two independent transcendental backends. Complete eventual-geometric remainder for polylog and zeta-minus-finite-prefix for Hurwitz; finite prefix alone is not the oracle. Conservative infinity is valid but uninformative on199 known-convergent polylog cases per precision. No malformed input, file-failure, promoted/huge exponent conversion, arbitrary parameters/state/overlap/thread, every source path, 32-bit/ARM/all rounding environments, new Rust full suite, matched Hyper performance/size, donor-only allocations or peak-memory claim. The upstream random suite was read, not newly run.'};
}
if(process.argv.includes('--mag-series-summary'))console.log(JSON.stringify(checkMagSeries()));
