// Independent BigInt expression/rounding reconstruction. No native backend.
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const lengths=[1,2,3,19,20,21,111,112,113,249,250,251],signsList=[0,1,6,15];
const names=['complex-re','complex-im','fallback-re','fallback-im','complex-left-re','complex-left-im',
 'complex-right-re','complex-right-im','square-re','square-im','square-inplace-re','square-inplace-im',
 'fma','fma-x','fma-initial','addmul','submul','sosq','add','sub','sum','sum-reversed','dot','dot-initial-sub'];
const map=[0,1,0,1,0,1,0,1,2,3,2,3,4,4,4,4,5,6,7,8,4,4,0,9];
const mask=(1n<<64n)-1n,offset=14695981039346656037n,prime=1099511628211n;
const abs=x=>x<0n?-x:x,bits=x=>x===0n?0:abs(x).toString(2).length;
function fixture(n,f,signs){
 const t=(1n<<BigInt(64*n-1))+(1n<<BigInt(32*n))+3n;
 const v=[t+1n,t,t-1n,t];
 if(f===1){v[2]=-t;v[3]=t-1n;}
 if(f===2){v[2]=t;v[3]=t+1n;}
 if(f===3){v[0]=0n;v[3]=0n;}
 if((f>=4&&f<=6)||f>=10){const gap=f===4?63:f===5?64:f===6?65:f===10?129:257;v[1]<<=BigInt(gap);v[2]<<=BigInt(gap);}
 if(f===7||f===8){const s=Math.max(1,n-(f===7?3:2));v[1]=(1n<<BigInt(64*s-1))+5n;v[3]=v[1]+2n;}
 if(f===9)v.fill(0n);
 return v.map((x,i)=>signs&(1<<i)?-x:x);
}
function rounded(value,exponent,precision,mode){
 const sign=value<0n,mag=abs(value),drop=precision===null?0:Math.max(0,bits(value)-precision),unit=1n<<BigInt(drop);
 let q=mag/unit;const r=mag%unit,inexact=r!==0n,tie=inexact&&mode===4&&2n*r===unit,odd=(q&1n)!==0n;
 const up=inexact&&(mode===1||(mode===2&&sign)||(mode===3&&!sign)||(mode===4&&(2n*r>unit||(tie&&odd))));
 if(up)q++;
 const carry=up&&bits(q)>precision;let e=exponent+drop;
 if(q===0n)e=0;
 else {while((q&mask)===0n){q>>=64n;e+=64;}while((q&1n)===0n){q>>=1n;e++;}}
 return {q:sign?-q:q,e,inexact:Number(inexact),tieEven:Number(tie&&!odd),tieOdd:Number(tie&&odd),carry:Number(carry)};
}
function mix(h,value){let v=BigInt.asUintN(64,BigInt(value));for(let i=0;i<8;i++){h=((h^(v&255n))*prime)&mask;v>>=8n;}return h;}
const mod=(x,p)=>((x%p)+p)%p;
function add(s,r){
 s.count++;s.exact+=1-r.inexact;s.tieEven+=r.tieEven;s.tieOdd+=r.tieOdd;s.carry+=r.carry;
 for(const v of [r.e,r.q===0n?1:r.q<0n?0:2,bits(r.q),mod(r.q,4294967291n),mod(r.q,4294967279n),r.inexact])s.trace=mix(s.trace,v);
}
const fresh=()=>({count:0,exact:0,tieEven:0,tieOdd:0,carry:0,failures:0,trace:offset});
const serial=s=>({...s,trace:s.trace.toString(16).padStart(16,'0')});
export function checkArfFusedBigInt(){
 const rows=readFileSync('results/arf-fused-native.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,13849);let index=0,zeroExpressions=0,unitExpressions=0;
 const totals=names.map(()=>({...fresh(),trace:0n}));
 for(const n of lengths)for(let family=0;family<12;family++)for(const signs of signsList){
  const [a,b,c,d]=fixture(n,family,signs),ab=a*b,initial=(family%2===0?-ab:ab)+BigInt(family%3+1);
  const unitExp=(family%3-1)*193,prodExp=unitExp*2;
  const values=[a*c-b*d,a*d+b*c,a*a-b*b,2n*ab,initial+ab,initial-ab,a*a+b*b,a+b,a-b,initial-(a*c-b*d)];
  zeroExpressions+=values.filter(x=>x===0n).length;unitExpressions+=values.filter(x=>abs(x)===1n).length;
  const stats=values.map(fresh),precs=[1,2,3,53,63,64,65,127,128,129,2*n*64,null];
  for(const p of precs)for(let mode=0;mode<5;mode++)for(let k=0;k<10;k++)add(stats[k],rounded(values[k],k===7||k===8?unitExp:prodExp,p,mode));
  for(let op=0;op<24;op++){
   const s=stats[map[op]];
   assert.deepEqual(rows[index++],{kind:'group',n,family,sign:signs,op:names[op],...serial(s)});
   for(const key of ['count','exact','tieEven','tieOdd','carry','failures'])totals[op][key]+=s[key];
  }
 }
 for(let op=0;op<24;op++)assert.deepEqual(rows[index++],{kind:'total',n:-1,family:-1,sign:-1,op:names[op],...serial(totals[op])});
 const summary={kind:'summary',outputs:829440,inputChecks:19584,inputFailures:0,flagFailures:0,failures:0};
 assert.deepEqual(rows[index++],summary);assert.equal(index,rows.length);
 return {oracle:'BigInt complete-expression rounding with compact output fingerprints',fixtures:576,groups:13824,
  independentArithmeticRounds:345600,zeroExpressions,unitExpressions,summary,totals:totals.map((s,i)=>({op:names[i],...serial(s)})),
  limits:'All membership, order, rounding decision counts and compact fingerprints checked; these are not collision-free value certificates. The separate GMP oracle compares every full decoded value and exactness flag. Fixtures include repeated signs/quantities/routes and do not prove all algorithms or states.'};
}
if(process.argv.includes('--arf-fused-bigint-summary'))console.log(JSON.stringify(checkArfFusedBigInt()));
