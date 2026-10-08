// Independent BigInt reconstruction; no GMP, FLINT, MPFR or child processes.
// Full value equality is checked by the C GMP oracle. This separately checks
// all decision counts and compact residue/sign/exponent/exactness fingerprints.
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const pairs=[[1,1],[1,2],[2,2],[2,3],[3,7],[19,20],[20,20],[20,21],[25,25],
 [25,26],[26,26],[26,27],[40,41],[499,500],[500,500],[500,501],[1000,1],[1001,1]];
const names=['set','set-inplace','neg','neg-inplace','mul','mul-swapped','mul-inplace','mpfr-mul','square','mpfr-square'];
const mask=(1n<<64n)-1n,offset=14695981039346656037n,prime=1099511628211n;
const abs=x=>x<0n?-x:x;
const bits=x=>x===0n?0:abs(x).toString(2).length;
function pattern(n,p,side){
 const b=BigInt(64*n),pow=k=>1n<<k;
 if(p===0)return 0n;
 if(p===1)return pow(b-1n);
 if(p===2)return pow(b)-1n;
 if(p===3)return pow(b-1n)+1n;
 if(p===4||p===5)return BigInt(p===4?5:7)<<(b-3n);
 if(p===6)return pow(b)-3n;
 if(p===7)return pow(b-1n)+pow(b/2n)+pow(b/2n-2n);
 let a=0n;
 for(let i=0;i<n;i++){
  const w=((0x9e3779b97f4a7c15n*BigInt(i+1+17*side))&mask)^((0xd1b54a32d192ed03n*BigInt(p+1))&mask);
  a=(a<<64n)+w;
 }
 a|=pow(b-1n)|1n;
 if(p===9)a&=~pow(b-1n);
 return a;
}
function round(value,exponent,precision,mode){
 const sign=value<0n,mag=abs(value),drop=precision===null?0:Math.max(0,bits(value)-precision);
 const d=BigInt(drop),unit=1n<<d;
 let q=mag/unit;const r=mag%unit,inexact=r!==0n;
 const tie=inexact&&mode===4&&2n*r===unit,odd=(q&1n)!==0n;
 const up=inexact&&(mode===1||(mode===2&&sign)||(mode===3&&!sign)||
  (mode===4&&(2n*r>unit||(tie&&odd))));
 if(up)q++;
 const carry=up&&bits(q)>precision;
 let e=exponent+drop;
 if(q===0n)e=0;
 else {
  // Removing zeros in 64-bit chunks avoids a per-bit scan for sparse patterns.
  while((q&mask)===0n){q>>=64n;e+=64;}
  while((q&1n)===0n){q>>=1n;e++;}
 }
 return {q:sign?-q:q,e,inexact:Number(inexact),tieEven:Number(tie&&!odd),tieOdd:Number(tie&&odd),carry:Number(carry)};
}
function mix(h,value){
 let v=BigInt.asUintN(64,BigInt(value));
 for(let i=0;i<8;i++){h=((h^(v&255n))*prime)&mask;v>>=8n;}
 return h;
}
const mod=(x,p)=>((x%p)+p)%p;
function add(s,r){
 s.count++;s.exact+=1-r.inexact;s.tieEven+=r.tieEven;s.tieOdd+=r.tieOdd;s.carry+=r.carry;
 for(const v of [r.e,r.q===0n?1:r.q<0n?0:2,bits(r.q),mod(r.q,4294967291n),mod(r.q,4294967279n),r.inexact])s.trace=mix(s.trace,v);
}
const fresh=()=>({count:0,exact:0,tieEven:0,tieOdd:0,carry:0,failures:0,trace:offset});
const serial=s=>({...s,trace:s.trace.toString(16).padStart(16,'0')});
export function checkArfRoundingBigInt(){
 const rows=readFileSync('results/arf-rounding-native.stdout','utf8').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,7211);
 let index=0;const totals=names.map(()=>({...fresh(),trace:0n}));
 for(let j=0;j<pairs.length;j++)for(let p=0;p<10;p++)for(let signs=0;signs<4;signs++){
  const a=pattern(pairs[j][0],p,0)*(signs&1?-1n:1n),b=pattern(pairs[j][1],p,1)*(signs&2?-1n:1n);
  const ae=(p%3-1)*257,be=((p+1)%3-1)*131,size=(pairs[j][0]+pairs[j][1])*64,cut=Math.floor(size*4/5);
  const precs=[1,2,3,53,63,64,65,127,128,129,cut-1,cut,cut+1,size-1,size,size+1,null];
  const quantities=[[a,ae],[-a,ae],[a*b,ae+be],[a*a,2*ae]],stats=quantities.map(fresh);
  for(const precision of precs)for(let mode=0;mode<5;mode++)
   for(let k=0;k<4;k++)add(stats[k],round(...quantities[k],precision,mode));
  for(let op=0;op<10;op++){
   const s=stats[op<2?0:op<4?1:op<8?2:3];
   assert.deepEqual(rows[index++],{kind:'group',pair:j,pattern:p,sign:signs,op:names[op],...serial(s)});
   for(const key of ['count','exact','tieEven','tieOdd','carry','failures'])totals[op][key]+=s[key];
  }
 }
 for(let op=0;op<10;op++)assert.deepEqual(rows[index++],{kind:'total',pair:-1,pattern:-1,sign:-1,op:names[op],...serial(totals[op])});
 const summary={kind:'summary',outputs:612000,inputChecks:2880,inputFailures:0,failures:0};
 assert.deepEqual(rows[index++],summary);assert.equal(index,rows.length);
 return {oracle:'BigInt rounding reconstruction with compact residue fingerprints',groups:7200,outputs:612000,
  independentArithmeticRounds:244800,summary,totals:totals.map((s,i)=>({op:names[i],...serial(s)})),
  limits:'All group membership, order, decision counts and fingerprints checked. Fingerprints are not collision-free certificates; full decoded values and flags are compared by the separate GMP oracle. This does not independently reimplement GMP multiplication or prove every donor branch.'};
}
if(process.argv.includes('--arf-rounding-bigint-summary'))console.log(JSON.stringify(checkArfRoundingBigInt()));
