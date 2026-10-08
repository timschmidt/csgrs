import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {checkHighProductBigint} from './high-product-bigint-oracle.mjs';
const read=p=>readFileSync(p,'utf8');
export function checkHighProductControls(){
 const s=read('results/high-product-native.stdout');
 assert.equal(s,read('results/high-product-memcheck.stdout'));assert.equal(read('results/high-product-native.stderr'),'');
 const all=s.trimEnd().split('\n').map(JSON.parse),header=all.shift(),summary=all.pop();
 assert.deepEqual(header,{configuration:true,limb_bits:64,lengths:144,patterns:32,mul_table:13,sqr_table:8,
  mulders_mul:40,mulders_sqr:90,full_mul:2000,full_sqr:2000,fft_small:0});
 const lengths=[...Array.from({length:128},(_,i)=>i+1),129,255,256,257,511,512,513,1023,1024,1025,1999,2000,2001,2047,2048,2049];
 assert.equal(new Set(lengths).size,144);assert.equal(all.length,53248);
 let index=0;const byOperation={},failed=[],fractionalOnly=[];
 for(const n of lengths)for(let p=0;p<32;p++)for(const norm of [0,1]){
  const ops=['mul','square','mul-scratch',...(n<=128?['mul-naive','mul-recursive']:[]),...(norm?['mul-normalised','square-normalised']:[])];
  for(const op of ops){
   const r=all[index++];assert.deepEqual([r.op,r.n,r.pattern,r.normalised_input],[op,n,p,norm]);
   assert([0,1].includes(r.shift));if(!op.endsWith('normalised'))assert.equal(r.shift,0);
   for(const k of ['exact','require_exact','top_ok','fractional_ok','ok'])assert([0,1].includes(r[k]));
   assert.equal(r.top_ok,1);const deficit=BigInt(r.deficit),scale=1<<r.shift;
   assert.equal(r.deficit,deficit.toString());assert(deficit>=0n);assert.equal(r.exact,Number(deficit===0n));
   assert.equal(r.bound,(n+2)*scale+scale-1);
   const exact=op==='mul-scratch'?n>=2000:['mul','square'].includes(op)?n>2000:['mul-naive','mul-recursive'].includes(op)?n<3:false;
   assert.equal(r.require_exact,Number(exact));if(exact)assert.equal(r.exact,1);
   assert.equal(r.ok,Number(deficit<=BigInt(r.bound)&&(!exact||r.exact)));
   const fullBound=BigInt((n+2)*scale);
   if(deficit<fullBound)assert.equal(r.fractional_ok,1);
   if(deficit>fullBound)assert.equal(r.fractional_ok,0);
   // A deficit strictly below 2n scaled guard ulps also bounds the
   // fractional tail, independently of the C checker's stronger status.
   assert(deficit<BigInt(2*n*scale));
   const g=byOperation[op]??={observations:0,integerFailures:0,fractionalFailures:0,exact:0,requiredExact:0,shifted:0,maxDeficit:0};
   g.observations++;g.integerFailures+=!r.ok;g.fractionalFailures+=!r.fractional_ok;g.exact+=r.exact;
   g.requiredExact+=r.require_exact;g.shifted+=r.shift;g.maxDeficit=Math.max(g.maxDeficit,Number(deficit));
   if(!r.ok)failed.push(r);else if(!r.fractional_ok)fractionalOnly.push(r);
  }
 }
 assert.equal(index,all.length);
 assert.deepEqual(summary,{summary:true,observations:53248,failures:24,fractional_failures:44,exact_outputs:15809,shifted_outputs:4727});
 assert.equal(failed.length,24);assert.equal(fractionalOnly.length,20);
 assert.deepEqual(Object.values(byOperation).map(g=>g.observations),[9216,9216,9216,8192,8192,4608,4608]);
 assert.deepEqual(Object.values(byOperation).map(g=>g.integerFailures),[0,8,0,6,6,0,4]);
 assert.deepEqual(Object.values(byOperation).map(g=>g.fractionalFailures),[2,10,2,12,12,1,5]);
 assert.equal(Object.values(byOperation).reduce((n,g)=>n+g.exact,0),summary.exact_outputs);
 assert.equal(Object.values(byOperation).reduce((n,g)=>n+g.shifted,0),summary.shifted_outputs);
 assert(failed.every(r=>r.pattern===13&&r.shift===0));assert(fractionalOnly.every(r=>r.pattern===13&&r.shift===0));
 const memory=read('results/high-product-memcheck.stderr');
 assert.match(memory,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(memory,/628,419 allocs, 628,419 frees, 891,786,227 bytes allocated/);
 assert.match(memory,/All heap blocks were freed/);assert.match(memory,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 const bigint=checkHighProductBigint();assert.equal(bigint.checked,3059);assert.equal(bigint.uniqueArithmeticCases,1180);
 assert.equal(bigint.integerFailures,24);assert.equal(bigint.fractionalFailures,44);
 assert.deepEqual(bigint,JSON.parse(read('results/high-product-bigint-oracle.stdout')));
 assert.equal(read('results/high-product-bigint-oracle.stderr'),'');
 return{summary,byOperation,inputCases:9216,inputPreservationChecks:18432,fullFallbackExactChecks:832,
  integerPublicSquareFailures:12,integerInternalReferenceFailures:12,fractionalOnly:20,
  noOverestimates:true,allWithinScaled2n:true,bigint,
  memory:{errors:0,liveBytes:0,allocations:628419,frees:628419,cumulativeBytesIncludingOracle:891786227},
  numericalGate:'Native and Memcheck exit 1 because mathematical bounds fail; clean memory diagnostics do not make these successful numerical gates.'};
}
if(process.argv.includes('--high-product-summary'))console.log(JSON.stringify(checkHighProductControls()));
