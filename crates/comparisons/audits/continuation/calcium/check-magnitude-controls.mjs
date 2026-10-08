import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
export function checkMagnitudeControls(){
 const read=p=>readFileSync('results/magnitude-'+p,'utf8'),native=read('native.stdout');
 assert.equal(native,read('memcheck.stdout'));assert.equal(read('native.stderr'),'');assert.equal(Buffer.byteLength(native),197917);
 const rows=native.trimEnd().split('\n').map(JSON.parse);assert.equal(rows.length,1805);
 const summary=rows.pop();assert.deepEqual(summary,{kind:'summary',boundChecks:505048,boundFailures:0,qualityFailures:0,
  comparisonChecks:14688,comparisonFailures:0,magnitudeInputChecks:31416,magnitudeInputFailures:0,
  ballResults:6480,ballFailures:0,ballInputChecks:1584,ballInputFailures:0});
 const names=['add','add-lower','sub','sub-lower','mul','mul-lower','div','div-lower','fast-mul','sqrt','sqrt-lower',
  'rsqrt','rsqrt-lower','inv','inv-lower','pow-ui','pow-ui-lower','pow-fmpz','pow-fmpz-lower','addmul','fast-addmul',
  'scale-si','scale-fmpz','fast-scale','add-pow2','fast-add-pow2','min','max'];
 const conversions=['fmpz','fmpz-lower','fmpz-scaled','fmpz-scaled-lower','arf','arf-lower','fast-arf',
  'ui','ui-lower','ui-scaled','add-ui','add-ui-lower','mul-ui','mul-ui-lower','div-ui','add-ui-scaled'];
 const balls=['fma','fma-x','fma-y','fma-initial','addmul','error-arf','error-ball','error-mag','error-pow2-si','error-pow2-fmpz'];
 let index=0,arithmeticCalls=0,conversionCalls=0,ballCalls=0;
 const row=(kind,a,b,op,calls)=>{
  const r=rows[index++];assert.equal(r.kind,kind);assert.equal(r.a,a);assert.equal(r.b,b);assert.equal(r.op,op);
  assert.equal(r.calls,calls);assert.equal(r.failed,0);assert(Number.isInteger(r.exact)&&r.exact>=0&&r.exact<=calls);
  assert.match(r.trace,/^[0-9a-f]{16}$/);return r;
 };
 for(let base=0;base<3;base++)for(let gi=0;gi<17;gi++)for(let op=0;op<28;op++){
  const signed=[-7,-1,0,1,2,3,8][(gi+base)%7];
  const skip=(op===6||op===7)||(op>=11&&op<=14)||((op===17||op===18)&&signed<0);
  const unary=(op>=9&&op<=18)||(op>=21&&op<=25);
  const count=(skip?132:144)*(unary?2:3);row('arithmetic',base,gi,names[op],count);arithmeticCalls+=count;
 }
 for(let op=0;op<16;op++){const count=op<7?680:op===14?280:320;row('conversion',0,0,conversions[op],count);conversionCalls+=count;}
 for(let wi=0;wi<6;wi++)for(let f=0;f<6;f++)for(let op=0;op<10;op++){
  const count=op<5?32:4;row('ball',wi,f,balls[op],count);ballCalls+=count;
 }
 assert.equal(index,1804);assert.equal(arithmeticCalls,497448);assert.equal(conversionCalls,7600);assert.equal(ballCalls,6480);
 assert.equal(arithmeticCalls+conversionCalls,summary.boundChecks);
 const m=read('memcheck.stderr');assert.match(m,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(m,/9,839 allocs, 9,839 frees, 6,879,352 bytes allocated/);assert.match(m,/All heap blocks were freed/);
 assert.match(m,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 for(const t of ['native','memcheck']){const g=JSON.parse(read(t+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);}
 return{summary,groups:index,arithmeticCalls,conversionCalls,ballCalls,ballEndpointComparisons:2*ballCalls,
  arithmeticInputPairs:7344,conversionFixtures:680,ballFixtures:144,identicalNumericalBytes:197917,
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:9839,frees:9839,cumulativeBytes:6879352},
  limits:'Finite 64-bit sequential inputs with bounded inline magnitude exponents and valid whole-object aliases. Complete GMP rational inequalities are the oracle; square/reciprocal-square roots are checked by exact squaring, not an approximate root. Quality checks impose a supplemental 1/1024 relative envelope (on the squared output for roots), not correct rounding or tightest bounds. GMP also underlies some FLINT integer work; no independent second integer backend or formal proof. Fingerprints supplement, not replace, full comparisons. FMA/error controls reuse the unchanged checkpoint-38 decoder and rectangle oracle, not its main. No promoted/extreme exponents, invalid raw preconditions/partial overlap, arbitrary thread, nonfinite sweep, direct hypot, every integer wrapper, large power, 32-bit/ARM/FFT, all rounding modes/libm environments, whole-stack CI, matched Hyper performance, donor-only allocation or peak-RSS claim.'};
}
if(process.argv.includes('--magnitude-summary'))console.log(JSON.stringify(checkMagnitudeControls()));
