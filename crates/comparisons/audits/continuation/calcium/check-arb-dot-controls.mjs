import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
export function checkArbDotControls(){
 const read=p=>readFileSync('results/arb-dot-'+p,'utf8');
 const native=read('native.stdout');assert.equal(native,read('memcheck.stdout'));assert.equal(read('native.stderr'),'');
 const rows=native.trimEnd().split('\n').map(l=>JSON.parse(l));assert.equal(rows.length,1225);
 const summary=rows.pop();assert.deepEqual(summary,{kind:'summary',fixtures:648,calls:198288,
  endpointComparisons:396576,failures:0,inputChecks:21349,inputFailures:0});
 const widths=[1,2,3,11,12,13,24,25,26,331,332,333],names=['dot','simple','precise','ui','si','uiui','siui','fmpz'];
 let row=0,calls=0;const routes=Object.fromEntries(names.map(n=>[n,{groups:0,calls:0,zeroRadius:0}]));
 for(let type=0;type<6;type++)for(const [wi,width]of widths.entries()){
  if(type&&![0,2,8].includes(wi))continue;
  for(let family=0;family<6;family++)for(const length of [0,1,2,5])
  for(let op=type?type+2:0;op<(type?type+3:3);op++){
   const r=rows[row++];assert.equal(r.kind,'group');assert.equal(r.type,type);assert.equal(r.width,width);
   assert.equal(r.family,family);assert.equal(r.length,length);assert.equal(r.op,names[op]);
   assert.equal(r.calls,162);assert.equal(r.failed,0);assert(Number.isInteger(r.exact)&&r.exact>=0&&r.exact<=r.calls);
   assert.match(r.trace,/^[0-9a-f]{16}$/);calls+=r.calls;
   const s=routes[r.op];s.groups++;s.calls+=r.calls;s.zeroRadius+=r.exact;
  }
 }
 assert.equal(row,1224);assert.equal(calls,summary.calls);
 const m=read('memcheck.stderr');assert.match(m,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(m,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);assert.match(m,/All heap blocks were freed/);
 const match=m.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);assert(match);
 const [allocations,frees,cumulativeBytes]=match.slice(1).map(s=>Number(s.replaceAll(',','')));assert.equal(allocations,frees);
 for(const tag of ['native','memcheck']){const g=JSON.parse(read(tag+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);}
 return{summary,groups:row,routes,identicalNumericalBytes:Buffer.byteLength(native),
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations,frees,cumulativeBytes},
  limits:'Every decoded midpoint/radius is checked against complete GMP rational rectangle endpoints. GMP is separate from Arb enclosure algorithms but also underlies some FLINT integer operations; no second integer backend or formal proof is claimed. Native/Memcheck trace agreement is supplemental, not the mathematical oracle. Counts include related routes, precisions and aliases. Input checks cover generated Arb arrays, initial and fmpz mirrors, not direct byte-preservation checks of every machine-word coefficient array. No partial overlap, raw invalid shape, negative length, self-dot correlation, nonfinite/huge exponent/radius-only-midpoint branch, direct FMA/add-error, arbitrary thread, 32-bit/ARM/FFT, correct-rounding or tightest-enclosure guarantee. Memory includes setup/oracle costs; no donor-only allocation, peak RSS, timing, binary-size improvement or production transfer.'};
}
if(process.argv.includes('--arb-dot-summary'))console.log(JSON.stringify(checkArbDotControls()));
