import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {checkArfFusedBigInt} from './arf-fused-bigint-oracle.mjs';
export function checkArfFusedControls(){
 const read=p=>readFileSync('results/arf-fused-'+p,'utf8');
 const native=read('native.stdout');assert.equal(native,read('memcheck.stdout'));assert.equal(read('native.stderr'),'');
 assert.equal(Buffer.byteLength(native),2176528);
 const m=read('memcheck.stderr');assert.match(m,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(m,/2,249,297 allocs, 2,249,297 frees, 3,530,423,920 bytes allocated/);
 assert.match(m,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);assert.match(m,/All heap blocks were freed/);
 const oracle=checkArfFusedBigInt();assert.deepEqual(JSON.parse(read('bigint-oracle.stdout')),oracle);assert.equal(read('bigint-oracle.stderr'),'');
 for(const tag of ['native','memcheck','bigint-oracle']){const g=JSON.parse(read(tag+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);}
 return {summary:oracle.summary,oracle,identicalNumericalBytes:2176528,
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:2249297,frees:2249297,cumulativeBytes:3530423920},
  limits:'Valid finite sequential inputs on the reused 64-bit ADX build. All full decoded values and flags are checked against complete GMP expressions followed by rounding; BigInt checks counts/fingerprints, not collision-free certificates. Memory totals include oracles, setup, destination reuse and exact product vectors; not donor-only cost or peak RSS. No approximate-dot, raw invalid overlap/size, extreme exponent, arbitrary thread, all-stride/length, 32-bit/ARM/FFT or whole-stack qualification.'};
}
if(process.argv.includes('--arf-fused-summary'))console.log(JSON.stringify(checkArfFusedControls()));
