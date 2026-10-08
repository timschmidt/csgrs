import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {checkArfRoundingBigInt} from './arf-rounding-bigint-oracle.mjs';
export function checkArfRoundingControls(){
 const read=p=>readFileSync('results/arf-rounding-'+p,'utf8');
 const native=read('native.stdout'),memory=read('memcheck.stdout');
 assert.equal(native,memory);assert.equal(read('native.stderr'),'');
 assert.equal(Buffer.byteLength(native),1142532);
 const m=read('memcheck.stderr');
 assert.match(m,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(m,/303,090 allocs, 303,090 frees, 976,772,600 bytes allocated/);
 assert.match(m,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 assert.match(m,/All heap blocks were freed/);
 const oracle=checkArfRoundingBigInt();
 assert.deepEqual(JSON.parse(read('bigint-oracle.stdout')),oracle);assert.equal(read('bigint-oracle.stderr'),'');
 for(const t of ['native','memcheck','bigint-oracle']){
  const g=JSON.parse(read(t+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);
 }
 return {summary:oracle.summary,oracle,identicalNumericalBytes:1142532,
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:303090,frees:303090,cumulativeBytes:976772600},
  limits:'Finite sequential corpus with an existing 64-bit ADX build. No invalid raw aliases, huge exponents, unsupported precisions, arbitrary thread histories, ARM/32-bit/FFT or whole-ARF qualification. Native and Memcheck costs include the oracle and destination seeding; no performance or peak-RSS claim.'};
}
if(process.argv.includes('--arf-rounding-summary'))console.log(JSON.stringify(checkArfRoundingControls()));
