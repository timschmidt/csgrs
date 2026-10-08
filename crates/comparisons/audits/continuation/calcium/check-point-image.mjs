// Reuse the unchanged checkpoint-52 mathematical oracle, then separately check
// the intended baseline-to-candidate completeness delta. No failed gate is edited.
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {checkPowerSumsPublic} from './check-power-sums-public.mjs';
const rows=p=>readFileSync(p,'utf8').trim().split('\n').map(s=>JSON.parse(s));
export function checkPointImage(baseline,candidate){
 const checked=checkPowerSumsPublic(candidate,candidate);assert.equal(checked.status,'pass');assert.deepEqual(checked.failures,[]);
 const a=rows(baseline),b=rows(candidate);assert.equal(a.length,6441);assert.equal(b.length,6441);
 let improved=0,unchanged=0;
 for(let i=0;i<a.length;i++){
  const before=a[i],after=b[i];
  if(before.report?.status==='InvalidTransformedEvidence'){
   assert.equal(before.report.message,'a collapsed refinement interval requires an exact witness');
   assert.equal(after.report.status,'Transformed');assert.equal(after.report.message,null);
   assert.deepEqual({...after,report:before.report},before);
   assert.equal(after.report.root.lower,after.report.root.upper);
   assert.equal(after.report.root.exact,after.report.root.lower);
   assert.equal(after.report.root.constraint,before.left.constraint);
   assert.equal(after.report.root.symbol,before.left.symbol);
   assert.equal(after.report.root.intervalIndex,before.left.intervalIndex);improved++;
  }else{assert.deepEqual(after,before);unchanged++;}
 }
 assert.equal(improved,825);assert.equal(unchanged,5616);
 return{status:'pass',improved,unchanged,mathematical:checked,
  limits:'The unchanged oracle independently certifies each candidate output. Its paired-record check is self-paired here, not baseline equality; the separate full-record comparison above qualifies exactly 825 improvements and no other changes. Original checkpoint-52 failed gate remains preserved.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 console.log(JSON.stringify(checkPointImage(process.argv[2],process.argv[3])));
}
