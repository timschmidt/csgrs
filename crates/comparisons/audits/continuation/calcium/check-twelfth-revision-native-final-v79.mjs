import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json} from './twelfth-revision-sources-v79.mjs';
import {key} from './twelfth-revision-cost-protocol-v79.mjs';
import {nativeEvidence} from './check-twelfth-revision-native-v79.mjs';

// Preserve the first recomputation, including its raw numerical/statistical
// evidence. Its equalResult flag meant identical complete certificate text,
// which incorrectly grouped unchanged NotEqual outcomes as changed answers.
// The final view separates the mathematical decision from certificate identity.
export function finalNativeEvidence(){
 const old=nativeEvidence();assert.deepEqual(old,json('twelfth-revision-native-summary-v79.json'));
 const cpu=old.cpu.map(r=>({...r,equalCertificate:r.equalResult,equalResult:r.transition[0]===r.transition[1]}));
 const indexed=new Map(cpu.map(r=>[r.reference+':'+key(r),r]));
 const allocations=old.allocations.map(r=>{const c=indexed.get(r.reference+':'+key(r));assert(c);return{...r,equalResult:c.equalResult,equalCertificate:c.equalCertificate};});
 const categories={};for(const r of cpu){
  const k=r.reference+':'+(r.equalResult?'same-decision:':'changed-decision:')+r.family;
  const t=categories[k]??={groups:0,ratios:[],changedCertificates:0,bothIntervalsFaster:0,bothIntervalsSlower:0,otherwise:0,shortMedianBelow100us:0};
  t.groups++;t.ratios.push(r.pairedMedianRatio);if(!r.equalCertificate)t.changedCertificates++;
  if(r.statistics.bootstrap.interval[1]<1&&r.statistics.orderStatistic.interval[1]<1)t.bothIntervalsFaster++;
  else if(r.statistics.bootstrap.interval[0]>1&&r.statistics.orderStatistic.interval[0]>1)t.bothIntervalsSlower++;else t.otherwise++;
  if(r.medianBatchNs<100000)t.shortMedianBelow100us++;
 }
 for(const t of Object.values(categories)){t.ratioRange=[Math.min(...t.ratios),Math.max(...t.ratios)];delete t.ratios;}
 assert.equal(cpu.filter(r=>r.reference==='baseline'&&r.family==='perturbed'&&r.equalResult&&!r.equalCertificate).length,192);
 assert(cpu.filter(r=>r.reference==='prior').every(r=>r.equalCertificate&&r.equalResult));
 return {...old,status:'revision-native-cost-recomputed-decision-and-certificate-separated',cpu,allocations,categories,
  classificationCorrection:'The initial equalResult flag meant identical certificate text. Final equalResult means identical mathematical outcome; equalCertificate separately records the full certificate comparison. All 192 perturbed baseline comparisons retain NotEqual with a changed certificate. Raw samples, ratios and both intervals are unchanged. No prior-candidate certificate changes.',
  limits:old.limits+' Same-decision comparisons need not have identical certificates; certificate identity is reported separately.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const e=finalNativeEvidence();if(process.argv.includes('--record'))writeFileSync('twelfth-revision-native-final-summary-v79.json',JSON.stringify(e,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(e,json('twelfth-revision-native-final-summary-v79.json'));
 console.log(JSON.stringify({checkpoint:79,status:'final-cost-summary-verified',campaign:e.campaign,categories:e.categories,classificationCorrection:e.classificationCorrection,retained:false}));
}
