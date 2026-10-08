// Post-campaign attribution control, not replacement data or CPU evidence.
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
import {groups,plan,validateRows,variantOrders,shuffle} from './zero-factor-cost-protocol-v71.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=costSources(),prior=json('results/zero-factor-native-check-v71.json');assert.equal(prior.code,0);assert.equal(prior.signal,null);
const binaries=json('zero-factor-cost-binaries-v71.json'),selected=[10,11,36,38,39,107,111],orders=variantOrders(4,'allocation-attribution');
const origin={checkpoint:71,recorded:new Date().toISOString(),source,selected,orders,pairsPerCase:4,
 scriptSha256:sha('probe-zero-factor-allocation-v71.mjs'),priorSummarySha256:sha('zero-factor-native-summary-v71.json'),
 scope:'Fresh process for one selected case at a time, both policies/lifecycles and one/sixteen iterations. Does not remove any main-campaign row, establish general cache attribution, or yield CPU evidence.'};
writeFileSync('zero-factor-allocation-probe-origin-v71.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const runs=[];
for(const which of selected)for(let round=0;round<4;round++){
 const rows=shuffle([1,16].flatMap(n=>groups(n).filter(g=>g.case===which)),'allocation-attribution-'+which+'-'+round),
  path='zero-factor-allocation-probe-'+which+'-'+round+'-plan-v71.json';plan(path,rows);
 for(const variant of orders[round]){
  const a=binaries.binaries.find(a=>a.variant===variant&&a.mode==='allocation');assert.equal(sha(a.path),a.sha256);
  const tag='zero-factor-allocation-probe-'+which+'-'+round+'-'+variant+'-v71';
  await captured(tag,'.','taskset',['-c','2',a.path,'allocation',resolve('zero-factor-cost-input-v71.json'),resolve(path)]);
  const checked=validateRows('results/'+tag+'.stdout',rows,variant,'allocation');
  runs.push({case:which,round,variant,tag,plan:path,planSha256:sha(path),rows:checked});
 }
}
assert.deepEqual(costSources(),source);assert.equal(origin.scriptSha256,sha('probe-zero-factor-allocation-v71.mjs'));
writeFileSync('zero-factor-allocation-probe-v71.json',JSON.stringify({checkpoint:71,originSha256:sha('zero-factor-allocation-probe-origin-v71.json'),
 runs,finished:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:71,status:'allocation-attribution-collected',processes:56,records:448,scope:origin.scope}));
