import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {groups,key,orders,shuffle,plan,validate} from './twelfth-cost-protocol-v78.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=sources(),binary=json('twelfth-cost-binaries-v78.json'),calibration=json('twelfth-native-calibration-v78.json');
assert.deepEqual(binary.source,source);
const origin={checkpoint:78,recorded:new Date().toISOString(),source,scriptSha256:sha('probe-twelfth-native-v78.mjs'),
 previousSummarySha256:sha('twelfth-native-summary-v78.json'),selected:[0,2,34,39,84,93],cpuPairs:12,allocationPairs:4,
 cpuOrders:orders(12,'isolated-cpu'),allocationOrders:orders(4,'isolated-allocation'),
 reason:'Post-selection case-isolated replication of two target forms, two large hot unequal regressions and two failed-proof controls. Same sources and global warmup protocol; one case per fresh process. Conditional intervals are not independent confirmatory significance claims.'};
writeFileSync('twelfth-isolated-origin-v78.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const runs=[];
for(const phase of ['cpu','allocation'])for(const id of origin.selected){
 const ordering=phase==='cpu'?origin.cpuOrders:origin.allocationOrders;
 for(let round=0;round<ordering.length;round++){
  const unshuffled=phase==='cpu'?groups().filter(g=>g.case===id).map(g=>({...g,iterations:calibration.iterations[key(g)]})):[1,16].flatMap(n=>groups(n).filter(g=>g.case===id));
  const g=shuffle(unshuffled,'isolated-'+phase+'-'+id+'-'+round);
  for(const variant of ordering[round]){
   const tag='twelfth-isolated-'+phase+'-'+id+'-'+round+'-'+variant+'-v78',path=tag+'-plan.json';plan(path,g);
   const b=binary.binaries.find(b=>b.variant===variant&&b.mode===phase);assert.equal(sha(b.path),b.sha256);
   await captured(tag,'.','taskset',['-c','2',b.path,phase,resolve('twelfth-cost-input-v78.json'),resolve(path)]);
   validate('results/'+tag+'.stdout',g,variant,phase);runs.push({phase,case:id,round,variant,tag,plan:path,planSha256:sha(path),groups:g.length});
  }
 }
 console.log(JSON.stringify({checkpoint:78,phase,case:id,status:'isolated-case-finished'}));
}
assert.deepEqual(sources(),source);
writeFileSync('twelfth-isolated-runs-v78.json',JSON.stringify({checkpoint:78,originSha256:sha('twelfth-isolated-origin-v78.json'),runs,
 started:origin.recorded,finished:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:78,status:'isolated-replication-collected',processes:runs.length,retained:false}));
