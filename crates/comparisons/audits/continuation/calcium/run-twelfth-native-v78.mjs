import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {config,groups,key,orders,shuffle,plan,validate} from './twelfth-cost-protocol-v78.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=sources(),binary=json('twelfth-cost-binaries-v78.json');assert.deepEqual(binary.source,source);
for(const b of binary.binaries)assert.equal(sha(b.path),b.sha256,b.path);
const origin={checkpoint:78,recorded:new Date().toISOString(),source,binariesSha256:sha('twelfth-cost-binaries-v78.json'),config,
 scriptSha256:sha('run-twelfth-native-v78.mjs'),environmentSha256:sha('twelfth-cost-environment-v78.mjs'),
 cpuOrders:orders(config.cpuPairs,'cpu-pairs'),allocationOrders:orders(config.allocationPairs,'allocation-pairs')};
writeFileSync('twelfth-native-origin-v78.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const runs=[];
async function run(phase,round,variant,mode,g){
 const tag='twelfth-native-'+phase+'-'+round+'-'+variant+'-v78',path=tag+'-plan.json';plan(path,g);
 const b=binary.binaries.find(b=>b.variant===variant&&b.mode===(mode==='allocation'?'allocation':'cpu'));
 await captured(tag,'.','taskset',['-c',String(config.cpu),b.path,mode,resolve('twelfth-cost-input-v78.json'),resolve(path)]);
 const observed=validate('results/'+tag+'.stdout',g,variant,mode);
 const record={phase,round,variant,mode,tag,plan:path,planSha256:sha(path),groups:g.length};runs.push(record);
 writeFileSync('twelfth-native-journal-v78.json',JSON.stringify({checkpoint:78,runs},null,2)+'\n');
 return observed;
}
await captured('twelfth-native-environment-before-v78','.','node',['twelfth-cost-environment-v78.mjs']);
const pilots=[];
for(let round=0;round<config.pilotIterations.length;round++){
 const g=shuffle(groups(config.pilotIterations[round]),'pilot-'+round);
 for(const variant of round%2?['candidate','baseline']:['baseline','candidate'])pilots.push({variant,round,rows:await run('pilot',round,variant,'cpu',g)});
}
const iterations={};
for(const g of groups()){
 const rates=pilots.map(p=>{const row=p.rows.find(r=>key(r)===key(g));assert(row);return row.elapsed_ns/row.iterations;});
 iterations[key(g)]=Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/Math.max(...rates))));
}
writeFileSync('twelfth-native-calibration-v78.json',JSON.stringify({checkpoint:78,iterations,pilotTags:runs.map(r=>r.tag),config},null,2)+'\n',{flag:'wx'});
for(let round=0;round<config.cpuPairs;round++){
 const g=shuffle(groups().map(g=>({...g,iterations:iterations[key(g)]})),'cpu-'+round);
 for(const variant of origin.cpuOrders[round])await run('cpu',round,variant,'cpu',g);
 console.log(JSON.stringify({checkpoint:78,phase:'cpu',completedPair:round+1,totalPairs:config.cpuPairs}));
}
await captured('twelfth-native-environment-between-v78','.','node',['twelfth-cost-environment-v78.mjs']);
for(let round=0;round<config.allocationPairs;round++){
 const g=shuffle(config.allocationIterations.flatMap(n=>groups(n)),'allocation-'+round);
 for(const variant of origin.allocationOrders[round])await run('allocation',round,variant,'allocation',g);
}
await captured('twelfth-native-environment-after-v78','.','node',['twelfth-cost-environment-v78.mjs']);
assert.deepEqual(sources(),source);for(const b of binary.binaries)assert.equal(sha(b.path),b.sha256);
const result={checkpoint:78,started:origin.recorded,finished:new Date().toISOString(),originSha256:sha('twelfth-native-origin-v78.json'),runs,
 note:'Collection only. Independent membership, source, timing chronology, allocation, calibration and statistical reanalysis must pass before interpreting results.'};
writeFileSync('twelfth-native-runs-v78.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:78,status:'native-collection-finished',groups:576,cpuPairs:24,allocationPairs:4,runs:runs.length,retained:false}));
