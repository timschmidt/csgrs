import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-revision-sources-v79.mjs';
import {config,groups,key,plan,validate,orders,shuffle} from './twelfth-revision-cost-protocol-v79.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=sources(),binary=json('twelfth-revision-binaries-v79.json');assert.deepEqual(binary.source,source);
for(const b of binary.binaries)assert.equal(sha(b.path),b.sha256);
const origin={checkpoint:79,recorded:new Date().toISOString(),source,config,binariesSha256:sha('twelfth-revision-binaries-v79.json'),
 scripts:Object.fromEntries(['run-twelfth-revision-native-v79.mjs','twelfth-revision-cost-protocol-v79.mjs','twelfth-revision-cost-protocol-v79.md',
  'twelfth-revision-environment-v79.mjs','twelfth-cost-protocol-v78.mjs','paired-statistics-v60.mjs'].map(p=>[p,sha(p)])),
 cpuOrders:orders(config.cpuTriples,'cpu-triples'),allocationOrders:orders(config.allocationTriples,'allocation-triples')};
writeFileSync('twelfth-revision-native-origin-v79.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const runs=[];
async function run(phase,round,variant,mode,g){
 const tag='twelfth-revision-'+phase+'-'+round+'-'+variant+'-v79',path=tag+'-plan.json';plan(path,g);
 const b=binary.binaries.find(b=>b.variant===variant&&b.mode===(mode==='allocation'?'allocation':'cpu'));
 await captured(tag,'.','taskset',['-c',String(config.cpu),b.path,mode,resolve('twelfth-cost-input-v78.json'),resolve(path)]);
 const observed=validate('results/'+tag+'.stdout',g,variant==='baseline'?'baseline':'candidate',mode);
 runs.push({phase,round,variant,mode,tag,plan:path,planSha256:sha(path),groups:g.length});
 writeFileSync('twelfth-revision-native-journal-v79.json',JSON.stringify({checkpoint:79,runs},null,2)+'\n');return observed;
}
await captured('twelfth-revision-environment-before-v79','.','node',['twelfth-revision-environment-v79.mjs']);
const pilots=[];
for(let round=0;round<config.pilotIterations.length;round++){
 const g=shuffle(groups(config.pilotIterations[round]),'pilot-'+round);
 for(const variant of round===0?['baseline','prior','candidate']:['candidate','prior','baseline'])pilots.push({variant,round,rows:await run('pilot',round,variant,'cpu',g)});
}
const iterations={};
for(const g of groups()){
 const rates=pilots.map(p=>{const r=p.rows.find(r=>key(r)===key(g));assert(r);return r.elapsed_ns/r.iterations;});
 iterations[key(g)]=Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/Math.max(...rates))));
}
writeFileSync('twelfth-revision-native-calibration-v79.json',JSON.stringify({checkpoint:79,config,iterations,pilotTags:runs.map(r=>r.tag)},null,2)+'\n',{flag:'wx'});
for(let round=0;round<config.cpuTriples;round++){
 const g=shuffle(groups().map(g=>({...g,iterations:iterations[key(g)]})),'cpu-'+round);
 for(const variant of origin.cpuOrders[round])await run('cpu',round,variant,'cpu',g);
 console.log(JSON.stringify({checkpoint:79,completedCpuTriple:round+1,total:config.cpuTriples}));
}
await captured('twelfth-revision-environment-between-v79','.','node',['twelfth-revision-environment-v79.mjs']);
for(let round=0;round<config.allocationTriples;round++){
 const g=shuffle(config.allocationIterations.flatMap(n=>groups(n)),'allocation-'+round);
 for(const variant of origin.allocationOrders[round])await run('allocation',round,variant,'allocation',g);
}
await captured('twelfth-revision-environment-after-v79','.','node',['twelfth-revision-environment-v79.mjs']);
sources();for(const b of binary.binaries)assert.equal(sha(b.path),b.sha256);
writeFileSync('twelfth-revision-native-runs-v79.json',JSON.stringify({checkpoint:79,started:origin.recorded,finished:new Date().toISOString(),
 originSha256:sha('twelfth-revision-native-origin-v79.json'),runs},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:79,status:'native-collection-complete',runs:runs.length,groups:576,rawRows:65664,retained:false}));
