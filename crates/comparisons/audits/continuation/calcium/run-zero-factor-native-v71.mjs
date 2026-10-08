import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
import {groups,plan,validateRows,variantOrders,shuffle,key,config} from './zero-factor-cost-protocol-v71.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=costSources(),build=json('results/zero-factor-cost-build-v71.json');assert.equal(build.code,0);assert.equal(build.signal,null);
assert.deepEqual(source,json('zero-factor-cost-origin-v71.json').source);
const binary=json('zero-factor-cost-binaries-v71.json'),byKey=new Map(binary.binaries.map(a=>[a.variant+':'+a.mode,a]));
for(const a of binary.binaries)assert.equal(sha(a.path),a.sha256);
assert.equal(readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.+)$/m)?.[1],'0');
const scripts=['run-zero-factor-native-v71.mjs','zero-factor-native-environment-v71.mjs'],origin={checkpoint:71,recorded:new Date().toISOString(),
 source,binariesSha256:sha('zero-factor-cost-binaries-v71.json'),scripts:Object.fromEntries(scripts.map(p=>[p,sha(p)])),config,
 orchestratorCpu:0,cpuOrders:variantOrders(config.cpuPairs,'cpu-order'),allocationOrders:variantOrders(config.allocationPairs,'allocation-order')};
writeFileSync('zero-factor-native-origin-v71.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
await captured('zero-factor-native-environment-before-v71','.','node',['zero-factor-native-environment-v71.mjs']);
const input=resolve('zero-factor-cost-input-v71.json'),runs=[];
async function run(phase,round,variant,mode,rows,planPath){
 const a=byKey.get(variant+':'+mode);assert.equal(sha(a.path),a.sha256);
 const tag='zero-factor-native-'+phase+'-'+round+'-'+variant+'-v71';
 await captured(tag,'.','taskset',['-c',String(config.cpu),a.path,mode,input,resolve(planPath)]);
 const values=validateRows('results/'+tag+'.stdout',rows,variant,mode);
 const g=json('results/'+tag+'.json'),lo=BigInt(Date.parse(g.started))*1000000n,hi=BigInt(Date.parse(g.finished)+1)*1000000n;
 for(const r of values){assert(BigInt(r.wall_before)>=lo);assert(BigInt(r.wall_after)<=hi);}
 runs.push({phase,round,variant,mode,tag,plan:planPath,planSha256:sha(planPath),rows:rows.length,started:g.started,finished:g.finished});
 return values;
}
const pilots={};
for(const iterations of config.pilotIterations){
 const rows=shuffle(groups(iterations),'pilot-'+iterations),path='zero-factor-native-pilot-'+iterations+'-plan-v71.json';plan(path,rows);
 for(const variant of iterations===1?['baseline','candidate']:['candidate','baseline'])
  pilots[variant+':'+iterations]=await run('pilot',iterations,variant,'cpu',rows,path);
}
const p32=Object.fromEntries(['baseline','candidate'].map(v=>[v,new Map(pilots[v+':32'].map(r=>[key(r),r]))]));
const iterationPlan=groups().map(g=>{const baseline=p32.baseline.get(key(g)),candidate=p32.candidate.get(key(g));
 const ns=Math.max(baseline.elapsed_ns/32,candidate.elapsed_ns/32),iterations=Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/ns)));
 return {...g,iterations,baselinePilotNs:baseline.elapsed_ns,candidatePilotNs:candidate.elapsed_ns};});
writeFileSync('zero-factor-native-iterations-v71.json',JSON.stringify(iterationPlan,null,2)+'\n',{flag:'wx'});
const measured=iterationPlan.map(({baselinePilotNs,candidatePilotNs,...g})=>g);
for(let round=0;round<config.cpuPairs;round++){
 const rows=shuffle(measured,'cpu-groups-'+round),path='zero-factor-native-cpu-'+round+'-plan-v71.json';plan(path,rows);
 for(const variant of origin.cpuOrders[round])await run('cpu',round,variant,'cpu',rows,path);
 console.log(JSON.stringify({checkpoint:71,phase:'cpu-pair-complete',round}));
}
await captured('zero-factor-native-environment-between-v71','.','node',['zero-factor-native-environment-v71.mjs']);
for(let round=0;round<config.allocationPairs;round++){
 const rows=shuffle(config.allocationIterations.flatMap(n=>groups(n)),'allocation-groups-'+round),path='zero-factor-native-allocation-'+round+'-plan-v71.json';plan(path,rows);
 for(const variant of origin.allocationOrders[round])await run('allocation',round,variant,'allocation',rows,path);
 console.log(JSON.stringify({checkpoint:71,phase:'allocation-pair-complete',round}));
}
await captured('zero-factor-native-environment-after-v71','.','node',['zero-factor-native-environment-v71.mjs']);
assert.deepEqual(costSources(),source);for(const[p,h]of Object.entries(origin.scripts))assert.equal(sha(p),h,p);
for(const a of binary.binaries)assert.equal(sha(a.path),a.sha256);
writeFileSync('zero-factor-native-runs-v71.json',JSON.stringify({checkpoint:71,originSha256:sha('zero-factor-native-origin-v71.json'),runs,
 finished:new Date().toISOString(),cpuRows:24*2*456,allocationRows:4*2*2*456,pilotRows:4*456},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:71,status:'native-collection-terminal',runs:runs.length,cpuRows:21888,allocationRows:7296,pilotRows:1824,
 limits:'All full reports checked; statistical analysis and retention decisions still pending. New-answer costs are not equal-work comparisons.'}));
