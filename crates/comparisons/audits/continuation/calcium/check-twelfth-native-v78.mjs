import {readFileSync,writeFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {config,groups,key,orders,shuffle,validate,rows} from './twelfth-cost-protocol-v78.mjs';
import {cases} from './twelfth-cost-input-v78.mjs';
import {median,correctedPairedStatistics} from './paired-statistics-v60.mjs';

export function nativeEvidence(){
 const source=sources(),binary=json('twelfth-cost-binaries-v78.json'),origin=json('twelfth-native-origin-v78.json'),run=json('twelfth-native-runs-v78.json');
 assert.deepEqual(binary.source,source);assert.deepEqual(origin.source,source);assert.deepEqual(origin.config,config);
 assert.equal(origin.binariesSha256,sha('twelfth-cost-binaries-v78.json'));assert.equal(origin.scriptSha256,sha('run-twelfth-native-v78.mjs'));
 assert.equal(origin.environmentSha256,sha('twelfth-cost-environment-v78.mjs'));assert.equal(run.originSha256,sha('twelfth-native-origin-v78.json'));
 assert.deepEqual(origin.cpuOrders,orders(24,'cpu-pairs'));assert.deepEqual(origin.allocationOrders,orders(4,'allocation-pairs'));
 assert.equal(run.runs.length,60);assert.deepEqual(json('twelfth-native-journal-v78.json').runs,run.runs);
 for(const b of binary.binaries)assert.equal(sha(b.path),b.sha256,b.path);
 const calibration=json('twelfth-native-calibration-v78.json');assert.deepEqual(calibration.config,config);
 const ordered=[];
 for(let r=0;r<2;r++)for(const v of r%2?['candidate','baseline']:['baseline','candidate'])ordered.push(['pilot',r,v]);
 for(let r=0;r<24;r++)for(const v of origin.cpuOrders[r])ordered.push(['cpu',r,v]);
 for(let r=0;r<4;r++)for(const v of origin.allocationOrders[r])ordered.push(['allocation',r,v]);
 assert.deepEqual(run.runs.map(r=>[r.phase,r.round,r.variant]),ordered);
 let previousFinish=Date.parse(origin.recorded);const observations=[];
 for(const r of run.runs){
  const g=r.phase==='pilot'?shuffle(groups(config.pilotIterations[r.round]),'pilot-'+r.round):r.phase==='cpu'?
   shuffle(groups().map(g=>({...g,iterations:calibration.iterations[key(g)]})),'cpu-'+r.round):shuffle([1,16].flatMap(n=>groups(n)),'allocation-'+r.round);
  assert.equal(sha(r.plan),r.planSha256);assert.deepEqual(json(r.plan),g);assert.equal(r.groups,g.length);
  const gate=json('results/'+r.tag+'.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);assert.equal(gate.command,'taskset');
  assert.equal(gate.cwd,resolve('.'));assert.equal(readFileSync('results/'+r.tag+'.stderr').length,0);
  const b=binary.binaries.find(b=>b.variant===r.variant&&b.mode===(r.phase==='allocation'?'allocation':'cpu'));
  assert.deepEqual(gate.args,['-c','2',b.path,r.mode,resolve('twelfth-cost-input-v78.json'),resolve(r.plan)]);
  assert(Date.parse(gate.started)>=previousFinish);assert(Date.parse(gate.finished)>=Date.parse(gate.started));previousFinish=Date.parse(gate.finished);
  for(const row of validate('results/'+r.tag+'.stdout',g,r.variant,r.mode)){
   assert(BigInt(row.wall_before)>=BigInt(Date.parse(gate.started))*1000000n);
   assert(BigInt(row.wall_after)<(BigInt(Date.parse(gate.finished))+1n)*1000000n);
   observations.push({...row,phase:r.phase,round:r.round,variant:r.variant,tag:r.tag});
  }
 }
 assert(Date.parse(run.finished)>=previousFinish);assert.equal(observations.length,39168);
 const pilots=observations.filter(r=>r.phase==='pilot');assert.equal(pilots.length,2304);
 assert.deepEqual(calibration.pilotTags,run.runs.filter(r=>r.phase==='pilot').map(r=>r.tag));
 for(const g of groups()){
  const rates=pilots.filter(r=>key(r)===key(g)).map(r=>r.elapsed_ns/r.iterations);assert.equal(rates.length,4);
  const n=Math.max(1,Math.min(2000,Math.ceil(2000000/Math.max(...rates))));assert.equal(calibration.iterations[key(g)],n);
 }
 const indexed=new Map();for(const r of observations){const k=key(r),a=indexed.get(k)??[];a.push(r);indexed.set(k,a);}
 const preflight={};for(const v of ['baseline','candidate'])for(const r of validate('results/twelfth-cost-'+v+'-cpu-check-v78.stdout',groups(),v,'check'))preflight[v+':'+key(r)]=r;
 const input=cases(),cpu=[],allocations=[];
 for(const g of groups()){
  const data=indexed.get(key(g));assert(data);
  for(const v of ['baseline','candidate'])for(const r of data.filter(r=>r.variant===v)){
   assert.equal(r.outcome,preflight[v+':'+key(g)].outcome);
   assert.equal(r.certificate,preflight[v+':'+key(g)].certificate);
  }
  const equalResult=preflight['baseline:'+key(g)].outcome===preflight['candidate:'+key(g)].outcome;
  const timed=data.filter(r=>r.phase==='cpu');assert.equal(timed.length,48);
  const ratios=[];
  for(let round=0;round<24;round++){
   const b=timed.find(r=>r.round===round&&r.variant==='baseline'),c=timed.find(r=>r.round===round&&r.variant==='candidate');assert(b&&c);
   assert.equal(b.iterations,c.iterations);ratios.push(c.elapsed_ns/b.elapsed_ns);
  }
  const stats=correctedPairedStatistics(ratios,'twelfth-native-v78:'+key(g));
  const variantNs=v=>median(timed.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations));
  cpu.push({...g,iterations:calibration.iterations[key(g)],family:input[g.case].family,equalResult,
   transition:[preflight['baseline:'+key(g)].outcome,preflight['candidate:'+key(g)].outcome],
   baselineNs:variantNs('baseline'),candidateNs:variantNs('candidate'),pairedMedianRatio:stats.pairedMedianRatio,statistics:stats,
   minBatchNs:Math.min(...timed.map(r=>r.elapsed_ns)),medianBatchNs:median(timed.map(r=>r.elapsed_ns)),maxBatchNs:Math.max(...timed.map(r=>r.elapsed_ns)),
   iterationCapReached:calibration.iterations[key(g)]===2000});
  for(const n of [1,16]){
   const allocated=data.filter(r=>r.phase==='allocation'&&r.iterations===n);assert.equal(allocated.length,8);
   const fields={};
   for(const [field,output,divide]of [['requests','requestsPerQuery',true],['requested_bytes','bytesPerQuery',true],['peak_delta','batchPeakAboveStart',false],['live_delta','batchLiveDelta',false]]){
    const values=v=>allocated.filter(r=>r.variant===v).map(r=>r[field]/(divide?n:1));
    const deltas=Array.from({length:4},(_,round)=>{const b=allocated.find(r=>r.round===round&&r.variant==='baseline'),c=allocated.find(r=>r.round===round&&r.variant==='candidate');return(c[field]-b[field])/(divide?n:1);});
    fields[output]={baseline:median(values('baseline')),candidate:median(values('candidate')),pairedDelta:median(deltas),
     baselineRange:[Math.min(...values('baseline')),Math.max(...values('baseline'))],candidateRange:[Math.min(...values('candidate')),Math.max(...values('candidate'))]};
   }
   allocations.push({...g,iterations:n,family:input[g.case].family,equalResult,fields});
  }
 }
 assert.equal(cpu.length,576);assert.equal(allocations.length,1152);
 const env=[];for(const name of ['before','between','after']){
  const tag='twelfth-native-environment-'+name+'-v78',g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
  assert.equal(g.command,'node');assert.deepEqual(g.args,['twelfth-cost-environment-v78.mjs']);assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
  const r=json('results/'+tag+'.stdout');assert.equal(r.checkpoint,78);assert.equal(r.cpu,2);assert(typeof r.processes==='string'&&r.processes.length>100);env.push({name,gate:g,record:r});
 }
 const first=json('results/'+run.runs[0].tag+'.json'),lastCpu=json('results/'+run.runs.filter(r=>r.phase==='cpu').at(-1).tag+'.json'),
  firstAlloc=json('results/'+run.runs.find(r=>r.phase==='allocation').tag+'.json'),last=json('results/'+run.runs.at(-1).tag+'.json');
 assert(Date.parse(env[0].gate.finished)<=Date.parse(first.started));assert(Date.parse(env[1].gate.started)>=Date.parse(lastCpu.finished));
 assert(Date.parse(env[1].gate.finished)<=Date.parse(firstAlloc.started));assert(Date.parse(env[2].gate.started)>=Date.parse(last.finished));
 const categories={};for(const r of cpu){const k=(r.equalResult?'same-result:':'changed-answer:')+r.family,t=categories[k]??={groups:0,ratios:[],bothIntervalsFaster:0,bothIntervalsSlower:0,otherwise:0,shortMedianBelow100us:0};
  t.groups++;t.ratios.push(r.pairedMedianRatio);if(r.statistics.bootstrap.interval[1]<1&&r.statistics.orderStatistic.interval[1]<1)t.bothIntervalsFaster++;
  else if(r.statistics.bootstrap.interval[0]>1&&r.statistics.orderStatistic.interval[0]>1)t.bothIntervalsSlower++;else t.otherwise++;
  if(r.medianBatchNs<100000)t.shortMedianBelow100us++;
 }
 for(const t of Object.values(categories)){t.ratioRange=[Math.min(...t.ratios),Math.max(...t.ratios)];delete t.ratios;}
 return {checkpoint:78,status:'native-cost-recomputed',source,originSha256:sha('twelfth-native-origin-v78.json'),
  campaign:{started:run.started,finished:run.finished,cases:96,groups:576,pilotRows:2304,cpuRows:27648,allocationRows:9216,rawRows:39168,cpuPairs:24,allocationPairs:4,cpu:2},
  categories,cpu,allocations,environment:env.map(r=>({name:r.name,started:r.gate.started,finished:r.gate.finished,governor:r.record.governor,frequency:r.record.frequency,load:r.record.load})),
  binaryBytes:binary.binaries.reduce((s,b)=>s+b.bytes,0),rawBytes:run.runs.reduce((s,r)=>s+statSync('results/'+r.tag+'.stdout').size,0),
  limits:'Same-result versus changed-answer comparisons separated. Twenty-four balanced randomized pairs; corrected deterministic bootstrap and independent median-order intervals are conditional, not multiplicity-adjusted. Short capped groups disclosed. Fresh means construction-inclusive in a warmed process; cold retains preconstructed pair memory outside timing. Allocation/peak excludes setup and is not RSS. No isolated worst-case attribution, whole-application, WASM runtime, consumer or representative size/retention claim.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const evidence=nativeEvidence();
 if(process.argv.includes('--record'))writeFileSync('twelfth-native-summary-v78.json',JSON.stringify(evidence,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(evidence,json('twelfth-native-summary-v78.json'));
 console.log(JSON.stringify({checkpoint:78,status:'verified',campaign:evidence.campaign,categories:evidence.categories,binaryBytes:evidence.binaryBytes,rawBytes:evidence.rawBytes,retained:false}));
}
