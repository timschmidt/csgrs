import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-revision-sources-v79.mjs';
import {config,groups,key,orders,shuffle,validate} from './twelfth-revision-cost-protocol-v79.mjs';
import {cases} from './twelfth-cost-input-v78.mjs';
import {median,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';
import {gate} from './check-twelfth-revision-tests-v79.mjs';

export function nativeEvidence(){
 const source=sources(),binary=json('twelfth-revision-binaries-v79.json'),origin=json('twelfth-revision-native-origin-v79.json'),run=json('twelfth-revision-native-runs-v79.json');
 assert.deepEqual(binary.source,source);assert.deepEqual(origin.source,source);assert.deepEqual(origin.config,config);
 assert.equal(origin.binariesSha256,sha('twelfth-revision-binaries-v79.json'));assert.equal(run.originSha256,sha('twelfth-revision-native-origin-v79.json'));
 for(const[p,h]of Object.entries(origin.scripts))assert.equal(sha(p),h,p);
 assert.deepEqual(origin.cpuOrders,orders(24,'cpu-triples'));assert.deepEqual(origin.allocationOrders,orders(6,'allocation-triples'));
 assert.equal(run.runs.length,96);assert.deepEqual(json('twelfth-revision-native-journal-v79.json').runs,run.runs);
 assert.equal(binary.binaries.length,6);for(const b of binary.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
 const calibration=json('twelfth-revision-native-calibration-v79.json');assert.deepEqual(calibration.config,config);
 const ordered=[];
 for(let r=0;r<2;r++)for(const v of r===0?['baseline','prior','candidate']:['candidate','prior','baseline'])ordered.push(['pilot',r,v]);
 for(let r=0;r<24;r++)for(const v of origin.cpuOrders[r])ordered.push(['cpu',r,v]);
 for(let r=0;r<6;r++)for(const v of origin.allocationOrders[r])ordered.push(['allocation',r,v]);
 assert.deepEqual(run.runs.map(r=>[r.phase,r.round,r.variant]),ordered);
 let previousFinish=Date.parse(origin.recorded);const observations=[];
 for(const r of run.runs){
  assert.equal(r.mode,r.phase==='allocation'?'allocation':'cpu');
  const g=r.phase==='pilot'?shuffle(groups(config.pilotIterations[r.round]),'pilot-'+r.round):r.phase==='cpu'?
   shuffle(groups().map(g=>({...g,iterations:calibration.iterations[key(g)]})),'cpu-'+r.round):shuffle([1,16].flatMap(n=>groups(n)),'allocation-'+r.round);
  assert.equal(r.tag,'twelfth-revision-'+r.phase+'-'+r.round+'-'+r.variant+'-v79');assert.equal(r.plan,r.tag+'-plan.json');
  assert.equal(sha(r.plan),r.planSha256);assert.deepEqual(json(r.plan),g);assert.equal(r.groups,g.length);
  const b=binary.binaries.find(b=>b.variant===r.variant&&b.mode===r.mode);assert(b);
  const capture=gate(r.tag,'taskset',['-c','2',b.path,r.mode,resolve('twelfth-cost-input-v78.json'),resolve(r.plan)],'.');
  assert.equal(readFileSync('results/'+r.tag+'.stderr').length,0);assert(Date.parse(capture.started)>=previousFinish);previousFinish=Date.parse(capture.finished);
  for(const row of validate('results/'+r.tag+'.stdout',g,r.variant==='baseline'?'baseline':'candidate',r.mode)){
   assert(BigInt(row.wall_before)>=BigInt(Date.parse(capture.started))*1000000n);
   assert(BigInt(row.wall_after)<(BigInt(Date.parse(capture.finished))+1n)*1000000n);
   observations.push({...row,phase:r.phase,round:r.round,variant:r.variant,tag:r.tag});
  }
 }
 assert(Date.parse(run.finished)>=previousFinish);assert.equal(observations.length,65664);
 const pilots=observations.filter(r=>r.phase==='pilot');assert.equal(pilots.length,3456);
 assert.deepEqual(calibration.pilotTags,run.runs.filter(r=>r.phase==='pilot').map(r=>r.tag));
 for(const g of groups()){
  const rates=pilots.filter(r=>key(r)===key(g)).map(r=>r.elapsed_ns/r.iterations);assert.equal(rates.length,6);
  assert.equal(calibration.iterations[key(g)],Math.max(1,Math.min(2000,Math.ceil(2000000/Math.max(...rates)))));
 }
 const indexed=new Map();for(const r of observations){const k=key(r),a=indexed.get(k)??[];a.push(r);indexed.set(k,a);}
 const preflight={};for(const v of ['baseline','prior','candidate']){
  const original=v==='baseline'?'baseline':'candidate';
  for(const mode of ['cpu','allocation']){
   const tag='twelfth-preflight-'+v+'-'+mode+'-v79',b=binary.binaries.find(b=>b.variant===v&&b.mode===mode);
   gate(tag,'taskset',['-c','2',b.path,'check',resolve('twelfth-cost-input-v78.json'),resolve('twelfth-cost-check-plan-v78.json')],'.');
   const current=validate('results/'+tag+'.stdout',groups(),original,'check');
   const old=validate('results/twelfth-cost-'+original+'-'+mode+'-check-v78.stdout',groups(),original,'check');
   assert.deepEqual(current.map(r=>[key(r),r.outcome,r.certificate]),old.map(r=>[key(r),r.outcome,r.certificate]));
   for(const r of current){if(mode==='cpu')preflight[v+':'+key(r)]=r;else assert.equal(r.certificate,preflight[v+':'+key(r)].certificate);}
  }
 }
 const input=cases(),cpu=[],allocations=[];
 for(const g of groups()){
  const data=indexed.get(key(g));assert(data);
  for(const r of data){const p=preflight[r.variant+':'+key(g)];assert.equal(r.outcome,p.outcome);assert.equal(r.certificate,p.certificate);}
  assert.equal(preflight['prior:'+key(g)].certificate,preflight['candidate:'+key(g)].certificate);
  const timed=data.filter(r=>r.phase==='cpu');assert.equal(timed.length,72);
  for(const reference of ['baseline','prior']){
   const equalResult=preflight[reference+':'+key(g)].certificate===preflight['candidate:'+key(g)].certificate;
   const ratios=[];for(let round=0;round<24;round++){
    const a=timed.find(r=>r.variant===reference&&r.round===round),b=timed.find(r=>r.variant==='candidate'&&r.round===round);assert(a&&b);
    assert.equal(a.iterations,b.iterations);ratios.push(b.elapsed_ns/a.elapsed_ns);
   }
   const stats=correctedPairedStatistics(ratios,'twelfth-revision-native-v79:'+reference+':'+key(g));
   const relevant=timed.filter(r=>r.variant===reference||r.variant==='candidate');
   const ns=v=>median(timed.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations));
   cpu.push({...g,reference,iterations:calibration.iterations[key(g)],family:input[g.case].family,equalResult,
    transition:[preflight[reference+':'+key(g)].outcome,preflight['candidate:'+key(g)].outcome],referenceNs:ns(reference),candidateNs:ns('candidate'),
    pairedMedianRatio:stats.pairedMedianRatio,statistics:stats,minBatchNs:Math.min(...relevant.map(r=>r.elapsed_ns)),
    medianBatchNs:median(relevant.map(r=>r.elapsed_ns)),maxBatchNs:Math.max(...relevant.map(r=>r.elapsed_ns)),iterationCapReached:calibration.iterations[key(g)]===2000});
   for(const n of [1,16]){
    const allocated=data.filter(r=>r.phase==='allocation'&&r.iterations===n);assert.equal(allocated.length,18);const fields={};
    for(const [field,output,divide]of [['requests','requestsPerQuery',true],['requested_bytes','bytesPerQuery',true],['peak_delta','batchPeakAboveStart',false],['live_delta','batchLiveDelta',false]]){
     const values=v=>allocated.filter(r=>r.variant===v).map(r=>r[field]/(divide?n:1));
     const deltas=Array.from({length:6},(_,round)=>{const a=allocated.find(r=>r.variant===reference&&r.round===round),b=allocated.find(r=>r.variant==='candidate'&&r.round===round);return(b[field]-a[field])/(divide?n:1);});
     fields[output]={reference:median(values(reference)),candidate:median(values('candidate')),pairedDelta:median(deltas),
      referenceRange:[Math.min(...values(reference)),Math.max(...values(reference))],candidateRange:[Math.min(...values('candidate')),Math.max(...values('candidate'))]};
    }
    allocations.push({...g,reference,iterations:n,family:input[g.case].family,equalResult,fields});
   }
  }
 }
 assert.equal(cpu.length,1152);assert.equal(allocations.length,2304);
 const env=[];for(const name of ['before','between','after']){
  const tag='twelfth-revision-environment-'+name+'-v79',g=gate(tag,'node',['twelfth-revision-environment-v79.mjs'],'.');
  assert.equal(readFileSync('results/'+tag+'.stderr').length,0);const r=json('results/'+tag+'.stdout');
  assert.equal(r.checkpoint,79);assert.equal(r.cpu,2);assert(typeof r.processes==='string'&&r.processes.length>100);env.push({name,gate:g,record:r});
 }
 const first=gate(run.runs[0].tag),lastCpu=gate(run.runs.filter(r=>r.phase==='cpu').at(-1).tag),firstAlloc=gate(run.runs.find(r=>r.phase==='allocation').tag),last=gate(run.runs.at(-1).tag);
 assert(Date.parse(env[0].gate.finished)<=Date.parse(first.started));assert(Date.parse(env[1].gate.started)>=Date.parse(lastCpu.finished));
 assert(Date.parse(env[1].gate.finished)<=Date.parse(firstAlloc.started));assert(Date.parse(env[2].gate.started)>=Date.parse(last.finished));
 const outer='twelfth-revision-native-campaign-v79',outerGate=gate(outer,'node',['run-twelfth-revision-native-v79.mjs'],'.');
 const outerRows=readFileSync('results/'+outer+'.stdout','utf8').trim().split('\n').map(JSON.parse);
 assert.equal(outerRows.length,223);assert.deepEqual(outerRows.at(-1),{checkpoint:79,status:'native-collection-complete',runs:96,groups:576,rawRows:65664,retained:false});
 const childTags=[...run.runs.map(r=>r.tag),...env.map(r=>'twelfth-revision-environment-'+r.name+'-v79')].sort();
 assert.deepEqual(outerRows.filter(r=>r.tag&&r.finished).map(r=>r.tag).sort(),childTags);
 assert.deepEqual(outerRows.filter(r=>r.tag&&!r.finished).map(r=>r.tag).sort(),childTags);
 assert(Date.parse(outerGate.finished)>=Date.parse(env[2].gate.finished));
 const categories={};for(const r of cpu){
  const k=r.reference+':'+(r.equalResult?'same-result:':'changed-answer:')+r.family;
  const t=categories[k]??={groups:0,ratios:[],bothIntervalsFaster:0,bothIntervalsSlower:0,otherwise:0,shortMedianBelow100us:0};
  t.groups++;t.ratios.push(r.pairedMedianRatio);
  if(r.statistics.bootstrap.interval[1]<1&&r.statistics.orderStatistic.interval[1]<1)t.bothIntervalsFaster++;
  else if(r.statistics.bootstrap.interval[0]>1&&r.statistics.orderStatistic.interval[0]>1)t.bothIntervalsSlower++;else t.otherwise++;
  if(r.medianBatchNs<100000)t.shortMedianBelow100us++;
 }
 for(const t of Object.values(categories)){t.ratioRange=[Math.min(...t.ratios),Math.max(...t.ratios)];delete t.ratios;}
 const controls=json('twelfth-cost-controls-v78.json');for(const c of controls.records){assert.equal(sha(c.path),c.sha256);
  if(c.accepted)validate(c.path,[controls.group],'baseline','cpu');else assert.throws(()=>validate(c.path,[controls.group],'baseline','cpu'));}
 return {checkpoint:79,status:'revision-native-cost-recomputed',originSha256:sha('twelfth-revision-native-origin-v79.json'),
  campaign:{started:run.started,finished:run.finished,cases:96,groups:576,pilotRows:3456,cpuRows:41472,allocationRows:20736,rawRows:65664,cpuTriples:24,allocationTriples:6},
  categories,cpu,allocations,statisticsSelfTest:statisticsSelfTest(),reusedStreamControls:{valid:1,rejected:12},
  environment:env.map(r=>({name:r.name,started:r.gate.started,finished:r.gate.finished,governor:r.record.governor,frequency:r.record.frequency,load:r.record.load})),
  newBinaryBytes:binary.binaries.filter(b=>b.variant==='candidate').reduce((s,b)=>s+b.bytes,0),rawBytes:run.runs.reduce((s,r)=>s+statSync('results/'+r.tag+'.stdout').size,0),
  limits:'All groups retained. New answers versus baseline are separated; every revised full certificate equals prior. Conditional bootstrap/order intervals are not multiplicity-adjusted. Warmed-process lifecycle definitions and short/capped rows disclosed. Allocation is not RSS. No isolated replication, WASM runtime, consumer, representative size or retention claim.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const evidence=nativeEvidence();if(process.argv.includes('--record'))writeFileSync('twelfth-revision-native-summary-v79.json',JSON.stringify(evidence,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(evidence,json('twelfth-revision-native-summary-v79.json'));
 console.log(JSON.stringify({checkpoint:79,status:'verified',campaign:evidence.campaign,categories:evidence.categories,newBinaryBytes:evidence.newBinaryBytes,rawBytes:evidence.rawBytes,retained:false}));
}
