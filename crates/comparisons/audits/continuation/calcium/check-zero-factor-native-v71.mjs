import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
import {groups,key,config,validateRows,reference,variantOrders,shuffle} from './zero-factor-cost-protocol-v71.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {statisticsSelfTest,correctedPairedStatistics,median} from './paired-statistics-v60.mjs';
const checkGate=t=>{const g=json('results/'+t+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.tag,t);return g;};
const range=xs=>[Math.min(...xs),Math.max(...xs)];
function shape(c){
 if(c.operation!=='Divide')return 'nondivision-bypass';
 const report=reference(c,0,'baseline');if(report.status==='DenominatorMayContainZero')return 'zero-divisor-guard';
 if(c.right.polynomial[0]!=='0/1')return 'nonzero-constant-bypass';
 return (c.left.polynomial.length-1)*(c.right.polynomial.length-1)>9?'deflated-oversized':'deflated-bounded';
}
function classification(stat){
 if(stat.bootstrap.interval[1]<1&&stat.orderStatistic.interval[1]<1)return 'below-one-both';
 if(stat.bootstrap.interval[0]>1&&stat.orderStatistic.interval[0]>1)return 'above-one-both';return 'not-directional-under-both';
}
export function nativeEvidence({statistics=true}={}){
 const source=costSources(),origin=json('zero-factor-native-origin-v71.json'),runs=json('zero-factor-native-runs-v71.json');
 assert.deepEqual(origin.source,source);assert.deepEqual(origin.config,config);assert.equal(runs.originSha256,sha('zero-factor-native-origin-v71.json'));
 for(const[p,h]of Object.entries(origin.scripts))assert.equal(sha(p),h,p);
 assert.deepEqual(origin.cpuOrders,variantOrders(24,'cpu-order'));assert.deepEqual(origin.allocationOrders,variantOrders(4,'allocation-order'));
 assert.equal(origin.cpuOrders.filter(o=>o[0]==='baseline').length,12);assert.equal(origin.allocationOrders.filter(o=>o[0]==='baseline').length,2);
 const outer=checkGate('zero-factor-native-campaign-v71'),build=checkGate('zero-factor-cost-build-v71');
 assert(Date.parse(outer.started)>Date.parse(build.finished));
 const binaries=json('zero-factor-cost-binaries-v71.json');assert.equal(sha('zero-factor-cost-binaries-v71.json'),origin.binariesSha256);
 for(const a of binaries.binaries){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);}
 const cases=costCases(),data=new Map(),counts={pilot:0,cpu:0,allocation:0},short={pilot:0,cpu:0,allocation:0};
 let last=Date.parse(outer.started),allocationStarted=false;
 assert.equal(runs.runs.length,60);
 for(const run of runs.runs){
  const g=checkGate(run.tag);assert.equal(g.started,run.started);assert.equal(g.finished,run.finished);
  assert(Date.parse(g.started)>=last);last=Date.parse(g.finished);
  if(run.phase==='allocation')allocationStarted=true;else assert(!allocationStarted);
  assert.equal(sha(run.plan),run.planSha256);const plan=json(run.plan);
  const a=binaries.binaries.find(a=>a.variant===run.variant&&a.mode===run.mode);
  assert.equal(g.command,'taskset');assert.deepEqual(g.args,['-c','2',a.path,run.mode,resolve('zero-factor-cost-input-v71.json'),resolve(run.plan)]);
  const rows=validateRows('results/'+run.tag+'.stdout',plan,run.variant,run.mode);assert.equal(rows.length,run.rows);
  const start=BigInt(Date.parse(g.started))*1000000n,end=BigInt(Date.parse(g.finished)+1)*1000000n;
  for(const r of rows){assert(BigInt(r.wall_before)>=start&&BigInt(r.wall_after)<=end);
   const k=[run.phase,run.round,run.variant,key(r),r.iterations].join('|');assert(!data.has(k));data.set(k,r);
   counts[run.phase]++;if(r.elapsed_ns<1000000)short[run.phase]++;
  }
 }
 assert(Date.parse(outer.finished)>=last);assert.deepEqual(counts,{pilot:1824,cpu:21888,allocation:7296});
 const iterationPlan=json('zero-factor-native-iterations-v71.json');assert.equal(iterationPlan.length,456);
 for(const g of iterationPlan){
  const b=data.get(['pilot',32,'baseline',key(g),32].join('|')),c=data.get(['pilot',32,'candidate',key(g),32].join('|'));
  assert.equal(g.baselinePilotNs,b.elapsed_ns);assert.equal(g.candidatePilotNs,c.elapsed_ns);
  assert.equal(g.iterations,Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/Math.max(b.elapsed_ns/32,c.elapsed_ns/32)))));
 }
 const bare=iterationPlan.map(({baselinePilotNs,candidatePilotNs,...g})=>g);
 for(let r=0;r<24;r++){
  const actual=runs.runs.filter(x=>x.phase==='cpu'&&x.round===r);assert.deepEqual(actual.map(x=>x.variant),origin.cpuOrders[r]);
  const expected=shuffle(bare,'cpu-groups-'+r);for(const run of actual)assert.deepEqual(json(run.plan),expected);
 }
 for(let r=0;r<4;r++){
  const actual=runs.runs.filter(x=>x.phase==='allocation'&&x.round===r);assert.deepEqual(actual.map(x=>x.variant),origin.allocationOrders[r]);
  const expected=shuffle([1,16].flatMap(n=>groups(n)),'allocation-groups-'+r);for(const run of actual)assert.deepEqual(json(run.plan),expected);
 }
 const selfTest=statistics?statisticsSelfTest():null,cpu=[],allocations=[],categories={};
 for(const g of iterationPlan){
  const item=cases[g.case],bReport=reference(item,g.policy,'baseline'),cReport=reference(item,g.policy,'candidate'),
   equal=JSON.stringify(bReport)===JSON.stringify(cReport),category=shape(item);
  if(!equal)assert.equal(cReport.status,'Transformed');
  const values={baseline:[],candidate:[]};for(let r=0;r<24;r++)for(const v of ['baseline','candidate'])
   values[v].push(data.get(['cpu',r,v,key(g),g.iterations].join('|')).elapsed_ns/g.iterations);
  const ratios=values.candidate.map((n,i)=>n/values.baseline[i]);
  const stat=statistics?correctedPairedStatistics(ratios,'zero-factor-native-v71:'+key(g)):null;
  const result={case:g.case,label:item.label,policy:g.policy,lifecycle:g.lifecycle,iterations:g.iterations,equalResult:equal,category,
   transition:bReport.status+' -> '+cReport.status,baselineNs:median(values.baseline),candidateNs:median(values.candidate),
   baselineRangeNs:range(values.baseline),candidateRangeNs:range(values.candidate),ratios,
   pairedMedianRatio:median(ratios),statistics:stat,classification:stat?classification(stat):null};
  cpu.push(result);
  const label=(equal?'equal:':'new-answer:')+category;
  const tally=categories[label]??={groups:0,below:0,above:0,inconclusive:0,ratios:[]};tally.groups++;tally.ratios.push(result.pairedMedianRatio);
  if(stat)tally[result.classification==='below-one-both'?'below':result.classification==='above-one-both'?'above':'inconclusive']++;
  for(const n of [1,16]){
   const pairs=Array.from({length:4},(_,r)=>['baseline','candidate'].map(v=>data.get(['allocation',r,v,key(g),n].join('|'))));
   const fields={};for(const [name,field,divisor]of [['requestsPerQuery','requests',n],['bytesPerQuery','requested_bytes',n],
    ['batchLiveDelta','live_delta',1],['batchPeakAboveStart','peak_delta',1]]){
    const b=pairs.map(p=>p[0][field]/divisor),c=pairs.map(p=>p[1][field]/divisor);
    fields[name]={baseline:median(b),candidate:median(c),baselineRange:range(b),candidateRange:range(c),pairedDelta:median(c.map((v,i)=>v-b[i])),
     pairedRatio:b.every(v=>v>0)?median(c.map((v,i)=>v/b[i])):null};
   }
   allocations.push({case:g.case,label:item.label,policy:g.policy,lifecycle:g.lifecycle,iterations:n,equalResult:equal,category,fields});
  }
 }
 const summary=Object.fromEntries(Object.entries(categories).map(([k,{ratios,...v}])=>[k,{...v,pairedMedianRange:range(ratios)}]));
 const env={};for(const phase of ['before','between','after']){
  const tag='zero-factor-native-environment-'+phase+'-v71',gate=checkGate(tag);env[phase]=json('results/'+tag+'.stdout');
  assert.equal(env[phase].affinity,'0');assert.equal(env[phase].cpu,2);
  if(phase==='before')assert(Date.parse(gate.finished)<=Date.parse(runs.runs[0].started));
  if(phase==='between'){
   const cpuRuns=runs.runs.filter(r=>r.phase==='cpu'),allocationRuns=runs.runs.filter(r=>r.phase==='allocation');
   assert(Date.parse(gate.started)>=Date.parse(cpuRuns.at(-1).finished));assert(Date.parse(gate.finished)<=Date.parse(allocationRuns[0].started));
  }
  if(phase==='after')assert(Date.parse(gate.started)>=Date.parse(runs.runs.at(-1).finished));
 }
 assert.deepEqual(costSources(),source);
 return {checkpoint:71,status:statistics?'native-cost-qualified':'raw-cost-verified',sourceSha256:sha('zero-factor-cost-origin-v71.json'),
  campaign:{started:outer.started,finished:outer.finished,pairs:24,allocationPairs:4,groups:456,cases:114,counts,subMillisecondRows:short,
   commonIterationRange:range(iterationPlan.map(g=>g.iterations)),orchestratorCpu:0,measuredCpu:2},selfTest,categories:summary,cpu,allocations,environment:env,
  newBinaryBytes:binaries.binaries.reduce((n,a)=>n+a.bytes,0),
  limits:'All rows retained. Full reports and source identities checked; instrumented durations excluded from CPU analysis. Fresh sources are not cold processes. Peak/live values are batch deltas above a nonzero starting footprint, not RSS. No universally independent samples, fixed frequency, multiplicity adjustment, representative consumer/size, WASM execution or seventh retention is claimed.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=nativeEvidence();writeFileSync('zero-factor-native-summary-v71.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:71,status:result.status,campaign:result.campaign,categories:result.categories,
  allocationComparisons:result.allocations.length,newBinaryBytes:result.newBinaryBytes,limits:result.limits}));
}
