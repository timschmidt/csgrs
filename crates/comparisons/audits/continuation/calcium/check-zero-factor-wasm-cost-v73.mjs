import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-wasm-cost-sources-v73.mjs';
import {config,flags,groups,key,shuffle,orders,modeOrders,validateRows} from './zero-factor-wasm-cost-protocol-v73.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {reference} from './zero-factor-cost-protocol-v71.mjs';
import {statisticsSelfTest,correctedPairedStatistics,median} from './paired-statistics-v60.mjs';
export function gate(t){const g=json('results/'+t+'.json');assert.equal(g.tag,t);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(g.elapsedSeconds>=0);return g;}
const range=xs=>[Math.min(...xs),Math.max(...xs)];
function shape(c){
 if(c.operation!=='Divide')return 'nondivision-bypass';
 if(reference(c,0,'baseline').status==='DenominatorMayContainZero')return 'zero-divisor-guard';
 if(c.right.polynomial[0]!=='0/1')return 'nonzero-constant-bypass';
 return (c.left.polynomial.length-1)*(c.right.polynomial.length-1)>9?'deflated-oversized':'deflated-bounded';
}
const classify=s=>s.bootstrap.interval[1]<1&&s.orderStatistic.interval[1]<1?'below':
 s.bootstrap.interval[0]>1&&s.orderStatistic.interval[0]>1?'above':'inconclusive';
export function costEvidence({statistics=true}={}){
 const source=costSources(),origin=json('zero-factor-wasm-cost-origin-v73.json'),collection=json('zero-factor-wasm-cost-runs-v73.json'),runs=collection.runs;
 assert.deepEqual(origin.source,source);assert.deepEqual(origin.config,config);assert.equal(collection.originSha256,sha('zero-factor-wasm-cost-origin-v73.json'));
 assert.deepEqual(origin.pairOrders,Object.fromEntries(config.instanceModes.map(m=>[m,orders(m+'-pair-order')])));assert.deepEqual(origin.modeOrders,modeOrders());
 for(const mode of config.instanceModes)assert.equal(origin.pairOrders[mode].filter(o=>o[0]==='baseline').length,12);
 assert.equal(origin.modeOrders.filter(o=>o[0]==='persistent').length,12);
 const outer=gate('zero-factor-wasm-cost-campaign-v73'),previous=gate('zero-factor-wasm-verify-v72');assert(Date.parse(outer.started)>Date.parse(previous.finished));
 const binary=json('zero-factor-wasm-binaries-v72.json');assert.equal(sha('zero-factor-wasm-binaries-v72.json'),origin.binariesSha256);
 for(const a of binary.artifacts){assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);}
 const expectedOrder=[];
 for(const instanceMode of config.instanceModes)for(const round of config.pilotIterations)for(const variant of round===1?['baseline','candidate']:['candidate','baseline'])
  expectedOrder.push({phase:'pilot',round,variant,instanceMode});
 for(let round=0;round<24;round++)for(const instanceMode of origin.modeOrders[round])for(const variant of origin.pairOrders[instanceMode][round])
  expectedOrder.push({phase:'cpu',round,variant,instanceMode});
 assert.equal(runs.length,104);assert.deepEqual(runs.map(({phase,round,variant,instanceMode})=>({phase,round,variant,instanceMode})),expectedOrder);
 const counts={pilot:0,cpu:0},short={pilot:0,cpu:0},data=new Map(),memory={},cases=costCases();let last=Date.parse(outer.started),rawBytes=0;
 for(const run of runs){
  const g=gate(run.tag);assert.equal(g.started,run.started);assert.equal(g.finished,run.finished);assert(Date.parse(g.started)>=last);last=Date.parse(g.finished);
  assert.equal(g.command,'taskset');assert.deepEqual(g.args,['-c','2','node',...flags,'zero-factor-wasm-cost-worker-v73.mjs',run.variant,run.instanceMode,resolve(run.plan)]);
  assert.equal(readFileSync('results/'+run.tag+'.stderr').length,0);assert.equal(sha(run.plan),run.planSha256);
  const rowsPath='results/'+run.tag+'.stdout',{rows,terminal}=validateRows(rowsPath,json(run.plan),run.variant,run.instanceMode);assert.equal(rows.length,run.rows);assert.equal(rows.length,456);
  assert.equal(terminal.moduleSha256,binary.artifacts.find(a=>a.variant===run.variant&&a.kind==='rational').sha256);
  assert.equal(terminal.inputSha256,sha('zero-factor-cost-input-v71.json'));assert.equal(terminal.planSha256,run.planSha256);
  assert(Date.parse(terminal.finished)<=Date.parse(g.finished));rawBytes+=statSync(rowsPath).size;
  for(const row of rows){
   assert(Date.parse(row.started)>=Date.parse(g.started)&&Date.parse(row.finished)<=Date.parse(g.finished));
   assert(row.elapsed_ns<=(Date.parse(row.finished)-Date.parse(row.started)+1)*1000000);
   const k=[run.phase,run.round,run.instanceMode,run.variant,key(row),row.iterations].join('|');assert(!data.has(k));data.set(k,row);
   counts[run.phase]++;if(row.elapsed_ns<1000000)short[run.phase]++;
   const m=memory[run.instanceMode+':'+run.variant]??={observations:0,beforeMin:Infinity,beforeMax:0,afterMin:Infinity,afterMax:0};m.observations++;
   m.beforeMin=Math.min(m.beforeMin,row.memoryAfterPrepare);m.beforeMax=Math.max(m.beforeMax,row.memoryAfterPrepare);
   m.afterMin=Math.min(m.afterMin,row.memoryAfterFinish);m.afterMax=Math.max(m.afterMax,row.memoryAfterFinish);
  }
 }
 assert(Date.parse(outer.finished)>=last);assert.deepEqual(counts,{pilot:3648,cpu:43776});assert.equal(collection.pilotRows,counts.pilot);assert.equal(collection.cpuRows,counts.cpu);
 const iterationPlans=json('zero-factor-wasm-cost-iterations-v73.json');assert.deepEqual(Object.keys(iterationPlans),config.instanceModes);
 for(const mode of config.instanceModes){
  const plan=iterationPlans[mode];assert.equal(plan.length,456);
  for(const [i,g]of plan.entries()){
   const {baselinePilotNs,candidatePilotNs,iterations,...group}=g,{iterations:unused,...expected}=groups()[i];assert.deepEqual(group,expected);
   const b=data.get(['pilot',32,mode,'baseline',key(g),32].join('|')),c=data.get(['pilot',32,mode,'candidate',key(g),32].join('|'));
   assert.equal(baselinePilotNs,b.elapsed_ns);assert.equal(candidatePilotNs,c.elapsed_ns);
   assert.equal(iterations,Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/Math.max(b.elapsed_ns/32,c.elapsed_ns/32)))));
  }
  for(const run of runs.filter(r=>r.instanceMode===mode)){
   const expected=run.phase==='pilot'?shuffle(groups(run.round),'pilot-'+mode+'-'+run.round):
    shuffle(plan.map(({baselinePilotNs,candidatePilotNs,...g})=>g),'cpu-'+mode+'-'+run.round);
   assert.deepEqual(json(run.plan),expected);
  }
 }
 const selfTest=statistics?statisticsSelfTest():null,cpu=[],categories={};
 for(const mode of config.instanceModes)for(const g of iterationPlans[mode]){
  const item=cases[g.case],bReport=reference(item,g.policy,'baseline'),cReport=reference(item,g.policy,'candidate'),equal=JSON.stringify(bReport)===JSON.stringify(cReport),category=shape(item);
  if(!equal){assert(['Undecided','UnsupportedDegree'].includes(bReport.status));assert.equal(cReport.status,'Transformed');}
  const values={baseline:[],candidate:[]};for(let round=0;round<24;round++)for(const v of ['baseline','candidate'])
   values[v].push(data.get(['cpu',round,mode,v,key(g),g.iterations].join('|')).elapsed_ns/g.iterations);
  const ratios=values.candidate.map((n,i)=>n/values.baseline[i]),stat=statistics?correctedPairedStatistics(ratios,'zero-factor-wasm-cost-v73:'+mode+':'+key(g)):null;
  const row={case:g.case,label:item.label,policy:g.policy,lifecycle:g.lifecycle,instanceMode:mode,iterations:g.iterations,equalResult:equal,category,
   transition:bReport.status+' -> '+cReport.status,baselineNs:median(values.baseline),candidateNs:median(values.candidate),
   baselineRangeNs:range(values.baseline),candidateRangeNs:range(values.candidate),pairedMedianRatio:median(ratios),ratios,statistics:stat,classification:stat?classify(stat):null};
  cpu.push(row);const k=mode+':'+(equal?'equal:':'new-answer:')+category,t=categories[k]??={groups:0,below:0,above:0,inconclusive:0,ratios:[]};
  t.groups++;t.ratios.push(row.pairedMedianRatio);if(stat)t[row.classification]++;
 }
 const environment={};
 for(const phase of ['before','after-pilots','after']){
  const tag='zero-factor-wasm-cost-environment-'+phase+'-v73',g=gate(tag),e=json('results/'+tag+'.stdout');assert.equal(e.checkpoint,73);assert.equal(e.affinity,'0');assert.equal(e.cpu,2);environment[phase]=e;
  if(phase==='before')assert(Date.parse(g.finished)<=Date.parse(runs[0].started));
  if(phase==='after-pilots'){assert(Date.parse(g.started)>=Date.parse(runs[7].finished));assert(Date.parse(g.finished)<=Date.parse(runs[8].started));}
  if(phase==='after')assert(Date.parse(g.started)>=Date.parse(runs.at(-1).finished));
  assert(Date.parse(g.started)>=Date.parse(outer.started)&&Date.parse(g.finished)<=Date.parse(outer.finished));
 }
 assert.deepEqual(costSources(),source);
 return {checkpoint:73,status:statistics?'matched-wasm-cost-qualified':'matched-wasm-raw-verified',sourceOriginSha256:sha('zero-factor-wasm-cost-origin-v73.json'),
  campaign:{started:outer.started,finished:outer.finished,workerProcesses:104,pairsPerInstanceMode:24,cases:114,groupsPerInstanceMode:456,counts,subMillisecondRows:short,
   commonIterationRanges:Object.fromEntries(config.instanceModes.map(m=>[m,range(iterationPlans[m].map(g=>g.iterations))])),orchestratorCpu:0,measuredCpu:2},selfTest,
  categories:Object.fromEntries(Object.entries(categories).map(([k,{ratios,...v}])=>[k,{...v,pairedMedianRange:range(ratios)}])),cpu,environment,memory,
  storage:{newBinaries:0,newSourceCopies:0,rawObservationBytes:rawBytes},
  limits:'All rows preserved. Conditional, non-multiplicity-adjusted intervals. Preconditioned query batches exclude module/JSON/prepare/GC/output costs. Fresh instance is not cold-start timing; fresh source is reconstructed per query. New answers are not equal-work comparisons. Memory is sampled linear capacity, not allocation/peak/RSS. No consumer/size qualification or new retention.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=costEvidence();writeFileSync('zero-factor-wasm-cost-summary-v73.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:73,status:result.status,campaign:result.campaign,categories:result.categories,memory:result.memory,storage:result.storage,limits:result.limits}));
}
