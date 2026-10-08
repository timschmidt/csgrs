import {readFileSync,statSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
import {nativeEvidence} from './check-zero-factor-native-v71.mjs';
import {groups,key,validateRows,variantOrders,shuffle,readRows} from './zero-factor-cost-protocol-v71.mjs';
export const buildTags=['baseline','candidate'].flatMap(v=>['lock','metadata','build','clippy','cpu-check','allocation-check'].map(k=>'zero-factor-cost-'+v+'-'+k+'-v71'));
function gate(tag){const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));assert(g.elapsedSeconds>=0);return g;}
function outer(tag,nested,extraRecords=0){
 const g=gate(tag),rows=readRows('results/'+tag+'.stdout'),begins=rows.filter(r=>r.pid),ends=rows.filter(r=>r.finished);
 assert.equal(rows.length,nested.length*2+extraRecords+1);assert.equal(rows.at(-1).checkpoint,71);
 assert.equal(readFileSync('results/'+tag+'.stderr').length,0);assert.deepEqual(ends.map(r=>r.tag),nested);
 let last=Date.parse(g.started);for(let i=0;i<nested.length;i++){
  const child=gate(nested[i]);assert.deepEqual(ends[i],child);assert.equal(begins[i].tag,nested[i]);
  assert.deepEqual({...begins[i],pid:undefined},{tag:child.tag,started:child.started,cwd:child.cwd,command:child.command,args:child.args,pid:undefined});
  assert(Number.isInteger(begins[i].pid)&&begins[i].pid>0);assert(Date.parse(child.started)>=last);last=Date.parse(child.finished);
 }assert(Date.parse(g.finished)>=last);return {tag,started:g.started,finished:g.finished,records:rows.length,nested:nested.length};
}
export function costEvidence(){
 const source=costSources(),native=nativeEvidence();assert.deepEqual(native,json('zero-factor-native-summary-v71.json'));
 const binary=json('zero-factor-cost-binaries-v71.json'),buildOrigin=json('zero-factor-cost-origin-v71.json');
 assert.deepEqual(buildOrigin.source,source);assert.equal(binary.originSha256,sha('zero-factor-cost-origin-v71.json'));
 assert.equal(buildOrigin.planSha256,sha('zero-factor-cost-check-plan-v71.json'));assert.deepEqual(json('zero-factor-cost-check-plan-v71.json'),groups());
 for(const a of binary.binaries){
  const symbols=execFileSync('nm',['-C',a.path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  assert.deepEqual(symbols,a.allocatorSymbols);if(a.mode==='cpu')assert.equal(symbols.length,0);
  validateRows('results/zero-factor-cost-'+a.variant+'-'+a.mode+'-check-v71.stdout',groups(),a.variant,'check');
 }
 const base=json('results/zero-factor-cost-baseline-metadata-v71.stdout'),candidate=JSON.parse(readFileSync('results/zero-factor-cost-candidate-metadata-v71.stdout','utf8')
  .replaceAll(resolve('zero-factor-cost-candidate-app-v71'),resolve('zero-factor-cost-baseline-app-v71'))
  .replaceAll(resolve('zero-factor-candidate-v70/hypersolve'),resolve('point-demand-candidate/hypersolve')));
 assert.deepEqual(candidate,base);assert.equal(base.packages.length,33);assert.equal(base.resolve.nodes.length,33);
 assert.equal(sha('zero-factor-cost-baseline-app-v71/Cargo.lock'),sha('zero-factor-cost-candidate-app-v71/Cargo.lock'));
 const runs=json('zero-factor-native-runs-v71.json').runs,campaignTags=['zero-factor-native-environment-before-v71',
  ...runs.filter(r=>r.phase!=='allocation').map(r=>r.tag),'zero-factor-native-environment-between-v71',
  ...runs.filter(r=>r.phase==='allocation').map(r=>r.tag),'zero-factor-native-environment-after-v71'];
 const captures=[outer('zero-factor-cost-build-v71',buildTags),outer('zero-factor-native-campaign-v71',campaignTags,28)];
 const probe=json('zero-factor-allocation-probe-v71.json'),po=json('zero-factor-allocation-probe-origin-v71.json');
 assert.deepEqual(po.source,source);assert.equal(po.scriptSha256,sha('probe-zero-factor-allocation-v71.mjs'));
 assert.equal(po.priorSummarySha256,sha('zero-factor-native-summary-v71.json'));assert.equal(probe.originSha256,sha('zero-factor-allocation-probe-origin-v71.json'));
 assert.deepEqual(po.selected,[10,11,36,38,39,107,111]);assert.deepEqual(po.orders,variantOrders(4,'allocation-attribution'));
 assert.equal(probe.runs.length,56);let pairChecks=0;
 const probeComparisons=[];
 for(const which of po.selected)for(let round=0;round<4;round++){
  const matching=probe.runs.filter(r=>r.case===which&&r.round===round);assert.deepEqual(matching.map(r=>r.variant),po.orders[round]);
  const expected=shuffle([1,16].flatMap(n=>groups(n).filter(g=>g.case===which)),'allocation-attribution-'+which+'-'+round);
  for(const r of matching){
   assert.equal(sha(r.plan),r.planSha256);assert.deepEqual(json(r.plan),expected);
   const actual=validateRows('results/'+r.tag+'.stdout',expected,r.variant,'allocation');assert.deepEqual(actual,r.rows);
   const a=binary.binaries.find(a=>a.variant===r.variant&&a.mode==='allocation'),g=gate(r.tag);
   assert.equal(g.command,'taskset');assert.deepEqual(g.args,['-c','2',a.path,'allocation',resolve('zero-factor-cost-input-v71.json'),resolve(r.plan)]);
  }
  const [b,c]=['baseline','candidate'].map(v=>matching.find(r=>r.variant===v));
  for(const x of b.rows){const y=c.rows.find(y=>key(y)===key(x)&&y.iterations===x.iterations);assert(y);
   assert.deepEqual(x.expected,y.expected);for(const field of ['requests','requested_bytes','peak_delta','live_delta'])assert.equal(x[field],y[field]);pairChecks++;
   if(round===0&&x.policy===0&&x.lifecycle==='retained'&&x.iterations===1)probeComparisons.push({case:which,requests:x.requests,bytes:x.requested_bytes,peak:x.peak_delta,live:x.live_delta});
  }
 }
 assert.equal(pairChecks,224);captures.push(outer('zero-factor-allocation-probe-v71',probe.runs.map(r=>r.tag)));
 assert(Date.parse(captures[2].started)>Date.parse(gate('zero-factor-native-check-v71').finished));
 const controls=json('zero-factor-cost-controls-v71.json');assert.equal(controls.valid,1);assert.equal(controls.rejected,9);
 for(const r of controls.results){assert.equal(sha(r.path),r.sha256);let error=null;
  try{validateRows(r.path,[controls.group],'baseline','cpu');}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error,r.error);assert.equal(error===null,r.accepted);
 }
 const allGates=[...buildTags,'zero-factor-cost-build-v71',...campaignTags,'zero-factor-native-campaign-v71','zero-factor-native-check-v71',
  ...probe.runs.map(r=>r.tag),'zero-factor-allocation-probe-v71','zero-factor-cost-controls-v71','zero-factor-cost-fmt-v71'];
 assert.equal(allGates.length,137);assert.equal(new Set(allGates).size,137);for(const t of allGates)gate(t);
 const allocationTallies={};for(const r of native.allocations){
  const category=(r.equalResult?'equal:':'new-answer:')+r.category+':n'+r.iterations,
   t=allocationTallies[category]??={groups:0,requests:{lower:0,equal:0,higher:0},bytes:{lower:0,equal:0,higher:0},peak:{lower:0,equal:0,higher:0},live:{lower:0,equal:0,higher:0}};
  t.groups++;for(const [a,b]of [['requests','requestsPerQuery'],['bytes','bytesPerQuery'],['peak','batchPeakAboveStart'],['live','batchLiveDelta']]){
   const delta=r.fields[b].pairedDelta;t[a][delta<0?'lower':delta>0?'higher':'equal']++;
  }
 }
 const examples=native.cpu.filter(r=>r.policy===0&&r.lifecycle==='retained'&&[3,23,35,41,104,113].includes(r.case)).map(r=>({
  case:r.case,label:r.label,equalResult:r.equalResult,category:r.category,transition:r.transition,baselineNs:r.baselineNs,candidateNs:r.candidateNs,
  pairedMedianRatio:r.pairedMedianRatio,bootstrap:r.statistics.bootstrap.interval,orderStatistic:r.statistics.orderStatistic.interval,
  allocation:native.allocations.find(a=>a.case===r.case&&a.policy===0&&a.lifecycle==='retained'&&a.iterations===1).fields}));
 assert.deepEqual(costSources(),source);
 return {checkpoint:71,status:'isolated-native-cost-qualified',sourceOriginSha256:sha('zero-factor-cost-origin-v71.json'),
  liveFiles:956,candidateFiles:176,retainedContinuationTransfers:6,newDonorLines:0,gates:allGates,captures,
  dependencyGraph:{packages:33,nodes:33,fullMetadataEqualAfterPaths:true,locksEqual:true},preflightChecks:1824,
  campaign:native.campaign,categories:native.categories,allocationComparisons:912,allocationTallies,examples,
  attribution:{cases:7,processes:56,records:448,pairedChecks:224,allCountsEqual:true,examples:probeComparisons,
   interpretation:'Small mixed-workload increases are not reproduced in case-isolated warmed processes. This supports dependence on preceding work/state, not a specified cache mechanism or universal absence of allocation costs. Original campaign rows are unchanged.'},
  negativeControls:{valid:1,rejected:9},binaries:{files:4,bytes:native.newBinaryBytes,dir:buildOrigin.dir,noNewSolverCopy:true},
  rawObservationBytes:runs.reduce((n,r)=>n+statSync('results/'+r.tag+'.stdout').size,0),
  limits:native.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(costEvidence()));
