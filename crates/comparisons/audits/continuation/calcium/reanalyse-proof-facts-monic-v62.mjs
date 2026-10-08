import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence as previousSources} from './reanalyse-retained-statistics-v61.mjs';
import {legacySamplerInventory} from './reanalyse-point-statistics-v60.mjs';
import {median,config,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';

export const reviewedScripts=['run-root-exp-proof-bench.mjs','verify-root-exp-proof-checkpoint.mjs',
 'run-root-exp-reuse-bench.mjs','run-root-exp-reuse-confirm-bench.mjs','run-root-exp-reuse-alloc-bench.mjs',
 'verify-root-exp-reuse-checkpoint.mjs','run-reuse-cost-bench.mjs','verify-reuse-cost-checkpoint.mjs',
 'run-polynomial-facts-cpu.mjs','verify-polynomial-facts.mjs','run-monic-costs.mjs','verify-monic-costs.mjs'];
export const manifests=['root-exp-proof-experiment.json','root-exp-reuse-experiment.json','reuse-cost-experiment.json',
 'polynomial-facts-experiment.json','monic-cost-experiment.json','retained-exp-proof.json','retained-polynomial-facts.json','retained-monic.json'];
const numerics=['exp-third','exp-sqrt2','exp-sine','exp-negative-sine','exp-multiradical','exp-large','exp-tiny',
 'exp-offset','exp-opaque-zero','ordinary-sqrt','ordinary-sine-root','perfect-square'];
const numericLives=['construct','fresh','warm','refine','hot-operand','construct-hot'];
const opaque=['identity','unresolved'].flatMap(k=>[1,8,32,128].map(d=>k+'-'+d));
const queryLives=['fresh','warm-pair','warm-difference'];
const proofQueries=['identity-small','identity-reduced','identity-large','identity-quotient','identity-product',
 'unresolved-near-exp','unresolved-trig-exp','unresolved-nonexp','separated-exp','ordinary-root','ordinary-rational'];
const product=(names,lives)=>names.flatMap(name=>lives.map(lifecycle=>({name,lifecycle})));
const facts=['rational-self','radical-self','log-self','log-plus-one','tiny-self','unknown-leading']
 .flatMap(name=>[1,8,16].flatMap(degree=>['fresh','retained'].map(lifecycle=>({name,degree,lifecycle}))));
const monic=Array.from({length:3},(_,kind)=>Array.from({length:27},(_,code)=>['fresh','retained'].map(lifecycle=>({kind,code,lifecycle})))).flat(2);
export const campaigns=[
 {id:'proofQuery',stem:'root-exp-proof-query',type:'proof',phase:'query',cases:product(proofQueries,queryLives),variants:['baseline','proof'],numerator:'proof'},
 {id:'proofNumeric',stem:'root-exp-proof-numeric',type:'proof',phase:'numeric',cases:product(numerics,numericLives),variants:['baseline','proof'],numerator:'proof'},
 {id:'reuseOverlapped',stem:'root-exp-reuse',type:'reuse',cases:product(opaque,queryLives),variants:['baseline','sign','reuse'],numerator:'reuse',rejected:true},
 {id:'reuseConfirm',stem:'root-exp-reuse-confirm',type:'reuse',cases:product(opaque,queryLives),variants:['baseline','sign','reuse'],numerator:'reuse'},
 {id:'reuseNumeric',stem:'reuse-cost-numeric',type:'cost',phase:'numeric',cases:product(numerics,numericLives).map(({name,lifecycle})=>({name,setting:lifecycle})),variants:['baseline','sign','reuse'],numerator:'reuse'},
 {id:'reuseFirstTouch',stem:'reuse-cost-first-touch',type:'cost',phase:'first-touch',cases:opaque.flatMap(name=>['independent','shared','common-tail'].map(setting=>({name,setting}))),variants:['baseline','sign','reuse'],numerator:'reuse'},
 {id:'polynomialFacts',stem:'polynomial-facts',type:'facts',cases:facts,variants:['baseline','trial','facts'],numerator:'facts'},
 {id:'monic',stem:'monic-cost',type:'monic',cases:monic,variants:['baseline','trial'],numerator:'trial'},
].map(c=>({...c,controls:c.variants.filter(v=>v!==c.numerator),clocks:c.phase==='first-touch'?['first_ns','warm_ns','lifecycle_ns']:['elapsed_ns']}));
export const paths=(c,mode)=>{
 const stem=c.type==='facts'&&mode==='alloc'?'polynomial-facts-allocation-bounded':c.stem+'-'+(mode==='alloc'&&c.type==='monic'?'allocation':mode);
 return{summary:stem+'-summary.json',raw:'results/'+stem+'.jsonl'};
};
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
const fileMap=m=>({...m.files,...m.sourceHashes,...m.evidenceHashes});

export function sourceEvidence(){
 const previous=json('retained-statistics-v61-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(previousSources(),previous.sources);
 const gate=json('results/retained-statistics-verify.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
 assert.equal(json('results/retained-statistics-verify.stdout').checkpoint,61);assert.equal(readFileSync('results/retained-statistics-verify.stderr').length,0);
 const historical=manifests.map(path=>{
  const m=json(path),files=fileMap(m);for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);
  return{path,sha256:sha(path),artifacts:Object.keys(files).length};
 });
 const retained=manifests.slice(-3).map(path=>{
  const m=json(path),root=m.frozenSnapshot??m.frozenSourceSnapshot,sources=m.liveSources??m.liveSourceHashes;
  for(const[p,h]of Object.entries(sources))assert.equal(sha(root+'/'+p),h,p);
  return{manifest:path,root,files:Object.keys(sources).length};
 });
 const inventory=legacySamplerInventory();assert.deepEqual(inventory,json('point-statistics-v60-manifest.json').inventory);
 const reads=reviewedScripts.map(path=>{
  assert(inventory.files.some(f=>f.path===path&&f.sha256===sha(path)));
  const text=readFileSync(path,'utf8'),lines=text.split('\n').length-Number(text.endsWith('\n'));
  return{path,sha256:sha(path),ranges:[[1,lines]],lines};
 });assert.equal(reads.reduce((n,r)=>n+r.lines,0),1222);
 // Re-estimation cannot make the explicitly rejected overlapping run eligible.
 const first=json('results/run-root-exp-reuse-cpu.json'),mem=json('results/root-exp-reuse-memcheck-deep-fresh.json'),confirm=json('results/run-root-exp-reuse-confirm-cpu.json');
 assert(Date.parse(first.started)<Date.parse(mem.finished)&&Date.parse(mem.started)<Date.parse(first.finished));
 assert(Date.parse(confirm.started)>Math.max(Date.parse(first.finished),Date.parse(mem.finished)));
 return{previousManifestSha256:sha('retained-statistics-v61-manifest.json'),previousArtifacts:48,previousFinished:gate.finished,
  liveFiles:956,pointCandidateFiles:175,historical,retained,reads,reviewedScriptLines:1222,inventoryMatches:inventory.count,
  supportingReads:[{path:'run-polynomial-facts-allocation-bounded.mjs',sha256:sha('run-polynomial-facts-allocation-bounded.mjs'),ranges:[[1,77]],lines:77}],
  rejectedOverlap:{first,memcheck:mem,confirmation:confirm}};
}

function metrics(c){
 if(c.type==='monic')return['requests','requested_bytes','live_delta','peak_delta'];
 if(c.type==='facts')return['alloc_calls','allocated_bytes','retained_bytes'];
 return c.phase==='first-touch'?['first_calls','first_bytes','warm_calls','warm_bytes']:['alloc_calls','allocated_bytes'];
}
function outcome(c,variant,descriptor){
 const name=descriptor.name;
 if(c.type==='facts')return ['rational-self','radical-self'].includes(name)||(variant!=='baseline'&&name!=='unknown-leading')?'Known':'Unknown';
 if(c.type==='monic')return descriptor.kind===2&&[7,8,15,16,17,19,20,21,22,23,24,25,26].includes(descriptor.code)?'Unknown':'Known';
 if(c.phase==='numeric')return null;
 if(c.type==='proof')return name.startsWith('identity')?(variant==='proof'?'Equal':'Unknown'):name.startsWith('unresolved')?'Unknown':'NotEqual';
 return name.startsWith('identity')&&variant!=='baseline'?'Equal':'Unknown';
}
function validateRow(c,g,r,variant,count,mode){
 const desc=Object.fromEntries(Object.keys(c.cases[0]).map(k=>[k,g[k]]));
 for(const[k,v]of Object.entries(desc))assert.equal(r[k==='name'?'case':k==='setting'?(c.phase==='numeric'?'lifecycle':'sharing'):k],v);
 assert.equal(r[c.phase==='first-touch'?'workers':'iterations'],count);
 for(const clock of c.clocks)assert(Number.isSafeInteger(r[clock])&&r[clock]>0);
 if(c.phase==='first-touch'){
  assert.equal(r.warm_queries_per_worker,16);assert(Number.isSafeInteger(r.clock_ns)&&r.clock_ns>0);
  assert(r.lifecycle_ns>=r.first_ns+r.warm_ns);
 }
 const expected=outcome(c,variant,desc);
 if(c.type==='facts'||c.type==='monic')assert.equal(r.known,expected==='Known'?count:0);
 else if(expected!==null)assert.deepEqual(r.outcomes,['Equal','NotEqual','Unknown'].map(v=>v===expected?count:0));
 if(c.type==='monic'){
  assert.equal(r.mode,mode==='cpu'?'cpu':'allocation');
  assert.equal(r.degree,[g.code%3,Math.floor(g.code/3)%3,Math.floor(g.code/9)%3].filter(Boolean).length);
 }
 if(c.type!=='facts'||mode==='alloc')for(const k of metrics(c)){
  assert(Number.isSafeInteger(r[k]));if(!['live_delta','retained_bytes'].includes(k))assert(r[k]>=0);
  if(mode==='cpu')assert.equal(r[k],0);
 }
}
function checkCalibration(c,g,mode){
 if(c.type==='facts'&&mode==='alloc'){
  assert.equal(g.iterations,100);assert.equal(g.pilots,undefined);
  return 'Fixed 100-query allocation batches, without timing calibration.';
 }
 if(!g.pilots){
  assert(['proof','reuse'].includes(c.type));
  assert(g.iterations>=20&&g.iterations<=100000||mode==='alloc'&&g.iterations===10);
  if(mode==='alloc')assert.equal(g.iterations,c.type==='reuse'&&g.lifecycle==='fresh'?10:100);
  return 'Pilot observations were not preserved in this historical summary; calibration cannot be independently reconstructed.';
 }
 assert.equal(g.pilots.length,c.variants.length);
 const n=c.type==='facts'||c.type==='monic'?10:c.phase==='first-touch'?2:20;
 g.pilots.forEach((r,i)=>validateRow(c,g,r,c.variants[i],n,mode));
 let expected;
 if(c.phase==='first-touch')expected=mode==='cpu'?32:16;
 else if(mode==='alloc')expected=c.type==='monic'?64:100;
 else expected=Math.max(n,Math.min(c.type==='facts'||c.type==='monic'?10000:100000,
  Math.ceil((c.type==='monic'?8e6:1e7)/Math.max(...g.pilots.map(r=>r.elapsed_ns/r.iterations)))));
 assert.equal(g.iterations,expected);return 'Recorded pilots and fixed/calibrated iteration count reconstructed.';
}
export function reconstruct(c,g,rows,mode){
 const blocks=mode==='cpu'?12:3,perBlock=mode==='alloc'&&c.type==='monic'?2:c.variants.length*2;
 assert.equal(rows.length,blocks*perBlock);assert.equal(g.observations,rows.length);
 assert(Number.isSafeInteger(g.iterations)&&g.iterations>0);
 const calibration=checkCalibration(c,g,mode);
 for(let b=0;b<blocks;b++){
  let order;
  if(mode==='alloc'&&c.type==='monic')order=c.variants;
  else if(c.variants.length===2)order=b%2?[c.numerator,c.controls[0],c.controls[0],c.numerator]:[c.controls[0],c.numerator,c.numerator,c.controls[0]];
  else{const rotation=c.variants.map((_,i)=>c.variants[(i+b)%3]);order=[...rotation,...rotation.toReversed()];}
  for(const[i,v]of order.entries()){
   const r=rows[b*perBlock+i];assert.equal(r.variant,v);assert.equal(r.block,b);validateRow(c,g,r,v,g.iterations,mode);
  }
 }
 const med=(v,k)=>median(rows.filter(r=>r.variant===v).map(r=>r[k]/g.iterations/(c.type==='cost'&&k.startsWith('warm_')?16:1)));
 const measured={};
 for(const v of c.variants){
  if(c.type==='proof'){
   measured[v]={ns:med(v,'elapsed_ns'),alloc_calls:med(v,'alloc_calls'),allocated_bytes:med(v,'allocated_bytes')};
   for(const[k,suffix]of [['ns','ns'],['alloc_calls','alloc_calls'],['allocated_bytes','alloc_bytes']])assert.equal(measured[v][k],g[v+'_'+suffix]);
  }else if(c.type==='reuse'){
   measured[v]={ns:med(v,'elapsed_ns'),alloc_calls:med(v,'alloc_calls'),allocated_bytes:med(v,'allocated_bytes')};assert.deepEqual(measured[v],g.measurements[v]);
  }else if(c.type==='cost'){
   const keys=c.phase==='numeric'?['elapsed_ns',...metrics(c)]:['first_ns','warm_ns','clock_ns','lifecycle_ns',...metrics(c)];
   measured[v]=Object.fromEntries(keys.map(k=>[k,med(v,k)]));assert.deepEqual(measured[v],g.measurements[v]);
  }else if(mode==='cpu'){
   measured[v]=med(v,'elapsed_ns');assert.equal(measured[v],c.type==='facts'?g.measurements[v]:g.nsPerQuery[v]);
  }else if(c.type==='facts'){
   const own=rows.filter(r=>r.variant===v);
   for(const r of own)assert.deepEqual([r.alloc_calls,r.allocated_bytes],[own[0].alloc_calls,own[0].allocated_bytes]);
   measured[v]={callsPerQuery:own[0].alloc_calls/g.iterations,bytesPerQuery:own[0].allocated_bytes/g.iterations,
    retainedByteDeltaRange:[Math.min(...own.map(r=>r.retained_bytes)),Math.max(...own.map(r=>r.retained_bytes))]};assert.deepEqual(measured[v],g.measurements[v]);
  }else{
   measured[v]=Object.fromEntries(metrics(c).map(k=>{const a=rows.filter(r=>r.variant===v).map(r=>r[k]);
    assert.equal(Math.min(...a),Math.max(...a));return[k,{min:Math.min(...a),max:Math.max(...a)}];}));assert.deepEqual(measured[v],g.measurements[v]);
  }
 }
 const comparisons=[];
 if(mode==='cpu'||!['facts','monic'].includes(c.type))for(const control of c.controls)for(const clock of c.clocks){
  const ratios=Array.from({length:blocks},(_,b)=>{
   const time=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r[clock]));return time(c.numerator)/time(control);
  });
  const old=c.type==='proof'||c.type==='monic'?g:c.type==='cost'?g.comparisons[control][clock]:g.comparisons[control];
  assert.equal(median(ratios),old.pairedMedianRatio);
  assert.equal(old.pairedMedianBootstrap95.length,2);assert(old.pairedMedianBootstrap95.every(x=>Number.isFinite(x)&&x>0));
  assert(old.pairedMedianBootstrap95[0]<=old.pairedMedianBootstrap95[1]);
  const a=outcome(c,c.numerator,g),b=outcome(c,control,g),relation=a===null?'numeric-contract-checked-separately':a===b?'matching-recorded-outcome':'additional-answer';
  comparisons.push({control,clock,relation,pairedBlockRatios:ratios,pairedMedianRatio:median(ratios),legacyBootstrap95:old.pairedMedianBootstrap95});
 }
 return{measured,comparisons,calibration};
}
const side=ci=>ci[1]<1?'below':ci[0]>1?'above':'includes';
function counts(comparisons){
 return{comparisons:comparisons.length,intervals:Object.fromEntries(['legacy','correctedBootstrap','orderStatistic'].map(method=>[method,
  Object.fromEntries(['below','above','includes'].map(k=>[k,comparisons.filter(c=>c.classifications[method]===k).length]))])),
  changedIntervals:comparisons.filter(c=>JSON.stringify(c.legacyBootstrap95)!==JSON.stringify(c.bootstrap.interval)).length,
  lostDirectional:comparisons.filter(c=>c.classifications.legacy!=='includes'&&c.classifications.correctedBootstrap==='includes').length,
  gainedDirectional:comparisons.filter(c=>c.classifications.legacy==='includes'&&c.classifications.correctedBootstrap!=='includes').length,
  reversedDirectional:comparisons.filter(c=>c.classifications.legacy!=='includes'&&c.classifications.correctedBootstrap!=='includes'&&c.classifications.legacy!==c.classifications.correctedBootstrap).length};
}

export function reanalyse(progress=()=>{}){
 const sources=sourceEvidence(),tests=statisticsSelfTest(),cpu=[],allocation=[];let rawRows=0,totalComparisons=0,allocationRows=0,withdrawnAllocationIntervals=0;
 for(const c of campaigns)for(const mode of ['cpu',...(c.rejected?[]:['alloc'])]){
  const p=paths(c,mode),m=json(p.summary),rows=readRows(p.raw),groups=[],blocks=mode==='cpu'?12:3,
   perGroup=blocks*(mode==='alloc'&&c.type==='monic'?2:c.variants.length*2);
  if(c.type!=='facts'||mode==='cpu')assert.equal(m.cpu,6);
  assert(Number.isFinite(Date.parse(m.started))&&Date.parse(m.finished)>=Date.parse(m.started));
  const keys=Object.keys(c.cases[0]);assert.deepEqual(m.summaries.map(g=>Object.fromEntries(keys.map(k=>[k,g[k]]))),c.cases);
  assert.equal(rows.length,c.cases.length*perGroup);
  for(const[i,g]of m.summaries.entries()){
   const rebuilt=reconstruct(c,g,rows.slice(i*perGroup,(i+1)*perGroup),mode),descriptor=Object.fromEntries(keys.map(k=>[k,g[k]]));
   const comparisons=rebuilt.comparisons.map(old=>{
    if(mode==='alloc')return{...old,inference:'Withdrawn; allocation-instrumented timings are not CPU evidence. No corrected interval with only three recorded blocks.'};
    const updated=correctedPairedStatistics(old.pairedBlockRatios,'proof-facts-monic-v62/'+c.id+'/'+keys.map(k=>g[k]).join(':')+'/'+old.control+'/'+old.clock);
    return{...old,...updated,classifications:{legacy:side(old.legacyBootstrap95),correctedBootstrap:side(updated.bootstrap.interval),orderStatistic:side(updated.orderStatistic.interval)}};
   });
   groups.push({...descriptor,iterations:g.iterations,observations:perGroup,calibration:rebuilt.calibration,measurements:rebuilt.measured,comparisons});
  }
  const bound={id:c.id,summary:p.summary,summarySha256:sha(p.summary),raw:p.raw,rawSha256:sha(p.raw),rawRows:rows.length,groups};
  if(mode==='cpu'){
   const flat=groups.flatMap(g=>g.comparisons),summary=counts(flat),strata=Object.fromEntries(['numeric-contract-checked-separately','matching-recorded-outcome','additional-answer']
    .map(relation=>[relation,counts(flat.filter(g=>g.relation===relation))]));
   cpu.push({...bound,inferenceEligible:!c.rejected,eligibility:c.rejected?'Previously rejected overlap with Memcheck; corrected intervals do not rehabilitate it.':'Retained historical CPU campaign, subject to original protocol limitations.',counts:summary,strata});
   rawRows+=rows.length;totalComparisons+=flat.length;progress({id:c.id,mode,rawRows:rows.length,inferenceEligible:!c.rejected,counts:summary,strata});
  }else{
   const timingIntervals=groups.reduce((n,g)=>n+g.comparisons.length,0);withdrawnAllocationIntervals+=timingIntervals;
   allocation.push({...bound,withdrawnTimingIntervals:timingIntervals,timingInference:'Not qualified as CPU evidence; original instrumented timing intervals remain withdrawn.'});
   allocationRows+=rows.length;progress({id:c.id,mode,rawRows:rows.length,groups:groups.length,withdrawnTimingIntervals:timingIntervals});
  }
 }
 assert.equal(rawRows,25776);assert.equal(totalComparisons,723);assert.equal(allocationRows,5040);assert.equal(withdrawnAllocationIntervals,441);
 return{status:'pass',config,sources,tests,rawRows,totalComparisons,allocationRows,withdrawnAllocationIntervals,cpu,allocation,
  limits:'Every recorded CPU point estimate and marginal measurement preserved; 8 campaigns include one already rejected overlapping run, which remains ineligible. Matching recorded outcomes are not necessarily identical full values, representations or downstream proof availability; numeric contract qualification is separate, and additional answers are different work. Corrected individual intervals use 20000 versus old 5000 replicates and require suitable independent/common-distribution blocks without multiplicity adjustment. Instrumented allocation timing intervals remain withdrawn, with no new CPU inference from three blocks. Allocation metrics are rechecked separately. No new measurements, production changes, complete retained-transfer statistical closure or ecosystem completion.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=reanalyse(x=>console.log(JSON.stringify(x)));
 writeFileSync('proof-facts-monic-v62-analysis.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:'pass',rawRows:result.rawRows,comparisons:result.totalComparisons,allocationRows:result.allocationRows,
  withdrawnAllocationIntervals:result.withdrawnAllocationIntervals,output:'proof-facts-monic-v62-analysis.json'}));
}
