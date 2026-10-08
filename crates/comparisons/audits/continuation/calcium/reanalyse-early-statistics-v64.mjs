import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence as previousSources} from './reanalyse-prototype-statistics-v63.mjs';
import {median,config,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';
export const reviewedScripts=['run-erf-bench.mjs','run-log-bench.mjs','run-polynomial-decision-bench.mjs',
 'run-root-exp-bench.mjs','run-root-exp-opaque-bench.mjs','run-root-exp-sign-bench.mjs',
 'verify-polynomial-decision.mjs','verify-root-exp-sign-checkpoint.mjs','verify-sign-work-cost-checkpoint.mjs'];
export const supportingScripts=['verify-checkpoint.mjs','verify-results.mjs','verify-erf-checkpoint.mjs','verify-root-exp-checkpoint.mjs'];
export const manifests=['erf-experiment.json','root-exp-experiment.json','root-exp-sign-experiment.json',
 'sign-work-cost-experiment.json','polynomial-decision-experiment.json'];
const logNames=['log-identity','algebraic-identity','log-near-nonzero','log-near-unknown','log-algebraic-near-unknown',
 'log-multiquadratic-unknown','log-transcendental-unknown','nonlog-unknown','ordinary-log','ordinary-rational','exp-cancellation'];
const numericNames=['exp-third','exp-sqrt2','exp-sine','exp-negative-sine','exp-multiradical','exp-large','exp-tiny',
 'exp-offset','exp-opaque-zero','ordinary-sqrt','ordinary-sine-root','perfect-square'];
const numericLives=['construct','fresh','warm','refine','hot-operand','construct-hot'];
const product=(names,lives)=>names.flatMap(name=>lives.map(lifecycle=>({name,lifecycle})));
export const campaigns=[
 {id:'logV1',stem:'log-paired',type:'log',runner:'run-log-bench.mjs',noAlloc:true,sourceLimited:true,
  cases:product(logNames.filter(n=>!['log-algebraic-near-unknown','log-multiquadratic-unknown','log-transcendental-unknown'].includes(n)),['fresh','warm'])},
 {id:'logV2',stem:'log-paired-v2',type:'log',runner:'run-log-bench.mjs',noAlloc:true,sourceLimited:true,
  cases:product(logNames.filter(n=>!['log-multiquadratic-unknown','log-transcendental-unknown'].includes(n)),['fresh','warm'])},
 {id:'logExpanded',stem:'log-paired-v2-expanded',type:'log',runner:'run-log-bench.mjs',cases:product(logNames,['fresh','warm'])},
 {id:'erf',stem:'erf-paired',type:'numeric',runner:'run-erf-bench.mjs',
  cases:product(['erf-zero','erf-third','erf-negative','erf-tiny','erf-tail','erfc-third','erfc-negative','erfc-tail','normal-cdf','complement','ordinary-add','ordinary-sqrt'],['construct','fresh','warm','refine'])},
 {id:'rootEager',stem:'root-exp-paired',type:'numeric',runner:'run-root-exp-bench.mjs',cases:product(numericNames,numericLives)},
 {id:'signQuery',stem:'root-exp-sign-query',type:'query',phase:'query',runner:'run-root-exp-sign-bench.mjs',numerator:'sign',
  cases:product(['identity-small','identity-reduced','identity-large','identity-quotient','identity-product','unresolved-near-exp','unresolved-trig-exp',
   'unresolved-nonexp','separated-exp','ordinary-root','ordinary-rational'],['fresh','warm-pair','warm-difference'])},
 {id:'signNumeric',stem:'root-exp-sign-numeric',type:'numeric',phase:'numeric',runner:'run-root-exp-sign-bench.mjs',numerator:'sign',cases:product(numericNames,numericLives)},
 {id:'opaque',stem:'root-exp-opaque',type:'query',phase:'query',runner:'run-root-exp-opaque-bench.mjs',numerator:'sign',
  cases:product(['identity','unresolved'].flatMap(n=>[1,8,32,128].map(d=>n+'-'+d)),['warm-pair','warm-difference'])},
 {id:'polynomial',stem:'polynomial-decision',type:'polynomial',runner:'run-polynomial-decision-bench.mjs',noAlloc:true,
  cases:['rational-self','radical-self','log-self','log-plus-one','tiny-self','unknown-leading'].flatMap(name=>[1,8,16].flatMap(degree=>['fresh','retained'].map(lifecycle=>({name,degree,lifecycle}))))},
].map(c=>({...c,numerator:c.numerator??'trial',variants:['baseline',c.numerator??'trial']}));
export const paths=(c,mode)=>({summary:c.stem+'-'+mode+'-summary.json',raw:'results/'+c.stem+'-'+mode+'.jsonl'});
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function sourceEvidence(){
 const previous=json('prototype-statistics-v63-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);assert.deepEqual(previousSources(),previous.sources);
 const gate=json('results/prototype-statistics-verify.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
 const output=readRows('results/prototype-statistics-verify.stdout');assert.equal(output.length,24);assert.equal(output.at(-1).checkpoint,63);
 assert.equal(readFileSync('results/prototype-statistics-verify.stderr').length,0);
 const historical=manifests.map(path=>{const m=json(path),files={...m.files,...m.sourceHashes,...m.evidenceHashes};
  for(const[p,h]of Object.entries(files))assert.equal(sha(p),h,p);return{path,sha256:sha(path),artifacts:Object.keys(files).length};});
 const record=path=>{const t=readFileSync(path,'utf8'),lines=t.split('\n').length-Number(t.endsWith('\n'));return{path,sha256:sha(path),lines,ranges:[[1,lines]]};};
 const reads=reviewedScripts.map(record),supportingReads=supportingScripts.map(record);
 assert.equal(reads.reduce((n,r)=>n+r.lines,0),880);assert.equal(supportingReads.reduce((n,r)=>n+r.lines,0),462);
 assert.deepEqual(reviewedScripts.toSorted(),previous.remainingMatches.toSorted());
 for(const r of reads)assert(json('point-statistics-v60-analysis.json').inventory.files.some(f=>f.path===r.path&&f.sha256===r.sha256));
 const baseline=json('baseline-hyperreal.json');for(const f of baseline.files)assert.equal(sha('baseline-hyperreal/'+f.path),f.sha256);
 const frozenPolynomial=json('polynomial-decision-cpu-summary.json').binaries;
 for(const b of Object.values(frozenPolynomial))assert.equal(sha(b.path),b.sha256);
 const old=json('polynomial-decision-cpu-summary.json'),rows=readRows('results/polynomial-decision-cpu.jsonl'),
  fmt=json('results/polynomial-decision-trial-fmt.json'),start=Date.parse(old.started),minMs=rows.slice(0,48).reduce((n,r)=>n+r.elapsed_ns/1e6,0);
 assert(Date.parse(fmt.finished)>start);assert(start+minMs-1>Date.parse(fmt.finished));
 return{previousManifestSha256:sha('prototype-statistics-v63-manifest.json'),previousArtifacts:95,previousFinished:gate.finished,
  liveFiles:956,pointCandidateFiles:175,historical,reads,supportingReads,reviewedScriptLines:880,supportingLines:462,
  baselineHyperrealFiles:baseline.files.length,polynomialBinaries:2,
  rejectedPolynomialOverlap:{group:old.summaries[0].name+':'+old.summaries[0].degree+':'+old.summaries[0].lifecycle,
   campaignStarted:old.started,formatFinished:fmt.finished,firstGroupMinimumElapsedMs:minMs},
  provenanceLimits:'Early CPU executables used reusable paths subsequently overwritten; their recorded hashes are not a claim those CPU binaries remain available. Only the separately frozen polynomial CPU binaries are rechecked here. Earlier 16/18-group log runner versions are not preserved by the surviving 22-group runner, so these two corrected datasets remain archival/source-limited. Most early pilots and all per-observation timestamps were not preserved. No reconstructed source history or fresh timing qualification.'};
}
function outcome(c,g,v){
 if(c.type==='numeric')return null;
 if(c.type==='polynomial')return ['rational-self','radical-self'].includes(g.name)||(v===c.numerator&&g.name!=='unknown-leading')?'Known':'Unknown';
 if(c.type==='query')return g.name.startsWith('identity')?(v===c.numerator?'Equal':'Unknown'):g.name.startsWith('unresolved')?'Unknown':'NotEqual';
 if(g.name==='algebraic-identity'||v==='trial'&&g.name==='log-identity')return 'Equal';
 if(['log-multiquadratic-unknown','log-transcendental-unknown','nonlog-unknown'].includes(g.name)||
  v==='baseline'&&['log-identity','log-near-unknown','log-algebraic-near-unknown'].includes(g.name)||c.id==='logV1'&&g.name==='log-near-unknown')return 'Unknown';
 return 'NotEqual';
}
export function reconstruct(c,g,rows,mode){
 const blocks=mode==='cpu'?12:3;assert.equal(rows.length,blocks*4);assert.equal(g.observations,rows.length);
 assert(Number.isSafeInteger(g.iterations)&&g.iterations>0);
 if(c.type==='polynomial'){
  assert.equal(mode,'cpu');assert.equal(g.pilots.length,2);
  g.pilots.forEach((r,i)=>validate(r,c.variants[i],10));
  assert.equal(g.iterations,Math.max(10,Math.min(10000,Math.ceil(1e7/Math.max(...g.pilots.map(r=>r.elapsed_ns/r.iterations))))));
 }else{
  assert.equal(g.pilots,undefined);assert.equal(g.pilot,undefined);
  assert(g.iterations>=20&&g.iterations<=100000);if(mode==='alloc')assert.equal(g.iterations,100);
 }
 function validate(r,v,n){
  for(const k of Object.keys(c.cases[0]))assert.equal(r[k==='name'?'case':k],g[k]);assert.equal(r.iterations,n);
  assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);assert.equal(r.started,undefined);assert.equal(r.finished,undefined);
  const expected=outcome(c,g,v);
  if(c.type==='polynomial')assert.equal(r.known,expected==='Known'?n:0);
  else{
   for(const k of ['alloc_calls','allocated_bytes']){assert(Number.isSafeInteger(r[k])&&r[k]>=0);if(mode==='cpu')assert.equal(r[k],0);}
   if(expected!==null)assert.deepEqual(r.outcomes,['Equal','NotEqual','Unknown'].map(s=>s===expected?n:0));
  }
 }
 for(let b=0;b<blocks;b++){
  const [base,trial]=c.variants,order=b%2?[trial,base,base,trial]:[base,trial,trial,base];
  for(let i=0;i<4;i++){const r=rows[b*4+i];assert.equal(r.variant,order[i]);assert.equal(r.block,b);validate(r,order[i],g.iterations);}
 }
 const med=(v,k)=>median(rows.filter(r=>r.variant===v).map(r=>r[k]/g.iterations));
 const measurements=Object.fromEntries(c.variants.map(v=>[v,c.type==='polynomial'?{ns:med(v,'elapsed_ns')}:
  {ns:med(v,'elapsed_ns'),alloc_calls:med(v,'alloc_calls'),allocated_bytes:med(v,'allocated_bytes')}]));
 if(c.type==='polynomial')assert.deepEqual(g.measurements,Object.fromEntries(c.variants.map(v=>[v,measurements[v].ns])));
 else for(const v of c.variants)for(const[k,legacy]of [['ns','ns'],['alloc_calls','alloc_calls'],['allocated_bytes','alloc_bytes']])assert.equal(measurements[v][k],g[v+'_'+legacy]);
 const ratios=Array.from({length:blocks},(_,b)=>{const time=v=>median(rows.filter(r=>r.variant===v&&r.block===b).map(r=>r.elapsed_ns));return time(c.numerator)/time('baseline');});
 assert.equal(median(ratios),g.pairedMedianRatio);assert.equal(g.pairedMedianBootstrap95.length,2);
 assert(g.pairedMedianBootstrap95.every(x=>Number.isFinite(x)&&x>0));assert(g.pairedMedianBootstrap95[0]<=g.pairedMedianBootstrap95[1]);
 const x=outcome(c,g,'baseline'),y=outcome(c,g,c.numerator),relation=x===null?'numeric-contract-checked-separately':x===y?'matching-recorded-outcome':'additional-answer';
 if(c.type==='log'){
  for(const[v,k]of [['baseline','baselineOutcomes'],['trial','trialOutcomes']])assert.deepEqual(g[k],['Equal','NotEqual','Unknown'].map(s=>s===outcome(c,g,v)?1:0));
  assert.equal(g.matched,x===y);
 }
 if(c.id==='signNumeric'&&mode==='alloc')for(const k of ['alloc_calls','allocated_bytes'])assert.equal(measurements.baseline[k],measurements.sign[k]);
 if(c.id==='opaque'&&mode==='alloc'&&g.name.startsWith('unresolved')&&g.lifecycle==='warm-difference')
  for(const k of ['alloc_calls','allocated_bytes'])assert.equal(measurements.baseline[k],measurements.sign[k]);
 return{measurements,ratios,relation,calibration:c.type==='polynomial'?'Recorded pilots reconstructed.':'Unpreserved pilots; only recorded iteration bounds/fixed allocation counts can be checked.'};
}
// Reproduce, never rehabilitate, the old estimator. Constants are read from
// the already archived runner, keeping this code out of the old-signature scan.
export function historicalEstimator(runner){
 const t=readFileSync(runner,'utf8'),initial=t.match(/let seed\s*=\s*(\d+)/),step=t.match(/Math\.imul\(seed,\s*(\d+)\)\s*\+\s*(\d+)/);
 assert(initial&&step);let seed=Number(initial[1]);const multiplier=Number(step[1]),increment=Number(step[2]);
 return ratios=>{
  const old=Array.from({length:5000},()=>median(ratios.map(()=>{seed=(Math.imul(seed,multiplier)+increment)>>>0;return ratios[seed%ratios.length];}))).sort((a,b)=>a-b);
  return[old[125],old[4875]];
 };
}
const side=ci=>ci[1]<1?'below':ci[0]>1?'above':'includes';
export function counts(gs){return{comparisons:gs.length,
 intervals:Object.fromEntries(['legacy','correctedBootstrap','orderStatistic'].map(k=>[k,Object.fromEntries(['below','above','includes'].map(s=>[s,gs.filter(g=>g.classifications[k]===s).length]))])),
 changedIntervals:gs.filter(g=>JSON.stringify(g.legacyBootstrap95)!==JSON.stringify(g.bootstrap.interval)).length,
 lostDirectional:gs.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap==='includes').length,
 gainedDirectional:gs.filter(g=>g.classifications.legacy==='includes'&&g.classifications.correctedBootstrap!=='includes').length,
 reversedDirectional:gs.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap!=='includes'&&g.classifications.legacy!==g.classifications.correctedBootstrap).length};}
export function reanalyse(progress=()=>{}){
 const sources=sourceEvidence(),tests=statisticsSelfTest(),cpu=[],allocation=[];let rawRows=0,totalComparisons=0,allocationRows=0;
 for(const c of campaigns)for(const mode of ['cpu',...(c.noAlloc?[]:['alloc'])]){
  const p=paths(c,mode),m=json(p.summary),rows=readRows(p.raw),n=mode==='cpu'?48:12,keys=Object.keys(c.cases[0]),groups=[],oldEstimator=historicalEstimator(c.runner);
  assert.equal(m.cpu,6);assert(Number.isFinite(Date.parse(m.started))&&Date.parse(m.finished)>=Date.parse(m.started));
  if(c.type!=='polynomial')assert.equal(m.mode,mode);if(c.phase)assert.equal(m.phase,c.phase);
  assert.deepEqual(m.summaries.map(g=>Object.fromEntries(keys.map(k=>[k,g[k]]))),c.cases);assert.equal(rows.length,c.cases.length*n);
  for(const[i,g]of m.summaries.entries()){
   const rebuilt=reconstruct(c,g,rows.slice(i*n,(i+1)*n),mode),descriptor=Object.fromEntries(keys.map(k=>[k,g[k]]));
   assert.deepEqual(oldEstimator(rebuilt.ratios),g.pairedMedianBootstrap95);
   const common={...descriptor,iterations:g.iterations,observations:n,relation:rebuilt.relation,measurements:rebuilt.measurements,calibration:rebuilt.calibration,legacyBootstrap95:g.pairedMedianBootstrap95};
   if(mode==='cpu'){
    const corrected=correctedPairedStatistics(rebuilt.ratios,'early-v64/'+c.id+'/'+keys.map(k=>g[k]).join(':'));
    const eligibility=c.sourceLimited?'archival-source-limited':c.id==='polynomial'&&i===0?'rejected-format-overlap':'historical-conditional';
    groups.push({...common,eligibility,...corrected,classifications:{legacy:side(g.pairedMedianBootstrap95),correctedBootstrap:side(corrected.bootstrap.interval),orderStatistic:side(corrected.orderStatistic.interval)}});
   }else groups.push({...common,pairedBlockRatios:rebuilt.ratios,pairedMedianRatio:g.pairedMedianRatio,inference:'Withdrawn: three-block allocation-instrumented timing is not CPU evidence.'});
  }
  const result={id:c.id,summary:p.summary,summarySha256:sha(p.summary),raw:p.raw,rawSha256:sha(p.raw),rawRows:rows.length,groups};
  if(mode==='cpu'){
   const strata=Object.fromEntries([...new Set(groups.map(g=>g.eligibility))].map(s=>[s,counts(groups.filter(g=>g.eligibility===s))]));
   cpu.push({...result,counts:counts(groups),strata});rawRows+=rows.length;totalComparisons+=groups.length;
   progress({id:c.id,mode,rawRows:rows.length,counts:counts(groups),strata});
  }else{allocation.push({...result,withdrawnTimingIntervals:groups.length});allocationRows+=rows.length;progress({id:c.id,mode,rawRows:rows.length,groups:groups.length,withdrawnTimingIntervals:groups.length});}
 }
 assert.equal(rawRows,15984);assert.equal(totalComparisons,333);assert.equal(allocationRows,3156);
 return{status:'pass',config,sources,tests,rawRows,totalComparisons,allocationRows,withdrawnAllocationIntervals:263,cpu,allocation,
  limits:'All nine early datasets corrected from raw paired rows; 34 comparisons in two older log versions remain archival/source-limited and the first polynomial group remains rejected for formatting overlap. No unpreserved pilots, timestamps, historical runner versions or CPU executables are invented. Other corrected cohorts retain their historical protocol limitations, not fresh source/binary/runtime qualification. Old inference is withdrawn even when reproduced exactly. Numeric contracts and additional answers are separate from matching recorded outcomes. Allocation clocks remain withdrawn. Corrected per-comparison intervals assume suitable independent/common-distribution blocks without multiplicity adjustment; 20000 versus old 5000 replicates. No new measurement, production change, retention decision, full estimator or ecosystem closure.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=reanalyse(x=>console.log(JSON.stringify(x)));
 writeFileSync('early-statistics-v64-analysis.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:'pass',rawRows:result.rawRows,comparisons:result.totalComparisons,allocationRows:result.allocationRows,
  withdrawnAllocationIntervals:result.withdrawnAllocationIntervals,output:'early-statistics-v64-analysis.json'}));
}
