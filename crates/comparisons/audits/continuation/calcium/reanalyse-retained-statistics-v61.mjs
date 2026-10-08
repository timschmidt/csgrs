import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json,sha,demandSources} from './point-demand-sources.mjs';
import {sources as plannerSources} from './e-plan-qualified-sources.mjs';
import {sources as derivativeSources} from './derivative-demand-sources.mjs';
import {legacySamplerInventory} from './reanalyse-point-statistics-v60.mjs';
import {median,config,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';

export const historicalManifests=['e-plan-experiment.json','e-qualified-experiment.json',
 'derivative-demand-experiment.json','derivative-qualification-experiment.json'];
export const reviewedScripts=['run-e-plan-costs.mjs','check-e-plan.mjs','run-e-qualified-controls.mjs',
 'run-e-qualified-wasm-costs.mjs','check-e-qualified.mjs','run-derivative-costs.mjs',
 'check-derivative-costs.mjs','run-derivative-endpoint.mjs','check-derivative-qualification-costs.mjs'];
const plannerCases=[];
for(const p of [0,-8,-32,-64,-128,-512,-4096,-16384,-65536,-262144]){
 for(const route of ['planner','kernel'])plannerCases.push([p,route,'fresh']);
 for(const lifecycle of ['fresh','warm','coarsen',...(p<0?['refine']:[])])plannerCases.push([p,'public',lifecycle]);
}
for(const p of [-64,-4096,-65536])for(const lifecycle of ['fresh','warm','coarsen','refine'])plannerCases.push([p,'pi-control',lifecycle]);
const followupCases=[[0,'public','warm'],[0,'public','coarsen'],[-64,'public','fresh'],[-4096,'public','fresh'],
 [-65536,'public','fresh'],[-262144,'public','fresh'],[-262144,'public','coarsen'],[-4096,'pi-control','warm'],
 [-65536,'pi-control','fresh'],[-65536,'public','warm']];
const wasmCases=[-64,-4096,-65536,-262144].flatMap(p=>[[0,'fresh'],[1,'fresh'],[2,'fresh'],[2,'warm']].map(([r,l])=>[p,r,l]))
 .concat([-64,-4096,-65536].flatMap(p=>['fresh','warm'].map(l=>[p,3,l])));
const derivativeCases=[],endpointCases=[];
for(const kind of [0,1])for(const degree of [1,3,8,24]){
 for(const order of [0,1,3,24,128])for(const lifecycle of ['retained_curve','fresh_curve'])derivativeCases.push([kind,degree,order,lifecycle]);
 for(const lifecycle of ['retained_graph','fresh_graph'])endpointCases.push([kind,degree,lifecycle]);
}
export const campaigns=[
 {id:'plannerOriginal',stem:'e-plan-cpu',fields:['p','route','lifecycle'],cases:plannerCases,blocks:12,clock:'ns'},
 {id:'plannerNativeFollowup',stem:'e-qualified-controls',fields:['p','route','lifecycle'],cases:followupCases,blocks:40,clock:'ns'},
 {id:'plannerWasm',stem:'e-qualified-wasm-cpu',fields:['p','route','lifecycle'],cases:wasmCases,blocks:12,clock:'ns'},
 {id:'derivativeMain',stem:'derivative-cost-cpu',fields:['kind','degree','order','lifecycle'],cases:derivativeCases,blocks:12,clock:'elapsed_ns'},
 {id:'derivativeEndpoint',stem:'derivative-endpoint-cpu',fields:['kind','degree','lifecycle'],cases:endpointCases,blocks:12,clock:'elapsed_ns'},
].map(c=>({...c,summary:c.stem+'-summary.json',raw:'results/'+c.stem+'.jsonl'}));
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export const classify=ci=>ci[1]<1?'below':ci[0]>1?'above':'includes';

export function sourceEvidence(){
 const previous=json('point-statistics-v60-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 const gate=json('results/point-statistics-verify.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
 const stdout=readRows('results/point-statistics-verify.stdout');assert.equal(stdout.length,13);assert.equal(stdout.at(-1).checkpoint,60);
 assert.equal(readFileSync('results/point-statistics-verify.stderr').length,0);
 demandSources();assert.deepEqual(plannerSources(false),json('e-qualified-experiment.json').sourceMap);
 const derivatives=derivativeSources();for(const p of historicalManifests.slice(2))assert.deepEqual(derivatives,json(p).candidateSources);
 const historical=historicalManifests.map(path=>{
  const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
  return{path,sha256:sha(path),artifacts:Object.keys(m.files).length};
 });
 const inventory=legacySamplerInventory();assert.deepEqual(inventory,previous.inventory);
 const reads=reviewedScripts.map(path=>{
  assert(inventory.files.some(f=>f.path===path&&f.sha256===sha(path)));
  const source=readFileSync(path,'utf8'),lines=source.split('\n').length-Number(source.endsWith('\n'));
  return{path,sha256:sha(path),lines,ranges:[[1,lines]]};
 });
 assert.equal(reads.reduce((s,r)=>s+r.lines,0),724);
 return{previousManifestSha256:sha('point-statistics-v60-manifest.json'),previousArtifacts:Object.keys(previous.files).length,
  priorVerificationFinished:gate.finished,liveFiles:956,demandFiles:175,plannerSourceFiles:[955,956],
  derivativeSourceFiles:Object.fromEntries(Object.entries(derivatives).map(([v,s])=>[v,Object.keys(s).length])),historical,
  reads,reviewedScriptLines:724,inventoryMatches:inventory.count};
}

function calibration(c,g){
 if(c.id==='plannerOriginal'){
  const single=['public','pi-control'].includes(g.route)&&['fresh','refine'].includes(g.lifecycle);
  return{pilot:single?1:2,iterations:single?1:Math.max(2,Math.min(1048576,Math.ceil(6e6/Math.max(...g.pilots.map(r=>r.ns/r.iterations)))))};
 }
 if(c.id==='plannerNativeFollowup'){
  const single=g.lifecycle==='fresh';return{pilot:single?1:8192,
   iterations:single?1:Math.max(8192,Math.min(1048576,Math.ceil(10e6/Math.max(...g.pilots.map(r=>r.ns/r.iterations)))))};
 }
 if(c.id==='plannerWasm'){
  const single=g.route>=2&&g.lifecycle==='fresh',pilot=single?1:g.lifecycle==='warm'?8192:Math.abs(g.p)>=4096?2:64;
  return{pilot,iterations:single?1:Math.max(2,Math.min(1048576,Math.ceil(6e6/Math.max(...g.pilots.map(r=>r.ns/r.iterations)))))};
 }
 const endpoint=c.id==='derivativeEndpoint';return{pilot:2,
  iterations:Math.max(2,Math.min(endpoint?500:5000,Math.ceil((endpoint?20e6:6e6)/Math.max(...g.pilots.map(r=>r.elapsed_ns/r.iterations)))))};
}

// Reconstruct pairing and point estimates without using either bootstrap.
// This is also the fail-closed entry point exercised by mutation controls.
export function reconstructGroup(c,g,rows,campaign){
 assert.equal(rows.length,c.blocks*4);assert.equal(g.observations,rows.length);assert.equal(g.pilots.length,2);
 assert(Number.isSafeInteger(g.iterations)&&g.iterations>0);
 const cal=calibration(c,g);assert.equal(g.iterations,cal.iterations);
 if(c.id==='plannerWasm')assert.equal(g.probeCount,cal.pilot);
 let last=Date.parse(campaign.started);assert(Number.isFinite(last));
 const end=Date.parse(campaign.finished);assert(Number.isFinite(end)&&end>=last);
 for(const [i,r]of [...g.pilots,...rows].entries()){
  const pilot=i<2,at=i-2,block=Math.floor(at/4);
  const order=block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
  assert.equal(r.variant,pilot?['baseline','candidate'][i]:order[at%4]);
  if(!pilot)assert.equal(r.block,block);
  assert.equal(r.iterations,pilot?cal.pilot:g.iterations);
  for(const field of c.fields)assert.equal(r[field],g[field]);
  assert(Number.isSafeInteger(r[c.clock])&&r[c.clock]>0);
  const begin=Date.parse(r.started),finish=Date.parse(r.finished);
  assert(begin>=last&&finish>=begin&&finish<=end);last=finish;
  if(c.clock==='elapsed_ns'){
   assert.equal(r.mode,'cpu');for(const key of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(r[key],0);
  }else if(c.id==='plannerWasm')assert(Number.isSafeInteger(r.linearMemoryBytes)&&r.linearMemoryBytes>0&&r.linearMemoryBytes%65536===0);
  else assert.equal(r.allocation,null);
 }
 if(c.clock==='ns')assert.equal(new Set([...g.pilots,...rows].map(r=>r.fingerprint+':'+r.answerBits)).size,1);
 const nsPerQuery=Object.fromEntries(['baseline','candidate'].map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r[c.clock]/r.iterations))]));
 assert.deepEqual(nsPerQuery,g.nsPerQuery);
 const ratios=Array.from({length:c.blocks},(_,block)=>{
  const clock=v=>median(rows.filter(r=>r.block===block&&r.variant===v).map(r=>r[c.clock]));return clock('candidate')/clock('baseline');
 });
 assert.equal(median(ratios),g.pairedMedianRatio);
 if(c.id==='plannerWasm')assert.deepEqual(g.linearMemoryBytes,Object.fromEntries(['baseline','candidate'].map(v=>{
  const bytes=rows.filter(r=>r.variant===v).map(r=>r.linearMemoryBytes);return[v,[Math.min(...bytes),Math.max(...bytes)]];
 })));
 assert.equal(g.pairedMedianBootstrap95.length,2);assert(g.pairedMedianBootstrap95.every(x=>Number.isFinite(x)&&x>0));
 assert(g.pairedMedianBootstrap95[0]<=g.pairedMedianBootstrap95[1]);
 return{ratios,nsPerQuery};
}

export function reanalyseRetainedStatistics(progress=()=>{}){
 const sources=sourceEvidence(),tests=statisticsSelfTest(),results=[];let rawRows=0,comparisons=0;
 for(const c of campaigns){
  const old=json(c.summary),rows=readRows(c.raw),groups=[];
  assert.deepEqual(old.summaries.map(g=>c.fields.map(k=>g[k])),c.cases);
  assert.equal(rows.length,c.cases.length*c.blocks*4);
  for(const [i,g]of old.summaries.entries()){
   const descriptor=Object.fromEntries(c.fields.map(k=>[k,g[k]])),key=c.fields.map(k=>g[k]).join(':'),
    observed=rows.slice(i*c.blocks*4,(i+1)*c.blocks*4),rebuilt=reconstructGroup(c,g,observed,old),
    corrected=correctedPairedStatistics(rebuilt.ratios,'retained-v61/'+c.id+'/'+key);
   groups.push({...descriptor,iterations:g.iterations,observations:observed.length,nsPerQuery:rebuilt.nsPerQuery,
    legacyBootstrap95:g.pairedMedianBootstrap95,...corrected,
    classifications:{legacy:classify(g.pairedMedianBootstrap95),correctedBootstrap:classify(corrected.bootstrap.interval),
     orderStatistic:classify(corrected.orderStatistic.interval)}});
  }
  const counts={groups:groups.length,intervals:Object.fromEntries(['legacy','correctedBootstrap','orderStatistic'].map(method=>[method,
   Object.fromEntries(['below','above','includes'].map(side=>[side,groups.filter(g=>g.classifications[method]===side).length]))])),
   lostDirectionalClaims:groups.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap==='includes').length,
   gainedDirectionalClaims:groups.filter(g=>g.classifications.legacy==='includes'&&g.classifications.correctedBootstrap!=='includes').length,
   reversedDirectionalClaims:groups.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap!=='includes'&&g.classifications.legacy!==g.classifications.correctedBootstrap).length,
   changedBootstrapIntervals:groups.filter(g=>JSON.stringify(g.legacyBootstrap95)!==JSON.stringify(g.bootstrap.interval)).length};
  results.push({id:c.id,summary:c.summary,summarySha256:sha(c.summary),raw:c.raw,rawSha256:sha(c.raw),rawRows:rows.length,
   blocks:c.blocks,counts,groups});rawRows+=rows.length;comparisons+=groups.length;progress({campaign:c.id,rawRows:rows.length,counts});
 }
 assert.equal(rawRows,10672);assert.equal(comparisons,199);
 return{status:'pass',config,sources,tests,rawRows,comparisons,campaigns:results,
  limits:'Five retained planner/derivative timing campaigns corrected from every original measured row; all point estimates and marginal medians preserved. Original bootstrap intervals are archival and withdrawn, not validated by reproduction. New bootstrap uses 20000 replicates versus 5000, so endpoint changes also include replicate-count/stream changes. Both interval methods require suitable independent common-distribution timing blocks and are individually unadjusted for multiplicity. Recorded allocation/numerical checks are separate. No fresh measurement, universal speedup, platform-wide qualification, whole historical impact closure, or new production/retention decision.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=reanalyseRetainedStatistics(x=>console.log(JSON.stringify(x)));
 writeFileSync('retained-statistics-v61-analysis.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:result.status,rawRows:result.rawRows,comparisons:result.comparisons,output:'retained-statistics-v61-analysis.json'}));
}
