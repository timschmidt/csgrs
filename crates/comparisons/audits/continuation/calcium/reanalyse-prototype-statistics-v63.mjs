import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence as previousSources} from './reanalyse-proof-facts-monic-v62.mjs';
import {sources as complexSources} from './snapshot-v43-complex-product-sources.mjs';
import {sources as complexV2Sources} from './snapshot-v43-complex-product-v2-sources.mjs';
import {signFilterSources} from './sign-filter-sources.mjs';
import {signFilterSources as maskSources} from './sign-filter-mask-sources.mjs';
import {verifyInventory} from './prototype-statistics-inventory-v63.mjs';
import {median,config,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';
export const reviewedScripts=['run-complex-product-costs.mjs','run-complex-product-v2-costs.mjs','run-rank-costs.mjs',
 'run-sign-filter-costs.mjs','run-sign-filter-mask-costs.mjs','check-complex-product.mjs','check-complex-product-v2.mjs',
 'check-sign-filter.mjs','check-sign-filter-mask.mjs','verify-rank-costs.mjs',
 'snapshot-v43-check-complex-product.mjs','snapshot-v43-check-complex-product-v2.mjs'];
export const supportingScripts=['complex-product-sources.mjs','complex-product-v2-sources.mjs',
 'snapshot-v43-complex-product-sources.mjs','snapshot-v43-complex-product-v2-sources.mjs',
 'sign-filter-sources.mjs','sign-filter-mask-sources.mjs','sign-filter-ring-oracle.mjs'];
const complexCases=[];
for(const bits of [64,192,256,1024,4096,16384])
 for(const[family,scale,mask]of [['dense','integer',0],['dense','dyadic',15],['dense','odd',6],['near-cancel','dyadic',0],
  ['sum-zero','integer',2],['unbalanced','integer',0],['mixed-scale','odd',0],['zero','integer',0]])
  for(const route of ['product','lattice'])for(const lifecycle of ['fresh','reused'])complexCases.push({bits,family,scale,mask,route,lifecycle});
const rankCases=[],signCases=[];
for(let which=0;which<7;which++)for(const width of [4,8,16,32])for(const lifecycle of ['fresh_problem','retained_analysis'])rankCases.push({case:which,width,lifecycle});
for(const fixture of ['rational','pi-far','pi-near','pi-reversed'])for(const vertices of [3,4,8,16,32,128])for(const lifecycle of ['retained','cloned-ring'])signCases.push({fixture,vertices,lifecycle});
export const campaigns=[
 {id:'complexV1',stem:'complex-product',type:'complex',cases:complexCases},
 {id:'complexV2',stem:'complex-product-v2',type:'complex',cases:complexCases},
 {id:'rank',stem:'rank-cost',type:'rank',cases:rankCases},
 {id:'signSummary',stem:'sign-filter',type:'sign',cases:signCases},
 {id:'signMask',stem:'sign-filter-mask',type:'sign',cases:signCases},
].map(c=>({...c,clock:c.type==='complex'?'ns':'elapsed_ns'}));
export const manifests=campaigns.map(c=>c.stem+'-experiment.json');
export const paths=(c,mode)=>({summary:c.stem+'-'+mode+'-summary.json',raw:'results/'+c.stem+'-'+mode+'.jsonl'});
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
const variants=['baseline','candidate'],metrics=['requests','requested_bytes','live_delta','peak_delta'];
export function sourceEvidence(){
 const previous=json('proof-facts-monic-v62-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(previousSources(),previous.sources);
 const gate=json('results/proof-facts-monic-verify.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
 const oldRows=readRows('results/proof-facts-monic-verify.stdout');assert.equal(oldRows.length,19);assert.equal(oldRows.at(-1).checkpoint,62);
 assert.equal(readFileSync('results/proof-facts-monic-verify.stderr').length,0);
 const historical=manifests.map(path=>{
  const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
  return{path,sha256:sha(path),artifacts:Object.keys(m.files).length};
 });
 const sourceCounts={};
 for(const[stem,fn]of [['complex-product',complexSources],['complex-product-v2',complexV2Sources]]){
  const original=readFileSync(stem+'-sources.mjs','utf8'),snapshot=readFileSync('snapshot-v43-'+stem+'-sources.mjs','utf8');
  assert.equal(snapshot,original.replace("sha(resolve(here,'../../../..',p))","sha(resolve(here,'derivative-demand-candidate',p))"));
  assert.notEqual(snapshot,original);
  assert.equal(readFileSync('snapshot-v43-check-'+stem+'.mjs','utf8'),
   readFileSync('check-'+stem+'.mjs','utf8').replace('./'+stem+'-sources.mjs','./snapshot-v43-'+stem+'-sources.mjs'));
  const sources=fn();assert.deepEqual(sources,json(stem+'-experiment.json').sourceMap);assert.deepEqual(sources,json(stem+'-binaries.json').sourceMap);
  sourceCounts[stem]=Object.fromEntries(variants.map(v=>[v,Object.keys(sources[v]).length]));
 }
 for(const[stem,fn]of [['sign-filter',signFilterSources],['sign-filter-mask',maskSources]]){
  const sources=fn();assert.deepEqual(sources,json(stem+'-experiment.json').sourceMaps);
  sourceCounts[stem]=Object.fromEntries(variants.map(v=>[v,Object.keys(sources[v]).length]));
 }
 const rank=json('matrix-solve-experiment.json').candidateSources;
 for(const[p,h]of Object.entries(rank))assert.equal(sha('rank-dominance-trial/'+p),h,p);
 sourceCounts.rankCandidate=Object.keys(rank).length;
 const inventory=verifyInventory(json('prototype-statistics-v63-inventory-corrected.json'));
 const record=path=>{
  const text=readFileSync(path,'utf8'),lines=text.split('\n').length-Number(text.endsWith('\n'));
  return{path,sha256:sha(path),lines,ranges:[[1,lines]]};
 };
 const reads=reviewedScripts.map(record),supportingReads=supportingScripts.map(record);
 const known=json('point-statistics-v60-analysis.json').inventory.files;
 for(const r of reads)assert(known.some(f=>f.path===r.path&&f.sha256===r.sha256));
 assert.equal(reads.reduce((n,r)=>n+r.lines,0),1034);assert.equal(supportingReads.reduce((n,r)=>n+r.lines,0),228);
 return{previousManifestSha256:sha('proof-facts-monic-v62-manifest.json'),previousArtifacts:78,previousFinished:gate.finished,
  liveFiles:956,pointCandidateFiles:175,historical,sourceCounts,reads,supportingReads,reviewedScriptLines:1034,supportingLines:228,inventory};
}
export function relation(c,g){
 if(c.type==='complex')return 'numeric-contract-checked-separately';
 if(c.type==='sign')return 'matching-full-sign-outcome';
 return [1,3].includes(g.case)?'additional-answer':'matching-known-status';
}
function expectedSign(g){
 const oracle=json('results/sign-filter-ring-oracle.stdout'),row=oracle.rows.find(r=>r.fixture===g.fixture&&r.vertices===g.vertices);assert(row);
 const sign=row.sign===1?'Positive':'Negative',stage=g.fixture==='rational'?'Exact':g.fixture==='pi-far'?'Structural':'Refined';
 assert([-1,1].includes(row.sign));return `Decided { value: ${sign}, certainty: Exact, stage: ${stage} }`;
}
export function reconstruct(c,g,rows,mode,campaign,last=Date.parse(campaign.started)){
 assert(['cpu','allocation'].includes(mode));assert.equal(campaign.mode,mode);assert.equal(campaign.cpu,6);
 const begin=Date.parse(campaign.started),end=Date.parse(campaign.finished);
 assert(Number.isFinite(begin)&&Number.isFinite(end)&&begin<=last&&last<=end);
 const pilot=c.type==='rank'?4:2,blocks=mode==='cpu'?12:3,perBlock=mode==='cpu'?4:2;
 assert.equal(rows.length,blocks*perBlock);assert.equal(g.observations,rows.length);assert.equal(g.pilots.length,2);
 assert(Number.isSafeInteger(g.iterations)&&g.iterations>0);
 const max=c.type==='complex'?4096:c.type==='rank'?5000:20000,min=c.type==='sign'?8:4,budget=c.type==='rank'?8e6:6e6;
 assert.equal(g.iterations,mode==='allocation'?16:Math.max(min,Math.min(max,Math.ceil(budget/Math.max(...g.pilots.map(r=>r[c.clock]/r.iterations))))));
 const sign=c.type==='sign'?expectedSign(g):null;if(sign!==null)assert.equal(g.outcome,sign);
 for(const[i,r]of [...g.pilots,...rows].entries()){
  const isPilot=i<2,at=i-2,block=Math.floor(at/perBlock),order=mode==='cpu'&&block%2?['candidate','baseline','baseline','candidate']:mode==='cpu'?['baseline','candidate','candidate','baseline']:variants;
  const v=isPilot?variants[i]:order[at%perBlock];assert.equal(r.variant,v);if(!isPilot)assert.equal(r.block,block);
  assert.equal(r.iterations,isPilot?pilot:g.iterations);
  for(const k of Object.keys(c.cases[0]))assert.equal(r[k],g[k]);
  assert(Number.isSafeInteger(r[c.clock])&&r[c.clock]>0);
  const start=Date.parse(r.started),finish=Date.parse(r.finished);assert(start>=last&&finish>=start&&finish<=end);last=finish;
  if(c.type==='complex'){
   assert.equal(r.preflight,true);
   if(mode==='cpu')assert.equal(r.allocation,null);
   else{assert.equal(r.allocation.length,4);assert(r.allocation.every(Number.isSafeInteger));
    for(const k of [0,1,3])assert(r.allocation[k]>=0);assert.equal(r.allocation[2],0);}
  }else{
   assert.equal(r.mode,mode);for(const k of metrics){assert(Number.isSafeInteger(r[k])&&r[k]>=0);if(mode==='cpu')assert.equal(r[k],0);}
   if(mode==='allocation')assert.equal(r.live_delta,0);
   if(c.type==='rank'){
    const known=[0,5,6].includes(g.case)||(v==='candidate'&&[1,3].includes(g.case));assert.equal(r.known,known?r.iterations:0);
    if(mode==='allocation')assert(r.requests>0&&r.requested_bytes>0&&r.peak_delta>0);
   }else assert.equal(r.outcome,sign);
  }
 }
 if(mode==='cpu'){
  const nsPerQuery=Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r[c.clock]/r.iterations))]));
  assert.deepEqual(nsPerQuery,g.nsPerQuery);
  const ratios=Array.from({length:blocks},(_,b)=>{const time=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r[c.clock]));return time('candidate')/time('baseline');});
  assert.equal(median(ratios),g.pairedMedianRatio);assert.equal(g.pairedMedianBootstrap95.length,2);
  assert(g.pairedMedianBootstrap95.every(x=>Number.isFinite(x)&&x>0));assert(g.pairedMedianBootstrap95[0]<=g.pairedMedianBootstrap95[1]);
  return{last,nsPerQuery,ratios,relation:relation(c,g)};
 }
 const measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(metrics.map((k,i)=>{
  const values=rows.filter(r=>r.variant===v).map(r=>c.type==='complex'?r.allocation[i]:r[k]);
  const min=Math.min(...values),max=Math.max(...values);assert.equal(min,max);return[k,{min,max}];
 }))]));
 const legacy=c.type==='complex'?Object.fromEntries(variants.map(v=>[v,Object.fromEntries(g.measurements[v].map(({min,max},i)=>[metrics[i],{min,max}]))])):g.measurements;
 if(c.type==='complex')for(const v of variants)assert.deepEqual(g.measurements[v].map(x=>x.name),['requests','requestedBytes','liveDelta','peakDelta']);
 assert.deepEqual(measurements,legacy);assert.equal(g.pairedMedianBootstrap95,undefined);
 if(c.type==='sign'){
  const b=measurements.baseline,q=measurements.candidate,reaches=['pi-near','pi-reversed'].includes(g.fixture);
  assert.equal(b.requests.min-q.requests.min,reaches?g.iterations:0);
  assert.equal(b.requested_bytes.min-q.requested_bytes.min,reaches?g.iterations*2*g.vertices:0);assert(q.peak_delta.min<=b.peak_delta.min);
  if(c.id==='signMask'){
   const old=json('sign-filter-allocation-summary.json').summaries.find(r=>r.fixture===g.fixture&&r.vertices===g.vertices&&r.lifecycle===g.lifecycle);
   assert.deepEqual(measurements,old.measurements);
  }
 }
 return{last,measurements,relation:relation(c,g)};
}
const side=ci=>ci[1]<1?'below':ci[0]>1?'above':'includes';
export function counts(groups){
 return{comparisons:groups.length,intervals:Object.fromEntries(['legacy','correctedBootstrap','orderStatistic'].map(method=>[method,
  Object.fromEntries(['below','above','includes'].map(k=>[k,groups.filter(g=>g.classifications[method]===k).length]))])),
  changedIntervals:groups.filter(g=>JSON.stringify(g.legacyBootstrap95)!==JSON.stringify(g.bootstrap.interval)).length,
  lostDirectional:groups.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap==='includes').length,
  gainedDirectional:groups.filter(g=>g.classifications.legacy==='includes'&&g.classifications.correctedBootstrap!=='includes').length,
  reversedDirectional:groups.filter(g=>g.classifications.legacy!=='includes'&&g.classifications.correctedBootstrap!=='includes'&&g.classifications.legacy!==g.classifications.correctedBootstrap).length};
}
export function reanalyse(progress=()=>{}){
 const sources=sourceEvidence(),tests=statisticsSelfTest(),cpu=[],allocation=[];let rawRows=0,totalComparisons=0,allocationRows=0;
 for(const c of campaigns)for(const mode of ['cpu','allocation']){
  const p=paths(c,mode),m=json(p.summary),rows=readRows(p.raw),groups=[],perGroup=mode==='cpu'?48:6,keys=Object.keys(c.cases[0]);
  assert.deepEqual(m.summaries.map(g=>Object.fromEntries(keys.map(k=>[k,g[k]]))),c.cases);assert.equal(rows.length,c.cases.length*perGroup);
  let last=Date.parse(m.started);
  for(const[i,g]of m.summaries.entries()){
   const rebuilt=reconstruct(c,g,rows.slice(i*perGroup,(i+1)*perGroup),mode,m,last);last=rebuilt.last;
   const descriptor=Object.fromEntries(keys.map(k=>[k,g[k]])),common={...descriptor,iterations:g.iterations,observations:perGroup,relation:rebuilt.relation};
   if(mode==='cpu'){
    const corrected=correctedPairedStatistics(rebuilt.ratios,'prototype-v63/'+c.id+'/'+keys.map(k=>g[k]).join(':'));
    groups.push({...common,nsPerQuery:rebuilt.nsPerQuery,legacyBootstrap95:g.pairedMedianBootstrap95,...corrected,
     classifications:{legacy:side(g.pairedMedianBootstrap95),correctedBootstrap:side(corrected.bootstrap.interval),orderStatistic:side(corrected.orderStatistic.interval)}});
   }else groups.push({...common,measurements:rebuilt.measurements});
  }
  const result={id:c.id,summary:p.summary,summarySha256:sha(p.summary),raw:p.raw,rawSha256:sha(p.raw),rawRows:rows.length,groups};
  if(mode==='cpu'){
   const strata=Object.fromEntries([...new Set(groups.map(g=>g.relation))].map(r=>[r,counts(groups.filter(g=>g.relation===r))]));
   cpu.push({...result,counts:counts(groups),strata});rawRows+=rows.length;totalComparisons+=groups.length;
   progress({id:c.id,mode,rawRows:rows.length,counts:counts(groups),strata});
  }else{
   const deltas=Object.fromEntries(metrics.map(k=>{const ds=groups.map(g=>g.measurements.candidate[k].min-g.measurements.baseline[k].min);
    return[k,{lower:ds.filter(d=>d<0).length,equal:ds.filter(d=>d===0).length,higher:ds.filter(d=>d>0).length}];}));
   allocation.push({...result,deltas});allocationRows+=rows.length;progress({id:c.id,mode,rawRows:rows.length,groups:groups.length,deltas});
  }
 }
 assert.equal(rawRows,25728);assert.equal(totalComparisons,536);assert.equal(allocationRows,3216);
 return{status:'pass',config,sources,tests,rawRows,totalComparisons,allocationRows,cpu,allocation,
  limits:'Five isolated, previously unselected prototype campaigns; every measured CPU row, recorded pilot, calibration, sequence, point estimate, and marginal median reconstructed. Complex numerical contracts are independently qualified separately; rank Known counts do not establish identical full results and 16 additional-answer groups are different work; sign outcomes include value/certainty/stage. Allocation requests/bytes/live/peak rechecked without timing inference. Individual corrected intervals require suitable independent/common-distribution blocks, are not multiplicity-adjusted, and use 20000 versus old 5000 replicates. Old intervals remain archival/withdrawn. No new CPU measurement, production or retention change, universal performance, whole historical statistical closure, or ecosystem completion.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=reanalyse(x=>console.log(JSON.stringify(x)));
 writeFileSync('prototype-statistics-v63-analysis.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:'pass',rawRows:result.rawRows,comparisons:result.totalComparisons,allocationRows:result.allocationRows,output:'prototype-statistics-v63-analysis.json'}));
}
