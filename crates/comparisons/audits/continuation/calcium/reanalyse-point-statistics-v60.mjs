import {createReadStream,readFileSync,readdirSync,writeFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {median,config,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';
import {orderFor} from './point-demand-cost-protocol.mjs';

export const campaigns=[
 {id:'point53',summary:'point-image-cost-cpu-summary.json',raw:'results/point-image-cost-cpu.jsonl',groups:80,
  variants:['baseline','candidate'],numerator:'candidate',references:['baseline'],fields:['case','lifecycle']},
 {id:'history56',summary:'point-history-cost-cpu-summary.json',raw:'results/point-history-cost-cpu.jsonl',groups:768,
  variants:['baseline','candidate'],numerator:'candidate',references:['baseline'],fields:['case','policy','history','lifecycle']},
 {id:'demand57',summary:'point-demand-cost-cpu-summary.json',raw:'results/point-demand-cost-cpu.jsonl',groups:768,
  variants:['baseline','eager','demand'],numerator:'demand',references:['baseline','eager'],fields:['case','policy','history','lifecycle']},
 {id:'wasm59',summary:'point-wasm-cost-summary.json',raw:'results/point-wasm-cost.jsonl',groups:768,
  variants:['baseline','eager','demand'],numerator:'demand',references:['baseline','eager'],fields:['case','policy','history','lifecycle']},
];
const classify=ci=>ci[1]<1?'below':ci[0]>1?'above':'includes';
async function* streamRows(path){
 const stream=createReadStream(path),rl=createInterface({input:stream,crlfDelay:Infinity});
 try{for await(const line of rl){assert(line.length);yield JSON.parse(line);}}finally{rl.close();stream.destroy();}
}
export function legacySamplerInventory(){
 const files=[];
 for(const p of readdirSync('.').filter(p=>p.endsWith('.mjs')&&!p.endsWith('-v60.mjs')).sort()){
  const lines=readFileSync(p,'utf8').split('\n'),matches=[];
  for(let i=0;i<lines.length;i++)if(/Math\.imul\([^;\n]*1664525/.test(lines[i]))matches.push({line:i+1,text:lines[i]});
  if(matches.length)files.push({path:p,sha256:sha(p),matches});
 }
 return{files,count:files.length,scope:'Top-level Calcium continuation JavaScript scripts matching the known LCG, excluding the new v60 test/reanalysis tools. A potential-impact inventory, not line-by-line impact closure or a claim that every match has the same sample size.'};
}
export async function reanalysePointStatistics(progress=()=>{}){
 const tests=statisticsSelfTest(),results=[];let totalRows=0,totalComparisons=0;
 for(const campaign of campaigns){
  const old=json(campaign.summary),summaries=old.summaries;assert.equal(summaries.length,campaign.groups);
  const iter=streamRows(campaign.raw)[Symbol.asyncIterator](),groups=[];let observed=0;
  for(const s of summaries){
   const descriptor=Object.fromEntries(campaign.fields.map(k=>[k,s[k]])),key=campaign.fields.map(k=>s[k]).join(':'),rows=[];
   for(let block=0;block<12;block++){
    const order=campaign.variants.length===3?orderFor('cpu',block):
     block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
    for(const variant of order){
     const next=await iter.next();assert(!next.done);const r=next.value;
     assert.equal(r.block,block);assert.equal(r.variant,variant);assert.equal(r.iterations,s.iterations);
     for(const[k,v]of Object.entries(descriptor))assert.equal(r[k],v);
     assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);rows.push(r);observed++;
    }
   }
   assert.equal(rows.length,s.observations);
   const reports=Object.fromEntries(campaign.variants.map(v=>[v,rows.find(r=>r.variant===v).actual??rows.find(r=>r.variant===v).expected]));
   for(const r of rows)assert.deepEqual(r.actual??r.expected,reports[r.variant]);
   const nsPerQuery=Object.fromEntries(campaign.variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]));
   assert.deepEqual(nsPerQuery,s.nsPerQuery);
   const comparisons={};
   for(const reference of campaign.references){
    const legacy=s.comparisons?.[reference]??s,ratios=Array.from({length:12},(_,block)=>{
     const clock=v=>median(rows.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));
     return clock(campaign.numerator)/clock(reference);
    });
    assert.equal(median(ratios),legacy.pairedMedianRatio);
    if(legacy.pairedBlockRatios)assert.deepEqual(ratios,legacy.pairedBlockRatios);
    const sameFullResult=JSON.stringify(reports[reference])===JSON.stringify(reports[campaign.numerator]);assert.equal(sameFullResult,legacy.sameFullResult);
    const corrected=correctedPairedStatistics(ratios,campaign.id+'/'+reference+'/'+key);
    comparisons[reference]={sameFullResult,legacyBootstrap95:legacy.pairedMedianBootstrap95,...corrected,
     classifications:{legacy:classify(legacy.pairedMedianBootstrap95),correctedBootstrap:classify(corrected.bootstrap.interval),
      orderStatistic:classify(corrected.orderStatistic.interval)}};totalComparisons++;
   }
   groups.push({...descriptor,iterations:s.iterations,observations:rows.length,nsPerQuery,comparisons});
   if(groups.length%128===0)progress({campaign:campaign.id,groups:groups.length});
  }
  assert((await iter.next()).done);assert.equal(observed,campaign.groups*12*campaign.variants.length*2);totalRows+=observed;
  const counts={};
  for(const reference of campaign.references){
   counts[reference]={};
   for(const same of [true,false]){
    const cs=groups.map(g=>g.comparisons[reference]).filter(c=>c.sameFullResult===same);
    counts[reference][same?'sameResult':'changedResult']={groups:cs.length,
     intervals:Object.fromEntries(['legacy','correctedBootstrap','orderStatistic'].map(method=>[method,
      Object.fromEntries(['below','above','includes'].map(side=>[side,cs.filter(c=>c.classifications[method]===side).length]))])),
     changedBootstrapClassification:cs.filter(c=>c.classifications.legacy!==c.classifications.correctedBootstrap).length,
     lostDirectionalBootstrapClaims:cs.filter(c=>c.classifications.legacy!=='includes'&&c.classifications.correctedBootstrap==='includes').length,
     changedBootstrapInterval:cs.filter(c=>JSON.stringify(c.legacyBootstrap95)!==JSON.stringify(c.bootstrap.interval)).length};
   }
  }
  results.push({id:campaign.id,summary:campaign.summary,summarySha256:sha(campaign.summary),raw:campaign.raw,rawSha256:sha(campaign.raw),
   rawRows:observed,counts,groups});progress({campaign:campaign.id,status:'recomputed',rawRows:observed,counts});
 }
 assert.equal(totalRows,151296);assert.equal(totalComparisons,3920);
 return{status:'pass',config,tests,campaigns:results,totalRows,totalComparisons,inventory:legacySamplerInventory(),
  limits:'Original timings/full records, allocation counts, block pairing, point estimates and marginal medians are preserved. Four point-family campaigns are recomputed from raw observations. Rejection-sampled deterministic pseudorandom bootstrap corrects the known low-bit resampling constraint; it does not establish independent identically distributed timing blocks. Order-statistic intervals are conservative conditional on the same independence/common-distribution assumptions. Individual intervals are unadjusted for multiple comparisons. Other inventoried historical intervals remain unqualified pending their own reanalysis; correctness and source integrity are separate.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=await reanalysePointStatistics(x=>console.log(JSON.stringify(x)));
 writeFileSync('point-statistics-v60-analysis.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:result.status,totalRows:result.totalRows,totalComparisons:result.totalComparisons,
  inventoryMatches:result.inventory.count,output:'point-statistics-v60-analysis.json'}));
}
