import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence,campaigns,readRows} from './reanalyse-retained-statistics-v61.mjs';
import {config} from './paired-statistics-v60.mjs';
import {additionalControls} from './check-retained-statistics-v61.mjs';
export function retainedStatisticsEvidence(){
 const sources=sourceEvidence(),analysis=json('retained-statistics-v61-analysis.json');
 assert.equal(analysis.status,'pass');assert.deepEqual(analysis.sources,sources);assert.deepEqual(analysis.config,config);
 assert.equal(analysis.rawRows,10672);assert.equal(analysis.comparisons,199);
 for(const c of analysis.campaigns){assert.equal(sha(c.summary),c.summarySha256);assert.equal(sha(c.raw),c.rawSha256);}
 const specifications=[
  ['retained-statistics-reanalysis','node',['reanalyse-retained-statistics-v61.mjs']],
  ['retained-statistics-check','node',['check-retained-statistics-v61.mjs']],
  ['retained-statistics-capacity','df',['-B1','/tmp','.']],
 ];
 const gates={};for(const[tag,command,args]of specifications){
  const g=json('results/'+tag+'.json');gates[tag]=g;
  for(const[k,v]of Object.entries({tag,command,args,cwd:resolve('.'),code:0,signal:null}))assert.deepEqual(g[k],v,tag+' '+k);
  assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
  assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
 }
 assert(Date.parse(gates['retained-statistics-reanalysis'].finished)<=Date.parse(gates['retained-statistics-check'].started));
 const checked=json('results/retained-statistics-check.stdout');assert.equal(checked.status,'pass');
 assert.equal(checked.rawRows,analysis.rawRows);assert.equal(checked.comparisons,analysis.comparisons);
 assert.deepEqual(checked.controls,additionalControls());
 assert.deepEqual(checked.campaigns,analysis.campaigns.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts})));
 const progress=readRows('results/retained-statistics-reanalysis.stdout');assert.equal(progress.length,6);
 assert.deepEqual(progress.slice(0,5),analysis.campaigns.map(c=>({campaign:c.id,rawRows:c.rawRows,counts:c.counts})));
 assert.deepEqual(progress.at(-1),{status:'pass',rawRows:10672,comparisons:199,output:'retained-statistics-v61-analysis.json'});
 const totals={lostDirectionalClaims:0,gainedDirectionalClaims:0,reversedDirectionalClaims:0,changedBootstrapIntervals:0};
 for(const c of analysis.campaigns)for(const k of Object.keys(totals))totals[k]+=c.counts[k];
 assert.deepEqual(totals,{lostDirectionalClaims:7,gainedDirectionalClaims:0,reversedDirectionalClaims:0,changedBootstrapIntervals:187});
 return{sources,config,rawRows:analysis.rawRows,comparisons:analysis.comparisons,totals,
  campaigns:checked.campaigns,controls:checked.controls,recordedEvidence:checked.recordedEvidence,
  gates:specifications.map(([tag])=>tag),analysisBytes:statSync('retained-statistics-v61-analysis.json').size,
  campaignBindings:campaigns.map(c=>({id:c.id,summarySha256:sha(c.summary),rawSha256:sha(c.raw)})),newTemporaryBytes:0,
  next:'Correct the retained exponential-proof/reuse, polynomial-facts and monic timing campaigns, then other historical experiments. Point-candidate runtime attribution and final consumer/size gates, separate power sums, donor/support reads and the entire original inventory remain open.'};
}
