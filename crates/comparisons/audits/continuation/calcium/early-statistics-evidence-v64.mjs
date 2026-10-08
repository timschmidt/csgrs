import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence,readRows,campaigns} from './reanalyse-early-statistics-v64.mjs';
export function evidence(){
 const sources=sourceEvidence(),a=json('early-statistics-v64-analysis.json');assert.equal(a.status,'pass');assert.deepEqual(a.sources,sources);
 for(const c of [...a.cpu,...a.allocation]){assert.equal(sha(c.summary),c.summarySha256);assert.equal(sha(c.raw),c.rawSha256);}
 const specs=[['early-statistics-historical','node',['verify-polynomial-decision.mjs']],
  ['early-statistics-reanalysis','node',['reanalyse-early-statistics-v64.mjs']],
  ['early-statistics-check','node',['check-early-statistics-v64.mjs']],
  ['early-statistics-binary-inventory','node',['inspect-early-binaries-v64.mjs']],
  ['early-statistics-capacity','df',['-B1','/tmp','.']]];
 const gates={};for(const[tag,command,args]of specs){
  const g=json('results/'+tag+'.json');gates[tag]=g;
  for(const[k,v]of Object.entries({tag,command,args,cwd:resolve('.'),code:0,signal:null}))assert.deepEqual(g[k],v,tag+' '+k);
  assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
  assert(readFileSync('results/'+tag+'.stdout').length>0);
 }
 const historical=readFileSync('results/early-statistics-historical.stdout','utf8'),checkedText=readFileSync('results/early-statistics-check.stdout','utf8');
 assert.equal(readRows('results/early-statistics-historical.stdout').length,14);assert(checkedText.startsWith(historical));
 assert.equal(historical,readFileSync('results/prototype-statistics-historical-rank.stdout','utf8').trimEnd().split('\n').slice(0,14).join('\n')+'\n');
 const rows=readRows('results/early-statistics-check.stdout');assert.equal(rows.length,15);const checked=rows.at(-1);
 assert.equal(checked.status,'pass');assert.equal(checked.controls.rejectedMutations.length,27);
 assert.equal(checked.strata['historical-conditional'].comparisons,298);assert.equal(checked.strata['historical-conditional'].lostDirectional,13);
 assert.equal(checked.strata['archival-source-limited'].comparisons,34);assert.equal(checked.strata['archival-source-limited'].lostDirectional,1);
 assert.equal(checked.strata['rejected-format-overlap'].comparisons,1);
 assert.equal(checked.rawRows,a.rawRows);assert.equal(checked.comparisons,a.totalComparisons);assert.equal(checked.allocationRows,a.allocationRows);
 assert.equal(checked.withdrawnAllocationIntervals,263);assert.equal(a.withdrawnAllocationIntervals,263);
 assert.deepEqual(checked.campaigns,a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts,strata:c.strata})));
 assert.deepEqual(checked.allocations,a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,withdrawnTimingIntervals:c.withdrawnTimingIntervals})));
 const expected=[];for(const c of a.cpu){
  expected.push({id:c.id,mode:'cpu',rawRows:c.rawRows,counts:c.counts,strata:c.strata});
  const alloc=a.allocation.find(g=>g.id===c.id);if(alloc)expected.push({id:alloc.id,mode:'alloc',rawRows:alloc.rawRows,groups:alloc.groups.length,withdrawnTimingIntervals:alloc.withdrawnTimingIntervals});
 }
 expected.push({status:'pass',rawRows:15984,comparisons:333,allocationRows:3156,withdrawnAllocationIntervals:263,output:'early-statistics-v64-analysis.json'});
 assert.deepEqual(readRows('results/early-statistics-reanalysis.stdout'),expected);
 assert(Date.parse(gates['early-statistics-historical'].finished)<Date.parse(gates['early-statistics-reanalysis'].started));
 assert(Date.parse(gates['early-statistics-reanalysis'].finished)<Date.parse(gates['early-statistics-check'].started));
 const binaries=json('early-statistics-v64-binary-inventory.json');assert.deepEqual(binaries,json('results/early-statistics-binary-inventory.stdout'));
 assert.equal(binaries.rows.length,18);assert.equal(binaries.matching,2);assert.equal(binaries.nonmatching,16);
 assert(Date.parse(binaries.inspected)>=Date.parse(gates['early-statistics-binary-inventory'].started)&&Date.parse(binaries.inspected)<=Date.parse(gates['early-statistics-binary-inventory'].finished));
 for(const [i,c]of campaigns.entries())for(const [j,v]of c.variants.entries()){
  const r=binaries.rows[i*2+j],summary=c.stem+'-cpu-summary.json',b=json(summary).binaries[v];
  assert.deepEqual([r.campaign,r.variant,r.summary,r.summarySha256,r.path,r.recordedSha256,r.recordedBytes],[c.id,v,summary,sha(summary),b.path,b.sha256,b.bytes]);
  assert(/^[a-f0-9]{64}$/.test(r.observedSha256));assert(Number.isSafeInteger(r.observedBytes)&&r.observedBytes>0);
  assert.equal(r.matches,r.observedSha256===r.recordedSha256);assert.equal(r.matches,c.id==='polynomial');
 }
 const all=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path),coverage=new Map();
 for(const p of ['point-demand-cost-protocol.mjs','point-image-cost-statistics.mjs','point-history-statistics.mjs'])coverage.set(p,60);
 for(const[version,path]of [[61,'retained-statistics-v61-manifest.json'],[62,'proof-facts-monic-v62-manifest.json'],[63,'prototype-statistics-v63-manifest.json']])
  for(const r of json(path).sources.reads){assert(!coverage.has(r.path));coverage.set(r.path,version);}
 for(const r of sources.reads){assert(!coverage.has(r.path));coverage.set(r.path,64);}
 assert.deepEqual([...coverage.keys()].sort(),all.toSorted());
 const addressedKnownMatches=all.map(path=>({path,checkpoint:coverage.get(path)}));
 return{sources,gates:specs.map(([tag])=>tag),rawRows:a.rawRows,comparisons:a.totalComparisons,allocationRows:a.allocationRows,
  withdrawnAllocationIntervals:263,strata:checked.strata,eligibleRelations:checked.eligibleRelations,campaigns:checked.campaigns,
  allocations:checked.allocations,selected:checked.selected,controls:checked.controls,addressedKnownMatches,remainingKnownMatches:[],
  broaderPotentialMatches:json('prototype-statistics-v63-inventory-corrected.json').otherPotentialFiles,
  binaryInventory:{inspected:binaries.inspected,matching:2,nonmatching:16,scope:binaries.scope},
  analysisBytes:statSync('early-statistics-v64-analysis.json').size,newTemporaryBytes:0,
  next:'Semantically classify the twenty additional top-level text matches and reconcile historical statistical coverage beyond the specific 45-script signature. Continue point-candidate runtime/consumer/size work, power sums, remaining donor reads and the entire original exact-real reference inventory. No statistical or ecosystem completion claim.'};
}
