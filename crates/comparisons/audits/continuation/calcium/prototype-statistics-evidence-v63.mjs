import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence,readRows} from './reanalyse-prototype-statistics-v63.mjs';
export const captureSpecs=[
 ['prototype-statistics-inventory','node',['prototype-statistics-v63-inventory.mjs'],0,'incomplete-inventory'],
 ['prototype-statistics-historical-rank','node',['verify-rank-costs.mjs'],0,'qualified-archival-integrity-only'],
 ['prototype-statistics-inventory-corrected','node',['prototype-statistics-inventory-v63.mjs'],1,'failed-overrestrictive-added-membership-assertion'],
 ['prototype-statistics-inventory-corrected-confirm','node',['prototype-statistics-inventory-v63.mjs'],0,'missing-output-not-qualified'],
 ['prototype-statistics-ring-oracle','node',['sign-filter-ring-oracle.mjs'],0,'missing-output-not-qualified'],
 ['prototype-statistics-inventory-qualified','node',['--input-type=module','-e',
  'import {verifyInventory} from "./prototype-statistics-inventory-v63.mjs"; import {json} from "./point-demand-sources.mjs"; console.log(JSON.stringify(verifyInventory(json("prototype-statistics-v63-inventory-corrected.json"))));'],0,'qualified-bounded-inventory'],
 ['prototype-statistics-ring-oracle-qualified','node',['sign-filter-ring-oracle.mjs'],0,'qualified-exact-sign-oracle'],
 ['prototype-statistics-reanalysis','node',['reanalyse-prototype-statistics-v63.mjs'],0,'qualified-reanalysis'],
 ['prototype-statistics-check','node',['check-prototype-statistics-v63.mjs'],0,'qualified-check'],
 ['prototype-statistics-capacity','df',['-B1','/tmp','.'],0,'qualified-capacity'],
];
export function evidence(){
 const sources=sourceEvidence(),a=json('prototype-statistics-v63-analysis.json');assert.equal(a.status,'pass');assert.deepEqual(a.sources,sources);
 for(const c of [...a.cpu,...a.allocation]){assert.equal(sha(c.summary),c.summarySha256);assert.equal(sha(c.raw),c.rawSha256);}
 const records={};for(const[tag,command,args,code,disposition]of captureSpecs){
  const g=json('results/'+tag+'.json');
  for(const[k,v]of Object.entries({tag,command,args,cwd:resolve('.'),code,signal:null}))assert.deepEqual(g[k],v,tag+' '+k);
  assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
  const stdout=readFileSync('results/'+tag+'.stdout','utf8'),stderr=readFileSync('results/'+tag+'.stderr','utf8');
  if(code===0)assert.equal(stderr,'');else assert.match(stderr,/ERR_ASSERTION/);
  if(disposition==='missing-output-not-qualified')assert.equal(stdout,'');
  else if(code===0)assert(stdout.length>0);
  records[tag]={...g,disposition};
 }
 assert.deepEqual(json('results/prototype-statistics-inventory.stdout'),{files:356,matchingFiles:60,knownLcgMatches:45,otherPotentialFiles:17,
  scope:'Frozen membership of top-level continuation .mjs files, excluding version-suffixed correction tools. Broad text matches, not an exhaustive workspace scan or semantic estimator classification. Future new files do not retroactively alter this snapshot.'});
 assert.deepEqual(json('results/prototype-statistics-inventory-qualified.stdout'),sources.inventory);
 const oracle=readFileSync('results/prototype-statistics-ring-oracle-qualified.stdout','utf8');
 assert.equal(oracle,readFileSync('results/sign-filter-ring-oracle.stdout','utf8'));assert.equal(JSON.parse(oracle).cases,24);
 const old=readFileSync('results/prototype-statistics-historical-rank.stdout','utf8'),checkedText=readFileSync('results/prototype-statistics-check.stdout','utf8');
 assert.equal(readRows('results/prototype-statistics-historical-rank.stdout').length,23);assert(checkedText.startsWith(old));
 const checkedRows=readRows('results/prototype-statistics-check.stdout');assert.equal(checkedRows.length,24);const checked=checkedRows.at(-1);
 assert.equal(checked.status,'pass');assert.equal(checked.controls.rejectedMutations.length,30);
 assert.deepEqual(checked.aggregate,{changedIntervals:486,lostDirectional:34,gainedDirectional:0,reversedDirectional:0});
 assert.equal(checked.rawRows,a.rawRows);assert.equal(checked.comparisons,a.totalComparisons);assert.equal(checked.allocationRows,a.allocationRows);
 assert.deepEqual(checked.campaigns,a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts,strata:c.strata})));
 assert.deepEqual(checked.allocations,a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,deltas:c.deltas})));
 const expected=[];for(const c of a.cpu){
  expected.push({id:c.id,mode:'cpu',rawRows:c.rawRows,counts:c.counts,strata:c.strata});
  const alloc=a.allocation.find(g=>g.id===c.id);expected.push({id:alloc.id,mode:'allocation',rawRows:alloc.rawRows,groups:alloc.groups.length,deltas:alloc.deltas});
 }
 expected.push({status:'pass',rawRows:25728,comparisons:536,allocationRows:3216,output:'prototype-statistics-v63-analysis.json'});
 assert.deepEqual(readRows('results/prototype-statistics-reanalysis.stdout'),expected);
 assert(Date.parse(records['prototype-statistics-reanalysis'].finished)<Date.parse(records['prototype-statistics-check'].started));
 const prior=json('proof-facts-monic-v62-manifest.json'),remainingMatches=prior.remainingMatches.filter(p=>!sources.reads.some(r=>r.path===p));
 assert.equal(remainingMatches.length,9);
 return{sources,captures:captureSpecs.map(([tag,,,code,disposition])=>({tag,code,disposition})),
  qualifiedGates:captureSpecs.filter(s=>s[4].startsWith('qualified')).map(s=>s[0]),
  rawRows:a.rawRows,comparisons:a.totalComparisons,allocationRows:a.allocationRows,aggregate:checked.aggregate,
  campaigns:checked.campaigns,allocations:checked.allocations,complexV2HistoricalDispatchStrata:checked.complexV2HistoricalDispatchStrata,
  unresolvedRankWidth32Retained:checked.unresolvedRankWidth32Retained,controls:checked.controls,historical:checked.historical,
  remainingMatches,analysisBytes:statSync('prototype-statistics-v63-analysis.json').size,newTemporaryBytes:0,
  next:'Review the remaining nine known sampler matches and the twenty other potential text matches, without confusing a top-level inventory with workspace-wide semantic closure. Continue point-candidate runtime/consumer/size qualification, power sums, remaining donor reads and the complete original reference inventory.'};
}
