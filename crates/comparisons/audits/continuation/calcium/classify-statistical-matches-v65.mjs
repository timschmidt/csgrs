import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {verifyInventory} from './prototype-statistics-inventory-v63.mjs';

// Human semantic review of every line; this table is not a regex proof that
// arbitrarily spelled estimators cannot exist elsewhere.
const findings={
 'bind-complex-product-v2.mjs':[63,'Calls the v2 checker; classifies existing selected/bypass intervals; historical limits prose.'],
 'bind-e-qualified.initial.mjs':[61,'Calls checkEQualified; bootstrap match is historical comparison/limits prose.'],
 'bind-e-qualified.mjs':[61,'Calls checkEQualified; bootstrap match is historical comparison/limits prose.'],
 'bind-rank-costs.mjs':[63,'Binds existing rank data; bootstrap match is limits prose, not an estimator.'],
 'bind-sign-filter-mask.mjs':[63,'Calls checkSignFilterMask; bootstrap match is historical limits prose.'],
 'bind-sign-filter.mjs':[63,'Calls checkSignFilter; bootstrap match is historical limits prose.'],
 'check-point-cold.mjs':[null,'Seed refers to constructor/refinement cache state; full-record semantic checks, no timing estimator.'],
 'check-point-demand-costs.mjs':[60,'Calls point-demand-cost-protocol summarizer and classifies existing intervals.'],
 'check-point-history.mjs':[60,'Calls point-history-statistics summarizer and classifies existing intervals.'],
 'check-point-image-costs.mjs':[60,'Calls point-image-cost-statistics summarizer and classifies existing intervals.'],
 'check-point-wasm-costs.mjs':[60,'Calls point-wasm-protocol re-export of point-demand-cost-protocol summarizer; classifies existing intervals.'],
 'prototype-statistics-v63-inventory.mjs':[null,'Matches its own scanning patterns; hashes and text inventory, no sampling.'],
 'run-point-demand-costs.mjs':[60,'Collects measurements and delegates statistics to point-demand-cost-protocol; limits prose.'],
 'run-point-history-costs.mjs':[60,'Collects measurements and delegates statistics to point-history-statistics; limits prose.'],
 'run-point-image-costs.mjs':[60,'Collects measurements and delegates statistics to point-image-cost-statistics; limits prose.'],
 'run-point-wasm-costs.mjs':[60,'Collects measurements and uses point-wasm-protocol re-exported demand summarizer; limits prose.'],
 'snapshot-v43-verify-complex-product-v2.mjs':[63,'Snapshot-bound checker call and existing-interval endpoint classifier, not a new sampler.'],
 'verify-charpoly-domain.mjs':[null,'Seed is a deterministic fixture dimension (0,1), reconstructing correctness records.'],
 'verify-complex-product-v2.mjs':[63,'Calls checkComplexProduct and classifies existing selected/bypass intervals.'],
 'verify-matrix-solves.mjs':[null,'Seed is a deterministic fixture dimension (0,1,2), reconstructing correctness records.'],
};
export function classification(){
 const inventory=json('prototype-statistics-v63-inventory-corrected.json');verifyInventory(inventory);
 const previous=json('early-statistics-v64-manifest.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(Object.keys(findings).sort(),inventory.otherPotentialFiles);
 assert.deepEqual(inventory.otherPotentialFiles,previous.broaderPotentialMatches);
 const files=inventory.otherPotentialFiles.map(path=>{
  const f=inventory.files.find(f=>f.path===path),[correctedAt,note]=findings[path];
  return{path,sha256:f.sha256,readRanges:[[1,f.lines]],lines:f.lines,
   matchedLines:f.matches.map(m=>m.line),additionalEstimator:false,correctedAt,note};
 });
 const original=readFileSync('verify-complex-product-v2.mjs','utf8');
 const rebound=original.replace('./verify-complex-product.mjs','./snapshot-v43-verify-complex-product.mjs')
  .replace('./complex-product-v2-sources.mjs','./snapshot-v43-complex-product-v2-sources.mjs')
  .replace('./check-complex-product-v2.mjs','./snapshot-v43-check-complex-product-v2.mjs')
  .replaceAll("resolve(here,'../../../..',r.path)","resolve(here,'derivative-demand-candidate',r.path)");
 assert.equal(rebound,readFileSync('snapshot-v43-verify-complex-product-v2.mjs','utf8'));
 assert.equal(files.length,20);assert.equal(files.reduce((n,f)=>n+f.lines,0),1371);
 return{checkpoint:65,inventorySha256:sha('prototype-statistics-v63-inventory-corrected.json'),
  previousSha256:sha('early-statistics-v64-manifest.json'),files,readLines:1371,additionalEstimators:0,
  scope:'All 20 additional text matches in the frozen 370-file inventory. Together with 45 known matches mapped in 60–64 this resolves 65 matched files. The 305 nonmatches were not thereby semantically reviewed; subdirectories and the whole workspace are not covered. Historical bound scripts remain unchanged; their old outputs are superseded only by scoped 60–64 corrections. No donor read credit.'};
}
const result=classification();
if(process.argv.includes('--record'))writeFileSync('statistical-matches-v65.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,json('statistical-matches-v65.json'));
console.log(JSON.stringify({checkpoint:65,classifiedFiles:result.files.length,readLines:result.readLines,additionalEstimators:0}));
