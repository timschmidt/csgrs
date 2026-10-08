import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json,sha,demandSources} from './point-demand-sources.mjs';
import {wasmBindings} from './point-wasm-protocol.mjs';
import {statisticsSelfTest,config} from './paired-statistics-v60.mjs';
import {legacySamplerInventory,campaigns} from './reanalyse-point-statistics-v60.mjs';
export function statisticsEvidence(){
 demandSources();wasmBindings();const analysis=json('point-statistics-v60-analysis.json');
 assert.equal(analysis.status,'pass');assert.deepEqual(analysis.config,config);assert.deepEqual(analysis.tests,statisticsSelfTest());
 assert.deepEqual(analysis.inventory,legacySamplerInventory());assert.equal(analysis.totalRows,151296);assert.equal(analysis.totalComparisons,3920);
 for(const c of analysis.campaigns){assert.equal(sha(c.summary),c.summarySha256);assert.equal(sha(c.raw),c.rawSha256);}
 const specs=[
  ['point-statistics-selftest','node',['test-paired-statistics-v60.mjs']],
  ['point-statistics-reanalysis','node',['reanalyse-point-statistics-v60.mjs']],
  ['point-statistics-check','node',['check-point-statistics-v60.mjs']],
  ['point-statistics-capacity','df',['-B1','/tmp','.']],
 ];
 const gates={};for(const[tag,command,args]of specs){const g=json('results/'+tag+'.json');gates[tag]=g;
  for(const[k,v]of Object.entries({tag,cwd:resolve('.'),command,args,code:0,signal:null}))assert.deepEqual(g[k],v,tag+' '+k);
  assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));
  assert.equal(statSync('results/'+tag+'.stderr').size,0);
 }
 assert(Date.parse(gates['point-statistics-reanalysis'].finished)<=Date.parse(gates['point-statistics-check'].started));
 assert.deepEqual(json('results/point-statistics-selftest.stdout'),analysis.tests);
 const checked={status:'pass',rawRows:analysis.totalRows,comparisons:analysis.totalComparisons,tests:analysis.tests,
  campaigns:analysis.campaigns.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts})),inventoryMatches:analysis.inventory.count,limits:analysis.limits};
 assert.deepEqual(json('results/point-statistics-check.stdout'),checked);
 const highlights=analysis.campaigns.flatMap(c=>c.groups.filter(g=>
  (c.id==='wasm59'&&((g.case===17&&g.policy===0&&g.history===2&&g.lifecycle==='fresh')
   ||(g.case===44&&g.policy===1&&g.history===3&&g.lifecycle==='retained')
   ||(g.case===37&&g.policy===1&&g.history===0&&g.lifecycle==='retained')
   ||(g.case===3&&g.policy===1&&g.history===3&&g.lifecycle==='retained'))))
  .map(g=>({campaign:c.id,...g})));
 return{gates:specs.map(s=>s[0]),sourceBindingSha256:sha('point-demand-source-binding.json'),sourceFiles:175,
  config,tests:analysis.tests,rawRows:analysis.totalRows,comparisons:analysis.totalComparisons,
  campaigns:analysis.campaigns.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts})),highlights,
  inventory:analysis.inventory,analysisBytes:statSync('point-statistics-v60-analysis.json').size,
  campaignBindings:campaigns.map(c=>({id:c.id,summarySha256:sha(c.summary),rawSha256:sha(c.raw)})),
  newTemporaryBytes:0,
  next:'Correct the remaining historical intervals and reassess their inference/retention implications; the potential-impact inventory is not closure. Then perform the disclosed bounded wall/process/thread-CPU diagnostic replay before final selected-version consumer/size and retention gates. No new candidate algorithm or replay has been launched. Separate power-sum work and the full original ecosystem inventory remain open.'};
}
