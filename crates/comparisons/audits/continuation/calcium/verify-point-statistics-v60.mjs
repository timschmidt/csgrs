import './verify-point-wasm.mjs';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {statisticsEvidence} from './point-statistics-evidence-v60.mjs';
import {reanalysePointStatistics} from './reanalyse-point-statistics-v60.mjs';
const m=json('point-statistics-v60-manifest.json');assert.equal(m.checkpoint,60);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(await reanalysePointStatistics(),json('point-statistics-v60-analysis.json'));
for(const[k,v]of Object.entries(statisticsEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:60,status:'source-evidence-and-corrected-statistics-pass',boundFiles:Object.keys(m.files).length,gates:m.gates.length,
 liveFiles:956,sourceFiles:175,rawRows:m.rawRows,comparisons:m.comparisons,campaigns:m.campaigns,
 inventoryMatches:m.inventory.count,newTemporaryBytes:0,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope}));
