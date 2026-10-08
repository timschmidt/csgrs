import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {retainedStatisticsEvidence} from './retained-statistics-evidence-v61.mjs';
import {checkRetainedStatistics} from './check-retained-statistics-v61.mjs';
const m=json('retained-statistics-v61-manifest.json');assert.equal(m.checkpoint,61);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(checkRetainedStatistics(),json('results/retained-statistics-check.stdout'));
for(const[k,v]of Object.entries(retainedStatisticsEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:61,status:'source-evidence-and-corrected-statistics-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,demandFiles:175,rawRows:m.rawRows,comparisons:m.comparisons,totals:m.totals,
 campaigns:m.campaigns,reviewedScripts:m.sources.reads.length,reviewedScriptLines:m.sources.reviewedScriptLines,
 recordedEvidence:m.recordedEvidence,controls:m.controls,newTemporaryBytes:0,coverage:m.coverageAtBinding,
 production:m.production,next:m.next,scope:m.scope,
 previousVerification:'Checkpoint 60 terminal output, bound artifacts and current sources rechecked; its full four-campaign reanalysis and the full historical 47-chain were not rerun.'}));
