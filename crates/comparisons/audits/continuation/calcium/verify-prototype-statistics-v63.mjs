import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {readRows} from './reanalyse-prototype-statistics-v63.mjs';
import {check} from './check-prototype-statistics-v63.mjs';
import {evidence} from './prototype-statistics-evidence-v63.mjs';
const m=json('prototype-statistics-v63-manifest.json');assert.equal(m.checkpoint,63);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(check(),readRows('results/prototype-statistics-check.stdout').at(-1));
for(const[k,v]of Object.entries(evidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:63,status:'source-evidence-and-corrected-statistics-pass',boundFiles:Object.keys(m.files).length,
 captures:m.captures.length,qualifiedGates:m.qualifiedGates.length,liveFiles:956,pointCandidateFiles:175,rawRows:m.rawRows,
 comparisons:m.comparisons,allocationRows:m.allocationRows,aggregate:m.aggregate,campaigns:m.campaigns,controls:m.controls,
 complexV2HistoricalDispatchStrata:m.complexV2HistoricalDispatchStrata,unresolvedRankWidth32Retained:m.unresolvedRankWidth32Retained,
 remainingMatches:m.remainingMatches,newTemporaryBytes:0,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope,
 previousVerification:'Historical 23-checkpoint rank chain plus four complex/sign checkers rerun; checkpoint 62 artifacts/current sources rechecked. No full checkpoint 60/61/62 statistical recomputations or historical 47-chain rerun.'}));
