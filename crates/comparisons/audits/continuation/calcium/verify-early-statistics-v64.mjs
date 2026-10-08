import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {readRows} from './reanalyse-early-statistics-v64.mjs';
import {check} from './check-early-statistics-v64.mjs';
import {evidence} from './early-statistics-evidence-v64.mjs';
const m=json('early-statistics-v64-manifest.json');assert.equal(m.checkpoint,64);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(check(),readRows('results/early-statistics-check.stdout').at(-1));
for(const[k,v]of Object.entries(evidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:64,status:'source-evidence-and-corrected-statistics-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,pointCandidateFiles:175,rawRows:m.rawRows,comparisons:m.comparisons,allocationRows:m.allocationRows,
 withdrawnAllocationIntervals:263,strata:m.strata,eligibleRelations:m.eligibleRelations,controls:m.controls,selected:m.selected,
 addressedKnownMatches:m.addressedKnownMatches.length,remainingKnownMatches:m.remainingKnownMatches,broaderPotentialMatches:m.broaderPotentialMatches,
 newTemporaryBytes:0,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope,
 previousVerification:'Fourteen-checkpoint historical chain rerun; checkpoint 63 artifacts/current sources rechecked. No full 60/61/62/63 statistical recomputation or historical 47-chain rerun.'}));
