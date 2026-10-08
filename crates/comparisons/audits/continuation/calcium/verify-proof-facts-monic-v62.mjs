import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {readRows} from './reanalyse-proof-facts-monic-v62.mjs';
import {check} from './check-proof-facts-monic-v62.mjs';
import {evidence} from './proof-facts-monic-evidence-v62.mjs';
const m=json('proof-facts-monic-v62-manifest.json');assert.equal(m.checkpoint,62);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(check(),readRows('results/proof-facts-monic-check.stdout').at(-1));
for(const[k,v]of Object.entries(evidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:62,status:'source-evidence-and-corrected-statistics-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,pointCandidateFiles:175,rawRows:m.rawRows,comparisons:m.comparisons,eligible:m.eligible,
 rejectedHistoricalOnly:m.rejectedHistoricalOnly,allocationRows:m.allocationRows,withdrawnAllocationIntervals:m.withdrawnAllocationIntervals,
 campaigns:m.campaigns,controls:m.controls,monicMetrics:m.monicMetrics,remainingMatches:m.remainingMatches,
 newTemporaryBytes:0,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope,
 previousVerification:'The historical 18-checkpoint verifier ran; checkpoint 61 bound artifacts/current sources rechecked. Checkpoints 60/61 full statistical recomputation and the historical 47-chain were not rerun.'}));
