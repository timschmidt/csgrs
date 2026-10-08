import './verify-point-history.mjs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointDemandEvidence} from './point-demand-evidence.mjs';
const m=json('point-demand-manifest.json');assert.equal(m.checkpoint,57);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(await pointDemandEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:57,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,gates:m.gates.length,
 liveFiles:956,sourceFiles:175,solverTests:m.solverTests,publicStatus:m.publicChecks.status,historyChecks:44992,
 cpuObservations:55296,cpuPilots:2304,allocationObservations:6912,
 comparisons:Object.fromEntries(Object.entries(m.costs.comparisons).map(([k,c])=>[k,{sameResult:c.sameResult,changedResult:c.changedResult,
  allocationCounts:c.allocationCounts,allocationDeltaRanges:c.allocationDeltaRanges}])),
 dedicatedBytes:m.dedicatedBytes,memories:m.memories,lineDeltas:m.lineDeltas,coverage:m.coverageAtBinding,
 production:m.production,next:m.next,scope:m.scope}));
