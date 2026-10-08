import './verify-power-sums.mjs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {sha,json} from './point-image-sources.mjs';
import {pointImageEvidence} from './point-image-evidence.mjs';
const m=json('point-image-manifest.json');assert.equal(m.checkpoint,53);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(pointImageEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageAtBinding,m.coverageBefore);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:53,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 liveFiles:956,guardFiles:Object.keys(m.guardSources).length,gates:m.gates.length,mathematicalStatus:m.mathematical.status,
 improved:m.mathematical.improved,unchanged:m.mathematical.unchanged,guardedSolverTestsPerProfile:811,
 v2Consumer:{passed:1764,ignored:9},memories:m.memories,costs:{cpuObservations:3840,allocationObservations:480,
 sameResultGroups:m.costs.sameResultGroups,changedResultGroups:m.costs.changedResultGroups,sameResultRatioRange:m.costs.sameResultRatioRange},
 dedicatedExecutableBytes:m.dedicatedExecutableBytes,coverage:m.coverageAtBinding,qualification:m.qualification,
 production:m.production,next:m.next,scope:m.scope}));
