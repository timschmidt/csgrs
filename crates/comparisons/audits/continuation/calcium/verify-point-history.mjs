import './verify-point-extended.mjs';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointHistoryEvidence} from './point-history-evidence.mjs';
const m=json('point-history-manifest.json');assert.equal(m.checkpoint,56);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(await pointHistoryEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageBefore,m.coverageAtBinding);assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:56,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,qualificationChecks:84864,cpuObservations:36864,cpuPilots:1536,allocationObservations:4608,
 sameResult:m.costs.sameResult,changedResult:m.costs.changedResult,allocationCounts:m.costs.allocationCounts,
 dedicatedBytes:m.dedicatedBytes,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope}));
