import './verify-point-demand.mjs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointColdEvidence} from './point-cold-evidence.mjs';
const m=json('point-cold-manifest.json');assert.equal(m.checkpoint,58);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(await pointColdEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:58,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,successfulGates:m.successfulGates,preservedFailures:m.preservedFailures,liveFiles:956,sourceFiles:175,
 queries:m.semantic.totalQueries,independentChecks:m.semantic.totalChecks,crossPlatform:m.semantic.crossPlatform,
 stateChanges:m.semantic.stateChanges.map(s=>({platform:s.platform,variant:s.variant,group:s.group,changedCalls:s.changedCalls})),
 dedicatedBytes:m.dedicatedBytes,rawBytes:m.rawBytes,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope}));
