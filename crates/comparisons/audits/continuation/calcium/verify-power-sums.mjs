import './verify-qqbar-arithmetic.mjs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {powerSumsEvidence,sha,json} from './power-sums-evidence.mjs';
const m=json('power-sums-manifest.json');assert.equal(m.checkpoint,52);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
const evidence=powerSumsEvidence();for(const[k,v]of Object.entries(evidence))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageAtBinding,m.coverageBefore);assert.equal(m.newDonorLines,0);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:52,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 liveFiles:956,candidateFiles:Object.keys(m.candidateSources).length,kernel:m.kernel,debugDefaultTests:m.debugDefaultTests,
 publicStatus:m.publicCheck.status,publicChecks:m.publicCheck.totalChecks,publicFailures:m.publicCheck.failures.length,
 publicCounts:m.publicCheck.counts,memory:m.memory,binaries:m.binaries,coverage:m.coverageAtBinding,
 qualification:m.qualification,production:m.production,scope:m.scope}));
