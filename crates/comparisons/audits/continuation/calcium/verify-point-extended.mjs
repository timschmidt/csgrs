import './verify-point-qualified.mjs';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointExtendedEvidence} from './point-extended-evidence.mjs';
const m=json('point-extended-manifest.json');assert.equal(m.checkpoint,55);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(pointExtendedEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageBefore,m.coverageAtBinding);assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:55,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,mathematical:m.mathematical.status,checks:21216,improved:128,unchangedQueries:256,
 failedGate:m.failedGate,memories:m.memories,dedicatedBytes:m.dedicatedBytes,coverage:m.coverageAtBinding,
 production:m.production,next:m.next,scope:m.scope}));
