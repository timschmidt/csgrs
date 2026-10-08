import './verify-point-image.mjs';
import assert from 'node:assert/strict';
import {effectiveSummary} from './effective-coverage.mjs';
import {sha,json} from './point-qualified-sources.mjs';
import {pointQualifiedEvidence} from './point-qualified-evidence.mjs';
const m=json('point-qualified-manifest.json');assert.equal(m.checkpoint,54);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(pointQualifiedEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageAtBinding,m.coverageBefore);assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);
 assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:54,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,
 gates:m.gates.length,liveFiles:956,consumer:m.consumer,strict:m.mathematical.status,approximate:m.approximateMathematical.status,
 applicationSizes:m.applicationSizes.map(({crate,example,byteDeltas})=>({crate,example,byteDeltas})),platformSizes:m.platformSizes,
 dedicatedBytes:m.dedicatedBytes,coverage:m.coverageAtBinding,production:m.production,next:m.next,scope:m.scope}));
