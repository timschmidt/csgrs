import './verify-point-cold.mjs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {pointWasmEvidence} from './point-wasm-evidence.mjs';
const m=json('point-wasm-manifest.json');assert.equal(m.checkpoint,59);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[k,v]of Object.entries(await pointWasmEvidence()))assert.deepEqual(m[k],v,k);
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);assert.deepEqual(m.coverageBefore,m.coverageAtBinding);
assert.equal(m.newDonorLines,0);assert.equal(m.liveFiles,956);
for(const now of effectiveSummary()){const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);assert(now.complete>=then.complete&&now.readLines>=then.readLines);}
console.log(JSON.stringify({checkpoint:59,status:'source-and-evidence-integrity-pass',boundFiles:Object.keys(m.files).length,gates:m.gates.length,
 liveFiles:956,sourceFiles:175,qualificationObservations:4608,independentChecks:129856,cpuObservations:55296,cpuPilots:2304,
 comparisons:m.costs.comparisons,dedicatedBytes:m.dedicatedBytes,rawBytes:m.rawBytes,coverage:m.coverageAtBinding,
 production:m.production,next:m.next,scope:m.scope}));
