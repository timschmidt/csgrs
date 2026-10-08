import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {plan,digest} from './snapshot-v43-spec.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
assert.equal(process.cwd(),here,'Run from the calcium audit directory.');
assert.deepEqual(process.argv.slice(2),['--e-plan-live'],'Historical live flags do not qualify current sources.');
const json=p=>JSON.parse(readFileSync(p,'utf8')),sha=p=>digest(readFileSync(p));
const m=json('e-qualified-retention-verification.json'),q=json('e-qualified-experiment.json'),p=plan();
for(const[f,h]of Object.entries(m.files))assert.equal(sha(f),h,f);
assert.equal(m.historicalSnapshot,'derivative-demand-candidate');
assert.deepEqual(m.historicalSources,q.sourceMap.baseline);
assert.deepEqual(m.liveSources,q.sourceMap.candidate);
assert.equal(Object.keys(m.historicalSources).length,955);
assert.equal(Object.keys(m.liveSources).length,956);
assert.deepEqual(m.modules,p.modules.map(({source,...r})=>r));assert.equal(m.entry,p.entry);
for(const g of p.modules) {
 assert.equal(sha(g.original),g.originalSha256,g.original);
 assert.equal(sha(g.path),g.sha256,g.path);
 assert.equal(readFileSync(g.path,'utf8'),g.source,g.path);
}
for(const[f,h]of Object.entries(m.historicalSources))assert.equal(sha(resolve(m.historicalSnapshot,f)),h,f);
for(const[f,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,f)),h,f);
await import('./'+m.entry);
// Repeat the live identity checks after every historical and current check runs.
for(const[f,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,f)),h,f);
console.log(JSON.stringify({binding:'Checkpoint 43 historical/current source separation',
 historicalFiles:955,liveFiles:956,versionedModules:m.modules.length,
 generatedBytes:m.modules.reduce((s,g)=>s+g.bytes,0),
 numericalAssertionsChanged:0,originalArtifactsChanged:0,
 status:'All 43 checkpoint verifiers passed; historical sources and current retention checked separately.'}));
