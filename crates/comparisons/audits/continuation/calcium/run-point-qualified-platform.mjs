import {statSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,json,sha} from './point-qualified-sources.mjs';
import {captured} from './point-qualified-capture.mjs';
const o=sources(),b=json('point-qualified-platform-binaries.json');
for(const variant of ['baseline','candidate']){
 const f=b.artifacts.find(f=>f.variant===variant&&f.platform==='native');
 assert.equal(statSync(f.path).size,f.bytes);assert.equal(sha(f.path),f.sha256);
 await captured('point-qualified-public-'+variant+'-native','.',f.path,[]);
 await captured('point-qualified-public-'+variant+'-wasm','.','node',['run-point-qualified-wasm.mjs',variant]);
}
await captured('point-qualified-public-check','.','node',['check-point-qualified-public.mjs']);
assert.deepEqual(sources(),o);
console.log(JSON.stringify({status:'pass',limits:'Full-value checks, not timed benchmark observations.'}));
