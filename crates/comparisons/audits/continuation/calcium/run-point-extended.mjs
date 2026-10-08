import {statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
import {captured} from './point-qualified-capture.mjs';
const o=sources(),b=json('point-extended-binaries.json');
assert.equal(b.harnessSha256,sha('point-extended.rs'));assert.equal(b.wrapperSha256,sha('point-extended-cpu.rs'));
for(const f of b.artifacts){
 assert.equal(sha(f.path),f.sha256);assert.equal(statSync(f.path).size,f.bytes);
 await captured('point-extended-public-'+f.variant,'.',f.path,['check']);
}
assert.deepEqual(sources(),o);
console.log(JSON.stringify({status:'collection-completed',limits:'Full raw values preserved; mathematical and state-history qualification is separate and pending.'}));
