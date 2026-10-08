import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {cases,selfTest} from './quadratic-extraction-oracle-v82.mjs';
const o=json('quadratic-extraction-origin-v82.json'),g=json('results/quadratic-extraction-prepare-v82.json');
assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.command,'node');assert.deepEqual(g.args,['prepare-quadratic-extraction-v82.mjs']);
assert(Date.parse(o.recorded)>=Date.parse(g.started)&&Date.parse(o.recorded)<=Date.parse(g.finished));
assert.deepEqual(retainedSources(),o.current);assert.equal(sha('scalar-boundary-v81-manifest.json'),o.previousSha256);
for(const[p,h]of Object.entries({...o.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);
for(const[k,h]of Object.entries(o.donorSources)){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),h,p);}
assert.deepEqual(json('quadratic-extraction-input-v82.json'),cases());assert.deepEqual(o.oracle,selfTest());
assert.equal(readFileSync('quadratic-extraction-input-v82.tsv','utf8'),cases().map(c=>[c.id,c.a,c.b,BigInt(c.d)*BigInt(c.s)**2n,c.q].join('\t')).join('\n')+'\n');
console.log(JSON.stringify({checkpoint:82,status:'prepared-files-confirmed',recorded:o.recorded,files:o.records.length,lines:644,cases:775,dir:o.dir,
 note:'Exclusive preparation is not replayed. Its zero-exit empty stdout is unusable as a result record; independently validate all written files and the captured preparation time.'}));
