import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {cases} from './twelfth-cost-input-v78.mjs';
export {sha,json};
export const harnessFiles=['twelfth-cost-protocol-v78.md','twelfth-cost-input-v78.mjs','twelfth-cost-input-v78.json',
 'twelfth-cost-v78.rs','twelfth-cost-cpu-v78.rs','twelfth-cost-allocation-v78.rs','twelfth-counting-allocator-v78.rs','twelfth-cost-initial-v78.rs',
 'twelfth-cost-sources-v78.mjs','twelfth-cost-protocol-v78.mjs','build-twelfth-cost-v78.mjs',
 'qqbar-trig-hyper-v76/src/main.rs','power-sums-allocation.rs','paired-statistics-v60.mjs','capture.mjs','point-qualified-capture.mjs',
 ...['baseline','candidate'].map(v=>'twelfth-cost-'+v+'-app-v78/Cargo.toml')];
export function sources(){
 const previous=json('twelfth-relation-v77-manifest.json'),current=retainedSources(),b=json('twelfth-final-binding-v77.json');
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(b.current,current);for(const[p,h]of Object.entries(b.sources))assert.equal(sha(b.root+'/'+p),h,p);
 assert.deepEqual(json('twelfth-cost-input-v78.json'),cases());
 const allocator=readFileSync('power-sums-allocation.rs','utf8').split('include!')[0];assert.equal(readFileSync('twelfth-counting-allocator-v78.rs','utf8'),allocator);
 return {checkpoint:78,current,candidateRoot:b.root,candidateSources:b.sources,previousSha256:sha('twelfth-relation-v77-manifest.json'),
  files:Object.fromEntries(harnessFiles.map(p=>[p,sha(p)]))};
}
