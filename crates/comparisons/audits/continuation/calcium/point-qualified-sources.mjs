import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './point-image-sources.mjs';
export {sha,json};
export function sources(){
 const o=json('point-qualified-origin.json');
 assert.equal(sha(o.predecessor),o.predecessorSha256);
 assert.equal(Object.keys(o.liveSources).length,956);
 assert.equal(Object.keys(o.candidateSources).length,956);
 const changed=[];
 for(const[p,h]of Object.entries(o.liveSources)){
  assert.equal(sha(resolve('../../../..',p)),h,'live '+p);
  assert.equal(sha(o.baseline+'/'+p),h,'baseline '+p);
  assert.equal(sha(o.candidate+'/'+p),o.candidateSources[p],'candidate '+p);
  if(h!==o.candidateSources[p])changed.push(p);
 }
 assert.deepEqual(changed,o.changed);
 assert.equal(sha(o.candidate+'/'+changed[0]),json(o.predecessor).guardSources[changed[0]]);
 return o;
}
