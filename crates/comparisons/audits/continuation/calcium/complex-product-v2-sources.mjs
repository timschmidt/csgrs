import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
export const here=dirname(fileURLToPath(import.meta.url));
export const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
export const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
export function sources() {
 const origin=json('complex-product-v2-origin.json'), old=json('complex-product-origin.json');
 const hashes={},changed=[];assert.equal(origin.copiedFiles,955);
 for(const[p,h]of Object.entries(origin.baselineHashes)) {
  assert.equal(sha(resolve(here,'../../../..',p)),h,p);
  assert.equal(sha(old.origin+'/'+p),h,p);
  assert.equal(sha(origin.origin+'/'+p),origin.sourceHashes[p],p);
  const ch=sha(origin.destination+'/'+p);hashes[p]=ch;if(ch!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hyperreal/src/rational/arithmetic/aggregate_products.rs']);
 return{baseline:origin.baselineHashes,candidate:hashes,changed};
}
