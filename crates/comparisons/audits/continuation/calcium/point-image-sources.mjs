import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
export const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
export const json=p=>JSON.parse(readFileSync(p,'utf8'));
export function checkPointImageSources(){
 const b=json('point-image-binaries.json'),changed='hypersolve/src/algebraic_binary.rs',hashes={};
 for(const[p,h]of Object.entries(b.sourceHashes)){
  const path='point-image-candidate/'+p;hashes[p]=sha(path);
  if(p!==changed)assert.equal(hashes[p],h,p);
 }
 const prior=readFileSync('point-image-v1-algebraic_binary.rs','utf8');
 assert.equal(sha('point-image-v1-algebraic_binary.rs'),b.sourceHashes[changed]);
 const old='(3 * b.unsigned_abs()) as u64';assert.equal(prior.split(old).length,2);
 const current=readFileSync('point-image-candidate/'+changed,'utf8');
 assert.equal(current,prior.replace(old,'3 * b.unsigned_abs()'));
 assert.equal(prior.split('#[cfg(test)]')[0],current.split('#[cfg(test)]')[0]);
 return hashes;
}
