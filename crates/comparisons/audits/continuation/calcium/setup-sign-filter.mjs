import {readFileSync,writeFileSync,mkdirSync,copyFileSync,symlinkSync,existsSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
const previous=json('nfloat-complex-experiment.json'),maps=sources();
assert.deepEqual(maps,previous.candidateSources);
for(const[p,h]of Object.entries(maps.candidate))assert.equal(sha(resolve('../../../..',p)),h,p);
for(const variant of ['baseline','candidate']) {
 const root='sign-filter-'+variant;assert(!existsSync(root));mkdirSync(root);
 for(const[p,h]of Object.entries(maps.candidate))if(p.startsWith('hyperlimit/')) {
  const from='derivative-demand-candidate/'+p,to=root+'/'+p;assert.equal(sha(from),h);
  mkdirSync(dirname(to),{recursive:true});copyFileSync(from,to);assert.equal(sha(to),h);
 }
 for(const crate of ['hyperreal','hyperlattice','hypertri','hypercurve','hypersolve'])
  symlinkSync(resolve('derivative-demand-candidate',crate),root+'/'+crate,'dir');
}
writeFileSync('sign-filter-origin.json',JSON.stringify({recorded:new Date().toISOString(),
 origin:'derivative-demand-candidate',sourceHashes:maps.candidate},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({snapshots:2,sourceFiles:Object.keys(maps.candidate).length,
 copiedPerSnapshot:Object.keys(maps.candidate).filter(p=>p.startsWith('hyperlimit/')).length,
 productionChanged:false}));
