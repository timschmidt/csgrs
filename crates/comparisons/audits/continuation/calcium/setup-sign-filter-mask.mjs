import {mkdirSync,copyFileSync,symlinkSync,existsSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import assert from 'node:assert/strict';
import {signFilterSources,sha} from './sign-filter-sources.mjs';
const maps=signFilterSources(),root='sign-filter-mask';assert(!existsSync(root));mkdirSync(root);
for(const[p,h]of Object.entries(maps.baseline))if(p.startsWith('hyperlimit/')) {
 const source='sign-filter-baseline/'+p,target=root+'/'+p;assert.equal(sha(source),h);
 mkdirSync(dirname(target),{recursive:true});copyFileSync(source,target);assert.equal(sha(target),h);
}
for(const crate of ['hyperreal','hyperlattice','hypertri','hypercurve','hypersolve'])
 symlinkSync(resolve('sign-filter-baseline',crate),root+'/'+crate,'dir');
console.log(JSON.stringify({copiedFiles:Object.keys(maps.baseline).filter(p=>p.startsWith('hyperlimit/')).length,
 sourceFiles:Object.keys(maps.baseline).length,productionChanged:false}));
