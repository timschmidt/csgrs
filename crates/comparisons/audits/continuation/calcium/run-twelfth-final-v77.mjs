import assert from 'node:assert/strict';
import {retainedSources,sha,json,workspace} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const b=json('twelfth-final-binding-v77.json');
function sources(){assert.deepEqual(retainedSources(),b.current);for(const[p,h]of Object.entries(b.sources))assert.equal(sha(b.root+'/'+p),h,p);for(const[p,h]of Object.entries(b.files))assert.equal(sha(p),h,p);}
sources();const cwd=b.root+'/hyperreal';
await captured('twelfth-clippy-final-v77',cwd,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
await captured('twelfth-fmt-final-v77',cwd,'cargo',['fmt','--all','--','--check']);
await captured('twelfth-private-fmt-final-v77',cwd,'rustfmt',['--edition','2024','--check','src/computable/node/twelfth_relation.rs','src/computable/node/twelfth_relation_tests.rs']);
for(const variant of ['candidate','baseline'])for(const profile of ['debug','release'])for(const features of ['default','all']){
 sources();const args=[...cargoEnv,'cargo','test','--locked','--offline',...(profile==='release'?['--release']:[]),...(features==='all'?['--all-features']:[]),'--lib','--tests'];
 await captured('twelfth-'+variant+'-'+features+'-'+profile+'-final-v77',variant==='candidate'?cwd:workspace+'/hyperreal','env',args);sources();
}
for(const profile of ['debug','release']){
 await captured('twelfth-capability-'+profile+'-final-v77','twelfth-capability-v77','env',[...cargoEnv,'cargo','run','--locked','--offline',...(profile==='release'?['--release']:[])]);sources();
}
console.log(JSON.stringify({checkpoint:77,status:'matched-final-source-regression-and-capability-captures-finished',candidateFiles:183,retained:false}));
