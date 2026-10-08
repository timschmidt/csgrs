import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const b=json('twelfth-relation-binding-v77.json');
function sources(){assert.deepEqual(retainedSources(),b.current);for(const[p,h]of Object.entries(b.sources))assert.equal(sha(b.root+'/'+p),h,p);for(const[p,h]of Object.entries(b.files))assert.equal(sha(p),h,p);}
sources();const cwd=b.root+'/hyperreal';
for(const [tag,command,args] of [
 ['twelfth-fmt-v77','cargo',['fmt','--all','--','--check']],
 ['twelfth-private-fmt-v77','rustfmt',['--edition','2024','--check','src/computable/node/twelfth_relation.rs','src/computable/node/twelfth_relation_tests.rs']],
 ['twelfth-default-debug-v77','env',[...cargoEnv,'cargo','test','--locked','--offline','--lib','--tests']],
 ['twelfth-all-debug-v77','env',[...cargoEnv,'cargo','test','--locked','--offline','--all-features','--lib','--tests']],
 ['twelfth-all-release-v77','env',[...cargoEnv,'cargo','test','--locked','--offline','--release','--all-features','--lib','--tests']],
 ['twelfth-clippy-v77','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']],
]){await captured(tag,cwd,command,args);sources();}
for(const profile of ['debug','release']){
 await captured('twelfth-capability-'+profile+'-final-v77','twelfth-capability-v77','env',[...cargoEnv,'cargo','run','--locked','--offline',...(profile==='release'?['--release']:[])]);sources();
}
console.log(JSON.stringify({checkpoint:77,status:'final-source-regression-and-capability-captures-finished',candidateFiles:183,retained:false}));
