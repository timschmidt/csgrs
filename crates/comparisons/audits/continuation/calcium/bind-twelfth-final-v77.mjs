import {writeFileSync,readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './zero-factor-retained-sources-v75.mjs';
const before=json('twelfth-relation-binding-v77.json'),test='hyperreal/src/computable/node/twelfth_relation_tests.rs';
assert.deepEqual(retainedSources(),before.current);
for(const[p,h]of Object.entries(before.sources))if(p!==test)assert.equal(sha(before.root+'/'+p),h,p);
for(const[p,h]of Object.entries(before.files))assert.equal(sha(p),h,p);
assert.equal(sha('twelfth-relation-tests-preclippy-v77.rs'),before.sources[test]);
const after={...before,recorded:new Date().toISOString(),previousBindingSha256:sha('twelfth-relation-binding-v77.json'),
 sources:{...before.sources,[test]:sha(before.root+'/'+test)},
 files:{...before.files,'bind-twelfth-final-v77.mjs':sha('bind-twelfth-final-v77.mjs'),'twelfth-relation-tests-preclippy-v77.rs':sha('twelfth-relation-tests-preclippy-v77.rs')},
 newFiles:before.added.map(p=>({path:p,bytes:statSync(before.root+'/'+p).size,lines:readFileSync(before.root+'/'+p,'utf8').split('\n').length-1})),
 note:'Final qualification binding. Only the test oracle loop spelling changed for Clippy; production candidate is byte-identical. Earlier binding/test source and failed Clippy/driver captures preserved.'};
assert.notEqual(after.sources[test],before.sources[test]);
writeFileSync('twelfth-final-binding-v77.json',JSON.stringify(after,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:77,files:183,changedSinceInitialBinding:[test],newFiles:after.newFiles}));
