import {readFileSync,writeFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {retainedSources,sha,json,workspace} from './zero-factor-retained-sources-v75.mjs';
const current=retainedSources(),o=json('twelfth-relation-origin-v77.json');
assert.deepEqual(current,o.current);assert.equal(sha('qqbar-trig-v76-manifest.json'),o.previousManifestSha256);
const changed=['hyperreal/src/computable/node.rs','hyperreal/src/computable/node/structural_analysis.rs'];
const added=['hyperreal/src/computable/node/twelfth_relation.rs','hyperreal/src/computable/node/twelfth_relation_tests.rs'];
const sources={};
for(const[p,h]of Object.entries(o.originalSources)){
 assert.equal(sha(workspace+'/'+p),h,p);sources[p]=sha(o.root+'/'+p);
 if(changed.includes(p))assert.notEqual(sources[p],h,p);else assert.equal(sources[p],h,p);
}
for(const p of added)sources[p]=sha(o.root+'/'+p);
const originalNode=readFileSync(workspace+'/'+changed[0],'utf8'),candidateNode=readFileSync(o.root+'/'+changed[0],'utf8');
assert.equal(candidateNode,originalNode.replace('include!("node/exp_relation_reuse.rs");','include!("node/exp_relation_reuse.rs");\ninclude!("node/twelfth_relation.rs");')
 .replace('include!("node/cache_rescale_tests.rs");','include!("node/cache_rescale_tests.rs");\ninclude!("node/twelfth_relation_tests.rs");'));
assert.equal(readFileSync(o.root+'/'+changed[1],'utf8'),readFileSync(workspace+'/'+changed[1],'utf8')
 .replace('}.or_else(|| node.exact_positive_exp_difference_sign());','}.or_else(|| node.exact_positive_exp_difference_sign())\n                     .or_else(|| node.exact_twelfth_relation_sign());'));
const files=['twelfth-relation-protocol-v77.md','prepare-twelfth-relation-v77.mjs','twelfth-relation-origin-v77.json',
 'bind-twelfth-relation-v77.mjs','twelfth-relation-tests-initial-v77.rs','twelfth-relation-tests-focused-v77.rs',
 'twelfth-capability-v77/Cargo.toml','twelfth-capability-v77/Cargo.lock','twelfth-capability-v77/src/main.rs','qqbar-trig-hyper-v76/src/main.rs'];
const binding={checkpoint:77,recorded:new Date().toISOString(),root:o.root,current,changed,added,sources,
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),originalFiles:181,candidateFiles:183,
 newFiles:added.map(p=>({path:p,bytes:statSync(o.root+'/'+p).size,lines:readFileSync(o.root+'/'+p,'utf8').split('\n').length-1})),
 note:'Final initial qualification source binding. Earlier development captures are historical, not final-source qualification. No live mutation or new donor credit.'};
writeFileSync('twelfth-relation-binding-v77.json',JSON.stringify(binding,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:77,root:o.root,files:183,changed,added,newFiles:binding.newFiles,current}));
