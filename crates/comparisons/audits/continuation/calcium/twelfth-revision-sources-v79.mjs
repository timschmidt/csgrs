import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './zero-factor-retained-sources-v75.mjs';
export {sha,json};
export const bindingPath='twelfth-revision-binding-v79.json';
const added='hyperreal/src/computable/node/twelfth_relation_reference_tests.rs';
const changed=['hyperreal/src/computable/node.rs','hyperreal/src/computable/node/twelfth_relation.rs'];
export function snapshot(){
 const origin=json('twelfth-revision-origin-v79.json'),prior=json('twelfth-cost-v78-manifest.json');
 assert.equal(sha('twelfth-cost-v78-manifest.json'),origin.previousManifestSha256);
 assert.equal(sha('prepare-twelfth-revision-v79.mjs'),origin.prepareSha256);
 for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
 assert.deepEqual(retainedSources(),origin.current);assert.deepEqual(prior.candidateSources,origin.originalSources);
 const files=[...Object.keys(origin.originalSources),added].sort(),sources={};
 for(const p of files){sources[p]=sha(origin.root+'/'+p);if(!changed.includes(p)&&p!==added)assert.equal(sources[p],origin.originalSources[p],p);}
 for(const p of changed)assert.notEqual(sources[p],origin.originalSources[p]);
 const read=p=>readFileSync(p,'utf8');
 const original=read(origin.originalRoot+'/hyperreal/src/computable/node.rs');
 assert.equal(read(origin.root+'/hyperreal/src/computable/node.rs'),original.replace('include!("node/twelfth_relation_tests.rs");',
  'include!("node/twelfth_relation_tests.rs");\ninclude!("node/twelfth_relation_reference_tests.rs");'));
 const reference=read(origin.root+'/'+added).split('    fn raw(')[0].replace('#[cfg(test)]\nmod twelfth_relation_reference_tests {\n    use super::*;\n','');
 const old=read(origin.originalRoot+'/hyperreal/src/computable/node/twelfth_relation.rs').split('\nimpl Computable {')[0];
 // Nesting the frozen source makes rustfmt wrap exactly this long match arm
 // with a block and remove its optional comma. Permit only that known change.
 const expected=old.replace(/\s/g,'').replace('Approximation::Constant(SharedConstant::Tau)=>(Rational::zero(),Rational::new(2)),',
  'Approximation::Constant(SharedConstant::Tau)=>{(Rational::zero(),Rational::new(2))}');
 assert.equal(reference.replace(/\s/g,''),expected,'frozen evaluator reference differs beyond the known rustfmt arm wrapping');
 assert.equal(files.length,184);
 return {checkpoint:79,root:origin.root,current:origin.current,originSha256:sha('twelfth-revision-origin-v79.json'),sources,changed,added:[added],
  details:[...changed,added].map(p=>({path:p,bytes:statSync(origin.root+'/'+p).size,lines:read(origin.root+'/'+p).split('\n').length-1})),
  files:Object.fromEntries(['prepare-twelfth-revision-v79.mjs','twelfth-revision-sources-v79.mjs','twelfth-revision-sources-initial-v79.mjs','twelfth-reference-initial-v79.rs',
   'twelfth-capability-v79/Cargo.toml','twelfth-capability-v79/src/main.rs','twelfth-cost-revision-app-v79/Cargo.toml'].map(p=>[p,sha(p)]))};
}
export function sources(){const b=json(bindingPath);assert.deepEqual(snapshot(),b);return b;}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 assert.deepEqual(process.argv.slice(2),['--record']);const b=snapshot();writeFileSync(bindingPath,JSON.stringify(b,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:79,status:'source-bound',candidateFiles:184,changed:b.changed,added:b.added,details:b.details}));
}
