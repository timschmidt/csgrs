import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
export {sha,json};
export function demandSources(requireBinding=true){
 const old=sources(),o=json('point-demand-origin.json');
 assert.equal(o.templateOriginSha256,sha('point-qualified-origin.json'));
 assert.equal(o.predecessorSha256,sha(o.predecessor));assert.equal(o.template,old.candidate);
 assert.equal(o.sourceFiles.length,175);assert.deepEqual(Object.keys(o.originalSources),o.sourceFiles);
 const now={},changed=[];
 for(const p of o.sourceFiles){
  assert.equal(sha(o.template+'/'+p),o.originalSources[p],p+' template');
  now[p]=sha(o.root+'/'+p);if(now[p]!==o.originalSources[p])changed.push(p);
 }
 assert.deepEqual(changed.sort(),['hypersolve/Cargo.toml','hypersolve/src/algebraic_binary.rs']);
 let manifest=readFileSync(o.template+'/hypersolve/Cargo.toml','utf8');
 for(const crate of ['hyperreal','hyperlattice','hyperlimit'])manifest=manifest.replace('path = "../'+crate+'"','path = "../../'+old.baseline+'/'+crate+'"');
 assert.equal(readFileSync(o.root+'/hypersolve/Cargo.toml','utf8'),manifest);
 const binding={originSha256:sha('point-demand-origin.json'),root:o.root,sources:now,changed};
 if(requireBinding)assert.deepEqual(json('point-demand-source-binding.json'),binding);
 return binding;
}
