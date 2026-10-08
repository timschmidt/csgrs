import {readFileSync,writeFileSync,mkdirSync,copyFileSync,constants,statSync,readdirSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-wasm-cost-sources-v73.mjs';
export {sha,json};
export const root='zero-factor-consumer-v74',crate=root+'/hypercurve',baseline='point-demand-consumer-v66/hypercurve';
const originPath='zero-factor-consumer-origin-v74.json',bindingPath='zero-factor-consumer-binding-v74.json';
export function previousSources(){
 const prior=json('zero-factor-wasm-cost-v73-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
 const current=costSources();assert.deepEqual(current,json('zero-factor-wasm-cost-origin-v73.json').source);
 const g=json('results/zero-factor-wasm-cost-verify-v73.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 const consumer=json('point-consumer-binding-v66.json');assert.equal(consumer.root,'point-demand-consumer-v66');assert.equal(Object.keys(consumer.sources).length,355);
 for(const[p,h]of Object.entries(consumer.sources))assert.equal(sha(consumer.root+'/'+p),h,p);
 return {current,previousSha256:sha('zero-factor-wasm-cost-v73-manifest.json'),consumerBindingSha256:sha('point-consumer-binding-v66.json'),consumer};
}
export function expectedManifest(){
 const text=readFileSync(baseline+'/Cargo.toml','utf8'),before='../../point-demand-candidate/hypersolve';
 assert.equal(text.split(before).length-1,2);return text.replaceAll(before,'../../zero-factor-candidate-v70/hypersolve');
}
export function consumerSources(requireBinding=true){
 const previous=previousSources(),o=json(originPath);assert.deepEqual(o.previous,previous);assert.equal(o.root,root);
 assert.equal(o.protocolSha256,sha('zero-factor-consumer-protocol-v74.md'));assert.equal(o.scriptSha256,sha('zero-factor-consumer-sources-v74.mjs'));
 const sources={},changed=[];let bytes=0;
 for(const[p,h]of Object.entries(previous.consumer.sources)){
  sources[p]=sha(root+'/'+p);bytes+=statSync(previous.consumer.root+'/'+p).size;
  if(sources[p]!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hypercurve/Cargo.toml']);assert.equal(readFileSync(crate+'/Cargo.toml','utf8'),expectedManifest());
 assert.equal(bytes,o.copiedBytes);
 const actual=readdirSync(root,{recursive:true,withFileTypes:true}).filter(d=>d.isFile()).map(d=>resolve(d.parentPath,d.name)).sort();
 assert.deepEqual(actual,Object.keys(sources).map(p=>resolve(root,p)).sort());
 const result={checkpoint:74,root,originSha256:sha(originPath),sources,changed,copiedBytes:bytes,
  solver:'zero-factor-candidate-v70/hypersolve',scalarRoot:'e-plan-qualified-candidate',
  limits:'Consumer manifest path changes only; all original sources/lockfile preserved. No live change or new retention.'};
 if(requireBinding)assert.deepEqual(result,json(bindingPath));return result;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 if(process.argv.includes('--copy')){
  const previous=previousSources();mkdirSync(root);let copiedBytes=0;
  for(const[p,h]of Object.entries(previous.consumer.sources)){
   const from=previous.consumer.root+'/'+p,to=root+'/'+p;mkdirSync(dirname(to),{recursive:true});
   copyFileSync(from,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);assert.equal(sha(to),h,p);copiedBytes+=statSync(from).size;
  }
  writeFileSync(originPath,JSON.stringify({checkpoint:74,root,previous,copiedBytes,protocolSha256:sha('zero-factor-consumer-protocol-v74.md'),
   scriptSha256:sha('zero-factor-consumer-sources-v74.mjs'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({checkpoint:74,copiedFiles:355,copiedBytes,root,next:'Apply the two path-only manifest edits, then bind.'}));
 }else if(process.argv.includes('--bind')){
  const result=consumerSources(false);writeFileSync(bindingPath,JSON.stringify(result,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({checkpoint:74,boundFiles:355,root,changed:result.changed}));
 }else{const result=consumerSources();console.log(JSON.stringify({checkpoint:74,status:'pass',files:355,root,copiedBytes:result.copiedBytes}));}
}
