import {readFileSync,writeFileSync,mkdirSync,copyFileSync,constants,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {demandSources,sha,json} from './point-demand-sources.mjs';
export {sha,json};
export const root='point-demand-consumer-v66',crate=root+'/hypercurve';
export const original='point-consumer-origin-v66.json',binding='point-consumer-binding-v66.json';
export function expectedManifest(){
 let text=readFileSync('e-plan-qualified-candidate/hypercurve/Cargo.toml','utf8');
 for(const name of ['hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri']){
  const old=`path = "../${name}"`,target=name==='hypersolve'?'point-demand-candidate':'e-plan-qualified-candidate';
  const n=text.split(old).length-1;assert.equal(n,name==='hyperlattice'?1:2);
  text=text.replaceAll(old,`path = "../../${target}/${name}"`);
 }
 return text;
}
export function consumerSources(requireBinding=true){
 demandSources();const o=json(original),source=json('point-qualified-origin.json');
 assert.equal(o.root,root);assert.equal(o.previousSha256,sha('point-attribution-v65-manifest.json'));
 assert.equal(o.demandSourceSha256,sha('point-demand-source-binding.json'));
 const selected=Object.fromEntries(Object.entries(source.liveSources).filter(([p])=>p.startsWith('hypercurve/')));
 assert.deepEqual(o.sources,selected);const current={};let bytes=0;
 for(const[p,h]of Object.entries(selected)){
  assert.equal(sha(source.baseline+'/'+p),h,p);assert.equal(sha(resolve('../../../..',p)),h,p+' live');
  current[p]=sha(root+'/'+p);bytes+=statSync(source.baseline+'/'+p).size;
  if(p!=='hypercurve/Cargo.toml')assert.equal(current[p],h,p+' copied');
 }
 assert.equal(Object.keys(selected).length,355);assert.equal(bytes,23841357);
 assert.equal(o.copiedBytes,bytes);assert.equal(readFileSync(crate+'/Cargo.toml','utf8'),expectedManifest());
 const result={root,originSha256:sha(original),sources:current,changed:['hypercurve/Cargo.toml'],
  solver:'point-demand-candidate/hypersolve',scalarRoot:source.baseline,
  solverSourceSha256:sha('point-demand-source-binding.json')};
 if(requireBinding)assert.deepEqual(result,json(binding));return result;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 if(process.argv.includes('--copy')){
  demandSources();const previous=json('point-attribution-v65-manifest.json');
  for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
  const source=json('point-qualified-origin.json'),selected=Object.fromEntries(Object.entries(source.liveSources).filter(([p])=>p.startsWith('hypercurve/')));
  assert.equal(Object.keys(selected).length,355);mkdirSync(root);let bytes=0;
  for(const[p,h]of Object.entries(selected)){
   const from=source.baseline+'/'+p,to=root+'/'+p;assert.equal(sha(from),h,p);
   mkdirSync(dirname(to),{recursive:true});copyFileSync(from,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
   assert.equal(sha(to),h,p);bytes+=statSync(to).size;
  }
  const o={root,recorded:new Date().toISOString(),previousSha256:sha('point-attribution-v65-manifest.json'),
   demandSourceSha256:sha('point-demand-source-binding.json'),sources:selected,copiedBytes:bytes,
   note:'Only Hypercurve is copied. After the separately applied path-only manifest edit, frozen scalar dependencies and the frozen demand solver remain shared. No copied source algorithm changes.'};
  writeFileSync(original,JSON.stringify(o,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({copiedFiles:355,copiedBytes:bytes,root,next:'Apply the path-only Cargo manifest edit, then --bind.'}));
 }else if(process.argv.includes('--bind')){
  const result=consumerSources(false);writeFileSync(binding,JSON.stringify(result,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify({boundFiles:Object.keys(result.sources).length,root,changed:result.changed}));
 }else console.log(JSON.stringify({status:'pass',root,files:Object.keys(consumerSources().sources).length}));
}
