import {writeFileSync,readdirSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {retainedSources,sha,json} from './point-retained-sources-v67.mjs';
export {sha,json};
export function zeroSources(){
 const o=json('zero-factor-origin-v70.json'),current=retainedSources(),source={},changed=[];
 assert.deepEqual(current,o.current);assert.equal(sha('power-wide-v69-manifest.json'),o.previousManifestSha256);
 for(const[p,h]of Object.entries(o.originalSources)){
  assert.equal(sha(o.baseline+'/'+p),h,p);source[p]=sha(o.root+'/'+p);if(source[p]!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hypersolve/src/algebraic_binary.rs']);
 const added='hypersolve/src/algebraic_binary/zero_factor_tests.rs';source[added]=sha(o.root+'/'+added);
 const files=readdirSync(o.root,{recursive:true,withFileTypes:true}).filter(d=>d.isFile()).map(d=>resolve(d.parentPath,d.name));
 assert.deepEqual(files.sort(),Object.keys(source).map(p=>resolve(o.root,p)).sort());
 return {checkpoint:70,root:o.root,baseline:o.baseline,originSha256:sha('zero-factor-origin-v70.json'),current,source,changed,added,
  limits:'Isolated zero-factor trial; no power-sum module or live edits. Source identity alone is not correctness/performance qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const s=zeroSources();if(process.argv.includes('--record'))writeFileSync('zero-factor-binding-v70.json',JSON.stringify(s,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(s,json('zero-factor-binding-v70.json'));
 console.log(JSON.stringify({checkpoint:70,files:Object.keys(s.source).length,changed:s.changed,added:s.added,liveFiles:s.current.liveFiles}));
}
