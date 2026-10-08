import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {retainedSources,sha,json} from './point-retained-sources-v67.mjs';
export {sha,json};
export const root='power-rebased-v68',binding='power-rebased-binding-v68.json';
export function rebasedSources(){
 const current=retainedSources(),o=json('power-rebased-origin-v68.json');
 assert.deepEqual(current,o.current);assert.equal(sha('point-retained-v67-manifest.json'),o.previousManifestSha256);
 assert.equal(sha('power-sums-manifest.json'),o.originalManifestSha256);
 for(const[p,h]of Object.entries(o.originalPowerSources))assert.equal(sha('power-sums-candidate/'+p),h,p);
 const source={},changed=[];
 for(const[p,h]of Object.entries(o.originalSources)){
  assert.equal(sha(o.baseline+'/'+p),h,p);source[p]=sha(root+'/'+p);if(source[p]!==h)changed.push(p);
 }
 assert.deepEqual(changed,['hypersolve/src/algebraic_binary.rs']);
 const added='hypersolve/src/algebraic_binary/power_sums.rs';source[added]=sha(root+'/'+added);
 assert.equal(source[added],o.originalPowerSources[added]);
 const base=readFileSync(o.baseline+'/'+changed[0],'utf8'),now=readFileSync(root+'/'+changed[0],'utf8'),
  original=readFileSync('power-sums-candidate/'+changed[0],'utf8');
 const begin='fn resultant_polynomial_for_binary_image(',end='fn binary_image_interval(';
 assert.equal(now.slice(0,now.indexOf(begin)).replace('mod power_sums;\n\n',''),base.slice(0,base.indexOf(begin)));
 assert.equal(now.slice(now.indexOf(end)),base.slice(base.indexOf(end)));
 assert.equal(now.slice(now.indexOf(begin),now.indexOf(end)),original.slice(original.indexOf(begin),original.indexOf(end)));
 const harness=['power-sums-public.rs','power-sums-cpu.rs','power-sums-allocation.rs',
  'point-qualified-approx.rs','point-demand-extended.rs','point-history-base.rs','point-extended.rs',
  'power-rebased-baseline-app-v68/Cargo.toml','power-rebased-candidate-app-v68/Cargo.toml'];
 return {checkpoint:68,root,baseline:o.baseline,originSha256:sha('power-rebased-origin-v68.json'),
  current,source,changed,added,harness:Object.fromEntries(harness.map(p=>[p,sha(p)])),
  limits:'Exact unchanged checkpoint-52 constructor plus retained point-witness repair. No live edit; source identity is not mathematical/performance qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const s=rebasedSources();
 if(process.argv.includes('--record'))writeFileSync(binding,JSON.stringify(s,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(s,json(binding));
 console.log(JSON.stringify({checkpoint:68,files:Object.keys(s.source).length,changed:s.changed,added:s.added,currentLiveFiles:956}));
}
