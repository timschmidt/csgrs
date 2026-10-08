import {existsSync,readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {plan,digest} from './snapshot-v43-spec.mjs';
const p=plan(),m=JSON.parse(readFileSync('e-qualified-experiment.json','utf8'));
assert.equal(m.draft,false);assert.equal(m.production.retained,true);
for(const[f,h]of Object.entries(m.files))assert.equal(digest(readFileSync(f)),h,f);
if(process.argv.includes('--patch')) {
 let patch='*** Begin Patch\n';for(const g of p.modules) {
  assert(!existsSync(g.path),g.path);
  assert(g.source.endsWith('\n'),g.path);
  patch+='*** Add File: '+g.path+'\n'+g.source.slice(0,-1).split('\n').map(l=>'+'+l).join('\n')+'\n';
 }console.log(patch+'*** End Patch');
}else {
 assert.deepEqual(process.argv.slice(2),['--bind']);
 for(const g of p.modules)assert.equal(digest(readFileSync(g.path)),g.sha256,g.path);
 const files={};for(const f of ['snapshot-v43-spec.mjs','prepare-snapshot-v43.mjs',
  'verify-e-qualified-retained.mjs','e-qualified-snapshot-verification.md','e-qualified-experiment.json'])
  files[f]=digest(readFileSync(f));
 const record={schema:1,recorded:new Date().toISOString(),files,
  historicalSnapshot:'derivative-demand-candidate',historicalSources:m.sourceMap.baseline,
  liveSources:m.sourceMap.candidate,entry:p.entry,
  modules:p.modules.map(({source,...r})=>r),
  note:'Explicit historical-source path and import substitutions only. Original scripts/manifests, all assertions, mathematical checkers, failures and benchmark observations remain unchanged. Current sources are separately matched to the 956-file retained map.'};
 writeFileSync('e-qualified-retention-verification.json',JSON.stringify(record,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({modules:p.modules.length,generatedBytes:p.modules.reduce((s,g)=>s+g.bytes,0),
  historicalFiles:Object.keys(record.historicalSources).length,liveFiles:Object.keys(record.liveSources).length}));
}
