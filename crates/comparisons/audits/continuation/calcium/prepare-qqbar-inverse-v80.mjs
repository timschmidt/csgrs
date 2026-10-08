import {readFileSync,writeFileSync,mkdtempSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json,retainedSources,workspace} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {cases,selfTest} from './qqbar-inverse-oracle-v80.mjs';
const prior=json('twelfth-revision-v79-manifest.json'),current=retainedSources(),coverage=effectiveCoverage(),inventory=json('inventory.json');
for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
for(const[p,h]of Object.entries(prior.candidateSources))assert.equal(sha(prior.candidateRoot+'/'+p),h,p);
const tag='qqbar-inverse-current79-before-v80',g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(json('results/'+tag+'.stdout').status,'verified-cheaper-revision-not-retained');assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
const notes={
 asin_pi:'Full degree-one/two exact tables and principal embeddings; generic finite 64-bit approximation, bounded mediant proposal, degree filter and exact reconstruction check. Recognition completeness and fixed-tolerance concerns remain distinct from sound accepted equality.',
 acos_pi:'Full asin composition with exact reduced 1/2-minus-angle branch and word-sized output arithmetic. Wrapper inherits asin recognition failures; NULL outputs are not documented here.',
 atan_pi:'Full bounded Stern-Brocot proposal helper and rational/quadratic tables; cached enclosure-side separation for quadratic conjugates, generic degree/parity and magnitude filters, and exact candidate validation. One-candidate tolerance can miss a different exact rational angle; public qualification follows separately.',
 acot_pi:'Full rational/quadratic tables and reciprocal-atan fallback. Range (-1/2,1/2], negative-input branch and explicit zero=1/2 reviewed. Abort/throw on unexpectedly broad low-degree cached enclosure noted without invalid-state reproduction.',
 log_pi_i:'Full root-of-unity phase recognition and normalization to (-1,1], retaining +1 at the negative real axis. This implementation was previously read; no new coverage credit.'};
const records=[],rereads=[],donorSources={};
for(const s of inventory.sources)for(const name of Object.keys(notes))for(const test of [false,true]){
 const path=(s.repo==='flint'?'src/':'')+'qqbar/'+(test?'test/t-':'')+name+'.c',f=s.files.find(f=>f.path===path);
 assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,path)),f.sha256);donorSources[s.repo+':'+path]=f.sha256;
 const old=coverage.find(r=>r.repo===s.repo&&r.path===path),record={repo:s.repo,path,ranges:[[1,f.lines]],note:'Checkpoint80: '+(test?
  'Full upstream test source: bounded random forward/inverse round trips, reduced p/q and principal-range assertions, pole exclusion and cleanup. Current test multiplier is one tenth the archived default. No negative-recognition or cache-history corpus here; source read, not newly executed.':notes[name])};
 if(old){assert.equal(name,'log_pi_i');assert(!test);assert.deepEqual(old.ranges,record.ranges);rereads.push(record);}else records.push(record);
}
const header=inventory.sources.find(s=>s.repo==='flint').files.find(f=>f.path==='src/qqbar/impl.h');
assert(header?.text&&!coverage.some(r=>r.repo==='flint'&&r.path===header.path));assert.equal(sha(workspace+'/exact-real-references/flint/'+header.path),header.sha256);
donorSources['flint:'+header.path]=header.sha256;records.push({repo:'flint',path:header.path,ranges:[[1,header.lines]],note:'Checkpoint80: Full private header: include guard and declarations for decimal-root formatting, bounded rational proposal and cyclotomic recognition. Only the proposal implementation is covered by the current source reads; declarations do not close their other callees.'});
assert.equal(records.length,19);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),1738);
const old=json('qqbar-trig-origin-v76.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const rereadRanges={'calcium:doc/source/qqbar.rst':[[608,695]],'flint:doc/source/qqbar.rst':[[645,728]]};
const hyperReadRanges={'hyperreal/src/real/arithmetic/elementary_functions.rs':[[2835,3240]]};
const files=['prepare-qqbar-inverse-v80.mjs','flint-qqbar-inverse-v80.c','flint-qqbar-inverse-counterexample-v80.c','qqbar-inverse-oracle-v80.mjs',
 'qqbar-inverse-protocol-v80.md','point-extended-field.mjs','inventory.json','coverage.json','effective-coverage.mjs','twelfth-revision-v79-manifest.json'];
const dir=mkdtempSync('/tmp/calcium-qqbar-inverse-v80.');
const origin={checkpoint:80,recorded:new Date().toISOString(),current,previousSha256:sha('twelfth-revision-v79-manifest.json'),
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),records,rereads,donorSources,coverageBefore:effectiveSummary(),extensionsBefore:json('coverage-extensions.json'),
 extensionsBeforeSha256:sha('coverage-extensions.json'),rereadRanges,rereadHashes:Object.fromEntries(Object.keys(rereadRanges).map(k=>{const[repo,p]=k.split(':');return[k,sha(workspace+'/exact-real-references/'+repo+'/'+p)];})),
 hyperReadRanges,hyperReadHashes:Object.fromEntries(Object.keys(hyperReadRanges).map(p=>[p,sha(workspace+'/'+p)])),libraries:old.libraries,configurationFiles:old.configurationFiles,
 dir,binary:dir+'/controls',counterexampleBinary:dir+'/counterexample',oracleSelfTest:selfTest(),
 note:'Pre-build source/read/library binding. Numeric correctness, completeness observation and coverage publication require later evidence; no production or donor edit.'};
writeFileSync('qqbar-inverse-input-v80.json',JSON.stringify(cases(),null,2)+'\n',{flag:'wx'});
writeFileSync('qqbar-inverse-origin-v80.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:80,status:'prepared',newFiles:19,newLines:1738,rereadImplementations:2,cases:1081,dir}));
