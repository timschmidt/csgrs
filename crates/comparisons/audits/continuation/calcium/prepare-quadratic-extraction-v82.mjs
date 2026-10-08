import {readFileSync,writeFileSync,mkdtempSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {cases,selfTest} from './quadratic-extraction-oracle-v82.mjs';
const prior=json('scalar-boundary-v81-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const gate=json('results/scalar-boundary-verify-v81.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);
assert.equal(json('results/scalar-boundary-verify-v81.stdout').status,'verified-scalar-boundary-source-and-capability');
assert.equal(readFileSync('results/scalar-boundary-verify-v81.stderr').length,0);
const inventory=json('inventory.json'),coverage=effectiveCoverage(),records=[],donorSources={};
for(const s of inventory.sources)for(const test of [false,true]){
 const p=(s.repo==='flint'?'src/':'')+'qqbar/'+(test?'test/t-':'')+'get_quadratic.c',f=s.files.find(f=>f.path===p);
 assert(f?.text&&!coverage.some(c=>c.repo===s.repo&&c.path===p));assert.equal(sha(workspace+'/exact-real-references/'+s.repo+'/'+p),f.sha256);
 donorSources[s.repo+':'+p]=f.sha256;
 records.push({repo:s.repo,path:p,ranges:[[1,f.lines]],note:'Checkpoint82: '+(test?
  'Full randomized degree-1/2 extraction across three factorization policies, reconstruction, denominator/content and residual-square normalization checks. Source read, not newly executed.':
  'Full rational fast path; degree precondition; discriminant; Gaussian-rational shortcut; power-of-two, full and smooth factorization policies; complex sign, pure-real sign and two-root enclosure separation; positive denominator/content reduction and cleanup. Comments contain discriminant-sign/square-root notation slips; actual arithmetic and selected-root identity qualified independently.')});
}
assert.equal(records.length,4);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),644);
const hyperReadRanges={'hyperreal/src/rational/arithmetic/squares_powers.rs':[[1,300]],
 'hyperreal/src/real/arithmetic/elementary_functions.rs':[[180,295]],
 'hyperreal/src/computable/node/structural_analysis.rs':[[1310,1555]],'hyperreal/src/real/tests.rs':[[480,560]]};
const manualRanges={'calcium:doc/source/qqbar.rst':[[740,805]],'flint:doc/source/qqbar.rst':[[775,840]]};
const old=json('scalar-boundary-origin-v81.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const files=['prepare-quadratic-extraction-v82.mjs','run-quadratic-extraction-v82.mjs','flint-quadratic-extraction-v82.c',
 'quadratic-extraction-protocol-v82.md','quadratic-extraction-oracle-v82.mjs','scalar-boundary-endpoints-v81.mjs',
 'qqbar-inverse-oracle-v80.mjs','point-extended-field.mjs','inventory.json','effective-coverage.mjs','coverage.json',
 'capture.mjs','point-qualified-capture.mjs','scalar-boundary-v81-manifest.json'];
const dir=mkdtempSync('/tmp/calcium-quadratic-extraction-v82.'),inputs=cases();
const origin={checkpoint:82,recorded:new Date().toISOString(),current:retainedSources(),previousSha256:sha('scalar-boundary-v81-manifest.json'),
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),records,donorSources,coverageBefore:effectiveSummary(),
 extensionsBefore:json('coverage-extensions.json'),extensionsBeforeSha256:sha('coverage-extensions.json'),
 hyperReadRanges,hyperReadHashes:Object.fromEntries(Object.keys(hyperReadRanges).map(p=>[p,sha(workspace+'/'+p)])),
 manualRanges,manualHashes:Object.fromEntries(Object.keys(manualRanges).map(k=>{const[repo,p]=k.split(':');return[k,sha(workspace+'/exact-real-references/'+repo+'/'+p)];})),
 libraries:old.libraries,configurationFiles:old.configurationFiles,dir,binary:dir+'/controls',oracle:selfTest()};
writeFileSync('quadratic-extraction-input-v82.json',JSON.stringify(inputs,null,2)+'\n',{flag:'wx'});
writeFileSync('quadratic-extraction-input-v82.tsv',inputs.map(c=>[c.id,c.a,c.b,BigInt(c.d)*BigInt(c.s)**2n,c.q].join('\t')).join('\n')+'\n',{flag:'wx'});
writeFileSync('quadratic-extraction-origin-v82.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:82,status:'prepared',newFiles:4,newLines:644,cases:775,rows:4650,dir}));
