import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {retainedSources,workspace,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const source=retainedSources(),prior=effectiveCoverage(),inventory=json('inventory.json');
const before=effectiveSummary();assert.deepEqual(before,json('qqbar-arithmetic-manifest.json').coverageAtBinding);
const notes={
 root_of_unity:'Full cyclotomic recognition, optional argument recovery and normalized construction with certified embedding; fixed 64-bit argument recovery and small-word domain noted.',
 exp_pi_i:'Full doubled-denominator wrapper; documented near-word-boundary exclusion applies, no claimed in-contract overflow defect.',
 cos_pi:'Full exact rational/period reduction, low-order constants, cos-minpoly construction and certified twice-cosine embedding followed by dyadic scaling.',
 sin_pi:'Full complementary-cosine wrapper with machine-sized reduction/doubling; documented word-boundary restriction retained.',
 tan_pi:'Full reduced-denominator poles/constants and generic root-of-unity reciprocal construction; explicit real-axis reset and temporary lifetime.',
 cot_pi:'Full gcd, integer pole/half-turn zero and inverse-tangent construction.',
 sec_pi:'Full cosine/pole/inverse wrapper; failed-call output is unspecified.',
 csc_pi:'Full sine/pole/inverse wrapper; failed-call output is unspecified.'};
const records=[],donorSources={};
for(const s of inventory.sources)for(const test of [false,true])for(const name of Object.keys(notes)){
 const path=(s.repo==='flint'?'src/':'')+'qqbar/'+(test?'test/t-':'')+name+'.c';
 const f=s.files.find(f=>f.path===path);assert(f?.text);assert(!prior.some(r=>r.repo===s.repo&&r.path===path));
 assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,path)),f.sha256);
 records.push({repo:s.repo,path,ranges:[[1,f.lines]],note:'Checkpoint76: '+(test?
  'Full upstream test source; small bounded random inputs, shared-backend overlap or root-reconstruction assertions, pole/return-flag coverage limits and cleanup reviewed. Source read, not newly executed.':notes[name])});
 donorSources[s.repo+':'+path]=f.sha256;
}
assert.equal(records.length,32);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),1964);
const old=json('qqbar-arithmetic-manifest.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const hyperReadRanges={
 'hyperreal/src/real/arithmetic/elementary_functions.rs':[[2390,2805]],
 'hyperreal/src/real/arithmetic/inversion.rs':[[125,323]],
 'hyperreal/src/real/arithmetic/representation.rs':[[465,588]]};
const hyperReadHashes=Object.fromEntries(Object.keys(hyperReadRanges).map(p=>[p,sha(resolve(workspace,p))]));
const rereadRanges={'calcium:doc/source/qqbar.rst':[[608,667]],'flint:doc/source/qqbar.rst':[[650,694]]};
const rereadHashes={};for(const k of Object.keys(rereadRanges)){const[repo,p]=k.split(':');rereadHashes[k]=sha(resolve(workspace,'exact-real-references',repo,p));}
const files=['prepare-qqbar-trig-v76.mjs','flint-qqbar-trig-v76.c','qqbar-trig-protocol-v76.md','point-extended-field.mjs',
 'zero-factor-retained-v75-manifest.json','zero-factor-retained-sources-v75.mjs','inventory.json','coverage.json','effective-coverage.mjs',
 ...['json','stdout','stderr'].map(e=>'results/zero-factor-retained-verify-v75.'+e)];
const origin={checkpoint:76,recorded:new Date().toISOString(),source,files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 records,donorSources,before,extensionsBefore:json('coverage-extensions.json'),extensionsBeforeSha256:sha('coverage-extensions.json'),
 hyperReadRanges,hyperReadHashes,rereadRanges,rereadHashes,libraries:old.libraries,configurationFiles:old.configurationFiles,
 binary:'/tmp/calcium-qqbar-trig-v76.xvGytz/controls',notes:'Pre-build source/configuration/read binding; numerical success and new coverage publication require later evidence.'};
writeFileSync('qqbar-trig-origin-v76.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:76,status:'prepared',files:32,newLines:1964,sourceFiles:source.liveFiles,binary:origin.binary}));
