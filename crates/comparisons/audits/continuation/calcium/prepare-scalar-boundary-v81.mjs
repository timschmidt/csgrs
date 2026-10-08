import {readFileSync,writeFileSync,mkdtempSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {selfTest,nativeCases} from './scalar-boundary-oracle-v81.mjs';
const prior=json('qqbar-inverse-v80-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const current=retainedSources(),g=json('results/inverse-reciprocal-current80-before-v81.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(json('results/inverse-reciprocal-current80-before-v81.stdout').status,'verified-inverse-source-and-completeness-audit');
assert.equal(readFileSync('results/inverse-reciprocal-current80-before-v81.stderr').length,0);
const inventory=json('inventory.json'),coverage=effectiveCoverage(),tests=['asec_pi','acsc_pi','floor','ceil','set_d','set_re_im_d'],
 helpers=['get_fmpq','get_fmpz','numerator','denominator','height','height_bits','swap','inlines','phi'];
const notes={
 asec_pi:'Full zero guard and reciprocal-acos composition. Failure is not negative proof; p/q are not nullable here. Principal branch inherited from acos; failed outputs are not inspected.',
 acsc_pi:'Full zero guard and reciprocal-asin composition. Failure is not negative proof; reciprocal poles are excluded and the principal asin range is inherited.',
 floor:'Full rational signed-division fast path, cached interval filter, magnitude-scaled refinement, half-offset integer proposal and authoritative exact real-part sign correction. Complex values use the real part. No unchecked approximate integer is accepted.',
 ceil:'Full dual floor architecture with negative half-offset proposal and positive exact real-part sign correction. Magnitude-aware precision avoids large-magnitude fractional uncertainty; numerical failure triggers archived abort/current throw.',
 set_d:'Full ARF finite classification and exact dyadic-to-rational conversion. Signed zero is mathematical zero. Non-finite failure does not authorize consuming the destination; no decimal conversion.',
 set_re_im_d:'Full real-only zero-imaginary shortcut, imaginary-first import and exact real addition. Non-finite real component can leave a changed destination despite returning failure; no unchanged-output contract is inferred.',
 get_fmpq:'Full exact degree-one coefficient extraction, signed numerator and positive denominator. Degree mismatch aborts/throws; no rational guessing.',
 get_fmpz:'Full degree-one and unit-leading-coefficient preconditions before exact signed coefficient extraction. No rounded conversion.',
 numerator:'Full algebraic-integer copy shortcut and multiplication by polynomial-leading-coefficient denominator. In-place and separate output qualified independently; not a minimal denominator operation.',
 denominator:'Full leading-coefficient extraction. This documented denominator need not be the smallest integer making the value integral; do not transfer rational-denominator semantics.',
 height:'Full minimal-polynomial coefficient-height wrapper; declaration/call does not newly credit its polynomial implementation.',
 height_bits:'Full maximum coefficient bit-length wrapper with sign-encoding absolute value; called polynomial implementation is not newly credited.',
 swap:'Full ownership transfer: archived swaps polynomial/enclosure separately; current swaps the complete qqbar_struct. Both representation and root-selection enclosure must travel together.',
 inlines:'Full inline-emission macro and public-header include. No completion credit for header bodies/callees from this shim.',
 phi:'Full minimal polynomial X^2-X-1 and positive golden-root enclosure construction, zero imaginary part. Embedding matters as well as polynomial identity.'};
const records=[],donorSources={};
for(const s of inventory.sources)for(const name of [...tests,...helpers])for(const test of tests.includes(name)?[false,true]:[false]){
 const p=(s.repo==='flint'?'src/':'')+'qqbar/'+(test?'test/t-':'')+name+'.c',f=s.files.find(f=>f.path===p);
 assert(f?.text&&!coverage.some(c=>c.repo===s.repo&&c.path===p));assert.equal(sha(workspace+'/exact-real-references/'+s.repo+'/'+p),f.sha256);
 donorSources[s.repo+':'+p]=f.sha256;records.push({repo:s.repo,path:p,ranges:[[1,f.lines]],note:'Checkpoint81: '+(test?
  name.includes('_pi')?'Full random reciprocal forward/inverse tests, reduced fractions, exact reconstruction and principal bounds. Archived acsc progress label incorrectly says asec; cosmetic only. Source read, not newly executed.':
  name==='floor'||name==='ceil'?'Full randomized real-part integer inequalities, including complex values and large rational offsets. Source read, not newly executed.':
  'Full randomized special-float import tests. Success checks exact 53-bit enclosure identity, failure requires non-finite source. These tests query the destination after failure but the new audit does not infer a failed-output value contract. Source read, not newly executed.':notes[name])});
}
assert.equal(records.length,42);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),1902);
const old=json('qqbar-inverse-origin-v80.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const hyperReadRanges={
 'hyperreal/src/real/arithmetic/representation.rs':[[105,133],[210,455]],'hyperreal/src/real/convert.rs':[[1,105]],
 'hyperreal/src/rational/convert.rs':[[1,205]],'hyperreal/src/computable/node/approximation_queries.rs':[[1,148]],
 'hyperreal/src/real/arithmetic/tests.rs':[[2460,2570]]};
const manualRanges=Object.fromEntries(['calcium','flint'].map(repo=>[repo+':doc/source/qqbar.rst',[[82,158],[367,390],[515,547],repo==='flint'?[716,741]:[681,706]]]));
const files=['prepare-scalar-boundary-v81.mjs','run-scalar-boundary-v81.mjs','flint-scalar-boundary-v81.c','flint-qqbar-inverse-v80.c',
 'scalar-boundary-protocol-v81.md','scalar-boundary-oracle-v81.mjs','qqbar-inverse-oracle-v80.mjs','point-extended-field.mjs',
 'inventory.json','coverage.json','effective-coverage.mjs','qqbar-inverse-v80-manifest.json','point-qualified-capture.mjs','capture.mjs'];
const dir=mkdtempSync('/tmp/calcium-scalar-boundary-v81.');
const origin={checkpoint:81,recorded:new Date().toISOString(),current,previousSha256:sha('qqbar-inverse-v80-manifest.json'),
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),records,donorSources,coverageBefore:effectiveSummary(),extensionsBefore:json('coverage-extensions.json'),
 extensionsBeforeSha256:sha('coverage-extensions.json'),hyperReadRanges,hyperReadHashes:Object.fromEntries(Object.keys(hyperReadRanges).map(p=>[p,sha(workspace+'/'+p)])),
 manualRanges,manualHashes:Object.fromEntries(Object.keys(manualRanges).map(k=>{const[repo,p]=k.split(':');return[k,sha(workspace+'/exact-real-references/'+repo+'/'+p)];})),
 libraries:old.libraries,configurationFiles:old.configurationFiles,dir,binary:dir+'/controls',oracle:selfTest(),
 note:'Pre-build source/read/configuration binding; no production/candidate/donor mutation. Coverage publication and correctness evidence are separate.'};
writeFileSync('scalar-boundary-input-v81.json',JSON.stringify(nativeCases(),null,2)+'\n',{flag:'wx'});
writeFileSync('scalar-boundary-origin-v81.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:81,status:'prepared',newFiles:42,newLines:1902,cases:19923,dir}));
