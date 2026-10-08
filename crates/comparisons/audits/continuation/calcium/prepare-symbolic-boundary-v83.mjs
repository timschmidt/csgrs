import {writeFileSync,readFileSync,mkdtempSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary}from './effective-coverage.mjs';
const prior=json('quadratic-extraction-v82-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const g=json('results/complex-symbolic-current82-before-v83.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(json('results/complex-symbolic-current82-before-v83.stdout').status,'verified-quadratic-extraction-source-and-capability');
assert.equal(readFileSync('results/complex-symbolic-current82-before-v83.stderr').length,0);
const records=[],donorSources={},coverage=effectiveCoverage(),inventory=json('inventory.json');
const notes={abs:'Full real sign, root-of-unity, pure-imaginary and conjugate-product square-root paths; force exact zero imaginary enclosure.',
 im:'Full zero/pure-imaginary paths and (x-conj(x))/(2i), including in-place operations and exact-real enclosure projection.',
 re:'Full real/pure-imaginary paths and (x+conj(x))/2, retaining exact-real enclosure.',
 re_im:'Full scheduling of two projections when the real output aliases input. Distinct outputs; upstream test does not invoke this combined wrapper or its input-alias paths.',
 sgn:'Full zero/real/imaginary shortcuts and x/abs(x). Complex sign is a unit phase, not csgn or a general real-valued ordering.',
 get_fexpr:'Full internal serialization, nearest/indexed root presentation, certified decimal selection, cyclotomic field search, flagged quadratic/deflation/separation formulas and recursion guards. Formula failure is not nonexistence proof; selected embedding must survive conversion.',
 set_fexpr:'Full dyadic-ball and decimal parsing; Pi-factor detection; indexed/nearest root conversion; all public expression cases, operation domains, coefficient/exponent bounds, variadic aggregation and cleanup. Suspected false-success Pi and large-coefficient branches and missing temporary fmpz cleanup selected for public native diagnostics.'};
for(const s of inventory.sources)for(const[test,names]of [[false,['abs','im','re','re_im','sgn','get_fexpr','set_fexpr']],[true,['abs','abs2','re_im','sgn','get_fexpr','get_fexpr_formula']]])for(const name of names){
 const p=(s.repo==='flint'?'src/':'')+'qqbar/'+(test?'test/t-':'')+name+'.c',f=s.files.find(f=>f.path===p);
 assert(f?.text&&!coverage.some(c=>c.repo===s.repo&&c.path===p));assert.equal(sha(workspace+'/exact-real-references/'+s.repo+'/'+p),f.sha256);
 donorSources[s.repo+':'+p]=f.sha256;records.push({repo:s.repo,path:p,ranges:[[1,f.lines]],note:'Checkpoint83: '+(test?
  'Full randomized '+name+' test: input construction, property/reconstruction comparisons, failure diagnostics and cleanup. Formula tests ignore the set_fexpr return flag and do not cover missing-Pi or huge coefficient guards. Source read, not newly executed.':notes[name])});
}
assert.equal(records.length,26);assert.equal(records.reduce((n,r)=>n+r.ranges[0][1],0),5728);
const hyperReadRanges={'hyperlattice/src/complex.rs':[[1,555]],'hyperreal/src/serde.rs':[[1,120]],
 'hyperreal/src/real/arithmetic/representation.rs':[[1,90],[245,290]],'hyperreal/src/real/arithmetic/classification.rs':[[1,90]]};
const rereadRanges={'calcium:doc/source/qqbar.rst':[[790,920]],'flint:doc/source/qqbar.rst':[[1,160],[828,950]],
 'flint:src/fexpr/arg.c':[[1,84]],'flint:src/fexpr.h':[[292,352]]};
const old=json('quadratic-extraction-origin-v82.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const files=['flint-symbolic-boundary-v83.c','symbolic-boundary-protocol-v83.md','prepare-symbolic-boundary-v83.mjs',
 'inventory.json','coverage.json','effective-coverage.mjs','capture.mjs','point-qualified-capture.mjs','quadratic-extraction-v82-manifest.json'];
const dir=mkdtempSync('/tmp/calcium-symbolic-boundary-v83.');
writeFileSync('symbolic-boundary-origin-v83.json',JSON.stringify({checkpoint:83,recorded:new Date().toISOString(),current:retainedSources(),
 previousSha256:sha('quadratic-extraction-v82-manifest.json'),files:Object.fromEntries(files.map(p=>[p,sha(p)])),records,donorSources,
 coverageBefore:effectiveSummary(),extensionsBefore:json('coverage-extensions.json'),extensionsBeforeSha256:sha('coverage-extensions.json'),
 hyperReadRanges,hyperReadHashes:Object.fromEntries(Object.keys(hyperReadRanges).map(p=>[p,sha(workspace+'/'+p)])),rereadRanges,
 rereadHashes:Object.fromEntries(Object.keys(rereadRanges).map(k=>{const[repo,p]=k.split(':');return[k,sha(workspace+'/exact-real-references/'+repo+'/'+p)];})),
 libraries:old.libraries,configurationFiles:old.configurationFiles,dir,binary:dir+'/controls'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:83,status:'prepared',readFiles:26,newLines:5728,dir}));
