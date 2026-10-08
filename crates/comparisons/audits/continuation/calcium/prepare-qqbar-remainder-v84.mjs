import {writeFileSync,readFileSync,mkdtempSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary}from './effective-coverage.mjs';
const prior=json('symbolic-boundary-v83-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
const g=json('results/qqbar-remainder-current83-before-v84.json');assert.equal(g.code,0);assert.equal(g.signal,null);
assert.equal(json('results/qqbar-remainder-current83-before-v84.stdout').status,'verified-source-and-classified-symbolic-defects');
assert.equal(readFileSync('results/qqbar-remainder-current83-before-v84.stderr').length,0);
const records=[],donorSources={},coverage=effectiveCoverage(),inventory=json('inventory.json');
const notes={
 eigenvalues_fmpq_mat:'Characteristic polynomial then exact roots; rational-matrix wrapper, shared flags and cleanup.',
 eigenvalues_fmpz_mat:'Integer characteristic polynomial then exact roots; shared flags and cleanup.',
 roots_fmpq_poly:'Borrowed numerator-polynomial view; common nonzero denominator cannot change roots. No copy or ownership transfer.',
 evaluate_fmpz_mpoly:'Full iterative/Horner scheduling, exponent representations, limits, status propagation, aliasing and cleanup. Current wrapper delegates to generic code, but omits its exponent-width dispatch guard.',
 randtest:'Rational bias, irreducible-polynomial generation, signature filtering, selected root isolation and cleanup; both archived version branches read. Not a uniform distribution contract.',
 print:'Polynomial plus existing enclosure diagnostic rendering; no new precision requested.',
 printn:'Decimal display requests approximation at 3.333*n+10 bits, clamps n>=1 and suppresses radius; not exact serialization.',
 write:'Stream/string display and ownership transfer; current helper visibility narrowed to static. Not exact serialization.',
 roots_poly_squarefree:'Full Cartesian conjugate annihilator, denominator bound, balanced product, unique-integer reconstruction, factor/candidate enumeration, cardinality-based certified elimination, sorting and cleanup. Unchecked final degree product selected for bounded diagnostic.'};
for(const s of inventory.sources){
 const prefix=s.repo==='flint'?'src/qqbar/':'qqbar/',files=s.files.filter(f=>f.text&&f.path.startsWith(prefix)&&!coverage.some(c=>c.repo===s.repo&&c.path===f.path));
 assert.equal(files.length,s.repo==='flint'?19:16);
 for(const f of files){const name=f.path.split('/').at(-1).replace(/\.c$/,'');
  const note=f.path.includes('/test/')?(name==='main'?'Complete include list and matching test registrations. No implicit execution claim.':
   'Complete '+name+' test, inputs, mathematical identities, status guards, randomized boundaries, diagnostics and cleanup. Root test success-frequency guard only applies when iterations >100; composed-op samples one pair rather than proving the whole polynomial. Source read, not newly executed.') : notes[name];
  assert(note);assert.equal(sha(workspace+'/exact-real-references/'+s.repo+'/'+f.path),f.sha256);
  donorSources[s.repo+':'+f.path]=f.sha256;records.push({repo:s.repo,path:f.path,ranges:[[1,f.lines]],note:'Checkpoint84: '+note});
 }
}
for(const p of ['src/gr_generic/fmpz_mpoly_evaluate.c','src/gr_generic/test/t-fmpz_mpoly_evaluate.c','src/gr/qqbar.c']){
 const f=inventory.sources.find(s=>s.repo==='flint').files.find(f=>f.path===p);assert(f?.text);assert.equal(sha(workspace+'/exact-real-references/flint/'+p),f.sha256);
 const old=coverage.find(r=>r.repo==='flint'&&r.path===p);if(p==='src/gr/qqbar.c')assert.deepEqual(old.ranges,[[1,100],[1352,1538]]);else assert(!old);
 donorSources['flint:'+p]=f.sha256;records.push({repo:'flint',path:p,ranges:p==='src/gr/qqbar.c'?[[101,1351]]:[[1,f.lines]],note:'Checkpoint84: '+
  (p==='src/gr/qqbar.c'?'All formerly unread methods now read: representation conversion, exact domain/Unknown/budget outcomes, arithmetic/powers, rounding and trig adapters, squarefree roots and other-ring root conversion. Existing setup/registration ranges reread; failure root-vector cleanup selected for diagnostic.':
   'Complete delegated generic multivariate evaluator/test: explicit Horner stack, scalar register tags, variable-power score, full exponent support, non-short-circuit status aggregation and width-aware outer dispatch. Tests may accept Unknown equality; donor-only test success is not an independent value oracle.')});
}
assert.equal(records.length,38);assert.equal(records.reduce((n,r)=>n+r.ranges.reduce((a,[b,c])=>a+c-b+1,0),0),5324);
const old=json('symbolic-boundary-origin-v83.json');for(const[p,h]of Object.entries({...old.libraries,...old.configurationFiles}))assert.equal(sha(p),h,p);
const files=['flint-qqbar-remainder-v84.c','qqbar-remainder-protocol-v84.md','prepare-qqbar-remainder-v84.mjs','flint-symbolic-boundary-fixed-v83.c',
 'inventory.json','coverage.json','effective-coverage.mjs','capture.mjs','point-qualified-capture.mjs','symbolic-boundary-v83-manifest.json'];
const dir=mkdtempSync('/tmp/calcium-qqbar-remainder-v84.');
writeFileSync('qqbar-remainder-origin-v84.json',JSON.stringify({checkpoint:84,recorded:new Date().toISOString(),current:retainedSources(),
 previousSha256:sha('symbolic-boundary-v83-manifest.json'),files:Object.fromEntries(files.map(p=>[p,sha(p)])),records,donorSources,
 coverageBefore:effectiveSummary(),extensionsBefore:json('coverage-extensions.json'),extensionsBeforeSha256:sha('coverage-extensions.json'),
 rereadRanges:{'flint:doc/source/qqbar.rst':[[550,670]]},rereadHashes:{'flint:doc/source/qqbar.rst':sha(workspace+'/exact-real-references/flint/doc/source/qqbar.rst')},
 libraries:old.libraries,configurationFiles:old.configurationFiles,dir,binary:dir+'/controls',sanitized:dir+'/sanitized'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:84,status:'prepared',readRecords:38,newCompleteFiles:37,completedPartialFiles:1,newLines:5324,dir}));
