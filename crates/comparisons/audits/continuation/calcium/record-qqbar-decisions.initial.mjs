import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkQqbarDecisions} from './check-qqbar-decisions.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const workspace=resolve('../../../..'),inv=json('inventory.json'),prior=effectiveCoverage();
const before=effectiveSummary();
assert.deepEqual(before,json('fexpr-formatting-manifest.json').coverageAtBinding);
const notes={
 'qqbar.h':'Entire algebraic object layout, inline canonical-polynomial predicates, limits, arithmetic/root/decision/enclosure/expression interfaces. No general computable-real equality contract.',
 'qqbar.rst':'Entire manual: minimal-polynomial/isolation invariant, expensive arithmetic, exact component polish and explicit cache contracts, canonical root ordering, heuristic versus certified relation APIs, trusted serialization and unimplemented formula flags.',
 'validate_enclosure':'Both interval-Newton validators; existence premise and inflated uniqueness differ from existence-and-uniqueness with axis checks. Source contracts, not arbitrary-ball qualification.',
 'enclosure_raw':'Whole cached-accuracy return, rational solve, iterative Newton/fallback all-conjugate recomputation; current real-specialized route absent in archive. Recursive numerical kernels remain open.',
 'equal':'Whole canonical polynomial, disjointness/containment, iterative refinement and unique-union equality algorithm; depends on representation invariant.',
 'sgn_re':'Whole rational/exact/separated-component shortcuts, odd-coefficient axis filter and existence-certified zero real part.',
 'sgn_im':'Whole rational/exact/separated-component shortcuts, refinement and existence-certified real axis; disabled alternative also read.',
 'cmp_re':'Whole real interval/rational/conjugacy shortcuts, refinement, delayed exact difference for nonreal equal components; readonly refined work not saved.',
 'cmp_im':'Whole imaginary interval/sign/equality/reflected-conjugate shortcuts, refinement and delayed exact-difference fallback.',
 'cmp_root_order':'Whole real-first, descending real, ascending imaginary magnitude and final sign tie-break. Equal-nonreal comparator contract checked independently, not sorting stress.',
 'cmpabs':'Whole axis special cases, numerical magnitude separation, delayed exact squared-magnitude comparison.',
 'cmpabs_re':'Whole borrowed temporary absolute-enclosure view and sign-conditioned exact real-component comparisons.',
 'cmpabs_im':'Whole borrowed temporary absolute-enclosure view and sign-conditioned exact imaginary-component comparisons.',
 'get_acb':'Whole rational output and component-sign polishing, precision growth, attempted dyadic reconstruction and final rounding. Exact-component failure independently recorded; containment passes stated corpus.',
 'cache_enclosure':'Whole explicit extra-precision polish and containment-only cache swap; not automatic readonly-query memoization.',
 'init':'Whole polynomial/enclosure initialization to exact zero.',
 'clear':'Whole owned polynomial/enclosure release.',
 'set':'Whole deep polynomial/enclosure copy, not shared expression-DAG identity.',
 'set_si':'Whole signed machine integer temporary and integer constructor.',
 'set_ui':'Whole unsigned machine integer temporary and integer constructor.',
 'set_fmpq':'Whole linear canonical rational polynomial and default-precision enclosure.',
 'set_fmpz':'Whole linear integer polynomial and default-precision enclosure.',
 'set_re_im':'Whole zero-imaginary shortcut and temporary-owned arithmetic construction of x+i*y.',
 'conj':'Whole unchanged minimal polynomial and conjugated enclosure.',
 'neg':'Whole parity-selected polynomial coefficient negation and enclosure negation.',
 'abs2':'Whole real/root-of-unity/pure-imaginary/conjugate-product routes and exact zero imaginary component.',
 'hash':'Whole minimal-polynomial-only integer hash, intentional conjugate collisions; not equality evidence.',
 'csgn':'Whole real-component sign followed by imaginary-axis sign.',
 'root_ui':'Whole principal-surd fast detection, rational/root-of-unity routes, factored annihilator and unique-root selection. Invalid/resource boundary branches read, not probed.',
 'fmpq_root_ui':'Whole square/root reduction of rational numerator and denominator, principal negative-input delegation and positive surd construction. Only bounded positive square roots executed here.',
 'i':'Whole canonical x^2+1 polynomial and exact positive imaginary-unit enclosure.',
 'get_arb':'Whole real admission and polished complex extraction, otherwise indeterminate output.',
 'get_arb_re':'Whole exact zero real shortcut or polished component extraction.',
 'get_arb_im':'Whole exact zero imaginary shortcut or polished component extraction.',
 'roots_fmpz_poly':'Whole factor/content/root-isolation paths, multiplicity expansion before final qsort. No new root-list numerical or sorting-failure claim.',
 'conjugates':'Whole rational copy or irreducible-minimal-polynomial root enumeration.'
};
const implementations=Object.keys(notes).filter(n=>!n.includes('.'));
assert.equal(implementations.length,34);
const tests=['cmp_re','cmp_im','cmpabs','cmpabs_re','cmpabs_im','sgn_re','csgn','get_acb','roots_fmpz_poly','conjugates'];
const records=[],donors={};
for(const s of inv.sources){
 const prefix=s.repo==='flint'?'src/':'';
 const paths=[prefix+'qqbar.h','doc/source/qqbar.rst',...implementations.map(n=>prefix+'qqbar/'+n+'.c'),...tests.map(n=>prefix+'qqbar/test/t-'+n+'.c')];
 for(const path of paths){
  const f=s.files.find(f=>f.path===path);assert(f?.text,path);assert(!prior.some(r=>r.repo===s.repo&&r.path===path),path);
  assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,path)),f.sha256,path);
  const basename=path.split('/').at(-1),key=basename.replace(/\.c$/,'');
  const note=path.includes('/test/')?'Entire '+basename+' source: shared-backend reference/control flow, random cases, failure checks and cleanup. Read only, not newly executed; independent corpus has separate oracle and stated scope.':notes[key];
  assert(note,path);records.push({repo:s.repo,path,ranges:[[1,f.lines]],note:'Checkpoint50: '+note});
  donors[s.repo+':'+path]=f.sha256;
 }
}
assert.equal(records.length,92);
const lines=records.reduce((n,r)=>n+r.ranges[0][1],0);assert.equal(lines,8646);
const checked=checkQqbarDecisions('results/qqbar-decisions-native.stdout');
assert.deepEqual(checked,json('results/qqbar-decisions-check.stdout'));
assert.equal(checked.totalChecks,35826);assert.equal(checked.failures.length,264);
assert.deepEqual(checked.failedByCheck,{copyRootOrder:84,rootOrder:84,'exact-dyadic-imag':48,'exact-dyadic-real':48});
assert(readFileSync('results/qqbar-decisions-native.stdout').equals(readFileSync('results/qqbar-decisions-memcheck.stdout')));
const gates=['compile','native','check','memcheck','linked'].map(s=>'qqbar-decisions-'+s);
for(const tag of gates){const r=json('results/'+tag+'.json');assert.equal(r.code,tag.endsWith('-check')?1:0);assert.equal(r.signal,null);}
const previous=json('mpoly-bridge-experiment.json');
for(const[p,h]of Object.entries({...previous.libraries,...previous.configurationFiles}))assert.equal(sha(p),h,p);
for(const[p,h]of Object.entries(previous.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
const hyperReadRanges={
 'hyperreal/src/computable/node/structural_analysis.rs':[[1,270]],
 'hyperreal/src/computable/node/approximation_queries.rs':[[330,460]],
 'hyperreal/src/real/arithmetic/comparison.rs':[[1,81]],
 'hypersolve/src/algebraic.rs':[[365,940],[3570,3645]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 assert(previous.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const files=['flint-qqbar-decision-controls.c','check-qqbar-decisions.mjs','record-qqbar-decisions.mjs',
 'verify-qqbar-decisions.mjs','qqbar-decisions-findings.md','qqbar-decisions-read-records.json',
 'capture.mjs','inventory.json','effective-coverage.mjs','fexpr-formatting-manifest.json',
 ...['json','stdout','stderr'].map(e=>'results/fexpr-formatting-verify.'+e),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(files.length,28);
for(const p of files.filter(p=>p!=='qqbar-decisions-read-records.json'))sha(p);
writeFileSync('qqbar-decisions-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
const binary='/tmp/calcium-qqbar-decisions.xUeflv/controls';
const m={schema:1,checkpoint:50,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 donorSources:donors,readRecords:records,newDonorLines:lines,coverageBefore:before,coverageAtBinding:effectiveSummary(),
 previousManifest:'fexpr-formatting-manifest.json',liveSourceManifest:'mpoly-bridge-experiment.json',liveFileCount:956,hyperReadRanges,
 libraries:previous.libraries,configurationFiles:previous.configurationFiles,
 binaries:[{path:binary,sha256:sha(binary),bytes:statSync(binary).size}],gates,checks:checked,
 qualification:'Independent valid-input mathematical gate fails264 assertions across two contract issues; containment/accuracy/ordinary decisions pass stated corpus. Focused Memcheck clean. Source/evidence integrity success is not mathematical success or a full47-chain rerun.',
 production:'No new Hyper/donor change, candidate or matched benchmark. One18152-byte executable reuses existing libraries; no broad build, cleanup/deletion, external report, commit or push.',
 scope:'92 new complete qqbar header/manual/implementation/test reads. Algebraic arithmetic/relations, recursive kernels and full original ecosystem remain incomplete.'};
writeFileSync('qqbar-decisions-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:50,boundFiles:files.length,readRecords:records.length,newDonorLines:lines,
 coverage:m.coverageAtBinding,mathematicalStatus:checked.status,failures:checked.failedByCheck,qualification:m.qualification}));
