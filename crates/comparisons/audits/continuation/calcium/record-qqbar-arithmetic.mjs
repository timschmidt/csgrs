import {readFileSync,writeFileSync,statSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkQqbarArithmetic} from './check-qqbar-arithmetic.mjs';
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const workspace=resolve('../../../..'),inv=json('inventory.json');
const before=effectiveSummary(),prior=effectiveCoverage(),previous=json('qqbar-decisions-manifest.json');
assert(!existsSync('qqbar-arithmetic-manifest.json'));
assert(!existsSync('qqbar-arithmetic-read-records.json'));
assert.deepEqual(before,previous.coverageAtBinding);
const notes={
 affine_transform:'Full affine substitution, primitive normalization, rational/zero-slope paths and selected-root enclosure validation.',
 composed_op:'Full power-sum/log-derivative, Borel/Hadamard reconstruction, factor/unique-root selection and exactly checked rational-guess fast path; disabled branches also read.',
 add:'Full zero/rational/fused-affine and generic algebraic addition dispatch.',
 sub:'Full zero/rational/fused-affine and generic algebraic subtraction dispatch.',
 mul:'Full zero/unit/rational/equal-input square, positive-surd and general product paths.',
 div:'Full nonzero-divisor precondition, zero/unit/rational/inverse/surd and general division paths; no invalid-input probes.',
 inv:'Full rational inversion, coefficient reversal/sign normalization and reciprocal-enclosure certification.',
 mul_2exp_si:'Full exact dyadic coefficient scaling and common two-valuation removal; resource guards read, not probed.',
 pow:'Full rational/root-of-unity/deflation/square/surd/polynomial-evaluation paths and partial algebraic-exponent admission.',
 fmpq_pow_si_ui:'Full rational-power helper, root/power order and sign-dependent reciprocal construction.',
 evaluate_fmpq_poly:'Full rational/constant/affine/remainder and number-field multiplication-matrix minimal polynomial evaluation; enclosure selection and lifetime review.',
 evaluate_fmpz_poly:'Full integer-polynomial evaluation wrapper with exact unit denominator.',
 equal_fmpq_poly_val:'Full modular polynomial-composition identity check and interval-uniqueness embedding certificate.',
 express_in_field:'Full numeric relation candidate, structural field filters and exact polynomial-value certificate; unused size/flags source observation, no direct LLL execution.',
 guess:'Full numerical relation guessing, factor/height filters, existence/uniqueness attempt and overlap selection; heuristic result requires caller certification.',
 acb_lindep:'Full finite-input/accuracy scaling, rounded lattice construction, LLL invocation and heuristic residual test; direct LLL qualification not performed.'
};
const tests=['add','sub','mul','div','inv','mul_2exp_si','pow','evaluate_fmpq_poly','equal_fmpq_poly_val','express_in_field','guess'];
const records=[],donors={};
for(const s of inv.sources){
 const prefix=s.repo==='flint'?'src/':'';
 for(const [test,names]of [[false,Object.keys(notes)],[true,tests]])for(const name of names){
  const path=prefix+'qqbar/'+(test?'test/t-':'')+name+'.c';
  const f=s.files.find(f=>f.path===path);assert(f?.text,path);
  assert(!prior.some(r=>r.repo===s.repo&&r.path===path),path);
  assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,path)),f.sha256,path);
  records.push({repo:s.repo,path,ranges:[[1,f.lines]],note:'Checkpoint51: '+(test?
   'Full upstream test source, including shared-backend identities, conditional assertions, alias calls and cleanup. Read only, not newly executed.':notes[name])});
  donors[s.repo+':'+path]=f.sha256;
 }
}
assert.equal(records.length,54);
const lines=records.reduce((n,r)=>n+r.ranges[0][1],0);assert.equal(lines,7223);
const checked=checkQqbarArithmetic('results/qqbar-arithmetic-native.stdout');
assert.deepEqual(checked,json('results/qqbar-arithmetic-check.stdout'));
assert.equal(checked.status,'pass');assert.equal(checked.totalChecks,40352);assert.equal(checked.records,10945);
assert(readFileSync('results/qqbar-arithmetic-native.stdout').equals(readFileSync('results/qqbar-arithmetic-memcheck.stdout')));
assert.equal(readFileSync('results/qqbar-arithmetic-native.stdout').length,2032198);
const memory=readFileSync('results/qqbar-arithmetic-memcheck.stderr','utf8');
assert(memory.includes('in use at exit: 0 bytes in 0 blocks'));
assert(memory.includes('3,180,130 allocs, 3,180,130 frees, 150,670,769 bytes allocated'));
assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
const gates=['compile','native','check','memcheck','linked'].map(s=>'qqbar-arithmetic-'+s);
for(const tag of gates){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 if(!tag.endsWith('-memcheck'))assert.equal(readFileSync('results/'+tag+'.stderr').length,0);}
const live=json('mpoly-bridge-experiment.json');assert.equal(Object.keys(live.liveSources).length,956);
for(const[p,h]of Object.entries(live.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const[p,h]of Object.entries({...previous.libraries,...previous.configurationFiles}))assert.equal(sha(p),h,p);
const hyperReadRanges={
 'hypersolve/src/algebraic_binary.rs':[[1,1089]],
 'hypersolve/src/integer_interpolation.rs':[[1,750]],
 'hypersolve/src/resultant.rs':[[185,340],[640,740],[1215,1275]]
};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 assert(live.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const files=['flint-qqbar-arithmetic-controls.c','check-qqbar-arithmetic.mjs','record-qqbar-arithmetic.mjs',
 'verify-qqbar-arithmetic.mjs','qqbar-arithmetic-findings.md','qqbar-arithmetic-read-records.json',
 'capture.mjs','inventory.json','effective-coverage.mjs','qqbar-decisions-manifest.json',
 ...['json','stdout','stderr'].map(e=>'results/qqbar-decisions-verify.'+e),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(e=>'results/'+t+'.'+e))];
assert.equal(files.length,28);
for(const p of files.filter(p=>p!=='qqbar-arithmetic-read-records.json'))sha(p);
const binary='/tmp/calcium-qqbar-arithmetic.oUgkCO/controls';
const binaries=[{path:binary,sha256:sha(binary),bytes:statSync(binary).size}];
writeFileSync('qqbar-arithmetic-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
const m={schema:1,checkpoint:51,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 donorSources:donors,readRecords:records,newDonorLines:lines,coverageBefore:before,coverageAtBinding:effectiveSummary(),
 previousManifest:'qqbar-decisions-manifest.json',liveSourceManifest:'mpoly-bridge-experiment.json',liveFileCount:956,
 hyperReadRanges,libraries:previous.libraries,configurationFiles:previous.configurationFiles,binaries,gates,checks:checked,
 qualification:'Independent biquadratic field oracle passes 40352 mathematical/state assertions including full minimal/composed polynomials and root enclosures. Focused Memcheck clean; no direct LLL, archived runtime, upstream test execution or performance claim.',
 production:'No production/donor change or retained transfer. One small executable reuses existing native libraries; no Rust/dependency build, source copy, cleanup/deletion, external report, commit or push.',
 candidate:'Power-sum/Newton construction is a genuine alternative to Hyper binary resultant sampling, but no Hyper prototype or matched benchmark yet. Preserve reducible/repeated carriers, zero-root divisor fallback, degree/policy/evidence contracts and qualify full public queries before retention.',
 scope:'54 full selected arithmetic/relation/test reads, not all qqbar or recursive support; full original ecosystem inventory remains incomplete.'};
writeFileSync('qqbar-arithmetic-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:51,boundFiles:files.length,readRecords:records.length,newDonorLines:lines,
 coverage:m.coverageAtBinding,checks:checked,binaries,candidate:m.candidate}));
