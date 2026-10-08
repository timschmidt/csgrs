import {readFileSync,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sha,json,retainedSources,workspace} from './zero-factor-retained-sources-v75.mjs';
import {target,cargoEnv} from './point-qualified-capture.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {capability} from './check-twelfth-capability-v77.mjs';
import {regressions} from './check-twelfth-regressions-v77.mjs';

export const developmentGates=['twelfth-tests-v77','twelfth-oracle-tests-v77','twelfth-tests-all-features-v77','twelfth-capability-debug-v77',
 'twelfth-fmt-v77','twelfth-private-fmt-v77','twelfth-default-debug-v77','twelfth-all-debug-v77','twelfth-all-release-v77'];
export const failedGates=['twelfth-tests-initial-v77','twelfth-clippy-v77','twelfth-regression-driver-v77'];
export const emptyGates=['twelfth-final-driver-v77','twelfth-support-driver-v77','twelfth-capability-check-v77','twelfth-regression-check-v77'];
export const finalGates=['twelfth-clippy-final-v77','twelfth-fmt-final-v77','twelfth-private-fmt-final-v77',
 ...['candidate','baseline'].flatMap(v=>['debug','release'].flatMap(p=>['default','all'].map(f=>'twelfth-'+v+'-'+f+'-'+p+'-final-v77'))),
 ...['debug','release'].map(p=>'twelfth-capability-'+p+'-final-v77'),
 ...['candidate','baseline'].map(v=>'twelfth-metadata-'+v+'-v77'),
 ...['rustc','cargo','valgrind'].map(v=>'twelfth-'+v+'-version-v77'),
 'twelfth-wasm-build-v77','twelfth-capability-memcheck-v77','twelfth-capability-check-approved-v77','twelfth-regression-check-approved-v77',
 'twelfth-final-driver-approved-v77','twelfth-support-driver-approved-v77'];
export const evidenceFiles=['prepare-twelfth-relation-v77.mjs','twelfth-relation-origin-v77.json','twelfth-relation-protocol-v77.md',
 'twelfth-relation-math-v77.md','bind-twelfth-relation-v77.mjs','twelfth-relation-binding-v77.json','bind-twelfth-final-v77.mjs','twelfth-final-binding-v77.json',
 'twelfth-relation-tests-initial-v77.rs','twelfth-relation-tests-focused-v77.rs','twelfth-relation-tests-preclippy-v77.rs',
 'run-twelfth-relation-v77.mjs','run-twelfth-final-v77.mjs','run-twelfth-support-v77.mjs','check-twelfth-capability-v77.mjs',
 'check-twelfth-regressions-v77.mjs','check-twelfth-evidence-v77.mjs','verify-twelfth-relation-v77.mjs',
 'twelfth-capability-v77/Cargo.toml','twelfth-capability-v77/Cargo.lock','twelfth-capability-v77/src/main.rs',
 'qqbar-trig-v76-manifest.json','qqbar-trig-hyper-v76/src/main.rs','capture.mjs','point-qualified-capture.mjs'];
const text=p=>readFileSync(p,'utf8');
function gate(tag,cwd,command,args){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);
 assert.equal(g.cwd,resolve(cwd));assert.equal(g.command,command);assert.deepEqual(g.args,args);
 assert(Number.isFinite(Date.parse(g.started))&&Date.parse(g.finished)>=Date.parse(g.started));
 return g;
}
export function twelfthEvidence(){
 const source=retainedSources(),b=json('twelfth-final-binding-v77.json'),before=json('twelfth-relation-binding-v77.json'),o=json('twelfth-relation-origin-v77.json');
 const previous=json('qqbar-trig-v76-manifest.json');assert.equal(sha('qqbar-trig-v76-manifest.json'),o.previousManifestSha256);
 for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 assert.deepEqual(source,b.current);assert.deepEqual(source,o.current);assert.equal(b.previousBindingSha256,sha('twelfth-relation-binding-v77.json'));
 assert.equal(Object.keys(b.sources).length,183);assert.equal(Object.keys(o.originalSources).length,181);
 assert.equal(sha('twelfth-relation-tests-preclippy-v77.rs'),before.sources['hyperreal/src/computable/node/twelfth_relation_tests.rs']);
 for(const[p,h]of Object.entries(b.sources)){assert.equal(sha(b.root+'/'+p),h,p);if(p!=='hyperreal/src/computable/node/twelfth_relation_tests.rs')assert.equal(h,before.sources[p]);}
 for(const binding of [before,b])for(const[p,h]of Object.entries(binding.files))assert.equal(sha(p),h,p);
 for(const[p,h]of Object.entries(o.originalSources)){assert.equal(sha(workspace+'/'+p),h,p);if(!b.changed.includes(p))assert.equal(b.sources[p],h,p);}
 const tests=regressions(),publicProbe=capability(),cwd=b.root+'/hyperreal';
 for(const v of ['candidate','baseline'])for(const p of ['debug','release'])for(const f of ['default','all']){
  gate('twelfth-'+v+'-'+f+'-'+p+'-final-v77',v==='candidate'?cwd:workspace+'/hyperreal','env',
   [...cargoEnv,'cargo','test','--locked','--offline',...(p==='release'?['--release']:[]),...(f==='all'?['--all-features']:[]),'--lib','--tests']);
 }
 for(const p of ['debug','release'])gate('twelfth-capability-'+p+'-final-v77','twelfth-capability-v77','env',[...cargoEnv,'cargo','run','--locked','--offline',...(p==='release'?['--release']:[])]);
 gate('twelfth-clippy-final-v77',cwd,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
 gate('twelfth-fmt-final-v77',cwd,'cargo',['fmt','--all','--','--check']);
 gate('twelfth-private-fmt-final-v77',cwd,'rustfmt',['--edition','2024','--check','src/computable/node/twelfth_relation.rs','src/computable/node/twelfth_relation_tests.rs']);
 gate('twelfth-wasm-build-v77',cwd,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--all-features','--lib','--target','wasm32-unknown-unknown']);
 const metadata={};
 for(const v of ['candidate','baseline']){
  gate('twelfth-metadata-'+v+'-v77',v==='candidate'?cwd:workspace+'/hyperreal','cargo',['metadata','--locked','--offline','--all-features','--format-version','1']);
  const m=json('results/twelfth-metadata-'+v+'-v77.stdout');assert.equal(m.packages.length,126);assert.equal(m.resolve.nodes.length,126);
  assert.equal(m.workspace_root,resolve(v==='candidate'?cwd:workspace+'/hyperreal'));
  metadata[v]=JSON.parse(JSON.stringify(m).replaceAll(m.workspace_root,'<HYPERREAL>'));
 }
 assert.deepEqual(metadata.candidate,metadata.baseline);
 for(const [cmd,args]of [['rustc',['--version','--verbose']],['cargo',['--version']],['valgrind',['--version']]]){
  gate('twelfth-'+cmd+'-version-v77','.',cmd,args);assert(text('results/twelfth-'+cmd+'-version-v77.stdout').length>10);
 }
 for(const[which,value]of [['capability',publicProbe],['regression',tests]]){
  gate('twelfth-'+which+'-check-approved-v77','.','node',['check-twelfth-'+(which==='capability'?'capability':'regressions')+'-v77.mjs']);
  assert.deepEqual(json('results/twelfth-'+which+'-check-approved-v77.stdout'),value);
  assert.equal(text('results/twelfth-'+which+'-check-approved-v77.stderr'),'');
 }
 for(const [name,script,count,status]of [
  ['final','run-twelfth-final-v77.mjs',14,'matched-final-source-regression-and-capability-captures-finished'],
  ['support','run-twelfth-support-v77.mjs',7,'metadata-toolchain-wasm-compilation-finished']]){
  gate('twelfth-'+name+'-driver-approved-v77','.','node',[script]);
  const rows=text('results/twelfth-'+name+'-driver-approved-v77.stdout').trimEnd().split('\n').map(s=>JSON.parse(s));
  assert.equal(rows.length,count);assert.equal(rows.at(-1).status,status);assert.equal(rows.at(-1).retained,false);
  for(const row of rows.slice(0,-1))assert.equal(row.finished,json('results/'+row.reusedGate+'.json').finished);
  assert.equal(text('results/twelfth-'+name+'-driver-approved-v77.stderr'),'');
 }
 const binary=target+'/release/twelfth-capability-v77';
 gate('twelfth-capability-memcheck-v77','.','valgrind',['--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary]);
 assert.equal(text('results/twelfth-capability-memcheck-v77.stdout'),text('results/twelfth-capability-release-final-v77.stdout'));
 const mem=text('results/twelfth-capability-memcheck-v77.stderr');
 assert(/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/.test(mem));
 for(const kind of ['definitely','indirectly','possibly'])assert(new RegExp(kind+' lost: 0 bytes in 0 blocks').test(mem));
 const heap=mem.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);assert(heap);
 const reachable=mem.match(/still reachable: ([\d,]+) bytes in ([\d,]+) blocks/);assert(reachable);
 const number=s=>Number(s.replaceAll(',',''));
 for(const tag of developmentGates){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);}
 assert.deepEqual(failedGates.map(t=>json('results/'+t+'.json').code),[101,101,1]);
 for(const tag of failedGates)assert.equal(json('results/'+tag+'.json').signal,null);
 assert(/no associated function or constant named `sqrt2`/.test(text('results/twelfth-tests-initial-v77.stderr')));
 assert(/needless.range.loop/.test(text('results/twelfth-clippy-v77.stderr')));
 for(const tag of emptyGates){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(text('results/'+tag+'.stdout'),'');}
 assert.deepEqual(effectiveSummary(),previous.coverage);
 return {checkpoint:77,status:'initial-candidate-correctness-qualified-not-retained',source,candidateFiles:183,originalFiles:181,
  changed:b.changed,added:b.added,newFiles:b.newFiles,coverage:previous.coverage,newDonorReadFiles:0,newDonorReadLines:0,
  publicProbe,regressions:tests,metadata:{packages:126,nodes:126,fullyEqualAfterRootNormalization:true},
  memory:{errors:0,lostBytes:0,allocations:number(heap[1]),frees:number(heap[2]),requestedBytes:number(heap[3]),
   stillReachableBytes:number(reachable[1]),stillReachableBlocks:number(reachable[2]),outputByteIdentical:true,scope:'Whole public probe including setup/output/caches; not matched allocation or peak-memory measurement.'},
  independentCases:{fieldSigns:6561,polynomialMatrixPairs:512,pellConvergents:256,pellMixedCases:512,rationalRootCases:336},
  wasm:'all-feature release compilation only; runtime qualification pending',finalGates,developmentGates,failedGates,unusableEmptyCompletionGates:emptyGates,
  executables:['debug','release'].map(p=>{const path=target+'/'+p+'/twelfth-capability-v77';return {path,bytes:statSync(path).size,sha256:sha(path)};}),
  retainedContinuationTransfers:7,productionChanges:0,
  limits:'No candidate timing, matched allocation, binary-size comparison, WASM execution, downstream consumer, doctest/bench/fuzz or full CI result. Historical source/evidence preserved; four empty sandbox completion captures rejected despite zero exit status. All source changes remain isolated.',
  next:'Measure intended-path and bypass costs with matched protocols, including cache/refinement/query-order cases, memory, native/WASM and consumer/size gates; retain only if worthwhile. Continue remaining references and inventory reconciliation.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(twelfthEvidence()));
