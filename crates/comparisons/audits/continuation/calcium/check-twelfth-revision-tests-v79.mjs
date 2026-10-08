import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,json} from './twelfth-revision-sources-v79.mjs';
import {workspace} from './zero-factor-retained-sources-v75.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
import {capability} from './check-twelfth-capability-v77.mjs';
export function gate(tag,command,args,cwd){
 const g=json('results/'+tag+'.json');assert.equal(g.code,0,tag);assert.equal(g.signal,null,tag);
 if(command)assert.equal(g.command,command);if(args)assert.deepEqual(g.args,args);if(cwd)assert.equal(g.cwd,resolve(cwd));
 assert(Number.isFinite(Date.parse(g.started))&&Date.parse(g.finished)>=Date.parse(g.started));return g;
}
export function regressionEvidence(){
 const b=sources(),root=b.root+'/hyperreal',gates=[],results=[],membership={};
 const check=(tag,command,args,cwd)=>{const g=gate(tag,command,args,cwd);gates.push(tag);return g;};
 for(const variant of ['candidate','baseline'])for(const profile of ['debug','release'])for(const features of ['default','all']){
  const tag='twelfth-'+variant+'-'+features+'-'+profile+'-v79',cwd=variant==='candidate'?root:workspace+'/hyperreal';
  const args=[...cargoEnv,'cargo','test','--locked','--offline',...(profile==='release'?['--release']:[]),...(features==='all'?['--all-features']:[]),'--lib','--tests'];
  const g=check(tag,'env',args,cwd),stdout=readFileSync('results/'+tag+'.stdout','utf8'),stderr=readFileSync('results/'+tag+'.stderr','utf8');
  assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(stderr));
  const names=[...stdout.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();
  const suites=[...stdout.matchAll(/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/gm)].map(m=>m.slice(1).map(Number));
  assert.equal(suites.length,features==='all'?14:13);const totals=suites.reduce((a,r)=>a.map((v,i)=>v+r[i]),[0,0,0,0,0]);
  assert.deepEqual(totals,[names.length,0,0,0,0]);assert.equal(names.length,variant==='baseline'?(features==='all'?859:756):(features==='all'?870:766));
  const oldTag='twelfth-'+variant+'-'+features+'-'+profile+'-final-v77';
  const oldNames=[...readFileSync('results/'+oldTag+'.stdout','utf8').matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();
  const added=names.filter(n=>n.startsWith('computable::node::twelfth_relation_reference_tests::'));
  assert.equal(added.length,variant==='candidate'?3:0);assert.deepEqual(names.filter(n=>!added.includes(n)),oldNames);
  const key=variant+':'+features;if(key in membership)assert.deepEqual(names,membership[key]);else membership[key]=names;
  results.push({variant,profile,features,passed:names.length,suites:suites.length,finished:g.finished});
 }
 check('twelfth-fmt-v79','cargo',['fmt','--all','--','--check'],root);
 check('twelfth-private-fmt-v79','rustfmt',['--edition','2024','--check','src/computable/node/twelfth_relation.rs','src/computable/node/twelfth_relation_tests.rs','src/computable/node/twelfth_relation_reference_tests.rs'],root);
 check('twelfth-clippy-v79','env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings'],root);
 for(const variant of ['candidate','baseline'])check('twelfth-metadata-'+variant+'-v79','cargo',['metadata','--locked','--offline','--all-features','--format-version','1'],variant==='candidate'?root:workspace+'/hyperreal');
 const base=json('results/twelfth-metadata-baseline-v79.stdout');
 const candidate=JSON.parse(readFileSync('results/twelfth-metadata-candidate-v79.stdout','utf8').replaceAll(resolve(root),workspace+'/hyperreal'));
 assert.deepEqual(candidate,base);assert.equal(base.packages.length,126);assert.equal(base.resolve.nodes.length,126);
 check('twelfth-wasm-build-v79','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--all-features','--lib','--target','wasm32-unknown-unknown'],root);
 check('twelfth-capability-lock-v79','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],'twelfth-capability-v79');
 const prior=capability();
 for(const profile of ['debug','release']){
  const tag='twelfth-capability-'+profile+'-v79';
  check(tag,'env',[...cargoEnv,'cargo','run','--locked','--offline',...(profile==='release'?['--release']:[])],'twelfth-capability-v79');
  assert.equal(readFileSync('results/'+tag+'.stdout','utf8'),readFileSync('results/twelfth-capability-'+profile+'-final-v77.stdout','utf8'));
 }
 for(const[tool,args]of [['rustc',['--version','--verbose']],['cargo',['--version']]]){
  check('twelfth-'+tool+'-version-v79',tool,args,'.');
  assert.equal(readFileSync('results/twelfth-'+tool+'-version-v79.stdout','utf8'),readFileSync('results/twelfth-'+tool+'-version-v77.stdout','utf8'));
 }
 const outer='twelfth-regressions-confirmed-v79';check(outer,'node',['run-twelfth-revision-tests-v79.mjs'],'.');
 const rows=readFileSync('results/'+outer+'.stdout','utf8').trim().split('\n').map(JSON.parse);
 assert.equal(rows.length,20);assert.deepEqual(rows.at(-1),{checkpoint:79,status:'revision-regression-captures-complete',retained:false,candidateFiles:184});
 assert.deepEqual(rows.slice(0,-1).map(r=>r.reusedGate).sort(),gates.filter(t=>t!==outer).sort());
 return {checkpoint:79,status:'revision-correctness-qualified',results,gates,metadataPackages:126,metadataNodes:126,
  publicRecordsPerProfile:prior.records,publicOutputsByteIdenticalTo77:true,independentOracleTestsRetained:true,
  differential:{sparsePairs:1024,coefficientBoundaryCases:160,tangentAndCotangentArguments:485,expressionCases:8760,unsupportedAndDeepCases:670},
  wasmCompilation:true,wasmRuntime:false,retained:false,
  limits:'Three added differential tests supplement unchanged independent mathematical tests; they do not prove arbitrary expression closure. Full CI/downstream/runtime/size gates remain separate.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(regressionEvidence()));
