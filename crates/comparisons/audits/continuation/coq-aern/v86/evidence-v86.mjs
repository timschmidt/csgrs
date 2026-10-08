import {readFileSync,readdirSync,writeFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const ws='/home/tim/Documents/GitHub/workspace',qc=ws+'/exactcore-hyper-comparison/audits/continuation/coq-aern',here=qc+'/v86',c=resolve(qc,'../calcium');
assert.equal(process.cwd(),here);
const read=p=>readFileSync(p,'utf8'),json=p=>JSON.parse(read(p)),sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const run=(script,cwd=here)=>JSON.parse(execFileSync(process.execPath,[script],{cwd,encoding:'utf8',timeout:120000,maxBuffer:1024*1024}));
const previous=run('evidence-v85.mjs',qc),old=json(qc+'/manifest-v85.json');
assert.equal(previous.status,'verified');assert.equal(previous.directArtifacts,62);assert.equal(previous.previousDirectArtifacts,4498);
assert.equal(previous.liveFiles,957);assert.equal(previous.isolatedCandidateFiles,184);
const coverage=run('read-records-v86.mjs');assert.deepEqual(coverage,{status:'verified',delta:{newComplete:8,completedPartial:1,newLines:4454},totals:{complete:30,partial:0,lines:9120}});
const readRecords=json('read-records-v86.json'),inv=json(qc+'/inventory-v85.json');
assert.equal(inv.files.filter(x=>x.kind==='text'&&!readRecords.cumulative.some(y=>y.path===x.path)).length,58);
const sqrt=read(inv.root+'/formalization/Analysis/Sqrt.v').split('\n');
assert.equal(sqrt[627].trim(),'Admitted.');assert.equal(sqrt[709].trim(),'Admitted.');assert.equal(sqrt.filter(x=>/^Admitted\./.test(x)).length,2);
const model=run('model-v86.mjs'),branches=run('branches-v86.mjs');
assert.equal(model.status,'verified-independent-exact-rational-model');
assert.deepEqual([model.normalizedInputs,model.normalizedBounds,model.fastBounds,model.plannerChecks,model.scaleCases,model.corruptionControls],[35,350,17920,65589,1161,6]);
assert.equal(model.sha256,'a54e7ce5625d38e1f1934b0911750d8788298a11f9def96b1bf4e12d6ffd4615');
assert.equal(branches.status,'verified-rational-branch-model');assert.equal(branches.branches,2048);assert.equal(branches.nonzeroInputs,1088);assert.equal(branches.ambiguousOutputs,240);
assert.equal(branches.corruptionControls,5);assert.equal(branches.sha256,'42c01ef9c211b4f7a0ffc56a43d2f0658dc380df0e97438e7fdb8eeed1fedb24');
const files={},gates=[];const bind=p=>{const absolute=resolve(here,p);files[absolute]=sha(absolute);};
const gate=(tag,cwd,command,args,classification='success',code=0)=>{
 const path=c+'/results/'+tag,g=json(path+'.json');
 assert.equal(g.tag,tag);assert.equal(g.cwd,cwd);assert.equal(g.command,command);assert.deepEqual(g.args,args);
 assert.equal(g.code,code);assert.equal(g.signal,null);assert(!g.error);assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);
 assert(Number.isFinite(Date.parse(g.started)));assert(Date.parse(g.finished)>=Date.parse(g.started));
 for(const ext of ['json','stdout','stderr'])bind(path+'.'+ext);
 gates.push({tag,classification,finished:g.finished});return path;
};
const denied=gate('coq-aern-current85-before-v86',qc,'node',['evidence-v85.mjs'],'environment-subprocess-denied',1);
assert(read(denied+'.stderr').includes('EPERM'));assert.equal(read(denied+'.stdout'),'');
const approved=gate('coq-aern-current85-before-approved-v86',qc,'node',['evidence-v85.mjs']);
assert.deepEqual(json(approved+'.stdout'),previous);assert.equal(read(approved+'.stderr'),'');
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const zero=gate('coq-aern-sqrt-regression-debug-v86',ws+'/hyperreal','env',[...env,'cargo','test','--offline','--locked','--lib','demand_sized_sqrt_seeds_match_directed_mpfr','--','--exact'],'unusable-zero-test-filter');
assert(read(zero+'.stdout').includes('running 0 tests'));assert(read(zero+'.stdout').includes('0 passed; 0 failed; 0 ignored; 0 measured; 698 filtered out'));
const tests={};const reusedExecutables=[];
for(const profile of ['debug','release']){
 const p=gate('coq-aern-sqrt-regression-'+profile+'-qualified-v86',ws+'/hyperreal','env',[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--lib','sqrt']);
 const out=read(p+'.stdout'),err=read(p+'.stderr');assert(out.includes('running 23 tests'));
 assert(out.includes('23 passed; 0 failed; 0 ignored; 0 measured; 675 filtered out'));
 const names=[...out.matchAll(/^test (\S+) \.\.\. ok$/gm)].map(m=>m[1]).sort();assert.equal(names.length,23);assert.equal(new Set(names).size,23);
 assert(names.includes('computable::node::tests::demand_sized_sqrt_seeds_match_directed_mpfr'));
 assert(names.includes('computable::node::tests::sqrt_square_unresolved_sign_reuses_one_child_and_does_not_cache_aborts'));
 assert(names.includes('computable::node::tests::sqrt_square_raw_nodes_preserve_exact_prefix_bounds_and_history'));
 assert(err.includes('Finished'));assert(!/Compiling|warning:|error:/.test(err));
 const match=err.match(/Running unittests src\/lib.rs \(([^)]+)\)/);assert(match);const binary=match[1];assert(binary.startsWith('/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/'+profile+'/deps/hyperreal-'));
 bind(binary);reusedExecutables.push({path:binary,bytes:statSync(binary).size,sha256:sha(binary)});tests[profile]=names;
}
assert.deepEqual(tests.debug,tests.release);
const mp=gate('coq-aern-model-v86',here,'node',['model-v86.mjs']);assert.deepEqual(json(mp+'.stdout'),model);assert.equal(read(mp+'.stderr'),'');
for(const good of [false,true]){
 const p=gate('coq-aern-branches'+(good?'-approved':'')+'-v86',here,'node',['branches-v86.mjs'],good?'success':'unusable-empty-sandbox-capture');
 assert.equal(read(p+'.stderr'),'');if(good)assert.deepEqual(json(p+'.stdout'),branches);else assert.equal(read(p+'.stdout'),'');
}
assert.equal(gates.length,8);assert.equal(new Set(gates.map(g=>g.tag)).size,8);
const rereads=[
 ['hyperreal/AGENTS.md',[[1,19]]],
 ['hyperreal/Cargo.toml',[[1,93]]],
 ['hyperreal/src/computable/approximation/exp_sqrt.rs',[[110,265]]],
 ['hyperreal/src/computable/node/roots_inverse_hyperbolic.rs',[[1,140]]],
 ['hyperreal/src/real/arithmetic/elementary_functions.rs',[[130,235]]],
 ['hyperreal/src/computable/node/tests.rs',[[815,945]]],
 ['hyperlimit/src/lib.rs',[[1,125]]],
 ['hyperlattice/src/lib.rs',[[1,97]]],
 ['hyperlattice/src/complex.rs',[[1,165]]],
 ['hyperlimit/STACK_RETAINED_FACT_AUDIT.md',[[1,80]]],
].map(([path,ranges])=>{const text=read(ws+'/'+path),lines=text.split('\n').length-(text.endsWith('\n')?1:0);
 const bounded=ranges.map(([a,b])=>[a,Math.min(b,lines)]);assert(bounded.every(([a,b])=>a<=b));bind(ws+'/'+path);return{path,ranges:bounded,sha256:sha(ws+'/'+path),newDonorReadCredit:false};});
const ownFiles=readdirSync('.').filter(p=>/\.(mjs|md|json)$/.test(p)&&p!=='manifest-v86.json').sort();
for(const p of ownFiles){bind(p);assert(!/[ \t]+$/m.test(read(p)),'authored trailing whitespace: '+p);}
for(const r of readRecords.records)bind(inv.root+'/'+r.path);
bind(qc+'/manifest-v85.json');bind(qc+'/inventory-v85.json');bind(qc+'/read-records-v85.json');bind(c+'/capture.mjs');
for(const [p,h]of Object.entries(old.liveSources))assert.equal(sha(ws+'/'+p),h,'live after checks '+p);
const result={checkpoint:86,status:'qualified-source-and-square-root-contract-models',previousSha256:sha(qc+'/manifest-v85.json'),previousDirectArtifacts:62,
 previousTransitiveArtifacts:4498,coverage:readRecords.totals,delta:readRecords.delta,remainingTextFiles:58,proofHoles:[{file:'formalization/Analysis/Sqrt.v',line:628,lemma:'csqrt_solutions'},{file:'formalization/Analysis/Sqrt.v',line:710,lemma:'csqrt_small'}],
 rereads,model,branches,tests,reusedExecutables,gates,files,liveFiles:957,isolatedCandidateFiles:184,productionChanges:0,newRetainedTransfers:0,retainedContinuationTransfers:7,
 limits:'Focused default Hyper tests and independent rational models only; no Coq/GHC build, proof repair, matched timing/memory/size improvement, all-feature/full-stack/WASM or whole-inventory completion.'};
if(process.argv.includes('--record'))writeFileSync('manifest-v86.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,json('manifest-v86.json'));
console.log(JSON.stringify({checkpoint:86,status:process.argv.includes('--record')?'sealed':'verified',directArtifacts:Object.keys(files).length,
 previousDirectArtifacts:62,previousTransitiveArtifacts:4498,liveFiles:957,isolatedCandidateFiles:184,delta:result.delta,coverage:result.coverage,
 captures:8,classifications:{success:5,environmentFailure:1,unusableZeroTestFilter:1,unusableEmptySandboxCapture:1},
 hyperTests:{debug:23,release:23},modelTrace:model.sha256,branchTrace:branches.sha256,productionChanges:0,fullAuditComplete:false}));
