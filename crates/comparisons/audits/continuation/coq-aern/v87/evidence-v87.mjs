import {readFileSync,readdirSync,writeFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const ws='/home/tim/Documents/GitHub/workspace',qc=ws+'/exactcore-hyper-comparison/audits/continuation/coq-aern',here=qc+'/v87',c=resolve(qc,'../calcium');
assert.equal(process.cwd(),here);
const read=p=>readFileSync(p,'utf8'),json=p=>JSON.parse(read(p)),sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const run=(script,cwd=here)=>JSON.parse(execFileSync(process.execPath,[script],{cwd,encoding:'utf8',timeout:120000,maxBuffer:1024*1024}));
const previous=run('evidence-v86.mjs',qc+'/v86');
assert.equal(previous.status,'verified');assert.equal(previous.directArtifacts,57);assert.equal(previous.previousDirectArtifacts,62);assert.equal(previous.previousTransitiveArtifacts,4498);
assert.equal(previous.liveFiles,957);assert.equal(previous.isolatedCandidateFiles,184);
const coverage=run('read-records-v87.mjs');assert.deepEqual(coverage,{status:'verified',delta:{newComplete:6,newLines:2864},totals:{complete:36,partial:0,lines:11984}});
const reads=json('read-records-v87.json'),inv=json(qc+'/inventory-v85.json');
assert.equal(inv.files.filter(f=>f.kind==='text'&&!reads.cumulative.some(r=>r.path===f.path)).length,52);
const model=run('model-v87.mjs'),bench=run('bench-contract-v87.mjs');
assert.equal(model.sha256,'35da1b6cb854c77c221298576f498b8fcfd98d3171ea21ae216be6f582f5ca34');
assert.deepEqual([model.scalarTriples,model.intervalCases,model.convexCases,model.vectorCases,model.pathChecks,model.rootCounterexamples,model.formalZeroChecks,model.corruptionControls],[35937,71874,2176,512,8448,510,16320,8]);
assert.equal(model.nativeDonorExecution,false);assert.equal(bench.runnerExecuted,false);assert.equal(bench.nativeHaskellExecuted,false);
assert.equal(bench.activeDispatchers.length,17);assert.equal(bench.selected.length,2);assert.equal(bench.plannedInvocationsIfAllSucceeded,100);
const files={},gates=[];const bind=p=>{const absolute=resolve(here,p);files[absolute]=sha(absolute);};
const gate=(tag,cwd,command,args,classification='success',code=0)=>{
 const path=c+'/results/'+tag,g=json(path+'.json');
 assert.equal(g.tag,tag);assert.equal(g.cwd,cwd);assert.equal(g.command,command);assert.deepEqual(g.args,args);
 assert.equal(g.code,code);assert.equal(g.signal,null);assert(!g.error);assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);
 assert(Number.isFinite(Date.parse(g.started)));assert(Date.parse(g.finished)>=Date.parse(g.started));
 for(const ext of ['json','stdout','stderr'])bind(path+'.'+ext);
 gates.push({tag,classification,finished:g.finished});return path;
};
const denied=gate('coq-aern-current86-before-v87',qc+'/v86','node',['evidence-v86.mjs'],'environment-subprocess-denied',1);
assert(read(denied+'.stderr').includes('EPERM'));assert.equal(read(denied+'.stdout'),'');
const approved=gate('coq-aern-current86-before-approved-v87',qc+'/v86','node',['evidence-v86.mjs']);
assert.deepEqual(json(approved+'.stdout'),previous);assert.equal(read(approved+'.stderr'),'');
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const tests={},executables=[];
for(const profile of ['debug','release']){
 const p=gate('coq-aern-vector-'+profile+'-v87',ws+'/hyperlattice','env',[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--test','vector','--test','api_surface_coverage']);
 const out=read(p+'.stdout'),err=read(p+'.stderr');
 for(const n of [6,21]){assert(out.includes('running '+n+' tests'));assert(out.includes(n+' passed; 0 failed; 0 ignored; 0 measured; 0 filtered out'));}
 const names=[...out.matchAll(/^test (\S+) \.\.\. ok$/gm)].map(m=>m[1]).sort();assert.equal(names.length,27);assert.equal(new Set(names).size,27);
 assert(names.includes('vector_facts_classify_squared_norm_zero_status_without_self_dot'));
 assert(names.includes('vector_public_surface_and_all_abort_sparse_masks_are_covered'));
 assert(err.includes('Finished'));assert(!/warning:|error:/.test(err));
 const matches=[...err.matchAll(/Running tests\/(\w+)\.rs \(([^)]+)\)/g)];assert.equal(matches.length,2);
 for(const [,target,path]of matches){assert(path.startsWith('/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/'+profile+'/deps/'));bind(path);executables.push({profile,target,path,bytes:statSync(path).size,sha256:sha(path)});}
 tests[profile]=names;
}
assert.deepEqual(tests.debug,tests.release);
for(const [name,value]of [['model',model],['bench-contract',bench]]){
 const bad=gate('coq-aern-'+name+'-v87',here,'node',[name+'-v87.mjs'],name==='model'?'unusable-empty-sandbox-capture':'environment-subprocess-denied',name==='model'?0:1);
 assert.equal(read(bad+'.stdout'),'');if(name==='model')assert.equal(read(bad+'.stderr'),'');else assert(read(bad+'.stderr').includes('EPERM'));
 const good=gate('coq-aern-'+name+'-approved-v87',here,'node',[name+'-v87.mjs']);
 assert.deepEqual(json(good+'.stdout'),value);assert.equal(read(good+'.stderr'),'');
}
const enBad=gate('coq-aern-environment-v87',here,'node',['environment-v87.mjs'],'environment-subprocess-denied',1);
assert(read(enBad+'.stderr').includes('EPERM'));assert.equal(read(enBad+'.stdout'),'');
const enGood=gate('coq-aern-environment-approved-v87',here,'node',['environment-v87.mjs']);
const environment=json(enGood+'.stdout');assert.equal(read(enGood+'.stderr'),'');
assert.equal(environment.status,'checked-known-native-tool-locations');assert.equal(environment.installedOrDownloaded,false);
for(const cmd of ['ghc','runghc','coqc','rocq','opam','cabal'])assert.equal(environment.commands[cmd],null);
assert.equal(environment.commands.stack,'/usr/bin/stack');assert.equal(environment.paths[0].exists,false);
assert.equal(environment.tmpAvailableBytes,13861122048);bind(environment.historicalBuildNote.path);assert.equal(sha(environment.historicalBuildNote.path),environment.historicalBuildNote.sha256);
assert.equal(gates.length,10);assert.equal(new Set(gates.map(g=>g.tag)).size,10);
const rereads=[
 ['hyperreal/AGENTS.md',[[1,14]]],
 ['hyperlattice/Cargo.toml',[[1,49]]],
 ['hyperlattice/src/vector.rs',[[1090,1245]]],
 ['hyperlattice/tests/vector.rs',[[110,175]]],
 ['hyperlattice/tests/api_surface_coverage.rs',[[270,345]]],
 ['exact-real-references/aern2/aern2-real/src/AERN2/Real/Limit.hs',[[1,62]]],
 ['exact-real-references/aern2/aern2-real/src/AERN2/Real/CKleenean.hs',[[25,199]]],
 ['exact-real-references/aern2/aern2-real/src/AERN2/Continuity/Principles.hs',[[1,73]]],
 ['exact-real-references/coq-aern/formalization/Analysis/Sqrt.v',[[330,378]]],
 ['exact-real-references/coq-aern/extracted-examples/src/Sqrt.hs',[[628,661]]],
].map(([path,ranges])=>{const t=read(ws+'/'+path),lines=t.split('\n').length-(t.endsWith('\n')?1:0);
 assert(ranges.every(([a,b])=>a>=1&&a<=b&&b<=lines),'read range '+path);bind(ws+'/'+path);return{path,ranges,sha256:sha(ws+'/'+path),newDonorReadCredit:false};});
for(const r of reads.records)bind(inv.root+'/'+r.path);
for(const p of readdirSync('.').filter(p=>/\.(md|mjs|json)$/.test(p)&&p!=='manifest-v87.json').sort()){
 bind(p);assert(!/[ \t]+$/m.test(read(p)),'authored trailing whitespace '+p);
}
for(const p of [qc+'/v86/manifest-v86.json',qc+'/v86/read-records-v86.json',qc+'/inventory-v85.json',c+'/capture.mjs'])bind(p);
const live=json(qc+'/manifest-v85.json').liveSources;assert.equal(Object.keys(live).length,957);
for(const [p,h]of Object.entries(live))assert.equal(sha(ws+'/'+p),h,'live after checks '+p);
const result={checkpoint:87,status:'qualified-metric-source-and-benchmark-contracts',previousManifestSha256:sha(qc+'/v86/manifest-v86.json'),
 previousDirectArtifacts:57,olderDirectArtifacts:[62,4498],coverage:reads.totals,delta:reads.delta,remainingTextFiles:52,
 model,bench,environment,tests,executables,executableBytes:executables.reduce((s,e)=>s+e.bytes,0),rereads,gates,files,
 liveFiles:957,isolatedCandidateFiles:184,productionChanges:0,newRetainedTransfers:0,retainedContinuationTransfers:7,
 limits:'Source counterexample and exact mathematical models; no native Haskell/Coq build or wrong-output reproduction. Selected default Hyperlattice tests only; no matched benchmark, memory/size improvement, full-stack/all-feature/WASM or whole-inventory completion.'};
if(process.argv.includes('--record'))writeFileSync('manifest-v87.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,json('manifest-v87.json'));
console.log(JSON.stringify({checkpoint:87,status:process.argv.includes('--record')?'sealed':'verified',directArtifacts:Object.keys(files).length,
 previousDirectArtifacts:57,olderDirectArtifacts:[62,4498],liveFiles:957,isolatedCandidateFiles:184,coverage:result.coverage,delta:result.delta,
 captures:10,classifications:{success:6,environmentFailure:3,unusableEmptySandboxCapture:1},hyperTests:{debug:27,release:27},
 modelTrace:model.sha256,executableBytes:result.executableBytes,productionChanges:0,fullAuditComplete:false}));
