import {readFileSync,readdirSync,writeFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {checkHyper} from './check-hyper-v85.mjs';

const ws='/home/tim/Documents/GitHub/workspace',qc=ws+'/exactcore-hyper-comparison/audits/continuation/coq-aern',c=resolve(qc,'../calcium');
assert.equal(process.cwd(),qc);
const read=p=>readFileSync(p,'utf8'),json=p=>JSON.parse(read(p)),sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const manifest='manifest-v85.json',record=process.argv.includes('--record'),previous=json(c+'/qqbar-remainder-v84-manifest.json');
const previousSha256=sha(c+'/qqbar-remainder-v84-manifest.json');
for(const [p,h]of Object.entries(previous.files))assert.equal(sha(resolve(c,p)),h,'previous '+p);
const live=json(c+'/zero-factor-retained-origin-v75.json').after;
assert.equal(Object.keys(live).length,957);for(const [p,h]of Object.entries(live))assert.equal(sha(ws+'/'+p),h,'live '+p);
const isolated=json(c+'/twelfth-revision-v79-manifest.json');
assert.equal(Object.keys(isolated.candidateSources).length,184);
for(const [p,h]of Object.entries(isolated.candidateSources))assert.equal(sha(resolve(c,isolated.candidateRoot,p)),h,'unretained candidate '+p);
const inventoryVerification=JSON.parse(execFileSync(process.execPath,['inventory.mjs'],{encoding:'utf8'}));
const readVerification=JSON.parse(execFileSync(process.execPath,['read-records-v85.mjs'],{encoding:'utf8'}));
assert.equal(inventoryVerification.status,'inventory-verified');assert.deepEqual(readVerification,{status:'read-records-verified',complete:21,partial:1,lines:4666});
const inventory=json('inventory-v85.json'),reads=json('read-records-v85.json');
const other=ws+'/exact-real-references/aern2',git=(root,args)=>execFileSync('git',args,{cwd:root,encoding:'utf8'});
assert.equal(git(other,['rev-parse','HEAD']).trim(),'d1ac3664bfb5c7f70fcf68f7fb412d288def65cb');assert.equal(git(other,['status','--short','--untracked-files=no']),'');
const crossReferenceRereads=[
 ['exact-real-references/aern2/aern2-real/src/AERN2/Real/Limit.hs',[[1,62]]],
 ['exact-real-references/aern2/aern2-real/src/AERN2/Real/CKleenean.hs',[[1,199]]],
 ['exact-real-references/aern2/aern2-real/src/AERN2/Continuity/Principles.hs',[[1,73]]],
 ['exact-real-references/aern2/aern2-mp/src/AERN2/Select.hs',[[1,79]]],
 ['hyperreal/src/real/arithmetic/representation.rs',[[95,135],[235,290],[470,560]]],
 ['hyperreal/src/real/arithmetic/comparison.rs',[[1,81]]],
 ['hyperreal/src/real/arithmetic/facts.rs',[[1310,1440]]],
].map(([path,ranges])=>({path,ranges,sha256:sha(ws+'/'+path),newCoqAernReadCredit:false}));

const cargo='exactcore-hyper-comparison/audits/continuation/coq-aern/hyper-v85/Cargo.toml';
const target='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
const env=['CARGO_TARGET_DIR='+target,'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const gates=[],files={};
const bind=p=>{const absolute=resolve(qc,p);files[absolute]=sha(absolute);};
const gate=(tag,cwd,command,args,classification='success')=>{
 const path=c+'/results/'+tag,g=json(path+'.json');
 assert.equal(g.tag,tag);assert.equal(g.cwd,cwd);assert.equal(g.command,command);assert.deepEqual(g.args,args);
 assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);assert(g.elapsedSeconds>=0);
 assert(Number.isFinite(Date.parse(g.started)));assert(Date.parse(g.finished)>=Date.parse(g.started));
 for(const ext of ['json','stdout','stderr'])bind(path+'.'+ext);
 gates.push({tag,classification,finished:g.finished});return path;
};
for(const profile of ['debug','release']){
 const tag='coq-aern-build-'+profile+'-v85';
 const path=gate(tag,ws,'env',[...env,'cargo','build',...(profile==='release'?['--release']:[]),'--offline','--manifest-path',cargo]);
 assert(read(path+'.stderr').includes('Compiling coq-aern-hyper-v85'));assert(read(path+'.stderr').includes('Finished'));
 const run=gate('coq-aern-run-'+profile+'-v85',ws,'timeout',['300s',target+'/'+profile+'/coq-aern-hyper-v85']);assert.equal(read(run+'.stderr'),'');
 bind(target+'/'+profile+'/coq-aern-hyper-v85');
}
const fmt=gate('coq-aern-fmt-v85',ws,'cargo',['fmt','--manifest-path',cargo,'--','--check']);
assert.equal(read(fmt+'.stdout')+read(fmt+'.stderr'),'');
const lint=gate('coq-aern-clippy-v85',ws,'env',[...env,'cargo','clippy','--offline','--manifest-path',cargo,'--all-targets','--','-D','warnings']);
assert(read(lint+'.stderr').includes('Checking coq-aern-hyper-v85'));assert(read(lint+'.stderr').includes('Finished'));assert(!/warning:|error:/.test(read(lint+'.stderr')));
const mem=gate('coq-aern-memcheck-v85',ws,'timeout',['300s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',target+'/release/coq-aern-hyper-v85']);
assert(readFileSync(mem+'.stdout').equals(readFileSync(c+'/results/coq-aern-run-release-v85.stdout')));
const memory=read(mem+'.stderr');assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts'));
for(const type of ['definitely','indirectly','possibly'])assert(new RegExp(type+' lost: 0 bytes in 0 blocks').test(memory));
assert(memory.includes('still reachable: 3,008 bytes in 25 blocks'));
assert(memory.includes('39,929 allocs, 39,904 frees, 5,175,511 bytes allocated'));
const numerical=checkHyper();
for(const approved of [false,true]){
 const tag='coq-aern-check-hyper'+(approved?'-approved':'')+'-v85';
 const path=gate(tag,ws,'node',['exactcore-hyper-comparison/audits/continuation/coq-aern/check-hyper-v85.mjs'],approved?'success':'unusable-empty-sandbox-capture');
 assert.equal(read(path+'.stderr'),'');if(approved)assert.deepEqual(json(path+'.stdout'),numerical);else assert.equal(read(path+'.stdout'),'');
}
let environment;
for(const approved of [false,true]){
 const tag='coq-aern-environment'+(approved?'-approved':'')+'-v85';
 const path=gate(tag,qc,'node',['environment-v85.mjs'],approved?'success':'unusable-empty-sandbox-capture');assert.equal(read(path+'.stderr'),'');
 if(approved)environment=json(path+'.stdout');else assert.equal(read(path+'.stdout'),'');
}
for(const key of ['coqc','rocq','opam','ghc','runghc','cabal'])assert.equal(environment.tools[key],null);
assert.deepEqual(environment.stackCompilerCache.entries,[]);assert(environment.symlinks.every(s=>!s.exists));
assert.equal(environment.symlinks.length,3);assert.equal(environment.auditBinaryBytes,5150272);
for(const b of environment.binaries)assert.equal(statSync(b.path).size,b.bytes);
const meta=gate('coq-aern-metadata-v85',ws,'env',[...env,'cargo','metadata','--offline','--format-version','1','--manifest-path',cargo]);
assert.equal(read(meta+'.stderr'),'');const graph=json(meta+'.stdout'),ids=new Set(graph.packages.map(p=>p.id));
assert.equal(ids.size,21);assert.equal(graph.resolve.nodes.length,21);assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());
for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
const hp=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.equal(hp.length,1);assert.equal(hp[0].name,'hyperreal');assert.equal(hp[0].manifest_path,ws+'/hyperreal/Cargo.toml');
assert.deepEqual(graph.resolve.nodes.find(n=>n.id===hp[0].id).features,['default']);
assert.equal(gates.length,12);assert.equal(new Set(gates.map(g=>g.tag)).size,12);
for(const p of readdirSync('.').filter(p=>/\.(mjs|md|json)$/.test(p)&&p!==manifest))bind(p);
for(const p of ['Cargo.toml','Cargo.lock','src/main.rs'])bind('hyper-v85/'+p);
for(const r of crossReferenceRereads)bind(ws+'/'+r.path);
bind(c+'/capture.mjs');bind(c+'/qqbar-remainder-v84-manifest.json');bind(c+'/zero-factor-retained-origin-v75.json');
const result={checkpoint:85,status:'qualified-source-contract-and-existing-Hyper-extrema-capability',previousSha256,
 previousDirectArtifacts:Object.keys(previous.files).length,inventory:inventoryVerification,readCoverage:reads.totals,
 crossReferenceRereads,liveSources:live,isolatedCandidateFiles:184,gates,numerical,
 memory:{errors:0,definitelyLost:0,indirectlyLost:0,possiblyLost:0,reachableBytes:3008,reachableBlocks:25,allocations:39929,frees:39904,requestedBytes:5175511},
 dependencyGraph:{packages:21,hyperPackages:['hyperreal'],features:['default']},environment,
 productionChanges:0,newRetainedTransfers:0,retainedContinuationTransfers:7,
 limits:'No native Coq/GHC execution, matched benchmark, full-stack/all-feature/WASM or product-size improvement claim. Full coq-aern and ecosystem source/transfer audit remains incomplete.',files};
if(record)writeFileSync(manifest,JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(result,json(manifest));
console.log(JSON.stringify({checkpoint:85,directArtifacts:Object.keys(files).length,previousDirectArtifacts:result.previousDirectArtifacts,
 captures:gates.length,classifications:{success:10,unusableEmptySandboxCaptures:2},sourceArtifacts:inventory.totals.files,
 coqAernReadCoverage:reads.totals,liveFiles:957,isolatedCandidateFiles:184,...numerical,status:record?'sealed':'verified',productionChanges:0,fullAuditComplete:false}));
