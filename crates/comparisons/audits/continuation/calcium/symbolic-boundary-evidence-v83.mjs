import {readFileSync,statSync}from 'node:fs';
import {createHash}from 'node:crypto';
import {execFileSync}from 'node:child_process';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary}from './effective-coverage.mjs';
import {cargoEnv,target}from './point-qualified-capture.mjs';
import {symbolicEvidence}from './check-symbolic-boundary-v83.mjs';
import {complexEvidence}from './check-complex-roundtrip-v83.mjs';
import {cleanupEvidence}from './check-symbolic-cleanup-control-v83.mjs';
import {hyperEvidence}from './check-symbolic-boundary-hyper-v83.mjs';
import {checkValue}from './symbolic-boundary-oracle-v83.mjs';
import {q,scalar,fzero}from './point-extended-field.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='symbolic-boundary-hyper-v83';
export const evidenceFiles=[
 'symbolic-boundary-protocol-v83.md','symbolic-boundary-hyper-protocol-v83.md','symbolic-boundary-v83-findings.md',
 'prepare-symbolic-boundary-v83.mjs','symbolic-boundary-origin-v83.json','symbolic-boundary-read-records-v83.json',
 'flint-symbolic-boundary-v83.c','flint-symbolic-boundary-fixed-v83.c','run-symbolic-boundary-v83.mjs','run-symbolic-boundary-fixed-v83.mjs',
 'symbolic-boundary-run-origin-v83.json','symbolic-boundary-run-origin-fixed-v83.json',
 'symbolic-boundary-oracle-v83.mjs','check-symbolic-boundary-v83.mjs',
 'flint-complex-roundtrip-v83.c','run-complex-roundtrip-v83.mjs','complex-roundtrip-origin-v83.json','check-complex-roundtrip-v83.mjs',
 'flint-set-fexpr-cleanup-control-v83.c','run-symbolic-cleanup-control-v83.mjs','symbolic-cleanup-control-origin-v83.json','check-symbolic-cleanup-control-v83.mjs',
 'flint-complex-abs-isolation-v83.c','run-complex-abs-isolation-v83.mjs','complex-abs-isolation-origin-v83.json',
 cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs','run-symbolic-boundary-hyper-v83.mjs','symbolic-boundary-hyper-origin-v83.json',
 'check-symbolic-boundary-hyper-v83.mjs','symbolic-boundary-evidence-v83.mjs','verify-symbolic-boundary-v83.mjs'];
export function boundaryEvidence(){
 const o=json('symbolic-boundary-origin-v83.json'),h=json('symbolic-boundary-hyper-origin-v83.json'),prior=json('quadratic-extraction-v82-manifest.json'),
  candidate=json('twelfth-revision-v79-manifest.json'),current=retainedSources(),lib=workspace+'/exact-real-references/flint';
 assert.deepEqual(current,o.current);assert.deepEqual(current,h.current);assert.equal(sha('quadratic-extraction-v82-manifest.json'),o.previousSha256);
 for(const[p,v]of Object.entries({...prior.files,...o.files,...h.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),v,p);
 for(const name of ['symbolic-boundary-run-origin-v83.json','symbolic-boundary-run-origin-fixed-v83.json','complex-roundtrip-origin-v83.json',
  'symbolic-cleanup-control-origin-v83.json','complex-abs-isolation-origin-v83.json']){
  const v=json(name);for(const[p,h]of Object.entries(v.files))assert.equal(sha(p),h,p);
  if(v.current)assert.deepEqual(v.current,current);
 }
 assert.equal(read('flint-symbolic-boundary-v83.c').replace(/\s/g,''),read('flint-symbolic-boundary-fixed-v83.c').replace(/\s/g,''));
 assert.equal(Object.keys(prior.liveSources).length,957);for(const[p,v]of Object.entries(prior.liveSources))assert.equal(sha(workspace+'/'+p),v,p);
 assert.equal(Object.keys(candidate.candidateSources).length,184);for(const[p,v]of Object.entries(candidate.candidateSources))assert.equal(sha(candidate.candidateRoot+'/'+p),v,p);
 const records=json('symbolic-boundary-read-records-v83.json'),ext=json('coverage-extensions.json'),inventory=json('inventory.json'),coverage=effectiveCoverage();
 assert.deepEqual(records,o.records);assert.deepEqual(ext.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(ext.slice(o.extensionsBefore.length,o.extensionsBefore.length+records.length),records);
 let lines=0;for(const r of records){
  const f=inventory.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);assert.deepEqual(r.ranges,[[1,f.lines]]);
  assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);assert.equal(sha(workspace+'/exact-real-references/'+r.repo+'/'+r.path),f.sha256);
  assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,r.ranges);lines+=f.lines;
 }
 assert.equal(records.length,26);assert.equal(lines,5728);
 for(const s of inventory.sources){const path=workspace+'/exact-real-references/'+s.repo;
  assert.equal(execFileSync('git',['rev-parse','HEAD'],{cwd:path,encoding:'utf8'}).trim(),s.commit);
  assert.equal(execFileSync('git',['status','--short','--untracked-files=no'],{cwd:path,encoding:'utf8'}),'');
 }
 for(const[k,v]of Object.entries(o.rereadHashes)){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),v);}
 for(const[p,v]of Object.entries({...o.hyperReadHashes,...h.extraHyperHashes}))assert.equal(sha(workspace+'/'+p),v);
 const coverageAtBinding=[{repo:'calcium',reviewed:613,complete:613,partial:0,readLines:66370},{repo:'flint',reviewed:945,complete:925,partial:20,readLines:129259}];
 for(const s of effectiveSummary()){const b=coverageAtBinding.find(t=>t.repo===s.repo);assert(s.complete>=b.complete&&s.readLines>=b.readLines);}
 const gates=[],gate=(tag,directory,command,args,code=0,signal=null,classification='success')=>{
  const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);assert.deepEqual(g.args,args);
  assert.equal(g.code,code);assert.equal(g.signal,signal);assert(!g.error);assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);
  assert(Number.isFinite(Date.parse(g.started))&&Date.parse(g.finished)>=Date.parse(g.started));
  gates.push({tag,classification});return g;
 };
 const node=(tag,script,code=0,classification='success')=>gate(tag,'.','node',[script],code,null,classification);
 const before=node('complex-symbolic-current82-before-v83','verify-quadratic-extraction-v82.mjs');
 assert.equal(json('results/'+before.tag+'.stdout').status,'verified-quadratic-extraction-source-and-capability');
 const prepare=node('symbolic-boundary-prepare-v83','prepare-symbolic-boundary-v83.mjs');
 assert(Date.parse(prepare.started)>=Date.parse(before.finished));assert(Date.parse(o.recorded)>=Date.parse(prepare.started)&&Date.parse(o.recorded)<=Date.parse(prepare.finished));
 assert.deepEqual(json('results/'+prepare.tag+'.stdout'),{checkpoint:83,status:'prepared',readFiles:26,newLines:5728,dir:o.dir});
 const cc=(source,binary,section=false)=>['-O2','-g0',...(section?['-ffunction-sections','-fdata-sections']:[]),'-Wall','-Wextra','-Werror','-isystem',lib+'/src',source,
  '-L',lib,'-Wl,-rpath,'+lib,...(section?['-Wl,--gc-sections']:[]),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary];
 gate('symbolic-boundary-compile-v83','.','gcc',cc('flint-symbolic-boundary-v83.c',o.binary),1,null,'harness-failure');
 assert(read('results/symbolic-boundary-compile-v83.stderr').includes('misleading-indentation'));
 node('symbolic-boundary-run-v83','run-symbolic-boundary-v83.mjs',1,'harness-failure');
 gate('symbolic-boundary-compile-fixed-v83','.','gcc',cc('flint-symbolic-boundary-fixed-v83.c',o.binary));
 gate('symbolic-boundary-linkage-v83','.','ldd',[o.binary]);
 const linked=read('results/symbolic-boundary-linkage-v83.stdout');for(const p of Object.keys(o.libraries))assert(linked.includes(p));
 const native=(binary,rest=[])=>['60s','prlimit','--as=1073741824','--cpu=30','--',binary,...rest];
 const mem=(binary,rest=[],seconds='120s')=>[seconds,'valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary,...rest];
 gate('symbolic-boundary-native-v83','.','timeout',native(o.binary,['angles']));
 gate('symbolic-boundary-memcheck-v83','.','timeout',mem(o.binary,['angles']));
 for(const which of [0,1,2])for(const count of [1,32,256]){
  const rest=['powers',String(which),String(count)];gate('symbolic-power-'+which+'-'+count+'-v83','.','timeout',native(o.binary,rest));
  gate('symbolic-power-mem-'+which+'-'+count+'-v83','.','timeout',mem(o.binary,rest),which?97:0,null,which?'diagnostic-leak':'success');
 }
 node('symbolic-boundary-run-fixed-v83','run-symbolic-boundary-fixed-v83.mjs');
 node('symbolic-boundary-check-v83','check-symbolic-boundary-v83.mjs');
 const components=o.dir+'/components';gate('complex-roundtrip-compile-v83','.','gcc',cc('flint-complex-roundtrip-v83.c',components,true));
 gate('complex-roundtrip-native-v83','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',components]);
 gate('complex-roundtrip-memcheck-v83','.','timeout',mem(components,[],'180s'),null,'SIGABRT','diagnostic-abort');
 node('complex-roundtrip-run-v83','run-complex-roundtrip-v83.mjs',1,'diagnostic-driver-failure');
 node('complex-roundtrip-check-v83','check-complex-roundtrip-v83.mjs');
 const control=o.dir+'/cleanup-control';
 gate('symbolic-cleanup-control-compile-v83','.','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-Wno-unused-parameter',
  '-Dqqbar_set_fexpr=qqbar_set_fexpr_cleanup_control','-isystem',lib+'/src','flint-symbolic-boundary-fixed-v83.c','flint-set-fexpr-cleanup-control-v83.c',
  '-L',lib,'-Wl,-rpath,'+lib,'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',control]);
 for(const which of [0,1,2]){
  gate('symbolic-cleanup-control-'+which+'-v83','.','timeout',native(control,['powers',String(which),'256']));
  gate('symbolic-cleanup-control-mem-'+which+'-v83','.','timeout',mem(control,['powers',String(which),'256']));
 }
 node('symbolic-cleanup-control-run-v83','run-symbolic-cleanup-control-v83.mjs');
 node('symbolic-cleanup-control-check-v83','check-symbolic-cleanup-control-v83.mjs');
 const isolation=o.dir+'/isolated-abs';gate('complex-abs-isolation-compile-v83','.','gcc',cc('flint-complex-abs-isolation-v83.c',isolation,true));
 for(const mode of ['native','none','memcheck']){
  const args=['60s','prlimit','--core=0','--cpu=30','--',...(mode==='native'?[]:['valgrind','--tool='+mode,...(mode==='memcheck'?['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97']:[])]),isolation];
  gate('complex-abs-isolation-'+mode+'-v83','.','timeout',args,mode==='native'?0:null,mode==='native'?null:'SIGABRT',mode==='native'?'success':'diagnostic-abort');
  if(mode!=='native'){assert.equal(read('results/complex-abs-isolation-'+mode+'-v83.stdout'),'');assert(read('results/complex-abs-isolation-'+mode+'-v83.stderr').includes('fmpz_lll_is_reduced_d'));}
 }
 node('complex-abs-isolation-run-v83','run-complex-abs-isolation-v83.mjs');
 const isolatedRows=read('results/complex-abs-isolation-native-v83.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(isolatedRows.length,2);assert.deepEqual(isolatedRows[1],{terminal:true,rows:1});assert.equal(isolatedRows[0].family,'isolated-abs');
 const isolatedChecks=checkValue(isolatedRows[0],{re:scalar(q(7,3)),im:fzero()});
 assert.deepEqual(h.cargoEnv,cargoEnv);
 gate('symbolic-boundary-hyper-fmt-v83',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);
 gate('symbolic-boundary-hyper-metadata-v83',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
 for(const profile of ['debug','release'])gate('symbolic-boundary-hyper-'+profile+'-v83',cwd,'timeout',['180s','env',...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);
 gate('symbolic-boundary-hyper-clippy-v83',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']);
 node('symbolic-boundary-hyper-run-v83','run-symbolic-boundary-hyper-v83.mjs');node('symbolic-boundary-hyper-check-v83','check-symbolic-boundary-hyper-v83.mjs');
 const graph=json('results/symbolic-boundary-hyper-metadata-v83.stdout'),ids=new Set(graph.packages.map(p=>p.id));assert.equal(ids.size,22);
 assert.equal(graph.resolve.nodes.length,22);assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());
 for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
 const hp=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.deepEqual(hp.map(p=>p.name).sort(),['hyperlattice','hyperreal']);
 for(const p of hp)assert.equal(p.manifest_path,workspace+'/'+p.name+'/Cargo.toml');
 const symbolic=symbolicEvidence(),complex=complexEvidence(),cleanup=cleanupEvidence(),hyper=hyperEvidence();
 for(const[tag,value]of [['symbolic-boundary-check-v83',symbolic],['complex-roundtrip-check-v83',complex],['symbolic-cleanup-control-check-v83',cleanup],['symbolic-boundary-hyper-check-v83',hyper]]){
  assert.deepEqual(value,json('results/'+tag+'.stdout'));assert.equal(read('results/'+tag+'.stderr'),'');
 }
 assert.equal(symbolic.checks,2124);assert.equal(complex.checks,15456);assert.equal(hyper.checks,3536);
 assert.deepEqual(hyper.norms,{Equal:144,Unknown:48});
 const angleMem=read('results/symbolic-boundary-memcheck-v83.stderr');
 assert(angleMem.includes('in use at exit: 0 bytes in 0 blocks'));assert(angleMem.includes('13,768 allocs, 13,768 frees, 438,556 bytes allocated'));
 assert(angleMem.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const classifiers={};for(const g of gates)classifiers[g.classification]=(classifiers[g.classification]??0)+1;
 assert.equal(gates.length,54);assert.equal(new Set(gates.map(g=>g.tag)).size,54);
 assert.deepEqual(classifiers,{success:42,'harness-failure':2,'diagnostic-leak':6,'diagnostic-abort':3,'diagnostic-driver-failure':1});
 for(const g of gates.filter(g=>g.classification==='success')){
  const err=read('results/'+g.tag+'.stderr');assert(!err.includes('warning:'),g.tag);
  if(!g.tag.includes('mem')&&!g.tag.includes('clippy')&&g.tag!=='complex-abs-isolation-native-v83')assert.equal(err,'',g.tag);
 }
 const executables=[o.binary,components,control,isolation,...['debug','release'].map(p=>target+'/'+p+'/symbolic-boundary-hyper-v83')].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}));
 assert.equal(executables.reduce((n,b)=>n+b.bytes,0),5971960);assert.deepEqual(retainedSources(),current);
 return{checkpoint:83,status:'qualified-source-and-classified-symbolic-defects',readFiles:26,newDonorLines:5728,coverageBefore:o.coverageBefore,coverageAtBinding,
  liveFiles:957,isolatedCandidateFiles:184,records,gates,classifiers,symbolic,complex,cleanup,hyper,isolatedChecks,
  angleMemory:{allocations:13768,frees:13768,requestedBytes:438556,liveBytes:0,errors:0,scope:'Whole angle collector only; not the aborted complex collector or per-call/peak cost.'},
  dependencyGraph:{packages:22,nodes:22,hyperPackages:['hyperlattice','hyperreal']},executables,
  libraries:{...o.libraries,'/lib64/ld-linux-x86-64.so.2':sha('/lib64/ld-linux-x86-64.so.2')},retainedContinuationTransfers:7,productionChanges:0,
  limits:'Pinned current FLINT executed; archive/upstream tests read only. Confirmed wrong successes and leaked Pow temporaries; complex Memcheck abort remains unresolved and is not a clean gate. Bounded default-feature Hyper comparison, no serde-ingestion test, new production transfer, benchmark, full-stack regression, WASM or product-size claim.',
  next:'Finish remaining qqbar/support files and other requested references; qualify scoped complementary-angle norm and rounding/proof-scheduling ideas before retention; reconcile complete inventory.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(boundaryEvidence()));
