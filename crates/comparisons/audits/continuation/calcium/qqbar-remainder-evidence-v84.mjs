import {readFileSync,readdirSync,statSync}from 'node:fs';
import {createHash}from 'node:crypto';
import {execFileSync}from 'node:child_process';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary}from './effective-coverage.mjs';
import {captured,cargoEnv,target}from './point-qualified-capture.mjs';
import {remainderEvidence}from './check-qqbar-remainder-fixed-v84.mjs';
import {hyperEvidence}from './check-qqbar-remainder-hyper-v84.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='qqbar-remainder-hyper-v84';
export const evidenceFiles=[...readdirSync('.').filter(p=>/-v84\.(mjs|json|md|c|rs)$/.test(p)&&p!=='qqbar-remainder-v84-manifest.json'),
 'qqbar-remainder-v84-findings.md',cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs'].sort();
export function evidence84(){
 const o=json('qqbar-remainder-origin-v84.json'),prior=json('symbolic-boundary-v83-manifest.json'),current=retainedSources(),lib=workspace+'/exact-real-references/flint';
 assert.deepEqual(current,o.current);assert.equal(sha('symbolic-boundary-v83-manifest.json'),o.previousSha256);
 for(const[p,h]of Object.entries({...prior.files,...o.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);
 for(const name of ['qqbar-remainder-run-origin-v84.json','qqbar-remainder-run-origin-fixed-v84.json','qqbar-remainder-run-origin-qualified-v84.json',
  'qqbar-remainder-run-origin-trap-v84.json','qqbar-cleanup-cubic-origin-v84.json','qqbar-normal-origin-v84.json','qqbar-normal-origin-fixed-v84.json',
  'qqbar-remainder-hyper-origin-v84.json','qqbar-remainder-hyper-origin-fixed-v84.json']){
  const origin=json(name);assert.deepEqual(origin.current,current);
  for(const[p,h]of Object.entries(origin.files))assert.equal(sha(name==='qqbar-remainder-hyper-origin-v84.json'&&p===cwd+'/src/main.rs'?'qqbar-remainder-hyper-before-lint-v84.rs':p),h,p);
  for(const[p,h]of Object.entries({...origin.hyperRereadHashes,...origin.extraHyperHashes}))assert.equal(sha(workspace+'/'+p),h,p);
 }
 assert.equal(read('qqbar-remainder-hyper-before-lint-v84.rs').replace('((5 * i + length) % 7) as i32','(5 * i + length) % 7'),read(cwd+'/src/main.rs'));
 assert.equal(read('flint-qqbar-remainder-v84.c').replace('(qqbar_srcptr) roots->entries','(qqbar_ptr) roots->entries'),read('flint-qqbar-remainder-fixed-v84.c'));
 assert.equal(read('flint-qqbar-remainder-fixed-v84.c').replace('gr_poly_set_coeff_si(p, 2, 1, ctx)','gr_poly_set_coeff_si(p, 3, 1, ctx)'),read('flint-qqbar-cleanup-cubic-v84.c'));
 assert.equal(read('flint-qqbar-normal-v84.c').replace('#include "fmpq_mat.h"','#include "fmpq_mat.h"\n#include "fmpq_poly.h"'),read('flint-qqbar-normal-fixed-v84.c'));
 assert.equal(read('check-qqbar-remainder-v84.mjs').replace('r.packedBits>64,exponent>=64','r.packedBits>64,exponent>=63'),read('check-qqbar-remainder-fixed-v84.mjs'));
 assert.equal(Object.keys(prior.liveSources).length,957);for(const[p,h]of Object.entries(prior.liveSources))assert.equal(sha(workspace+'/'+p),h,p);
 const candidate=json('twelfth-revision-v79-manifest.json');assert.equal(Object.keys(candidate.candidateSources).length,184);
 for(const[p,h]of Object.entries(candidate.candidateSources))assert.equal(sha(candidate.candidateRoot+'/'+p),h,p);
 const records=json('qqbar-remainder-read-records-v84.json'),extensions=json('coverage-extensions.json'),inventory=json('inventory.json'),coverage=effectiveCoverage();
 assert.deepEqual(records,o.records);assert.equal(records.length,38);assert.deepEqual(extensions.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(extensions.slice(o.extensionsBefore.length,o.extensionsBefore.length+38),records);let newLines=0;
 for(const r of records){const f=inventory.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
  assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);assert.equal(sha(workspace+'/exact-real-references/'+r.repo+'/'+r.path),f.sha256);
  assert.deepEqual(coverage.find(v=>v.repo===r.repo&&v.path===r.path).ranges,[[1,f.lines]]);
  newLines+=r.ranges.reduce((n,[a,b])=>n+b-a+1,0);
 }assert.equal(newLines,5324);
 const qqbar=[];for(const s of inventory.sources){const path=workspace+'/exact-real-references/'+s.repo;
  assert.equal(execFileSync('git',['rev-parse','HEAD'],{cwd:path,encoding:'utf8'}).trim(),s.commit);
  assert.equal(execFileSync('git',['status','--short','--untracked-files=no'],{cwd:path,encoding:'utf8'}),'');
  const files=s.files.filter(f=>f.text&&f.path.startsWith(s.repo==='flint'?'src/qqbar/':'qqbar/'));let lines=0;
  for(const f of files){assert.equal(sha(path+'/'+f.path),f.sha256);assert.deepEqual(coverage.find(r=>r.repo===s.repo&&r.path===f.path).ranges,[[1,f.lines]]);lines+=f.lines;}
  qqbar.push({repo:s.repo,files:files.length,lines,unread:0});
 }
 assert.deepEqual(qqbar,[{repo:'calcium',files:149,lines:14405,unread:0},{repo:'flint',files:153,lines:14086,unread:0}]);
 for(const[k,h]of Object.entries(o.rereadHashes)){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),h,p);}
 const coverageAtBinding=[{repo:'calcium',reviewed:629,complete:629,partial:0,readLines:68181},{repo:'flint',reviewed:966,complete:947,partial:19,readLines:132772}];
 for(const s of effectiveSummary()){const b=coverageAtBinding.find(x=>x.repo===s.repo);assert(s.complete>=b.complete&&s.readLines>=b.readLines);}
 const gates=[],gate=(tag,directory,command,args,code=0,signal=null,classification='success')=>{
  const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);assert.deepEqual(g.args,args);
  assert.equal(g.code,code);assert.equal(g.signal,signal);assert(!g.error);assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);
  assert(Number.isFinite(Date.parse(g.started))&&Date.parse(g.finished)>=Date.parse(g.started));gates.push({tag,classification});return g;
 };
 const node=(tag,script,code=0,classification='success')=>gate(tag,'.','node',[script],code,null,classification);
 for(const which of ['before','after']){const tag='qqbar-remainder-current83-'+which+'-v84';node(tag,'verify-symbolic-boundary-v83.mjs');assert.equal(json('results/'+tag+'.stdout').status,'verified-source-and-classified-symbolic-defects');}
 node('qqbar-remainder-prepare-v84','prepare-qqbar-remainder-v84.mjs');
 const cc=['-O2','-g1','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src'],
  link=['-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread'];
 gate('qqbar-remainder-compile-v84','.','gcc',[...cc,'flint-qqbar-remainder-v84.c',...link,'-o',o.binary],1,null,'harness-failure');
 gate('qqbar-remainder-compile-fixed-v84','.','gcc',[...cc,'flint-qqbar-remainder-fixed-v84.c',...link,'-o',o.binary]);
 for(const mode of ['original','qualified','trap']){
  const tag='qqbar-remainder-sanitized-compile'+(mode==='original'?'':'-'+mode)+'-v84';
  gate(tag,'.','gcc',[...cc,...(mode==='original'?[]:['-Wno-error=sign-compare']),'-fsanitize=undefined',mode==='trap'?'-fsanitize-undefined-trap-on-error':'-fno-sanitize-recover=undefined',
   '-D_qqbar_roots_poly_squarefree=qqbar_roots_squarefree_sanitized_v84','flint-qqbar-remainder-fixed-v84.c',lib+'/src/qqbar/roots_poly_squarefree.c',...link,'-o',o.sanitized],mode==='trap'?0:1,null,mode==='trap'?'success':mode==='original'?'harness-failure':'environment-failure');
 }
 for(const mode of ['original','fixed','qualified','trap']){
  const suffix=mode==='original'?'':'-'+mode;node('qqbar-remainder-run'+suffix+'-v84','run-qqbar-remainder'+suffix+'-v84.mjs',mode==='trap'?0:1,mode==='trap'?'success':mode==='qualified'?'environment-failure':'harness-failure');
 }
 assert(read('results/qqbar-remainder-compile-v84.stderr').includes('discarded-qualifiers'));
 assert(read('results/qqbar-remainder-sanitized-compile-v84.stderr').includes('sign-compare'));
 assert(read('results/qqbar-remainder-sanitized-compile-qualified-v84.stderr').includes('cannot find /usr/lib64/libubsan.so.1.0.0'));
 gate('qqbar-remainder-linkage-v84','.','ldd',[o.binary]);for(const p of Object.keys(o.libraries))assert(read('results/qqbar-remainder-linkage-v84.stdout').includes(p));
 const native=(binary,rest)=>['20s','prlimit','--core=0','--as=1073741824','--cpu=10','--',binary,...rest],
  mem=(binary,rest=[],seconds='120s')=>[seconds,'prlimit','--core=0','--','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary,...rest];
 for(const generic of [0,1])for(const exponent of [0,30,63,64,80,256])for(const terms of [1,2]){
  const fail=generic===0&&exponent>=64&&terms===1;
  gate('qqbar-monomial-'+generic+'-'+exponent+'-'+terms+'-v84','.','timeout',native(o.binary,['monomial',String(generic),String(exponent),String(terms)]),fail?null:0,fail?'SIGABRT':null,fail?'diagnostic-abort':'success');
 }
 for(const degree of [57,58,59,60,61,62,63])gate('qqbar-root-limit-sanitized-'+degree+'-v84','.','timeout',native(o.sanitized,['limit',String(degree),'0']),degree<=61?null:0,degree<=61?'SIGILL':null,degree<=61?'diagnostic-overflow-trap':'success');
 for(const mode of ['native','sanitized'])gate('qqbar-root-limit-bounded-'+mode+'-v84','.','timeout',native(mode==='native'?o.binary:o.sanitized,['limit','61','1']));
 gate('qqbar-root-overflow-gdb-v84','.','timeout',['20s','prlimit','--core=0','--as=1073741824','--cpu=10','--','gdb','-batch','-nx','-ex','set debuginfod enabled off','-ex','run','-ex','bt','-ex','x/4i $pc','--args',o.sanitized,'limit','57','0']);
 for(const limit of [1,2,4])for(const count of [1,32,128]){
  const rest=['cleanup',String(limit),String(count)];gate('qqbar-root-cleanup-'+limit+'-'+count+'-v84','.','timeout',native(o.binary,rest));gate('qqbar-root-cleanup-mem-'+limit+'-'+count+'-v84','.','timeout',mem(o.binary,rest));
 }
 const cubic=o.dir+'/cleanup-cubic';gate('qqbar-cleanup-cubic-compile-v84','.','gcc',[...cc,'flint-qqbar-cleanup-cubic-v84.c',...link,'-o',cubic]);node('qqbar-cleanup-cubic-run-v84','run-qqbar-cleanup-cubic-v84.mjs');
 for(const limit of [4,8])for(const count of [1,32,128]){
  const rest=['cleanup',String(limit),String(count)];gate('qqbar-cleanup-cubic-native-'+limit+'-'+count+'-v84','.','timeout',native(cubic,rest));
  gate('qqbar-cleanup-cubic-mem-'+limit+'-'+count+'-v84','.','timeout',mem(cubic,rest),limit===4?97:0,null,limit===4?'diagnostic-leak':'success');
 }
 const normal=o.dir+'/normal';for(const fixed of [false,true]){
  const suffix=fixed?'-fixed':'';gate('qqbar-normal-compile'+suffix+'-v84','.','gcc',[...cc,'flint-qqbar-normal'+suffix+'-v84.c',...link,'-o',normal],fixed?0:1,null,fixed?'success':'harness-failure');
  node('qqbar-normal-run'+suffix+'-v84','run-qqbar-normal'+suffix+'-v84.mjs',fixed?0:1,fixed?'success':'harness-failure');
 }
 assert(read('results/qqbar-normal-compile-v84.stderr').includes('implicit declaration'));
 gate('qqbar-normal-native-v84','.','timeout',['120s','prlimit','--core=0','--as=1073741824','--cpu=90','--',normal]);gate('qqbar-normal-mem-v84','.','timeout',mem(normal,[],'180s'));
 node('qqbar-remainder-check-v84','check-qqbar-remainder-v84.mjs',1,'harness-failure');node('qqbar-remainder-check-fixed-v84','check-qqbar-remainder-fixed-v84.mjs');
 for(const fixed of [false,true]){
  const suffix=fixed?'-fixed':'';
  gate('qqbar-remainder-hyper-fmt'+suffix+'-v84',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']);
  gate('qqbar-remainder-hyper-metadata'+suffix+'-v84',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']);
  for(const profile of ['debug','release'])gate('qqbar-remainder-hyper-'+profile+suffix+'-v84',cwd,'timeout',['180s','env',...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]);
  gate('qqbar-remainder-hyper-clippy'+suffix+'-v84',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings'],fixed?0:101,null,fixed?'success':'harness-failure');
  node('qqbar-remainder-hyper-run'+suffix+'-v84','run-qqbar-remainder-hyper'+suffix+'-v84.mjs',fixed?0:1,fixed?'success':'harness-failure');
 }
 assert(read('results/qqbar-remainder-hyper-clippy-v84.stderr').includes('unnecessary_cast'));node('qqbar-remainder-hyper-check-v84','check-qqbar-remainder-hyper-v84.mjs');
 const graph=json('results/qqbar-remainder-hyper-metadata-fixed-v84.stdout'),ids=new Set(graph.packages.map(p=>p.id));assert.equal(ids.size,21);assert.equal(graph.resolve.nodes.length,21);
 assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
 const hp=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.deepEqual(hp.map(p=>p.name).sort(),['hyperreal']);assert.equal(hp[0].manifest_path,workspace+'/hyperreal/Cargo.toml');
 const numerical=remainderEvidence(),hyper=hyperEvidence();
 for(const[tag,v]of [['qqbar-remainder-check-fixed-v84',numerical],['qqbar-remainder-hyper-check-v84',hyper]]){assert.deepEqual(json('results/'+tag+'.stdout'),v);assert.equal(read('results/'+tag+'.stderr'),'');}
 const classifiers={};for(const g of gates)classifiers[g.classification]=(classifiers[g.classification]??0)+1;
 assert.equal(gates.length,100);assert.equal(new Set(gates.map(g=>g.tag)).size,100);
 assert.deepEqual(classifiers,{success:78,'harness-failure':9,'environment-failure':2,'diagnostic-abort':3,'diagnostic-overflow-trap':5,'diagnostic-leak':3});
 const executables=[o.binary,o.sanitized,cubic,normal,...['debug','release'].map(p=>target+'/'+p+'/qqbar-remainder-hyper-v84')].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}));
 assert.deepEqual(retainedSources(),current);
 return{checkpoint:84,status:'qualified-qqbar-source-remainder-and-three-donor-defect-families',records,newCompleteFiles:37,completedPartialFiles:1,newLines:5324,coverageAtBinding,qqbar,
  liveFiles:957,isolatedCandidateFiles:184,gates,classifiers,numerical,hyper,dependencyGraph:{packages:21,nodes:21,hyperPackages:['hyperreal']},executables,
  executableBytes:executables.reduce((n,b)=>n+b.bytes,0),retainedContinuationTransfers:7,productionChanges:0,
  limits:'Current donor executed, archived and randomized upstream tests read only. No production change, matched benchmark, full-stack regression, WASM or product-size claim. Full ecosystem inventory still open.',
  next:'Finish wider Calcium/FLINT support and other requested references; qualify bounded relation/proof-scheduling hypotheses before retention; reconcile whole inventory.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(evidence84()));
