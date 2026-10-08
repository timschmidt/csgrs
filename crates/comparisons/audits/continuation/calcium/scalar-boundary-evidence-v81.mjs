import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {cargoEnv,target} from './point-qualified-capture.mjs';
import {nativeCases} from './scalar-boundary-oracle-v81.mjs';
import {scalarBoundaryEvidence} from './check-scalar-boundary-fixed-v81.mjs';
import {hyperBoundaryEvidence} from './check-scalar-boundary-hyper-fixed-v81.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='scalar-boundary-hyper-v81';
export const evidenceFiles=['scalar-boundary-protocol-v81.md','scalar-boundary-hyper-protocol-v81.md','scalar-boundary-hyper-correction-v81.md',
 'prepare-scalar-boundary-v81.mjs','scalar-boundary-origin-v81.json','scalar-boundary-input-v81.json','scalar-boundary-read-records-v81.json',
 'flint-scalar-boundary-v81.c','run-scalar-boundary-v81.mjs','scalar-boundary-oracle-v81.mjs','scalar-boundary-endpoints-v81.mjs',
 'check-scalar-boundary-v81.mjs','check-scalar-boundary-fixed-v81.mjs','run-scalar-boundary-hyper-v81.mjs','run-scalar-boundary-hyper-fixed-v81.mjs',
 'scalar-boundary-hyper-origin-v81.json','scalar-boundary-hyper-origin-fixed-v81.json','scalar-boundary-hyper-initial-v81.rs',
 cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs','check-scalar-boundary-hyper-v81.mjs','check-scalar-boundary-hyper-fixed-v81.mjs',
 'scalar-boundary-evidence-v81.mjs','verify-scalar-boundary-v81.mjs','scalar-boundary-v81-findings.md'];
function gate(tag,directory,command,args,code=0){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,code);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Number.isFinite(Date.parse(g.started)));
 assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
export function boundaryEvidence(){
 const o=json('scalar-boundary-origin-v81.json'),old=json('scalar-boundary-hyper-origin-v81.json'),h=json('scalar-boundary-hyper-origin-fixed-v81.json'),
  prior=json('qqbar-inverse-v80-manifest.json'),candidate=json('twelfth-revision-v79-manifest.json'),current=retainedSources();
 assert.deepEqual(current,o.current);assert.deepEqual(current,old.current);assert.deepEqual(current,h.current);assert.equal(current.liveFiles,957);
 assert.equal(sha('qqbar-inverse-v80-manifest.json'),o.previousSha256);
 for(const[p,v]of Object.entries({...prior.files,...o.files,...h.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),v,p);
 for(const[p,v]of Object.entries(candidate.candidateSources))assert.equal(sha(candidate.candidateRoot+'/'+p),v,p);
 assert.equal(Object.keys(candidate.candidateSources).length,184);
 for(const[p,v]of Object.entries(old.files))assert.equal(sha(p===cwd+'/src/main.rs'?'scalar-boundary-hyper-initial-v81.rs':p),v,p);
 const compact=s=>s.replace(/\s/g,'');assert.equal(compact(read(cwd+'/src/main.rs')),compact(read('scalar-boundary-hyper-initial-v81.rs'))
  .replace('value.as_f64_lossy()','value.to_f64_lossy()').replace('angle.sec_pi()','Real::one()/angle.cos_pi()').replace('angle.csc_pi()','Real::one()/angle.sin_pi()'));
 for(const origin of [old,h])assert.deepEqual(origin.cargoEnv,cargoEnv);
 assert.deepEqual(json('scalar-boundary-input-v81.json'),nativeCases());
 const records=json('scalar-boundary-read-records-v81.json');assert.deepEqual(records,o.records);
 const ext=json('coverage-extensions.json');assert.deepEqual(ext.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(ext.slice(o.extensionsBefore.length,o.extensionsBefore.length+records.length),records);
 const inventory=json('inventory.json'),coverage=effectiveCoverage();let lines=0;
 for(const r of records){
  const f=inventory.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);assert.deepEqual(r.ranges,[[1,f.lines]]);
  assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);assert.equal(sha(workspace+'/exact-real-references/'+r.repo+'/'+r.path),f.sha256);
  assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,r.ranges);lines+=f.lines;
 }
 assert.equal(records.length,42);assert.equal(lines,1902);
 for(const[k,v]of Object.entries(o.manualHashes)){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),v);}
 for(const[p,v]of Object.entries({...o.hyperReadHashes,...old.extraHyperHashes,...h.extraHyperHashes}))assert.equal(sha(workspace+'/'+p),v);
 const extraReads={'hyperreal/src/real/arithmetic/mul_div.rs':[[500,560]],'hyperreal/src/problem.rs':[[1,56]]};
 for(const[p,ranges]of Object.entries(extraReads)){
  assert.equal(sha(workspace+'/'+p),prior.liveSources[p]);const count=read(workspace+'/'+p).split('\n').length;
  for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<count);
 }
 const coverageAtBinding=[{repo:'calcium',reviewed:598,complete:598,partial:0,readLines:63158},
  {repo:'flint',reviewed:930,complete:910,partial:20,readLines:126099}];
 for(const s of effectiveSummary()){const b=coverageAtBinding.find(t=>t.repo===s.repo);assert(s.complete>=b.complete&&s.readLines>=b.readLines);}
 const currentGates=[],push=(...a)=>{const g=gate(...a);currentGates.push(g);return g;};
 const before=push('inverse-reciprocal-current80-before-v81','.','node',['verify-qqbar-inverse-v80.mjs']);
 assert.equal(json('results/'+before.tag+'.stdout').status,'verified-inverse-source-and-completeness-audit');
 const preparation=push('scalar-boundary-prepare-v81','.','node',['prepare-scalar-boundary-v81.mjs']);
 assert.deepEqual(json('results/'+preparation.tag+'.stdout'),{checkpoint:81,status:'prepared',newFiles:42,newLines:1902,cases:19923,dir:o.dir});
 assert(Date.parse(preparation.started)>=Date.parse(before.finished));assert(Date.parse(o.recorded)>=Date.parse(preparation.started)&&Date.parse(o.recorded)<=Date.parse(preparation.finished));
 const lib=workspace+'/exact-real-references/flint',native=[];
 native.push(push('scalar-boundary-compile-v81','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror',
  '-isystem',lib+'/src','flint-scalar-boundary-v81.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]));
 native.push(push('scalar-boundary-linkage-v81','.','ldd',[o.binary]));
 native.push(push('scalar-boundary-native-v81','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',o.binary]));
 native.push(push('scalar-boundary-memcheck-v81','.','timeout',['180s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary]));
 const driver=push('scalar-boundary-run-v81','.','node',['run-scalar-boundary-v81.mjs']),log=read('results/'+driver.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(log.length,9);for(let i=0;i<native.length;i++){
  assert.deepEqual(log[2*i+1],native[i]);assert(Date.parse(native[i].started)>=Date.parse(i?native[i-1].finished:o.recorded));}
 assert(Date.parse(driver.started)<=Date.parse(native[0].started)&&Date.parse(driver.finished)>=Date.parse(native.at(-1).finished));
 assert.deepEqual(log.at(-1),{checkpoint:81,status:'native-collected',expectedRows:19924,productionChanges:0});
 const raw=readFileSync('results/scalar-boundary-native-v81.stdout');assert(raw.equals(readFileSync('results/scalar-boundary-memcheck-v81.stdout')));assert.equal(raw.length,9799577);
 const mem=read('results/scalar-boundary-memcheck-v81.stderr');
 assert(mem.includes('in use at exit: 0 bytes in 0 blocks'));assert(mem.includes('1,682,521 allocs, 1,682,521 frees, 2,220,444,799 bytes allocated'));
 assert(mem.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const linked=read('results/scalar-boundary-linkage-v81.stdout');for(const p of Object.keys(o.libraries))assert(linked.includes(p));assert(linked.includes('/lib64/ld-linux-x86-64.so.2'));
 push('scalar-boundary-check-fixed-v81','.','node',['check-scalar-boundary-fixed-v81.mjs']);
 const mathematical=scalarBoundaryEvidence();assert.deepEqual(mathematical,json('results/scalar-boundary-check-fixed-v81.stdout'));
 assert.equal(mathematical.totalChecks,163328);assert.equal(mathematical.corruptionControlsRejected,18);
 const hyper=[];
 hyper.push(push('scalar-boundary-hyper-fmt-fixed-v81',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']));
 hyper.push(push('scalar-boundary-hyper-metadata-fixed-v81',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']));
 for(const profile of ['debug','release'])hyper.push(push('scalar-boundary-hyper-'+profile+'-fixed-v81',cwd,'env',[
  ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]));
 hyper.push(push('scalar-boundary-hyper-clippy-fixed-v81',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 hyper.push(push('scalar-boundary-hyper-environment-fixed-v81','.','node',['point-qualified-environment.mjs']));
 const hd=push('scalar-boundary-hyper-run-fixed-v81','.','node',['run-scalar-boundary-hyper-fixed-v81.mjs']);
 assert(Date.parse(h.recorded)>=Date.parse(hd.started)&&Date.parse(h.recorded)<=Date.parse(hyper[0].started));
 const hl=read('results/'+hd.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(hl.length,13);
 for(let i=0;i<hyper.length;i++){assert.deepEqual(hl[2*i+1],hyper[i]);if(i)assert(Date.parse(hyper[i].started)>=Date.parse(hyper[i-1].finished));}
 assert(Date.parse(hd.finished)>=Date.parse(hyper.at(-1).finished));
 assert.deepEqual(hl.at(-1),{checkpoint:81,status:'hyper-capability-collected',expectedRows:19341,liveFiles:957,productionChanges:0});
 push('scalar-boundary-hyper-check-fixed-v81','.','node',['check-scalar-boundary-hyper-fixed-v81.mjs']);
 const hyperMathematical=hyperBoundaryEvidence();assert.deepEqual(hyperMathematical,json('results/scalar-boundary-hyper-check-fixed-v81.stdout'));
 assert.equal(hyperMathematical.corruptionControlsRejected,12);
 const development=[gate('scalar-boundary-hyper-fmt-v81',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']),
  gate('scalar-boundary-hyper-metadata-v81',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1'])];
 const failed=[gate('scalar-boundary-check-v81','.','node',['check-scalar-boundary-v81.mjs'],1),
  gate('scalar-boundary-hyper-debug-v81',cwd,'env',[...cargoEnv,'cargo','run','--offline','--locked','--quiet'],101),
  gate('scalar-boundary-hyper-run-v81','.','node',['run-scalar-boundary-hyper-v81.mjs'],1),
  gate('scalar-boundary-hyper-check-v81','.','node',['check-scalar-boundary-hyper-v81.mjs'],1)];
 assert(read('results/scalar-boundary-check-v81.stderr').includes('assert(abs(e)<=8192n)'));
 for(const name of ['as_f64_lossy','sec_pi','csc_pi'])assert(read('results/scalar-boundary-hyper-debug-v81.stderr').includes('no method named `'+name+'`'));
 assert(read('results/scalar-boundary-hyper-check-v81.stderr').includes("+ 'DivideByZero'"));
 for(const g of failed.filter(g=>g.tag!=='scalar-boundary-hyper-run-v81'))assert.equal(read('results/'+g.tag+'.stdout'),'');
 const graph=json('results/scalar-boundary-hyper-metadata-fixed-v81.stdout');assert.deepEqual(graph,json('results/scalar-boundary-hyper-metadata-v81.stdout'));
 const ids=new Set(graph.packages.map(p=>p.id));assert.equal(ids.size,21);assert.equal(graph.resolve.nodes.length,21);
 assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
 const hs=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.equal(hs.length,1);assert.equal(hs[0].manifest_path,workspace+'/hyperreal/Cargo.toml');
 for(const g of [...currentGates,...development]){
  const err=read('results/'+g.tag+'.stderr');assert(!err.includes('warning:'));
  if(!['scalar-boundary-memcheck-v81','scalar-boundary-hyper-clippy-fixed-v81'].includes(g.tag))assert.equal(err,'',g.tag);
 }
 assert.equal(currentGates.length,16);assert.equal(development.length,2);assert.equal(failed.length,4);
 const executables=[o.binary,...['debug','release'].map(p=>target+'/'+p+'/scalar-boundary-hyper-v81')].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}));
 assert.deepEqual(retainedSources(),current);
 return{checkpoint:81,status:'qualified-scalar-boundary-source-and-capability',liveFiles:957,isolatedCandidateFiles:184,
  readFiles:42,newDonorLines:1902,coverageBefore:o.coverageBefore,coverageAtBinding,records,extraReads,mathematical,hyper:hyperMathematical,
  successfulGates:currentGates.map(g=>g.tag),developmentGates:development.map(g=>g.tag),failedGates:failed.map(g=>g.tag),
  memory:{errors:0,liveBytes:0,allocations:1682521,frees:1682521,requestedBytes:2220444799,
   scope:'Entire native collector, including import, construction, exact algebraic operations, caches and output; cumulative requests, not peak/RSS or per-operation cost.'},
  executables,environment:json('results/scalar-boundary-hyper-environment-fixed-v81.stdout'),dependencyGraph:{packages:21,nodes:21,hyperrealOnly:true},
  libraries:{...o.libraries,'/lib64/ld-linux-x86-64.so.2':sha('/lib64/ld-linux-x86-64.so.2')},retainedContinuationTransfers:7,productionChanges:0,
  limits:'42 selected full donor files. Only current FLINT executed; upstream tests read, not newly run. Bounded IEEE mantissa samples, not exhaustive float patterns. Derived Hyper reciprocal functions are not absent public APIs. No new production transfer, full-stack regression, benchmark, WASM runtime or representative product-size qualification.',
  next:'Investigate a certificate-preserving two-candidate directional rounding proposal using existing near_integer, with isolated differential correctness and matched costs before any retention. Continue remaining qqbar quadratic/expression/complex-support files and all formal, symbolic, historical references and inventory reconciliation.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(boundaryEvidence()));
