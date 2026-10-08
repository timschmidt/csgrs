import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {cargoEnv,target} from './point-qualified-capture.mjs';
import {cases,selfTest} from './quadratic-extraction-oracle-v82.mjs';
import {quadraticEvidence} from './check-quadratic-extraction-v82.mjs';
import {hyperQuadraticEvidence} from './check-quadratic-extraction-hyper-v82.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='quadratic-extraction-hyper-v82';
export const evidenceFiles=['flint-quadratic-extraction-v82.c','quadratic-extraction-protocol-v82.md','quadratic-extraction-oracle-v82.mjs',
 'prepare-quadratic-extraction-v82.mjs','confirm-quadratic-extraction-v82.mjs','run-quadratic-extraction-v82.mjs',
 'quadratic-extraction-origin-v82.json','quadratic-extraction-input-v82.json','quadratic-extraction-input-v82.tsv',
 'quadratic-extraction-read-records-v82.json','check-quadratic-extraction-v82.mjs','quadratic-extraction-hyper-protocol-v82.md',
 cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs','run-quadratic-extraction-hyper-v82.mjs','quadratic-extraction-hyper-origin-v82.json',
 'check-quadratic-extraction-hyper-v82.mjs','quadratic-extraction-evidence-v82.mjs','verify-quadratic-extraction-v82.mjs','quadratic-extraction-v82-findings.md'];
function gate(tag,directory,command,args){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Number.isFinite(Date.parse(g.started)));
 assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
export function extractionEvidence(){
 const o=json('quadratic-extraction-origin-v82.json'),h=json('quadratic-extraction-hyper-origin-v82.json'),
  prior=json('scalar-boundary-v81-manifest.json'),candidate=json('twelfth-revision-v79-manifest.json'),current=retainedSources();
 assert.deepEqual(current,o.current);assert.deepEqual(current,h.current);assert.equal(current.liveFiles,957);
 assert.equal(sha('scalar-boundary-v81-manifest.json'),o.previousSha256);
 for(const[p,v]of Object.entries({...prior.files,...o.files,...h.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),v,p);
 assert.equal(Object.keys(prior.liveSources).length,957);for(const[p,v]of Object.entries(prior.liveSources))assert.equal(sha(workspace+'/'+p),v,p);
 for(const[p,v]of Object.entries(candidate.candidateSources))assert.equal(sha(candidate.candidateRoot+'/'+p),v,p);
 assert.equal(Object.keys(candidate.candidateSources).length,184);assert.deepEqual(h.cargoEnv,cargoEnv);
 const inputs=cases();assert.deepEqual(json('quadratic-extraction-input-v82.json'),inputs);assert.deepEqual(o.oracle,selfTest());
 assert.equal(read('quadratic-extraction-input-v82.tsv'),inputs.map(c=>[c.id,c.a,c.b,BigInt(c.d)*BigInt(c.s)**2n,c.q].join('\t')).join('\n')+'\n');
 const records=json('quadratic-extraction-read-records-v82.json');assert.deepEqual(records,o.records);
 const ext=json('coverage-extensions.json');assert.deepEqual(ext.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(ext.slice(o.extensionsBefore.length,o.extensionsBefore.length+records.length),records);
 const inventory=json('inventory.json'),coverage=effectiveCoverage();let lines=0;
 for(const s of inventory.sources){
  const path=workspace+'/exact-real-references/'+s.repo;
  assert.equal(execFileSync('git',['rev-parse','HEAD'],{cwd:path,encoding:'utf8'}).trim(),s.commit);
  assert.equal(execFileSync('git',['status','--short','--untracked-files=no'],{cwd:path,encoding:'utf8'}),'');
 }
 for(const r of records){
  const f=inventory.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);assert.deepEqual(r.ranges,[[1,f.lines]]);
  assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);assert.equal(sha(workspace+'/exact-real-references/'+r.repo+'/'+r.path),f.sha256);
  assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,r.ranges);lines+=f.lines;
 }
 assert.equal(records.length,4);assert.equal(lines,644);
 for(const[k,v]of Object.entries(o.manualHashes)){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),v);}
 for(const[p,v]of Object.entries({...o.hyperReadHashes,...h.extraHyperHashes}))assert.equal(sha(workspace+'/'+p),v);
 const additionalHyperReads={'hyperreal/src/structural.rs':[[75,230]]};
 for(const p of Object.keys(additionalHyperReads))assert.equal(sha(workspace+'/'+p),prior.liveSources[p]);
 const coverageAtBinding=[{repo:'calcium',reviewed:600,complete:600,partial:0,readLines:63484},
  {repo:'flint',reviewed:932,complete:912,partial:20,readLines:126417}];
 for(const s of effectiveSummary()){const b=coverageAtBinding.find(t=>t.repo===s.repo);assert(s.complete>=b.complete&&s.readLines>=b.readLines);}
 const successful=[],push=(...a)=>{const g=gate(...a);successful.push(g);return g;};
 const before=push('scalar-boundary-verify-v81','.','node',['verify-scalar-boundary-v81.mjs']);
 assert.equal(json('results/'+before.tag+'.stdout').status,'verified-scalar-boundary-source-and-capability');
 const preparation=gate('quadratic-extraction-prepare-v82','.','node',['prepare-quadratic-extraction-v82.mjs']);
 assert(Date.parse(preparation.started)>=Date.parse(before.finished));assert(Date.parse(o.recorded)>=Date.parse(preparation.started)&&Date.parse(o.recorded)<=Date.parse(preparation.finished));
 const confirm=push('quadratic-extraction-confirm-v82','.','node',['confirm-quadratic-extraction-v82.mjs']);
 const confirmed=json('results/'+confirm.tag+'.stdout');assert.equal(confirmed.status,'prepared-files-confirmed');assert.equal(confirmed.recorded,o.recorded);
 assert.equal(confirmed.dir,o.dir);assert.equal(confirmed.files,4);assert.equal(confirmed.lines,644);assert.equal(confirmed.cases,775);
 const lib=workspace+'/exact-real-references/flint',input='quadratic-extraction-input-v82.tsv',native=[];
 native.push(push('quadratic-extraction-compile-v82','.','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-isystem',lib+'/src',
  'flint-quadratic-extraction-v82.c','-L',lib,'-Wl,-rpath,'+lib,'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]));
 native.push(push('quadratic-extraction-linkage-v82','.','ldd',[o.binary]));
 native.push(push('quadratic-extraction-native-v82','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',o.binary,input]));
 native.push(push('quadratic-extraction-memcheck-v82','.','timeout',['180s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary,input]));
 const firstDriver=gate('quadratic-extraction-run-v82','.','node',['run-quadratic-extraction-v82.mjs']);
 assert(Date.parse(firstDriver.started)>=Date.parse(preparation.finished)&&Date.parse(firstDriver.started)<=Date.parse(native[0].started));
 for(let i=1;i<native.length;i++)assert(Date.parse(native[i].started)>=Date.parse(native[i-1].finished));
 assert(Date.parse(firstDriver.finished)>=Date.parse(native.at(-1).finished));
 const driver=push('quadratic-extraction-run-approved-v82','.','node',['run-quadratic-extraction-v82.mjs']),log=read('results/'+driver.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(log.length,5);assert(Date.parse(driver.started)>=Date.parse(firstDriver.finished));
 for(let i=0;i<4;i++)assert.deepEqual(log[i],{reusedGate:native[i].tag,finished:native[i].finished});
 assert.deepEqual(log.at(-1),{checkpoint:82,status:'native-collected',expectedRows:4651,productionChanges:0});
 const mem=read('results/quadratic-extraction-memcheck-v82.stderr');assert(mem.includes('in use at exit: 0 bytes in 0 blocks'));
 assert(mem.includes('1,276,837 allocs, 1,276,837 frees, 280,998,786 bytes allocated'));
 assert(mem.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const linked=read('results/quadratic-extraction-linkage-v82.stdout');for(const p of Object.keys(o.libraries))assert(linked.includes(p));
 assert(linked.includes('/lib64/ld-linux-x86-64.so.2'));
 push('quadratic-extraction-check-v82','.','node',['check-quadratic-extraction-v82.mjs']);
 const mathematical=quadraticEvidence();assert.deepEqual(mathematical,json('results/quadratic-extraction-check-v82.stdout'));
 assert.equal(mathematical.checks,99587);assert.equal(mathematical.corruptions,20);assert.equal(mathematical.bytes,22104752);
 const hyper=[];
 hyper.push(push('quadratic-extraction-hyper-fmt-v82',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']));
 hyper.push(push('quadratic-extraction-hyper-metadata-v82',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']));
 for(const profile of ['debug','release'])hyper.push(push('quadratic-extraction-hyper-'+profile+'-v82',cwd,'env',[
  ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]));
 hyper.push(push('quadratic-extraction-hyper-clippy-v82',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 hyper.push(push('quadratic-extraction-hyper-environment-v82','.','node',['point-qualified-environment.mjs']));
 const hd=push('quadratic-extraction-hyper-run-v82','.','node',['run-quadratic-extraction-hyper-v82.mjs']);
 assert(Date.parse(h.recorded)>=Date.parse(hd.started)&&Date.parse(h.recorded)<=Date.parse(hyper[0].started));
 const hl=read('results/'+hd.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(hl.length,13);
 for(let i=0;i<hyper.length;i++){assert.deepEqual(hl[2*i+1],hyper[i]);if(i)assert(Date.parse(hyper[i].started)>=Date.parse(hyper[i-1].finished));}
 assert(Date.parse(hd.finished)>=Date.parse(hyper.at(-1).finished));
 assert.deepEqual(hl.at(-1),{checkpoint:82,status:'hyper-capability-collected',expectedRows:935,liveFiles:957,productionChanges:0});
 push('quadratic-extraction-hyper-check-v82','.','node',['check-quadratic-extraction-hyper-v82.mjs']);
 const hyperMathematical=hyperQuadraticEvidence();assert.deepEqual(hyperMathematical,json('results/quadratic-extraction-hyper-check-v82.stdout'));
 assert.equal(hyperMathematical.corruptions,13);assert.equal(hyperMathematical.bytesPerProfile,303998);assert.equal(hyperMathematical.uncertain.length,0);
 const graph=json('results/quadratic-extraction-hyper-metadata-v82.stdout'),ids=new Set(graph.packages.map(p=>p.id));
 assert.equal(ids.size,21);assert.equal(graph.resolve.nodes.length,21);assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());
 for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
 const hs=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.equal(hs.length,1);assert.equal(hs[0].manifest_path,workspace+'/hyperreal/Cargo.toml');
 const unusable=[preparation,firstDriver];for(const g of unusable){assert.equal(read('results/'+g.tag+'.stdout'),'');assert.equal(read('results/'+g.tag+'.stderr'),'');}
 for(const g of successful){const err=read('results/'+g.tag+'.stderr');assert(!err.includes('warning:'));
  if(!['quadratic-extraction-memcheck-v82','quadratic-extraction-hyper-clippy-v82'].includes(g.tag))assert.equal(err,'',g.tag);}
 assert.equal(successful.length,16);assert.equal(unusable.length,2);
 const executables=[o.binary,...['debug','release'].map(p=>target+'/'+p+'/quadratic-extraction-hyper-v82')].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}));
 assert.deepEqual(retainedSources(),current);
 return{checkpoint:82,status:'qualified-quadratic-extraction-source-and-capability',liveFiles:957,isolatedCandidateFiles:184,
  readFiles:4,newDonorLines:644,coverageBefore:o.coverageBefore,coverageAtBinding,records,additionalHyperReads,mathematical,hyper:hyperMathematical,
  successfulGates:successful.map(g=>g.tag),unusableGates:unusable.map(g=>g.tag),
  memory:{errors:0,liveBytes:0,allocations:1276837,frees:1276837,requestedBytes:280998786,
   scope:'Whole native collector including construction, three extraction modes, reconstruction, enclosures and output. Cumulative requests, not peak/RSS or per-operation cost.'},
  executables,environment:json('results/quadratic-extraction-hyper-environment-v82.stdout'),dependencyGraph:{packages:21,nodes:21,hyperrealOnly:true},
  libraries:{...o.libraries,'/lib64/ld-linux-x86-64.so.2':sha('/lib64/ld-linux-x86-64.so.2')},retainedContinuationTransfers:7,productionChanges:0,
  limits:'Four complete pinned donor files. Only current FLINT executed; archive/upstream tests read, not newly run. Hyper public default-feature scalar capability only. No new production transfer, full-stack regression, matched benchmark, WASM runtime or representative product-size claim.',
  next:'Continue remaining qqbar expression/complex-support source and all formal, symbolic, historical references plus inventory reconciliation. Existing two-candidate rounding and proof-scheduling hypotheses remain unretained pending qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(extractionEvidence()));
