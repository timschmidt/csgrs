import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {cargoEnv,target} from './point-qualified-capture.mjs';
import {cases} from './qqbar-inverse-oracle-v80.mjs';
import {checkQqbarInverse} from './check-qqbar-inverse-v80.mjs';
import {counterexampleReplay} from './qqbar-inverse-counterexample-replay-v80.mjs';
import {hyperInverseEvidence} from './check-qqbar-inverse-hyper-v80.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='qqbar-inverse-hyper-v80';
export const evidenceFiles=[
 'qqbar-inverse-protocol-v80.md','qqbar-inverse-hyper-protocol-v80.md','prepare-qqbar-inverse-v80.mjs','qqbar-inverse-origin-v80.json',
 'qqbar-inverse-input-v80.json','qqbar-inverse-read-records-v80.json','flint-qqbar-inverse-v80.c','flint-qqbar-inverse-counterexample-v80.c',
 'run-qqbar-inverse-v80.mjs','qqbar-inverse-oracle-v80.mjs','check-qqbar-inverse-v80.mjs','check-qqbar-inverse-counterexample-v80.mjs',
 'qqbar-inverse-counterexample-replay-v80.mjs','run-qqbar-inverse-hyper-v80.mjs','qqbar-inverse-hyper-origin-v80.json',
 cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs','check-qqbar-inverse-hyper-v80.mjs','qqbar-inverse-evidence-v80.mjs',
 'verify-qqbar-inverse-v80.mjs','qqbar-inverse-v80-findings.md'];
function gate(tag,directory,command,args){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,0);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Number.isFinite(Date.parse(g.started)));
 assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
export function inverseEvidence(){
 const o=json('qqbar-inverse-origin-v80.json'),h=json('qqbar-inverse-hyper-origin-v80.json'),prior=json('twelfth-revision-v79-manifest.json');
 const source=retainedSources();assert.deepEqual(source,o.current);assert.deepEqual(source,h.source);
 assert.equal(sha('twelfth-revision-v79-manifest.json'),o.previousSha256);
 for(const[p,v]of Object.entries({...prior.files,...o.files,...h.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),v,p);
 for(const[p,v]of Object.entries(prior.candidateSources))assert.equal(sha(prior.candidateRoot+'/'+p),v,p);
 assert.deepEqual(json('qqbar-inverse-input-v80.json'),cases());assert.deepEqual(h.cargoEnv,cargoEnv);
 const records=json('qqbar-inverse-read-records-v80.json');assert.deepEqual(records,o.records);
 const ext=json('coverage-extensions.json');assert.deepEqual(ext.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(ext.slice(o.extensionsBefore.length,o.extensionsBefore.length+records.length),records);
 const inventory=json('inventory.json'),coverage=effectiveCoverage();let newLines=0;
 for(const r of records){
  const f=inventory.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);assert.deepEqual(r.ranges,[[1,f.lines]]);
  assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);assert.equal(sha(workspace+'/exact-real-references/'+r.repo+'/'+r.path),f.sha256);
  assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,r.ranges);newLines+=f.lines;
 }
 assert.equal(records.length,19);assert.equal(newLines,1738);assert.equal(o.rereads.length,2);
 for(const[k,v]of Object.entries({...o.donorSources,...o.rereadHashes})){const[repo,p]=k.split(':');assert.equal(sha(workspace+'/exact-real-references/'+repo+'/'+p),v);}
 for(const[p,v]of Object.entries(o.hyperReadHashes))assert.equal(sha(workspace+'/'+p),v);
 const coverageAtBinding=[{repo:'calcium',reviewed:577,complete:577,partial:0,readLines:62179},
  {repo:'flint',reviewed:909,complete:889,partial:20,readLines:125176}];
 for(const s of effectiveSummary()){const old=coverageAtBinding.find(t=>t.repo===s.repo);assert(s.complete>=old.complete&&s.readLines>=old.readLines);}
 const gates=[],push=(...a)=>{const g=gate(...a);gates.push(g);return g;};
 const before=push('qqbar-inverse-current79-before-v80','.','node',['verify-twelfth-revision-v79.mjs']);
 assert.equal(json('results/'+before.tag+'.stdout').status,'verified-cheaper-revision-not-retained');
 const prepare=push('qqbar-inverse-prepare-v80','.','node',['prepare-qqbar-inverse-v80.mjs']);
 assert.deepEqual(json('results/'+prepare.tag+'.stdout'),{checkpoint:80,status:'prepared',newFiles:19,newLines:1738,rereadImplementations:2,cases:1081,dir:o.dir});
 assert(Date.parse(o.recorded)>=Date.parse(prepare.started)&&Date.parse(o.recorded)<=Date.parse(prepare.finished));
 assert(Date.parse(prepare.started)>=Date.parse(before.finished));
 const lib=workspace+'/exact-real-references/flint',nativeGates=[];
 for(const[name,input,binary]of [['corpus','flint-qqbar-inverse-v80.c',o.binary],['counterexample','flint-qqbar-inverse-counterexample-v80.c',o.counterexampleBinary]]){
  nativeGates.push(push('qqbar-inverse-'+name+'-compile-v80','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror',
   '-isystem',lib+'/src',input,'-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]));
  nativeGates.push(push('qqbar-inverse-'+name+'-linkage-v80','.','ldd',[binary]));
  const linked=read('results/qqbar-inverse-'+name+'-linkage-v80.stdout');for(const p of Object.keys(o.libraries))assert(linked.includes(p));
  assert(linked.includes('/lib64/ld-linux-x86-64.so.2'));
 }
 nativeGates.push(push('qqbar-inverse-native-v80','.',o.binary,[]));
 nativeGates.push(push('qqbar-inverse-memcheck-v80','.','valgrind',['--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary]));
 nativeGates.push(push('qqbar-inverse-counterexample-native-v80','.','timeout',['45s','prlimit','--as=1073741824','--cpu=30','--',o.counterexampleBinary]));
 const driver=push('qqbar-inverse-run-v80','.','node',['run-qqbar-inverse-v80.mjs']);
 const log=read('results/'+driver.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(log.length,15);
 for(let i=0;i<nativeGates.length;i++){assert.deepEqual(log[2*i+1],nativeGates[i]);assert(Date.parse(nativeGates[i].started)>=Date.parse(i?nativeGates[i-1].finished:o.recorded));}
 assert(Date.parse(driver.started)<=Date.parse(nativeGates[0].started)&&Date.parse(driver.finished)>=Date.parse(nativeGates.at(-1).finished));
 assert.deepEqual(log.at(-1),{checkpoint:80,status:'native-collection-complete',expectedRows:2163,executables:2,retained:false,
  note:'Collection only; independent mathematical checking and the public counterexample proof must pass separately.'});
 const raw=readFileSync('results/qqbar-inverse-native-v80.stdout');assert(raw.equals(readFileSync('results/qqbar-inverse-memcheck-v80.stdout')));assert.equal(raw.length,593765);
 const memory=read('results/qqbar-inverse-memcheck-v80.stderr');
 assert(memory.includes('in use at exit: 0 bytes in 0 blocks'));assert(memory.includes('1,492,610 allocs, 1,492,610 frees, 74,269,892 bytes allocated'));
 assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const unusable=[];
 for(const[tag,script,arg]of [
  ['qqbar-inverse-check-v80','check-qqbar-inverse-v80.mjs','results/qqbar-inverse-native-v80.stdout'],
  ['qqbar-inverse-counterexample-check-v80','check-qqbar-inverse-counterexample-v80.mjs','results/qqbar-inverse-counterexample-native-v80.stdout']]){
  unusable.push(gate(tag,'.','node',[script,arg]));assert.equal(read('results/'+tag+'.stdout'),'');assert.equal(read('results/'+tag+'.stderr'),'');
 }
 push('qqbar-inverse-check-confirmed-v80','.','node',['check-qqbar-inverse-v80.mjs','results/qqbar-inverse-native-v80.stdout']);
 const mathematical=checkQqbarInverse('results/qqbar-inverse-native-v80.stdout');assert.deepEqual(mathematical,json('results/qqbar-inverse-check-confirmed-v80.stdout'));
 assert.equal(mathematical.status,'pass');assert.equal(mathematical.totalChecks,25216);assert.equal(mathematical.corruptionControlsRejected,15);
 push('qqbar-inverse-counterexample-check-confirmed-v80','.','node',['check-qqbar-inverse-counterexample-v80.mjs','results/qqbar-inverse-counterexample-native-v80.stdout']);
 push('qqbar-inverse-counterexample-replay-v80','.','node',['qqbar-inverse-counterexample-replay-v80.mjs']);
 const counterexample=counterexampleReplay();assert.deepEqual(counterexample,json('results/qqbar-inverse-counterexample-replay-v80.stdout'));
 const hyperGates=[];
 hyperGates.push(push('qqbar-inverse-hyper-fmt-v80',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']));
 hyperGates.push(push('qqbar-inverse-hyper-metadata-v80',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1']));
 for(const profile of ['debug','release'])hyperGates.push(push('qqbar-inverse-hyper-'+profile+'-v80',cwd,'env',[
  ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]));
 hyperGates.push(push('qqbar-inverse-hyper-clippy-v80',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 hyperGates.push(push('qqbar-inverse-hyper-environment-v80','.','node',['point-qualified-environment.mjs']));
 const hd=push('qqbar-inverse-hyper-run-v80','.','node',['run-qqbar-inverse-hyper-v80.mjs']);
 assert(Date.parse(h.recorded)>=Date.parse(hd.started)&&Date.parse(h.recorded)<=Date.parse(hyperGates[0].started));
 const hl=read('results/'+hd.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(hl.length,13);
 for(let i=0;i<hyperGates.length;i++){assert.deepEqual(hl[2*i+1],hyperGates[i]);if(i)assert(Date.parse(hyperGates[i].started)>=Date.parse(hyperGates[i-1].finished));}
 assert(Date.parse(hd.finished)>=Date.parse(hyperGates.at(-1).finished));
 assert.deepEqual(hl.at(-1),{checkpoint:80,status:'hyper-capability-collected',expectedRows:1849,profiles:['debug','release'],retained:false});
 push('qqbar-inverse-hyper-check-v80','.','node',['check-qqbar-inverse-hyper-v80.mjs']);
 const hyper=hyperInverseEvidence();assert.deepEqual(hyper,json('results/qqbar-inverse-hyper-check-v80.stdout'));
 const graph=json('results/qqbar-inverse-hyper-metadata-v80.stdout'),ids=new Set(graph.packages.map(p=>p.id));assert.equal(ids.size,21);assert.equal(graph.resolve.nodes.length,21);
 assert.deepEqual([...ids].sort(),graph.resolve.nodes.map(n=>n.id).sort());for(const n of graph.resolve.nodes)for(const d of n.dependencies)assert(ids.has(d));
 const hs=graph.packages.filter(p=>p.name.startsWith('hyper'));assert.equal(hs.length,1);assert.equal(hs[0].manifest_path,workspace+'/hyperreal/Cargo.toml');
 for(const g of gates){const err=read('results/'+g.tag+'.stderr');assert(!err.includes('warning:'));if(!['qqbar-inverse-memcheck-v80','qqbar-inverse-hyper-clippy-v80'].includes(g.tag))assert.equal(err,'',g.tag);}
 assert.equal(gates.length,21);assert.equal(unusable.length,2);
 const executables=[o.binary,o.counterexampleBinary,...['debug','release'].map(p=>target+'/'+p+'/qqbar-inverse-hyper-v80')]
  .map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}));
 assert.deepEqual(retainedSources(),source);
 return{checkpoint:80,status:'qualified-inverse-source-and-completeness-audit',liveFiles:Object.keys(source).length,
  readFiles:records.length,newDonorLines:newLines,coverageBefore:o.coverageBefore,coverageAtBinding,records,
  mathematical,counterexample,hyper,successfulGates:gates.map(g=>g.tag),preservedUnusableGates:unusable.map(g=>g.tag),
  memory:{errors:0,liveBytes:0,allocations:1492610,frees:1492610,requestedBytes:74269892,
   scope:'Entire corpus collector, including construction, caching and output. Not per-inverse costs, peak/RSS, counterexample Memcheck or matched performance.'},
  executables,environment:json('results/qqbar-inverse-hyper-environment-v80.stdout'),dependencyGraph:{packages:21,nodes:21,hyperrealOnly:true},
  libraries:{...o.libraries,'/lib64/ld-linux-x86-64.so.2':sha('/lib64/ld-linux-x86-64.so.2')},retainedContinuationTransfers:7,productionChanges:0,
  limits:'19 new full donor files plus two implementation rereads. Only current FLINT executed; upstream tests read, not newly run. Hyper capability probe is not full-stack regression or a benchmark. The documented recognition completeness failure is not an unsound accepted equality. No production/donor edit, new retention, cleanup, commit or push.',
  next:'Continue remaining qqbar inverse/root/recognition and supporting files, then remaining formal, symbolic and historical references; preserve proof certificates and Unknown fallbacks. Full inventory reconciliation and unresolved worthwhile transfers remain required.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(inverseEvidence()));
