import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {retainedSources,workspace,sha,json} from './zero-factor-retained-sources-v75.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkQqbarTrig} from './check-qqbar-trig-v76.mjs';
import {hyperEvidence} from './check-qqbar-trig-hyper-v76.mjs';
import {cargoEnv} from './point-qualified-capture.mjs';
const read=p=>readFileSync(p,'utf8'),cwd='qqbar-trig-hyper-v76';
export const evidenceFiles=['qqbar-trig-protocol-v76.md','qqbar-trig-hyper-protocol-v76.md','flint-qqbar-trig-v76.c',
 'prepare-qqbar-trig-v76.mjs','qqbar-trig-origin-v76.json','qqbar-trig-read-records-v76.json','check-qqbar-trig-v76.mjs',
 'run-qqbar-trig-hyper-v76.mjs','run-qqbar-trig-hyper-fixed-v76.mjs','qqbar-trig-hyper-origin-v76.json','qqbar-trig-hyper-origin-fixed-v76.json',
 'qqbar-trig-hyper-initial-v76.rs','qqbar-trig-hyper-pretypefix-v76.rs','check-qqbar-trig-hyper-v76.mjs',
 'check-qqbar-trig-evidence-v76.mjs','check-qqbar-trig-evidence-fixed-v76.mjs','verify-qqbar-trig-v76.mjs','qqbar-trig-v76-findings.md',
 cwd+'/Cargo.toml',cwd+'/Cargo.lock',cwd+'/src/main.rs'];
function gate(tag,directory,command,args,code=0){
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.cwd,resolve(directory));assert.equal(g.command,command);
 assert.deepEqual(g.args,args);assert.equal(g.code,code);assert.equal(g.signal,null);assert(!g.error);
 assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Number.isFinite(Date.parse(g.started)));
 assert(Date.parse(g.finished)>=Date.parse(g.started));return g;
}
export function trigEvidence(){
 const o=json('qqbar-trig-origin-v76.json'),source=retainedSources();assert.deepEqual(o.source,source);
 for(const[p,h]of Object.entries(o.files))assert.equal(sha(p),h,p);
 const retained=json('zero-factor-retained-v75-manifest.json');for(const[p,h]of Object.entries(retained.files))assert.equal(sha(p),h,p);
 for(const[p,h]of Object.entries({...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);
 const inv=json('inventory.json'),records=json('qqbar-trig-read-records-v76.json');assert.deepEqual(records,o.records);
 const ext=json('coverage-extensions.json');assert.deepEqual(ext.slice(0,o.extensionsBefore.length),o.extensionsBefore);
 assert.equal(createHash('sha256').update(JSON.stringify(o.extensionsBefore,null,2)+'\n').digest('hex'),o.extensionsBeforeSha256);
 assert.deepEqual(ext.slice(o.extensionsBefore.length,o.extensionsBefore.length+records.length),records);
 const coverage=effectiveCoverage();let newLines=0;
 for(const r of records){
  const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
  assert.deepEqual(r.ranges,[[1,f.lines]]);newLines+=f.lines;assert.equal(o.donorSources[r.repo+':'+r.path],f.sha256);
  assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
  assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,r.ranges);
 }
 assert.equal(records.length,32);assert.equal(newLines,1964);
 const coverageAtBinding=[{repo:'calcium',reviewed:568,complete:568,partial:0,readLines:61301},
  {repo:'flint',reviewed:899,complete:879,partial:20,readLines:124316}];
 for(const s of effectiveSummary()){const then=coverageAtBinding.find(t=>t.repo===s.repo);assert(s.complete>=then.complete&&s.readLines>=then.readLines);}
 const extraHyperReads={'hyperreal/src/real/arithmetic/facts.rs':[[1170,1348]],
  'hyperreal/src/real/arithmetic/comparison.rs':[[1,81]],'hyperreal/src/real/arithmetic/representation.rs':[[588,595]]};
 for(const ranges of [o.hyperReadRanges,extraHyperReads])for(const[p,rs]of Object.entries(ranges)){
  const text=read(resolve(workspace,p)),lines=text.split('\n').length-Number(text.endsWith('\n'));
  for(const[a,b]of rs)assert(a>=1&&a<=b&&b<=lines,p);
  assert.equal(sha(resolve(workspace,p)),retained.liveSources[p]);
 }
 for(const[p,h]of Object.entries(o.hyperReadHashes))assert.equal(sha(resolve(workspace,p)),h);
 for(const[k,h]of Object.entries(o.rereadHashes)){const[repo,p]=k.split(':');assert.equal(sha(resolve(workspace,'exact-real-references',repo,p)),h);}
 const donor=[];
 donor.push(gate('qqbar-trig-compile-v76','../../..','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
  'audits/continuation/calcium/flint-qqbar-trig-v76.c','-L','../exact-real-references/flint',
  '-Wl,-rpath,'+workspace+'/exact-real-references/flint','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]));
 assert(Date.parse(donor[0].started)>=Date.parse(o.recorded));
 donor.push(gate('qqbar-trig-native-v76','.',o.binary,[]));
 donor.push(gate('qqbar-trig-linked-v76','.','ldd',[o.binary]));
 donor.push(gate('qqbar-trig-check-v76','.','node',['check-qqbar-trig-v76.mjs','results/qqbar-trig-native-v76.stdout']));
 donor.push(gate('qqbar-trig-memcheck-v76','.','valgrind',['--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',o.binary]));
 for(const g of donor.filter(g=>!g.tag.includes('memcheck')))assert.equal(read('results/'+g.tag+'.stderr'),'');
 const native=readFileSync('results/qqbar-trig-native-v76.stdout');assert(native.equals(readFileSync('results/qqbar-trig-memcheck-v76.stdout')));
 assert.equal(native.length,910636);const memory=read('results/qqbar-trig-memcheck-v76.stderr');
 assert(memory.includes('in use at exit: 0 bytes in 0 blocks'));
 assert(memory.includes('397,808 allocs, 397,808 frees, 41,257,680 bytes allocated'));
 assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
 const mathematical=checkQqbarTrig('results/qqbar-trig-native-v76.stdout');assert.deepEqual(mathematical,json('results/qqbar-trig-check-v76.stdout'));
 assert.equal(mathematical.status,'pass');assert.equal(mathematical.totalChecks,28586);assert.equal(mathematical.corruptionControlsRejected,10);
 const linked=read('results/qqbar-trig-linked-v76.stdout');
 for(const p of Object.keys(o.libraries))assert(linked.includes(p));
 const loader='/lib64/ld-linux-x86-64.so.2';assert(linked.includes(loader));
 const original=json('qqbar-trig-hyper-origin-v76.json'),fixed=json('qqbar-trig-hyper-origin-fixed-v76.json');
 for(const[p,h]of Object.entries(original.files))assert.equal(sha(p===cwd+'/src/main.rs'?'qqbar-trig-hyper-pretypefix-v76.rs':p),h,p);
 for(const[p,h]of Object.entries(fixed.files))assert.equal(sha(p),h,p);
 for(const origin of [original,fixed]){assert.deepEqual(origin.source,source);assert.deepEqual(origin.cargoEnv,cargoEnv);}
 const compact=s=>s.replace(/\s+/g,'');
 assert.equal(compact(read('qqbar-trig-hyper-initial-v76.rs')).replace('turns*2),);','turns*2));'),compact(read('qqbar-trig-hyper-pretypefix-v76.rs')));
 assert.equal(compact(read(cwd+'/src/main.rs')),compact(read('qqbar-trig-hyper-pretypefix-v76.rs')).replace('fraction(k,12)','fraction(k.into(),12)'));
 const failed=[gate('qqbar-trig-hyper-fmt-initial-v76',cwd,'cargo',['fmt','--','--check'],1),
  gate('qqbar-trig-hyper-debug-v76',cwd,'env',[...cargoEnv,'cargo','run','--offline','--locked','--quiet'],101),
  gate('qqbar-trig-hyper-gates-v76','.','node',['run-qqbar-trig-hyper-v76.mjs'],1)];
 failed.push(gate('qqbar-trig-evidence-v76','.','node',['check-qqbar-trig-evidence-v76.mjs'],1));
 assert(read('results/qqbar-trig-evidence-v76.stderr').includes('ERR_ASSERTION'));
 assert.equal(read('results/qqbar-trig-evidence-v76.stdout'),'');
 assert(read('results/qqbar-trig-hyper-fmt-initial-v76.stdout').includes('Diff in '));
 assert(read('results/qqbar-trig-hyper-debug-v76.stderr').includes('expected `i64`, found `i32`'));
 assert.equal(read('results/qqbar-trig-hyper-debug-v76.stdout'),'');
 const oldPass=[gate('qqbar-trig-hyper-fmt-v76',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']),
  gate('qqbar-trig-hyper-metadata-v76',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1'])];
 const live=[gate('qqbar-trig-hyper-fmt-fixed-v76',cwd,'env',[...cargoEnv,'cargo','fmt','--','--check']),
  gate('qqbar-trig-hyper-metadata-fixed-v76',cwd,'env',[...cargoEnv,'cargo','metadata','--offline','--locked','--format-version','1'])];
 for(const profile of ['debug','release'])live.push(gate('qqbar-trig-hyper-'+profile+'-fixed-v76',cwd,'env',[
  ...cargoEnv,'cargo','run','--offline','--locked','--quiet',...(profile==='release'?['--release']:[])]));
 live.push(gate('qqbar-trig-hyper-clippy-fixed-v76',cwd,'env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--all-features','--','-D','warnings']));
 live.push(gate('qqbar-trig-hyper-environment-fixed-v76','.','node',['point-qualified-environment.mjs']));
 for(const g of [...oldPass,...live])assert(!read('results/'+g.tag+'.stderr').includes('warning:'));
 for(const profile of ['debug','release'])assert.equal(read('results/qqbar-trig-hyper-'+profile+'-fixed-v76.stderr'),'');
 const driver=gate('qqbar-trig-hyper-gates-fixed-v76','.','node',['run-qqbar-trig-hyper-fixed-v76.mjs']);
 assert(Date.parse(fixed.recorded)>=Date.parse(driver.started));assert(Date.parse(fixed.recorded)<=Date.parse(live[0].started));
 for(let i=1;i<live.length;i++)assert(Date.parse(live[i].started)>=Date.parse(live[i-1].finished));
 assert(Date.parse(driver.finished)>=Date.parse(live.at(-1).finished));assert.equal(read('results/'+driver.tag+'.stderr'),'');
 const log=read('results/'+driver.tag+'.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(log.length,13);
 for(let i=0;i<live.length;i++)assert.deepEqual(log[2*i+1],live[i]);assert.deepEqual(log.at(-1),{checkpoint:76,status:'hyper-probe-gates-pass',source});
 const hyperCheck=gate('qqbar-trig-hyper-check-v76','.','node',['check-qqbar-trig-hyper-v76.mjs']);
 assert.equal(read('results/'+hyperCheck.tag+'.stderr'),'');const hyper=hyperEvidence();assert.deepEqual(hyper,json('results/'+hyperCheck.tag+'.stdout'));
 const graph=json('results/qqbar-trig-hyper-metadata-fixed-v76.stdout');assert.equal(graph.packages.length,10);assert.equal(graph.resolve.nodes.length,10);
 assert.equal(new Set(graph.packages.map(p=>p.id)).size,10);const hs=graph.packages.filter(p=>p.name.startsWith('hyper'));
 assert.equal(hs.length,1);assert.equal(hs[0].name,'hyperreal');assert.equal(hs[0].manifest_path,workspace+'/hyperreal/Cargo.toml');
 const oldGraph=json('results/qqbar-trig-hyper-metadata-v76.stdout');assert.deepEqual(graph,oldGraph);
 const environment=json('results/qqbar-trig-hyper-environment-fixed-v76.stdout');
 assert.deepEqual(environment.versions,retained.evidence.environment.after.versions);
 const binary={path:o.binary,sha256:sha(o.binary),bytes:statSync(o.binary).size};assert.equal(binary.bytes,13872);
 assert.deepEqual(retainedSources(),source);
 return {checkpoint:76,status:'source-and-independent-qualification-pass',source,readFiles:32,newDonorLines:1964,coverageBefore:o.before,coverageAtBinding,
  records,extraHyperReads,mathematical,hyper,successfulGates:[...donor,...oldPass,...live,driver,hyperCheck].map(g=>g.tag),
  preservedFailedGates:failed.map(g=>g.tag),memory:{errors:0,liveBytes:0,allocations:397808,requestedBytes:41257680,
   scope:'Entire bounded collector process including construction, caching and output; not per-operation costs or peak/RSS.'},
  libraries:{...o.libraries,[loader]:sha(loader)},binary,dependencyGraph:{packages:10,nodes:10,hyperrealOnly:true},environment,
  retainedContinuationTransfers:7,productionChanges:0,
  next:'Consider a certificate-preserving twelfth-turn radical relation candidate; do not replace SinPi/TanPi or infer performance benefits without qualification. Continue remaining qqbar/support/formal/historical references and full inventory reconciliation.',
  limits:'32 selected full donor files, not all qqbar. Only current FLINT executed; upstream tests were read, not newly run. Hyper probe is bounded default-feature debug/release capability, not whole-stack regression. Documented word-boundary exclusions respected. No benchmark, new retained transfer, cleanup, commit or push.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(trigEvidence()));
