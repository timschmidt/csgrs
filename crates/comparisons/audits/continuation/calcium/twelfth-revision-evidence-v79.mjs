import {readFileSync,statSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-revision-sources-v79.mjs';
import {regressionEvidence,gate} from './check-twelfth-revision-tests-v79.mjs';
import {finalNativeEvidence} from './check-twelfth-revision-native-final-v79.mjs';
import {cargoEnv,target} from './point-qualified-capture.mjs';
export const developmentGates=['twelfth-focused-v79','twelfth-clippy-initial-v79','twelfth-revision-native-check-v79'];
export const failedGates=['twelfth-focused-initial-v79','twelfth-bind-v79'];
export const unusableGates=['twelfth-prepare-v79','twelfth-bind-final-v79','twelfth-regressions-v79','twelfth-regression-evidence-v79'];
export const files=['prepare-twelfth-revision-v79.mjs','twelfth-revision-origin-v79.json','twelfth-reference-initial-v79.rs',
 'twelfth-revision-sources-initial-v79.mjs','twelfth-revision-sources-v79.mjs','twelfth-revision-binding-v79.json','run-twelfth-revision-tests-v79.mjs',
 'check-twelfth-revision-tests-v79.mjs','build-twelfth-revision-cost-v79.mjs','twelfth-revision-build-origin-v79.json','twelfth-revision-binaries-v79.json',
 'twelfth-revision-cost-protocol-v79.md','twelfth-revision-cost-protocol-v79.mjs','twelfth-revision-environment-v79.mjs','run-twelfth-revision-native-v79.mjs',
 'twelfth-revision-native-origin-v79.json','twelfth-revision-native-calibration-v79.json','twelfth-revision-native-journal-v79.json','twelfth-revision-native-runs-v79.json',
 'check-twelfth-revision-native-v79.mjs','check-twelfth-revision-native-final-v79.mjs','twelfth-revision-native-summary-v79.json','twelfth-revision-native-final-summary-v79.json',
 'twelfth-revision-evidence-v79.mjs','verify-twelfth-revision-v79.mjs','twelfth-capability-v79/Cargo.toml','twelfth-capability-v79/Cargo.lock','twelfth-capability-v79/src/main.rs',
 'twelfth-cost-revision-app-v79/Cargo.toml','twelfth-cost-revision-app-v79/Cargo.lock'];
export function revisionEvidence(){
 const source=sources(),regression=regressionEvidence(),native=finalNativeEvidence();
 assert.deepEqual(native,json('twelfth-revision-native-final-summary-v79.json'));
 gate('twelfth-regression-evidence-confirmed-v79','node',['check-twelfth-revision-tests-v79.mjs'],'.');
 assert.deepEqual(json('results/twelfth-regression-evidence-confirmed-v79.stdout'),regression);
 const finalCheck='twelfth-revision-native-final-check-v79';gate(finalCheck,'node',['check-twelfth-revision-native-final-v79.mjs','--record'],'.');
 const finalOutput=json('results/'+finalCheck+'.stdout');assert.equal(finalOutput.status,'final-cost-summary-verified');assert.deepEqual(finalOutput.categories,native.categories);
 const b=json('twelfth-revision-binaries-v79.json'),build=json('twelfth-revision-build-origin-v79.json');
 assert.deepEqual(b.source,source);assert.deepEqual(build.source,source);assert.equal(b.originSha256,sha('twelfth-revision-build-origin-v79.json'));
 assert.equal(build.scriptSha256,sha('build-twelfth-revision-cost-v79.mjs'));assert.equal(build.oldBinariesSha256,sha('twelfth-cost-binaries-v78.json'));
 const old=json('twelfth-cost-binaries-v78.json'),buildGates=[];
 for(const variant of ['baseline','prior','candidate']){
  const app=variant==='candidate'?'twelfth-cost-revision-app-v79':'twelfth-cost-'+(variant==='prior'?'candidate':'baseline')+'-app-v78';
  for(const[name,args]of [['metadata',['metadata','--locked','--offline','--format-version','1']],['build',['build','--locked','--offline','--release','--bins']]]){
   const tag='twelfth-cost-'+variant+'-'+name+'-v79';gate(tag,'env',[...cargoEnv,'cargo',...args],app);buildGates.push(tag);
   assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(readFileSync('results/'+tag+'.stderr','utf8')));
  }
  for(const mode of ['cpu','allocation'])buildGates.push('twelfth-preflight-'+variant+'-'+mode+'-v79');
 }
 gate('twelfth-cost-lock-v79','env',[...cargoEnv,'cargo','generate-lockfile','--offline'],'twelfth-cost-revision-app-v79');buildGates.push('twelfth-cost-lock-v79');
 gate('twelfth-cost-clippy-v79','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--','-D','warnings'],'twelfth-cost-revision-app-v79');buildGates.push('twelfth-cost-clippy-v79');
 assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(readFileSync('results/twelfth-cost-clippy-v79.stderr','utf8')));
 const metadata=json('results/twelfth-cost-baseline-metadata-v79.stdout');assert.equal(metadata.packages.length,21);assert.equal(metadata.resolve.nodes.length,21);
 for(const variant of ['prior','candidate']){
  const app=variant==='prior'?'twelfth-cost-candidate-app-v78':'twelfth-cost-revision-app-v79';
  const root=variant==='prior'?'twelfth-relation-candidate-v77/hyperreal':'twelfth-revision-candidate-v79/hyperreal';
  const normalized=readFileSync('results/twelfth-cost-'+variant+'-metadata-v79.stdout','utf8')
   .replaceAll(resolve(app),resolve('twelfth-cost-baseline-app-v78')).replaceAll(resolve(root),resolve('../../../../hyperreal'));
  assert.deepEqual(JSON.parse(normalized),metadata);assert.equal(sha(app+'/Cargo.lock'),sha('twelfth-cost-baseline-app-v78/Cargo.lock'));
 }
 assert.equal(b.lockSha256,sha('twelfth-cost-revision-app-v79/Cargo.lock'));
 for(const binary of b.binaries){
  assert.equal(sha(binary.path),binary.sha256);assert.equal(statSync(binary.path).size,binary.bytes);
  if(binary.variant!=='candidate'){
   const previous=old.binaries.find(x=>x.variant===(binary.variant==='prior'?'candidate':'baseline')&&x.mode===binary.mode);
   assert.deepEqual({...binary,variant:previous.variant},previous);
  }else assert.equal(binary.path,build.dir+'/candidate-'+binary.mode);
  const symbols=execFileSync('nm',['-C',binary.path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  assert.deepEqual(symbols,binary.allocatorSymbols);if(binary.mode==='cpu')assert.equal(symbols.length,0);
  else for(const name of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(s=>s.endsWith('::'+name)));
 }
 const checkOuter=(tag,status,children,length)=>{
  const g=gate(tag),rows=readFileSync('results/'+tag+'.stdout','utf8').trim().split('\n').map(JSON.parse);
  assert.equal(readFileSync('results/'+tag+'.stderr').length,0);assert.equal(rows.length,length);assert.equal(rows.at(-1).status,status);
  const ends=rows.filter(r=>r.tag&&r.finished);assert.deepEqual(ends.map(r=>r.tag).sort(),[...children].sort());
  let previous=Date.parse(g.started);for(const row of ends){assert.deepEqual(row,gate(row.tag));assert(Date.parse(row.started)>=previous);previous=Date.parse(row.finished);}
  assert(Date.parse(g.finished)>=previous);
 };
 checkOuter('twelfth-cost-build-v79','native-preflight-qualified',buildGates,29);buildGates.push('twelfth-cost-build-v79');
 const runs=json('twelfth-revision-native-runs-v79.json').runs,env=['before','between','after'].map(n=>'twelfth-revision-environment-'+n+'-v79');
 checkOuter('twelfth-revision-native-campaign-v79','native-collection-complete',[...runs.map(r=>r.tag),...env],223);
 const mem='twelfth-capability-memcheck-v79';gate(mem,'valgrind',['--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',target+'/release/twelfth-capability-v79'],'.');
 assert.equal(readFileSync('results/'+mem+'.stdout','utf8'),readFileSync('results/twelfth-capability-release-v79.stdout','utf8'));
 const report=readFileSync('results/'+mem+'.stderr','utf8');assert(report.includes('ERROR SUMMARY: 0 errors'));
 for(const kind of ['definitely','indirectly','possibly'])assert(new RegExp(kind+' lost: 0 bytes in 0 blocks').test(report));
 assert(report.includes('still reachable: 19,000 bytes in 148 blocks'));assert(report.includes('41,200 allocs, 41,052 frees, 4,092,804 bytes allocated'));
 const before='twelfth-current78-before-v79';gate(before,'node',['verify-twelfth-cost-v78.mjs'],'.');assert.equal(json('results/'+before+'.stdout').status,'verified-current-version-not-selected');
 const currentGates=[before,...regression.gates,'twelfth-regression-evidence-confirmed-v79',...buildGates,...runs.map(r=>r.tag),...env,
  'twelfth-revision-native-campaign-v79',finalCheck,mem];
 assert.equal(currentGates.length,new Set(currentGates).size);for(const t of currentGates)gate(t);
 for(const t of developmentGates)gate(t);
 for(const t of unusableGates){gate(t);assert.equal(readFileSync('results/'+t+'.stdout').length,0);assert.equal(readFileSync('results/'+t+'.stderr').length,0);}
 assert.equal(json('results/twelfth-focused-initial-v79.json').code,101);assert.equal(json('results/twelfth-bind-v79.json').code,1);
 assert(readFileSync('results/twelfth-focused-initial-v79.stderr','utf8').includes('no associated function or constant named `sqrt2`'));
 assert(readFileSync('results/twelfth-bind-v79.stderr','utf8').includes('frozen evaluator reference differs beyond formatting'));
 const examples=native.cpu.filter(r=>r.precision===-64&&r.lifecycle==='retained'&&[0,2,34,39,84,93].includes(r.case)).map(r=>({case:r.case,reference:r.reference,
  family:r.family,equalResult:r.equalResult,equalCertificate:r.equalCertificate,referenceNs:r.referenceNs,candidateNs:r.candidateNs,ratio:r.pairedMedianRatio,
  bootstrap:r.statistics.bootstrap.interval,order:r.statistics.orderStatistic.interval,
  allocation:native.allocations.find(a=>a.case===r.case&&a.reference===r.reference&&a.precision===r.precision&&a.lifecycle===r.lifecycle&&a.iterations===1).fields}));
 const tally={};for(const a of native.allocations){const k=a.reference+':'+a.family+':n'+a.iterations,
  t=tally[k]??={groups:0,requests:{lower:0,equal:0,higher:0},bytes:{lower:0,equal:0,higher:0},peak:{lower:0,equal:0,higher:0}};t.groups++;
  for(const[field,name]of [['requestsPerQuery','requests'],['bytesPerQuery','bytes'],['batchPeakAboveStart','peak']])t[name][a.fields[field].pairedDelta<0?'lower':a.fields[field].pairedDelta>0?'higher':'equal']++;
 }
 return {checkpoint:79,status:'revision-qualified-not-selected-for-retention',currentGates,developmentGates,failedGates,unusableGates,
  source,regression,preflightRecords:3456,benchmarkMetadata:{packages:21,nodes:21,locksEqual:true},campaign:native.campaign,categories:native.categories,examples,allocationTallies:tally,
  boundedGroups:native.cpu.filter(r=>r.reference==='baseline'&&r.iterationCapReached).length,
  shortGroups:native.cpu.filter(r=>r.reference==='baseline'&&r.medianBatchNs<100000).length,
  classificationCorrection:native.classificationCorrection,controls:native.reusedStreamControls,
  memcheck:{errors:0,lostBytes:0,allocations:41200,frees:41052,requestedBytes:4092804,stillReachableBytes:19000,stillReachableBlocks:148,outputMatches:true},
  newBinaryBytes:native.newBinaryBytes,rawBytes:native.rawBytes,liveFiles:957,candidateFiles:184,retainedContinuationTransfers:7,newDonorLines:0,productionChanges:0,
  decision:'Preserve the cheaper isolated revision and all evidence, but do not retain this version: hot same-decision tan/cot comparisons remain about four times live baseline with higher allocation demand; failed-proof CPU costs remain. Full exact answers and prior certificates are preserved.',
  failures:'One test-only missing sqrt2/sqrt3 helper compile failure, one overly strict source-format comparison, four status-zero empty Node captures, and an initial answer/certificate classification mistake are preserved. An uncaptured parse of empty evidence failed. Approved confirmation validates individual gates and source maps without repeating exclusive source preparation or binding. No mathematical misdecision or raw-sample removal.',
  limits:native.limits+' No representative binary-size, consumer or WASM runtime retention qualification. New production proof is unretained; previous seven transfers unchanged.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(revisionEvidence()));
