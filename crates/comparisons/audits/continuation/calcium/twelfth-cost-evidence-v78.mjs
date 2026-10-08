import {readFileSync,statSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sources,sha,json,harnessFiles} from './twelfth-cost-sources-v78.mjs';
import {groups,validate,rows} from './twelfth-cost-protocol-v78.mjs';
import {nativeEvidence} from './check-twelfth-native-v78.mjs';
import {isolatedEvidence} from './check-twelfth-isolated-v78.mjs';
import {statisticsSelfTest} from './paired-statistics-v60.mjs';
export const buildGates=[...['baseline','candidate'].flatMap(v=>['lock','metadata','build','clippy','cpu-check','allocation-check'].map(k=>'twelfth-cost-'+v+'-'+k+'-v78')),
 'twelfth-cost-fmt-v78','twelfth-cost-baseline-rebuild-v78','twelfth-cost-candidate-rebuild-v78','twelfth-cost-build-final-v78'];
export const developmentGates=['twelfth-cost-initial-build-v78','twelfth-cost-initial-check-v78'];
export const failedGates=['twelfth-cost-build-resumed-v78'];
export const files=[...harnessFiles,'resume-twelfth-cost-v78.mjs','finalize-twelfth-cost-v78.mjs','twelfth-cost-resume-origin-v78.json',
 'twelfth-cost-origin-v78.json','twelfth-cost-binaries-v78.json','twelfth-cost-check-plan-v78.json','twelfth-cost-development-plan-v78.json',
 'twelfth-cost-environment-v78.mjs','run-twelfth-native-v78.mjs','twelfth-native-origin-v78.json','twelfth-native-calibration-v78.json',
 'twelfth-native-journal-v78.json','twelfth-native-runs-v78.json','twelfth-native-summary-v78.json','check-twelfth-native-v78.mjs',
 'probe-twelfth-native-v78.mjs','twelfth-isolated-origin-v78.json','twelfth-isolated-runs-v78.json','twelfth-isolated-summary-v78.json','check-twelfth-isolated-v78.mjs',
 'check-twelfth-controls-v78.mjs','twelfth-cost-controls-v78.json','twelfth-cost-evidence-v78.mjs','verify-twelfth-cost-v78.mjs',
 ...['baseline','candidate'].map(v=>'twelfth-cost-'+v+'-app-v78/Cargo.lock')];
function gate(tag){const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(g.signal,null);return g;}
export function costEvidence(){
 const source=sources(),native=nativeEvidence(),isolated=isolatedEvidence();
 assert.deepEqual(native,json('twelfth-native-summary-v78.json'));assert.deepEqual(isolated,json('twelfth-isolated-summary-v78.json'));
 const binary=json('twelfth-cost-binaries-v78.json'),origin=json('twelfth-cost-origin-v78.json');
 assert.deepEqual(source,binary.source);assert.deepEqual(source,origin.source);assert.equal(binary.originSha256,sha('twelfth-cost-origin-v78.json'));
 assert.equal(binary.finalizerSha256,sha('finalize-twelfth-cost-v78.mjs'));assert.equal(origin.checkPlanSha256,sha('twelfth-cost-check-plan-v78.json'));
 assert.deepEqual(json('twelfth-cost-check-plan-v78.json'),groups());
 const base=json('results/twelfth-cost-baseline-metadata-v78.stdout'),candidate=JSON.parse(readFileSync('results/twelfth-cost-candidate-metadata-v78.stdout','utf8')
  .replaceAll(resolve('twelfth-cost-candidate-app-v78'),resolve('twelfth-cost-baseline-app-v78')).replaceAll(resolve('twelfth-relation-candidate-v77/hyperreal'),resolve('../../../../hyperreal')));
 assert.deepEqual(base,candidate);assert.equal(base.packages.length,21);assert.equal(base.resolve.nodes.length,21);
 assert.equal(sha('twelfth-cost-baseline-app-v78/Cargo.lock'),sha('twelfth-cost-candidate-app-v78/Cargo.lock'));assert.equal(binary.lockSha256,sha('twelfth-cost-baseline-app-v78/Cargo.lock'));
 for(const b of binary.binaries){
  assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);
  const symbols=execFileSync('nm',['-C',b.path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  assert.deepEqual(symbols,b.allocatorSymbols);if(b.mode==='cpu')assert.equal(symbols.length,0);
  else for(const s of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(x=>x.endsWith('::'+s)));
  validate('results/twelfth-cost-'+b.variant+'-'+b.mode+'-check-v78.stdout',groups(),b.variant,'check');
 }
 const nativeRuns=json('twelfth-native-runs-v78.json').runs,isolatedRuns=json('twelfth-isolated-runs-v78.json').runs;
 const currentGates=[...buildGates,...nativeRuns.map(r=>r.tag),...['before','between','after'].map(n=>'twelfth-native-environment-'+n+'-v78'),
  'twelfth-native-campaign-v78',...isolatedRuns.map(r=>r.tag),'twelfth-isolated-campaign-v78'];
 assert.equal(currentGates.length,273);assert.equal(new Set(currentGates).size,273);for(const t of currentGates)gate(t);
 for(const t of developmentGates)gate(t);
 const failed=json('results/twelfth-cost-build-resumed-v78.json');assert.equal(failed.code,1);assert.equal(failed.signal,null);
 assert(readFileSync('results/twelfth-cost-build-resumed-v78.stderr','utf8').includes('21 !== 20'));
 assert(readFileSync('results/twelfth-cost-initial-build-v78.stderr','utf8').includes('unused import'));
 for(const v of ['baseline','candidate'])for(const kind of ['build','clippy','rebuild'])assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(readFileSync('results/twelfth-cost-'+v+'-'+kind+'-v78.stderr','utf8')));
 const outerBuild=rows('results/twelfth-cost-build-final-v78.stdout');assert.equal(outerBuild.length,5);assert.equal(outerBuild.at(-1).fullReportChecks,2304);
 assert.deepEqual(outerBuild.at(-1).binaries,binary.binaries);assert.equal(outerBuild.at(-1).metadataPackages,21);
 const campaign=rows('results/twelfth-native-campaign-v78.stdout'),isolation=rows('results/twelfth-isolated-campaign-v78.stdout');
 assert.equal(campaign.length,151);assert.equal(campaign.at(-1).status,'native-collection-finished');assert.equal(campaign.at(-1).runs,60);
 assert.equal(isolation.length,397);assert.equal(isolation.at(-1).status,'isolated-replication-collected');assert.equal(isolation.at(-1).processes,192);
 for(const [tag,stream]of [['twelfth-native-campaign-v78',campaign],['twelfth-isolated-campaign-v78',isolation]]){
  assert.equal(readFileSync('results/'+tag+'.stderr').length,0);let previous=Date.parse(gate(tag).started);
  for(const end of stream.filter(r=>r.finished)){const inner=gate(end.tag);assert.deepEqual(end,inner);assert(Date.parse(inner.started)>=previous);previous=Date.parse(inner.finished);}
  assert(Date.parse(gate(tag).finished)>=previous);
 }
 const controls=json('twelfth-cost-controls-v78.json');assert.equal(controls.valid,1);assert.equal(controls.rejected,12);
 for(const c of controls.records){assert.equal(sha(c.path),c.sha256);let error=null;try{validate(c.path,[controls.group],'baseline','cpu');}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(c.error,error);assert.equal(c.accepted,error===null);
 }
 const selfTest=statisticsSelfTest();assert.equal(selfTest.status,'pass');
 const examples=native.cpu.filter(r=>r.precision===-64&&r.lifecycle==='retained'&&[0,2,34,39,84,93].includes(r.case)).map(r=>({case:r.case,family:r.family,equalResult:r.equalResult,
  baselineNs:r.baselineNs,candidateNs:r.candidateNs,pairedMedianRatio:r.pairedMedianRatio,bootstrap:r.statistics.bootstrap.interval,order:r.statistics.orderStatistic.interval,
  allocation:native.allocations.find(a=>a.case===r.case&&a.precision===r.precision&&a.lifecycle===r.lifecycle&&a.iterations===1).fields,
  replication:isolated.comparisons.filter(a=>a.case===r.case&&a.precision===r.precision&&a.lifecycle===r.lifecycle).map(a=>({baselineNs:a.baselineNs,candidateNs:a.candidateNs,
   pairedMedianRatio:a.statistics.pairedMedianRatio,bootstrap:a.statistics.bootstrap.interval,order:a.statistics.orderStatistic.interval,allocations:a.allocations}))}));
 const allocationTallies={};for(const r of native.allocations){const k=(r.equalResult?'same-result:':'changed-answer:')+r.family+':n'+r.iterations,
  t=allocationTallies[k]??={groups:0,requests:{lower:0,equal:0,higher:0},bytes:{lower:0,equal:0,higher:0},peak:{lower:0,equal:0,higher:0}};t.groups++;
  for(const[a,b]of [['requests','requestsPerQuery'],['bytes','bytesPerQuery'],['peak','batchPeakAboveStart']])t[a][r.fields[b].pairedDelta<0?'lower':r.fields[b].pairedDelta>0?'higher':'equal']++;
 }
 return {checkpoint:78,status:'native-cost-qualified-current-version-not-selected',source,currentGates,developmentGates,failedGates,
  mathematicalPreflightRecords:2304,dependencyGraph:{packages:21,nodes:21,fullyEqualAfterPaths:true,locksEqual:true},
  campaign:native.campaign,categories:native.categories,allocationComparisons:1152,allocationTallies,examples,
  isolated:{cases:6,processes:192,cpuRows:864,allocationRows:576,comparisons:36,limits:isolated.limits},
  controls:{valid:1,rejected:12},statisticalSelfTest:selfTest.status,
  binaries:{files:4,bytes:native.binaryBytes,directory:origin.dir,noNewHyperrealCopy:true},rawObservationBytes:native.rawBytes+isolatedRuns.reduce((s,r)=>s+statSync('results/'+r.tag+'.stdout').size,0),
  liveFiles:957,candidateFiles:183,retainedContinuationTransfers:7,productionChanges:0,newDonorLines:0,
  decision:'Do not retain this version. Preserve its exact answers and numerical nodes while reducing proof-arithmetic and failed-proof overhead; requalify any revision against this complete corpus. The original candidate and all evidence remain intact.',
  failures:'Initial unused-import warnings fixed in the harness. One patch context mismatch applied no edit. An uncaptured build stopped on sandbox nm EPERM after copying one binary; approved resume reused it. The resume then failed an expected package count of 20 versus actual 21 including the benchmark root; mathematical gates passed. Correct finalization and byte-identical rebuilds pass. No source or raw record was silently replaced.',
  limits:native.limits+' The case-isolated replication now qualifies six selected cases separately; it is post-selection, not an unconditioned significance claim. No WASM/consumer/representative linked-size or new retention result.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(costEvidence()));
