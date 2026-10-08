import {writeFileSync,statSync,readFileSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {groups,validate} from './twelfth-cost-protocol-v78.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=sources(),origin=json('twelfth-cost-origin-v78.json');assert.deepEqual(source,origin.source);
const failed=json('results/twelfth-cost-build-resumed-v78.json');assert.equal(failed.code,1);assert.equal(failed.signal,null);
assert(readFileSync('results/twelfth-cost-build-resumed-v78.stderr','utf8').includes('21 !== 20'));
const binaries=[];
for(const variant of ['baseline','candidate']){
 const app='twelfth-cost-'+variant+'-app-v78';
 await captured('twelfth-cost-'+variant+'-rebuild-v78',app,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--bins']);
 for(const mode of ['cpu','allocation']){
  const path=origin.dir+'/'+variant+'-'+mode,product=target+'/release/twelfth-'+mode+'-v78';assert.equal(sha(path),sha(product));
  const symbols=execFileSync('nm',['-C',path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  if(mode==='cpu')assert.equal(symbols.length,0);else for(const name of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(s=>s.endsWith('::'+name)));
  binaries.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path),allocatorSymbols:symbols});
  const tag='twelfth-cost-'+variant+'-'+mode+'-check-v78';assert.equal(json('results/'+tag+'.json').code,0);
  validate('results/'+tag+'.stdout',groups(),variant,'check');
 }
}
const base=json('results/twelfth-cost-baseline-metadata-v78.stdout'),candidate=JSON.parse(readFileSync('results/twelfth-cost-candidate-metadata-v78.stdout','utf8')
 .replaceAll(resolve('twelfth-cost-candidate-app-v78'),resolve('twelfth-cost-baseline-app-v78')).replaceAll(resolve('twelfth-relation-candidate-v77/hyperreal'),resolve('../../../../hyperreal')));
assert.deepEqual(candidate,base);assert.equal(base.packages.length,21);assert.equal(base.resolve.nodes.length,21);
assert.equal(base.packages.filter(p=>p.name==='twelfth-cost-v78').length,1);assert.equal(base.packages.filter(p=>p.name==='hyperreal').length,1);
assert.equal(sha('twelfth-cost-baseline-app-v78/Cargo.lock'),sha('twelfth-cost-candidate-app-v78/Cargo.lock'));
assert.deepEqual(sources(),source);
writeFileSync('twelfth-cost-binaries-v78.json',JSON.stringify({checkpoint:78,recorded:new Date().toISOString(),binaries,source,
 originSha256:sha('twelfth-cost-origin-v78.json'),finalizerSha256:sha('finalize-twelfth-cost-v78.mjs'),
 lockSha256:sha('twelfth-cost-baseline-app-v78/Cargo.lock'),metadataPackages:21,metadataNodes:21,
 note:'Correct full graph count includes the root benchmark package. Both rebuilt products match the already captured immutable binaries; no mathematical/source change or new snapshot.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:78,status:'native-preflight-qualified',cases:96,groups:576,fullReportChecks:2304,metadataPackages:21,binaries,
 limits:'Preflight elapsed values are not benchmark evidence; live/candidate sources unchanged, no retention.'}));
