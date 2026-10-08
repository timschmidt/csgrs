import {writeFileSync,readFileSync,mkdtempSync,copyFileSync,constants,statSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-revision-sources-v79.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
import {config,groups,validate} from './twelfth-cost-protocol-v78.mjs';
const source=sources(),old=json('twelfth-cost-binaries-v78.json');
const dir=mkdtempSync('/tmp/calcium-twelfth-revision-v79.');
writeFileSync('twelfth-revision-build-origin-v79.json',JSON.stringify({checkpoint:79,recorded:new Date().toISOString(),dir,source,
 oldBinariesSha256:sha('twelfth-cost-binaries-v78.json'),scriptSha256:sha('build-twelfth-revision-cost-v79.mjs')},null,2)+'\n',{flag:'wx'});
const binaries=[];
for(const variant of ['baseline','prior','candidate']){
 const app=variant==='candidate'?'twelfth-cost-revision-app-v79':'twelfth-cost-'+(variant==='prior'?'candidate':'baseline')+'-app-v78';
 if(variant==='candidate')await captured('twelfth-cost-lock-v79',app,'env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
 for(const [name,args]of [['metadata',['metadata','--locked','--offline','--format-version','1']],['build',['build','--locked','--offline','--release','--bins']]])
  await captured('twelfth-cost-'+variant+'-'+name+'-v79',app,'env',[...cargoEnv,'cargo',...args]);
 const original=variant==='baseline'?'baseline':'candidate';
 for(const mode of ['cpu','allocation']){
  const previous=old.binaries.find(b=>b.variant===original&&b.mode===mode),product=target+'/release/twelfth-'+mode+'-v78';
  assert.equal(sha(previous.path),previous.sha256);
  let path=previous.path;
  if(variant==='candidate'){
   path=dir+'/candidate-'+mode;copyFileSync(product,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  }else assert.equal(sha(product),previous.sha256,'rebuilt frozen '+variant+' '+mode);
  const symbols=execFileSync('nm',['-C',path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  if(mode==='cpu')assert.equal(symbols.length,0);else for(const name of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(s=>s.endsWith('::'+name)));
  binaries.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path),allocatorSymbols:symbols});
  const tag='twelfth-preflight-'+variant+'-'+mode+'-v79';
  await captured(tag,'.','taskset',['-c',String(config.cpu),path,'check',resolve('twelfth-cost-input-v78.json'),resolve('twelfth-cost-check-plan-v78.json')]);
  const current=validate('results/'+tag+'.stdout',groups(),original,'check');
  const reference=validate('results/twelfth-cost-'+original+'-'+mode+'-check-v78.stdout',groups(),original,'check');
  assert.deepEqual(current.map(r=>[r.case,r.precision,r.lifecycle,r.outcome,r.certificate]),reference.map(r=>[r.case,r.precision,r.lifecycle,r.outcome,r.certificate]));
 }
 sources();
}
await captured('twelfth-cost-clippy-v79','twelfth-cost-revision-app-v79','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--all-targets','--','-D','warnings']);
const base=json('results/twelfth-cost-baseline-metadata-v79.stdout');assert.equal(base.packages.length,21);assert.equal(base.resolve.nodes.length,21);
for(const variant of ['prior','candidate']){
 const app=variant==='prior'?'twelfth-cost-candidate-app-v78':'twelfth-cost-revision-app-v79';
 const root=variant==='prior'?'twelfth-relation-candidate-v77/hyperreal':'twelfth-revision-candidate-v79/hyperreal';
 const normalized=readFileSync('results/twelfth-cost-'+variant+'-metadata-v79.stdout','utf8')
  .replaceAll(resolve(app),resolve('twelfth-cost-baseline-app-v78')).replaceAll(resolve(root),resolve('../../../../hyperreal'));
 assert.deepEqual(JSON.parse(normalized),base);assert.equal(sha(app+'/Cargo.lock'),sha('twelfth-cost-baseline-app-v78/Cargo.lock'));
}
const binding={checkpoint:79,recorded:new Date().toISOString(),source,binaries,metadataPackages:21,metadataNodes:21,
 lockSha256:sha('twelfth-cost-revision-app-v79/Cargo.lock'),originSha256:sha('twelfth-revision-build-origin-v79.json'),
 preflightRecords:3456,limits:'Unchanged 78 harness and inputs; baseline/prior rebuilt byte-identically to frozen products. Revised binary snapshots only. Preflight is not timing evidence.'};
writeFileSync('twelfth-revision-binaries-v79.json',JSON.stringify(binding,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:79,status:'native-preflight-qualified',variants:3,binaries:6,groups:576,preflightRecords:3456,
 newSnapshotBytes:binaries.filter(b=>b.variant==='candidate').reduce((s,b)=>s+b.bytes,0),retained:false}));
