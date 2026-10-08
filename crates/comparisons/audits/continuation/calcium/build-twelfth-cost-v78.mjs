import {mkdtempSync,copyFileSync,constants,writeFileSync,statSync,readFileSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {config,groups,plan,validate} from './twelfth-cost-protocol-v78.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=sources(),checkPlan='twelfth-cost-check-plan-v78.json';plan(checkPlan,groups());
const dir=mkdtempSync('/tmp/calcium-twelfth-cost-v78.'),origin={checkpoint:78,recorded:new Date().toISOString(),dir,source,config,checkPlanSha256:sha(checkPlan)};
writeFileSync('twelfth-cost-origin-v78.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const binaries=[];
for(const variant of ['baseline','candidate']){
 const app='twelfth-cost-'+variant+'-app-v78',tag='twelfth-cost-'+variant;
 for(const [name,args]of [['lock',['generate-lockfile','--offline']],['metadata',['metadata','--offline','--locked','--format-version','1']],
  ['build',['build','--offline','--locked','--release','--bins']],['clippy',['clippy','--offline','--locked','--all-targets','--','-D','warnings']]])
  await captured(tag+'-'+name+'-v78',app,'env',[...cargoEnv,'cargo',...args]);
 for(const mode of ['cpu','allocation']){
  const path=dir+'/'+variant+'-'+mode;copyFileSync(target+'/release/twelfth-'+mode+'-v78',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  const symbols=execFileSync('nm',['-C',path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  if(mode==='cpu')assert.equal(symbols.length,0);else for(const name of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(s=>s.endsWith('::'+name)));
  binaries.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path),allocatorSymbols:symbols});
  await captured(tag+'-'+mode+'-check-v78','.','taskset',['-c',String(config.cpu),path,'check',resolve('twelfth-cost-input-v78.json'),resolve(checkPlan)]);
  validate('results/'+tag+'-'+mode+'-check-v78.stdout',groups(),variant,'check');
 }
 assert.deepEqual(sources(),source);
}
await captured('twelfth-cost-fmt-v78','.','rustfmt',['--edition','2024','--check','twelfth-cost-v78.rs','twelfth-cost-cpu-v78.rs','twelfth-cost-allocation-v78.rs','twelfth-counting-allocator-v78.rs']);
const base=json('results/twelfth-cost-baseline-metadata-v78.stdout'),candidate=JSON.parse(readFileSync('results/twelfth-cost-candidate-metadata-v78.stdout','utf8')
 .replaceAll(resolve('twelfth-cost-candidate-app-v78'),resolve('twelfth-cost-baseline-app-v78')).replaceAll(resolve('twelfth-relation-candidate-v77/hyperreal'),resolve('../../../../hyperreal')));
assert.deepEqual(candidate,base);assert.equal(base.packages.length,20);assert.equal(base.resolve.nodes.length,20);
assert.equal(sha('twelfth-cost-baseline-app-v78/Cargo.lock'),sha('twelfth-cost-candidate-app-v78/Cargo.lock'));
const binding={checkpoint:78,recorded:new Date().toISOString(),binaries,originSha256:sha('twelfth-cost-origin-v78.json'),source,
 lockSha256:sha('twelfth-cost-baseline-app-v78/Cargo.lock'),metadataPackages:20,metadataNodes:20};
writeFileSync('twelfth-cost-binaries-v78.json',JSON.stringify(binding,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:78,status:'native-preflight-qualified',cases:96,groups:576,fullReportChecks:2304,binaries,
 limits:'Preflight elapsed values are not benchmark evidence; live/candidate sources unchanged, no retention.'}));
