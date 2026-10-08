import {mkdtempSync,copyFileSync,constants,writeFileSync,statSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
import {groups,plan,validateRows,config} from './zero-factor-cost-protocol-v71.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
import {checkZeroFactor} from './check-zero-factor-v70.mjs';
const source=costSources();const checkPlan='zero-factor-cost-check-plan-v71.json';plan(checkPlan,groups());
const dir=mkdtempSync('/tmp/calcium-zero-cost.'),origin={checkpoint:71,recorded:new Date().toISOString(),dir,source,config,planSha256:sha(checkPlan)};
writeFileSync('zero-factor-cost-origin-v71.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
const binaries=[];
for(const variant of ['baseline','candidate']){
 const app='zero-factor-cost-'+variant+'-app-v71',tag='zero-factor-cost-'+variant;
 for(const [name,args]of [['lock',['generate-lockfile','--offline']],['metadata',['metadata','--offline','--locked','--format-version','1']],
  ['build',['build','--offline','--locked','--release','--bins']],['clippy',['clippy','--offline','--locked','--all-targets','--','-D','warnings']]])
  await captured(tag+'-'+name+'-v71',app,'env',[...cargoEnv,'cargo',...args]);
 for(const mode of ['cpu','allocation']){
  const path=dir+'/'+variant+'-'+mode;copyFileSync(target+'/release/zero-factor-'+mode+'-v71',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  const symbols=execFileSync('nm',['-C',path],{encoding:'utf8'}).split('\n').filter(s=>s.includes('instrumentation::allocation::'));
  if(mode==='cpu')assert.equal(symbols.length,0);else for(const name of ['CALLS','BYTES','LIVE','PEAK'])assert(symbols.some(s=>s.endsWith('::'+name)));
  binaries.push({variant,mode,path,bytes:statSync(path).size,sha256:sha(path),allocatorSymbols:symbols});
  await captured(tag+'-'+mode+'-check-v71','.','taskset',['-c',String(config.cpu),path,'check',resolve('zero-factor-cost-input-v71.json'),resolve(checkPlan)]);
  validateRows('results/'+tag+'-'+mode+'-check-v71.stdout',groups(),variant,'check');
 }
}
assert.equal(checkZeroFactor().status,'pass');assert.deepEqual(costSources(),source);
writeFileSync('zero-factor-cost-binaries-v71.json',JSON.stringify({checkpoint:71,binaries,originSha256:sha('zero-factor-cost-origin-v71.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:71,status:'native-preflight-qualified',cases:114,groups:456,fullReportChecks:1824,binaries,
 limits:'Preflight elapsed values are not benchmark evidence. Pilots, CPU/allocation campaign and consumer/WASM/size qualification remain open.'}));
