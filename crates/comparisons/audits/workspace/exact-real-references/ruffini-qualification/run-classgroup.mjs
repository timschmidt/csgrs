import {readFileSync,writeFileSync,mkdirSync,readdirSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,repo=resolve(root,'../Ruffini'),build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
if(existsSync(root+'/classgroup-runs.json'))throw Error('refusing to overwrite classgroup evidence');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const output=build+'/classgroup';mkdirSync(output,{recursive:true});const cp=output+':'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const sources=[root+'/ClassGroupContracts.java',...['ClassGroup','QuadraticForm'].map(n=>repo+'/class-group/src/main/java/dk/jonaslindstrom/ruffini/quadraticform/'+n+'.java')];
const compile=(inputs,log)=>{const r=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',cp,'-d',output,...inputs],{encoding:'utf8',timeout:60000});writeFileSync(root+'/'+log,(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');return r;};
const main=compile(sources,'classgroup-build.log');if(main.error||main.status!==0)throw Error('classgroup main build failed');
const originalTest=repo+'/class-group/test/java/QuadraticFormTests.java';const test=compile([originalTest],'classgroup-original-test-build.log');
const manifest={sources:sources.concat(originalTest).map(p=>[p,hash(p)]),classes:readdirSync(output,{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(output+'/'+p)]),originalTestBuild:{status:test.status,error:test.error?.code??null}};
writeFileSync(root+'/classgroup-build-manifest.json',JSON.stringify(manifest,null,2)+'\n');
const runs=[];for(const mode of ['jit','interpreter']){const args=['-ea','-Xmx256m','-Xss1m',...(mode==='interpreter'?['-Xint']:[]),'-cp',cp,'ClassGroupContracts'];const r=spawnSync('java',args,{encoding:'utf8',timeout:15000,killSignal:'SIGKILL'});const file=mode+'-classgroup.log';writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');const result={mode,status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file)};runs.push(result);writeFileSync(root+'/classgroup-runs.json',JSON.stringify(runs,null,2)+'\n');console.log(JSON.stringify(result));if(r.error||r.status!==0)throw Error('classgroup harness failed');}
