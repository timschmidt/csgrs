import {readFileSync,writeFileSync,readdirSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,repo=resolve(root,'../Ruffini'),build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const sources=[repo+'/common/src/test/java/AlgorithmsTests.java',root+'/RuffiniShared.java',...['BigIntegers','BigIntegersModuloN','BigRationals'].map(n=>repo+'/integers/src/main/java/dk/jonaslindstrom/ruffini/integers/structures/'+n+'.java')];
const compiled=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',build+'/classes:'+build+'/deps/*','-d',build+'/tests',...sources],{encoding:'utf8',timeout:60000,maxBuffer:8*1024*1024});
writeFileSync(root+'/shared-build.log',(compiled.stdout??'')+(compiled.stderr??'')+'\n'+JSON.stringify({status:compiled.status,error:compiled.error?.message??null})+'\n');
if(compiled.error||compiled.status!==0)throw Error('supplement build failed');
const classes=readdirSync(build+'/tests',{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(build+'/tests/'+p)]);
writeFileSync(root+'/shared-build-manifest.json',JSON.stringify({sources:sources.map(p=>[p,hash(p)]),classes},null,2)+'\n');
const results=[];
for(const mode of ['jit','interpreter'])for(const test of ['junit','shared','dag','binary-gcd-zero']) {
 const main=test==='junit'?['org.junit.runner.JUnitCore','AlgorithmsTests']:['RuffiniShared',test];
 const args=['-Xmx256m','-Xss512k','-ea',...(mode==='interpreter'?['-Xint']:[]),'-cp',build+'/tests:'+build+'/classes:'+build+'/deps/*',...main];
 const r=spawnSync('java',args,{encoding:'utf8',timeout:test==='binary-gcd-zero'?2000:60000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 writeFileSync(root+'/'+mode+'-'+test+'.log',r.stdout??'');writeFileSync(root+'/'+mode+'-'+test+'.stderr',r.stderr??'');
 const result={mode,test,status:r.status,error:r.error?.code??null,signal:r.signal,sha256:hash(root+'/'+mode+'-'+test+'.log')};
 results.push(result);writeFileSync(root+'/shared-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(r.error&&r.error.code!=='ETIMEDOUT')throw r.error;
 if(test!=='junit'&&test!=='binary-gcd-zero'&&r.status!==0)throw Error('unexpected shared qualification failure');
}
