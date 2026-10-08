import {readFileSync,writeFileSync,readdirSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,repo=resolve(root,'../Ruffini'),build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const integerRoot=repo+'/integers/src/main/java';
const sources=[root+'/RuffiniMatrix.java',repo+'/integers/src/test/java/TestIntegers.java',...readdirSync(integerRoot,{recursive:true}).filter(p=>p.endsWith('.java')&&!p.endsWith('/IntegerPolynomial.java')).map(p=>integerRoot+'/'+p)];
const r=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',build+'/classes:'+build+'/deps/*','-d',build+'/matrix',...sources],{encoding:'utf8',timeout:60000,maxBuffer:8*1024*1024});
writeFileSync(root+'/matrix-build.log',(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');
if(r.error||r.status!==0)throw Error('matrix build failed');
const classes=readdirSync(build+'/matrix',{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(build+'/matrix/'+p)]);
writeFileSync(root+'/matrix-build-manifest.json',JSON.stringify({sources:sources.map(p=>[p,hash(p)]),classes},null,2)+'\n');
const results=[];
for(const mode of ['jit','interpreter'])for(const test of ['arithmetic','boundaries','transforms','junit-integers']) {
 const main=test==='junit-integers'?['org.junit.runner.JUnitCore','TestIntegers']:['RuffiniMatrix',test];
 const args=['-Xmx512m','-Xss1m','-ea','-Djava.util.concurrent.ForkJoinPool.common.parallelism=1',...(mode==='interpreter'?['-Xint']:[]),'-cp',build+'/matrix:'+build+'/classes:'+build+'/deps/*',...main];
 const ran=spawnSync('java',args,{encoding:'utf8',timeout:120000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 writeFileSync(root+'/'+mode+'-matrix-'+test+'.log',ran.stdout??'');writeFileSync(root+'/'+mode+'-matrix-'+test+'.stderr',ran.stderr??'');
 const result={mode,test,status:ran.status,error:ran.error?.code??null,signal:ran.signal,sha256:hash(root+'/'+mode+'-matrix-'+test+'.log')};
 results.push(result);writeFileSync(root+'/matrix-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(ran.error||ran.status!==0)throw Error('unexpected matrix qualification process failure');
}
