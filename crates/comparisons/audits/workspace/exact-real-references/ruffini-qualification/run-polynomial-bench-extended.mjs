import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
if(existsSync(root+'/polynomial-bench-extended-runs.json'))throw Error('refusing to overwrite benchmark evidence');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const cp=build+'/polynomial:'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const r=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',cp,'-d',build+'/polynomial',root+'/PolynomialBenchExtended.java'],{encoding:'utf8',timeout:60000});
writeFileSync(root+'/polynomial-bench-extended-build.log',(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');
if(r.error||r.status!==0)throw Error('extended polynomial benchmark compilation failed');
const results=[];
for(const n of [16,64,128])for(const bits of [32,2048])for(const density of ['dense','sparse']) {
 const args=['-c','6','java','-ea','-Xms256m','-Xmx512m','-Xss1m','-Djava.util.concurrent.ForkJoinPool.common.parallelism=1','-cp',cp,'PolynomialBenchExtended',String(n),String(bits),density];
 const started=new Date().toISOString();const ran=spawnSync('taskset',args,{encoding:'utf8',timeout:180000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 const file='polynomial-bench-extended-'+n+'-'+bits+'-'+density+'.tsv';writeFileSync(root+'/'+file,ran.stdout??'');writeFileSync(root+'/'+file+'.stderr',ran.stderr??'');
 const result={n,bits,density,args,started,finished:new Date().toISOString(),status:ran.status,error:ran.error?.code??null,signal:ran.signal,file,sha256:hash(root+'/'+file),sourceSha256:hash(root+'/PolynomialBenchExtended.java'),classSha256:hash(build+'/polynomial/PolynomialBenchExtended.class'),baseSourceSha256:hash(root+'/PolynomialBench.java'),baseClassSha256:hash(build+'/polynomial/PolynomialBench.class'),oracleSourceSha256:hash(root+'/PolynomialContracts.java')};
 results.push(result);writeFileSync(root+'/polynomial-bench-extended-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify({n,bits,density,status:ran.status,error:ran.error?.code??null}));
 if(ran.error||ran.status!==0)throw Error('extended polynomial benchmark failed');
}
