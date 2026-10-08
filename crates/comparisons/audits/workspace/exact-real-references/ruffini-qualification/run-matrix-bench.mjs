import {readFileSync,writeFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const cp=build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const r=spawnSync('javac',['--release','16','-cp',cp,'-d',build+'/matrix',root+'/MatrixBench.java'],{encoding:'utf8',timeout:60000});
writeFileSync(root+'/matrix-bench-build.log',(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');
if(r.error||r.status!==0)throw Error('benchmark compilation failed');
const results=[];
for(const n of [4,8,16])for(const bits of [32,256]) {
 const args=['-c','6','java','-Xms256m','-Xmx512m','-Xss1m','-ea','-Djava.util.concurrent.ForkJoinPool.common.parallelism=1','-cp',cp,'MatrixBench',String(n),String(bits)];
 const ran=spawnSync('taskset',args,{encoding:'utf8',timeout:120000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 const file='matrix-bench-'+n+'-'+bits+'.tsv';writeFileSync(root+'/'+file,ran.stdout??'');writeFileSync(root+'/'+file+'.stderr',ran.stderr??'');
 const result={n,bits,args,status:ran.status,error:ran.error?.code??null,signal:ran.signal,file,sha256:hash(root+'/'+file),sourceSha256:hash(root+'/MatrixBench.java'),classSha256:hash(build+'/matrix/MatrixBench.class')};
 results.push(result);writeFileSync(root+'/matrix-bench-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify({n,bits,status:ran.status,observations:(ran.stdout??'').split('\n').filter(s=>s.startsWith('BENCH\t')).length}));
 if(ran.error||ran.status!==0)throw Error('matrix benchmark failed');
}
