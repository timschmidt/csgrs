import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
if(existsSync(root+'/polynomial-progress-runs.json'))throw Error('refusing to overwrite progress evidence');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const cp=build+'/polynomial:'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const r=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',cp,'-d',build+'/polynomial',root+'/PolynomialProgress.java'],{encoding:'utf8',timeout:60000});
writeFileSync(root+'/polynomial-progress-build.log',(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');
if(r.error||r.status!==0)throw Error('progress compilation failed');
const manifest={sourceSha256:hash(root+'/PolynomialProgress.java'),classes:['PolynomialProgress.class','PolynomialProgress$Budget.class','PolynomialProgress$Univariate.class','PolynomialProgress$Multivariate.class'].map(p=>[p,hash(build+'/polynomial/'+p)])};
writeFileSync(root+'/polynomial-progress-build-manifest.json',JSON.stringify(manifest,null,2)+'\n');
const results=[];
for(const mode of ['jit','interpreter']) {
 const args=['-ea','-Xmx256m','-Xss1m',...(mode==='interpreter'?['-Xint']:[]),'-cp',cp,'PolynomialProgress'];
 const ran=spawnSync('java',args,{encoding:'utf8',timeout:15000,killSignal:'SIGKILL'});
 const file=mode+'-polynomial-progress.log';writeFileSync(root+'/'+file,ran.stdout??'');writeFileSync(root+'/'+file+'.stderr',ran.stderr??'');
 const row={mode,status:ran.status,error:ran.error?.code??null,signal:ran.signal,file,sha256:hash(root+'/'+file)};results.push(row);writeFileSync(root+'/polynomial-progress-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(row));
 if(ran.error||ran.status!==0)throw Error('progress prefix check failed');
}
