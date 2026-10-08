import {readFileSync,writeFileSync,existsSync,copyFileSync} from 'node:fs';
import {spawn,spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,pilot=resolve(root,'..'),ws=resolve(root,'../../../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/extended-runs.json'))throw Error('refusing to overwrite extended evidence');
const failed=JSON.parse(readFileSync(root+'/runs.json','utf8'));if(failed.length!==1||failed[0].error!=='ETIMEDOUT')throw Error('unexpected original run');
const sources=failed[0].sources;for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness changed '+p);
if(hash(root+'/binaries/debug')!==failed[0].binarySha256)throw Error('debug binary changed');
const snapshot=JSON.parse(readFileSync(pilot+'/snapshot.json','utf8'));
const validate=()=>{for(const[p,h]of snapshot.files)if(hash(pilot+'/snapshot/'+p)!==h)throw Error('frozen source drift '+p);};validate();
const c=spawnSync('cargo',['build','--offline','--release','--manifest-path',root+'/Cargo.toml','--target-dir',target],{encoding:'utf8',timeout:120000,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'}});
writeFileSync(root+'/extended-release-build.log',(c.stdout??'')+(c.stderr??'')+'\n'+JSON.stringify({status:c.status,error:c.error?.code??null})+'\n');
if(c.error||c.status!==0)throw Error('release build failed');
const release=root+'/binaries/release';if(existsSync(release))throw Error('existing release binary');copyFileSync(target+'/release/ruffini_hyper_polynomial_coefficient_oracle',release);
const results=[];
for(const mode of ['release','debug']){
 const binary=root+'/binaries/'+mode,capMs=mode==='debug'?1200000:300000;
 const started=new Date().toISOString();const child=spawn(binary,[],{stdio:['ignore','pipe','pipe']});
 console.log(JSON.stringify({mode,pid:child.pid,started,capMs}));
 let stdout='',stderr='',expired=false,error=null;child.stdout.on('data',b=>{stdout+=b;process.stdout.write(b);});child.stderr.on('data',b=>{stderr+=b;process.stderr.write(b);});
 child.on('error',e=>{error=e.code??e.message;});const timer=setTimeout(()=>{expired=true;child.kill('SIGKILL');},capMs);
 const ended=await new Promise(resolve=>child.on('close',(status,signal)=>resolve({status,signal})));clearTimeout(timer);
 const file='extended-'+mode+'.log';writeFileSync(root+'/'+file,stdout);writeFileSync(root+'/'+file+'.stderr',stderr);
 const record={mode,binary,binarySha256:hash(binary),capMs,started,finished:new Date().toISOString(),...ended,error:expired?'ETIMEDOUT':error,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sources,lockSha256:hash(root+'/Cargo.lock')};results.push(record);writeFileSync(root+'/extended-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(record));
 if(record.error||record.status!==0)throw Error('extended coefficient failure '+mode);
}
validate();for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness changed '+p);
