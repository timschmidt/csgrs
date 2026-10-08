import {readFileSync,writeFileSync,existsSync,copyFileSync,mkdirSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,pilot=resolve(root,'..'),ws=resolve(root,'../../../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/runs.json'))throw Error('refusing to overwrite coefficient evidence');
const snapshot=JSON.parse(readFileSync(pilot+'/snapshot.json','utf8'));
for(const[p,h]of snapshot.files)if(hash(pilot+'/snapshot/'+p)!==h)throw Error('source drift '+p);
const sources=['main.rs','Cargo.toml','../main.rs','../baseline.rs'].map(p=>[p,hash(root+'/'+p)]),results=[];
mkdirSync(root+'/binaries',{recursive:true});
for(const mode of ['debug','release']){
 const args=['build','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',target,...(mode==='release'?['--release']:[])];
 const c=spawnSync('cargo',args,{encoding:'utf8',timeout:120000,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'}});
 writeFileSync(root+'/'+mode+'-build.log',(c.stdout??'')+(c.stderr??'')+'\n'+JSON.stringify({status:c.status,error:c.error?.code??null})+'\n');
 if(c.error||c.status!==0)throw Error('build failed '+mode);
 const binary=root+'/binaries/'+mode;if(existsSync(binary))throw Error('existing binary');copyFileSync(target+'/'+mode+'/ruffini_hyper_polynomial_coefficient_oracle',binary);
 const started=new Date().toISOString();const r=spawnSync(binary,[],{encoding:'utf8',timeout:300000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 const file=mode+'.log';writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const record={mode,binary,binarySha256:hash(binary),started,finished:new Date().toISOString(),status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sources,lockSha256:hash(root+'/Cargo.lock'),buildSha256:hash(root+'/'+mode+'-build.log')};results.push(record);writeFileSync(root+'/runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(record));
 if(r.error||r.status!==0)throw Error('coefficient oracle failed '+mode);
}
for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness drift '+p);
for(const[p,h]of snapshot.files)if(hash(pilot+'/snapshot/'+p)!==h)throw Error('source drift '+p);
