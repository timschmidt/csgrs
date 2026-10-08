import {readFileSync,writeFileSync,existsSync,copyFileSync} from 'node:fs';
import {spawn} from 'node:child_process';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,scratch='/home/tim/Documents/GitHub/workspace/.audit-square-root.C42NO5BC',hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const snapshot=JSON.parse(readFileSync(root+'/../snapshot.json','utf8')),sources=['main.rs','Cargo.toml','../main.rs','../diagonal.rs','../baseline.rs','../shared.rs'].map(p=>[p,hash(root+'/'+p)]);
const validate=()=>{for(const[p,h]of snapshot.files)if(hash(root+'/../snapshot/'+p)!==h)throw Error('snapshot drift '+p);for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness drift '+p);};validate();
if(existsSync(root+'/runs.json'))throw Error('refusing to overwrite evidence');const results=[];
const binary=scratch+'/square-root-memory';
for(const mode of ['build','check']){
 const args=mode==='build'?['build','--release','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',scratch+'/target']:[];
 const command=mode==='build'?'cargo':binary,started=new Date().toISOString(),child=spawn(command,args,{stdio:['ignore','pipe','pipe'],env:{...process.env,TMPDIR:scratch,CCACHE_DISABLE:'1',CARGO_INCREMENTAL:'0'}});
 console.log(JSON.stringify({mode,pid:child.pid,started}));let stdout='',stderr='',error=null;
 child.stdout.on('data',b=>stdout+=b);child.stderr.on('data',b=>{stderr+=b;if(mode==='build')process.stderr.write(b);});child.on('error',e=>error=e.code??e.message);
 const timer=setTimeout(()=>{error='ETIMEDOUT';child.kill('SIGKILL');},240000),ended=await new Promise(resolve=>child.on('close',(status,signal)=>resolve({status,signal})));clearTimeout(timer);
 const file=mode+'.log';writeFileSync(root+'/'+file,stdout);writeFileSync(root+'/'+file+'.stderr',stderr);
 const r={mode,command,args,started,finished:new Date().toISOString(),...ended,error,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sources,lockSha256:hash(root+'/Cargo.lock'),runnerSha256:hash(import.meta.filename)};results.push(r);writeFileSync(root+'/runs.json',JSON.stringify(results,null,2)+'\n');if(r.status!==0||r.error)throw Error('failed '+mode);
 if(mode==='build'){if(existsSync(binary))throw Error('existing binary');copyFileSync(scratch+'/target/release/ruffini_hyper_square_root_memory',binary);}r.binary=binary;r.binarySha256=hash(binary);writeFileSync(root+'/runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(r));
}
validate();
