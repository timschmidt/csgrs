import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawn} from 'node:child_process';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,scratch='/home/tim/Documents/GitHub/workspace/.audit-square-root.C42NO5BC',target=scratch+'/target',bin=scratch+'/v2-binaries';
const phase=process.argv[2],hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const snapshot=JSON.parse(readFileSync(root+'/../snapshot.json','utf8'));
const sources=['main.rs','Cargo.toml','../main.rs','../diagonal.rs','../baseline.rs','../shared.rs'].map(p=>[p,hash(root+'/'+p)]);
function validate(){for(const[p,h]of snapshot.files)if(hash(root+'/../snapshot/'+p)!==h)throw Error('frozen source drift '+p);for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness drift '+p);}
validate();const manifest=root+'/'+phase+'-runs.json';if(existsSync(manifest))throw Error('refusing to overwrite '+manifest);const results=[];
async function run(name,command,args,capMs){
 const file=phase+'-'+name+'.log';if(existsSync(root+'/'+file))throw Error('existing evidence '+file);
 const started=new Date().toISOString(),child=spawn(command,args,{stdio:['ignore','pipe','pipe'],env:{...process.env,TMPDIR:scratch,CCACHE_DISABLE:'1',CARGO_INCREMENTAL:'0'}});
 console.log(JSON.stringify({name,pid:child.pid,started,capMs}));let stdout='',stderr='',error=null,expired=false;
 child.stdout.on('data',b=>{stdout+=b;if(phase!=='bench')process.stdout.write(b);});child.stderr.on('data',b=>{stderr+=b;if(phase==='build')process.stderr.write(b);});child.on('error',e=>error=e.code??e.message);
 const timer=setTimeout(()=>{expired=true;child.kill('SIGKILL');},capMs),ended=await new Promise(resolve=>child.on('close',(status,signal)=>resolve({status,signal})));clearTimeout(timer);
 writeFileSync(root+'/'+file,stdout);writeFileSync(root+'/'+file+'.stderr',stderr);
 const r={name,command,args,capMs,started,finished:new Date().toISOString(),...ended,error:expired?'ETIMEDOUT':error,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sources,runnerSha256:hash(import.meta.filename),lockSha256:hash(root+'/Cargo.lock')};results.push(r);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(r));if(r.status!==0||r.error)throw Error('failed '+name);return r;
}
if(phase==='build'){
 mkdirSync(bin,{recursive:true});for(const mode of ['debug','release']){
  const r=await run(mode,'cargo',['build','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',target,...(mode==='release'?['--release']:[])],240000),binary=bin+'/'+mode;
  if(existsSync(binary))throw Error('existing binary');copyFileSync(target+'/'+mode+'/ruffini_hyper_square_root_oracle_v2',binary);r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
}else if(phase==='check'){
 for(const mode of ['release','debug']){const binary=bin+'/'+mode,r=await run(mode,binary,['check'],mode==='debug'?1200000:300000);r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');}
}else if(phase==='bench'){
 const families=[[1,32,'dense','square'],[2,32,'dense','square'],[4,32,'dense','square'],[8,32,'dense','square'],[16,256,'dense','square'],[32,32,'dense','square'],[16,256,'sparse','square'],[8,256,'fraction','nonsquare']];
 for(const seed of [137,149,163])for(const[degree,bits,kind,shape]of families)for(const lifetime of ['fresh','reused']){
  const c=[degree,bits,kind,lifetime,shape,seed],binary=bin+'/release',r=await run(c.join('-'),'taskset',['-c','6',binary,'bench',...c.map(String),'0.25','6'],240000);r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
}else throw Error('unknown phase');validate();
