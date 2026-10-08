import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawn} from 'node:child_process';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,pilot=root+'/..',scratch='/home/tim/Documents/GitHub/workspace/.audit-square-root.C42NO5BC',target=scratch+'/target',bin=scratch+'/public-binaries';
const phase=process.argv[2],hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const snapshot=JSON.parse(readFileSync(pilot+'/snapshot.json','utf8')),origin=JSON.parse(readFileSync(pilot+'/public-copy-origin.json','utf8'));
const sources=['main.rs','Cargo.toml','../diagonal.rs','../baseline.rs','../hypersolve-diagonal/Cargo.toml','../hypersolve-diagonal/src/curve_resultant.rs'].map(p=>[p,hash(root+'/'+p)]);
function validate(){
 for(const[p,h]of snapshot.files)if(hash(pilot+'/snapshot/'+p)!==h)throw Error('snapshot drift '+p);
 for(const[p,h]of origin.files){
  const actual=readFileSync(pilot+'/hypersolve-diagonal/'+p,'utf8');let expected=readFileSync(pilot+'/snapshot/hypersolve/'+p,'utf8');
  if(p==='Cargo.toml'){expected=expected.replace('name = "hypersolve"','name = "hypersolve-diagonal-pilot"');for(const name of ['hyperreal','hyperlattice','hyperlimit'])expected=expected.replace('path = "../'+name+'"','path = "../snapshot/'+name+'"');}
  else if(p==='src/curve_resultant.rs'){const before=readFileSync(pilot+'/baseline.rs','utf8'),after=readFileSync(pilot+'/diagonal.rs','utf8');if(!expected.includes(before))throw Error('missing baseline');expected=expected.replace(before,after);}
  else if(hash(pilot+'/hypersolve-diagonal/'+p)!==h)throw Error('unexpected candidate change '+p);
  if(actual!==expected)throw Error('unexpected candidate content '+p);
 }
 for(const[p,h]of sources)if(hash(root+'/'+p)!==h)throw Error('harness drift '+p);
}
validate();const manifest=root+'/'+phase+'-runs.json';if(existsSync(manifest))throw Error('refusing to overwrite '+manifest);const results=[];
async function run(name,command,args,capMs){
 const file=phase+'-'+name+'.log';if(existsSync(root+'/'+file))throw Error('existing evidence '+file);
 const started=new Date().toISOString(),child=spawn(command,args,{stdio:['ignore','pipe','pipe'],env:{...process.env,TMPDIR:scratch,CCACHE_DISABLE:'1',CARGO_INCREMENTAL:'0'}});
 console.log(JSON.stringify({name,pid:child.pid,started,capMs}));let stdout='',stderr='',error=null,expired=false;
 child.stdout.on('data',b=>{stdout+=b;if(phase==='check')process.stdout.write(b);});child.stderr.on('data',b=>{stderr+=b;if(phase==='build')process.stderr.write(b);});child.on('error',e=>error=e.code??e.message);
 const timer=setTimeout(()=>{expired=true;child.kill('SIGKILL');},capMs),ended=await new Promise(resolve=>child.on('close',(status,signal)=>resolve({status,signal})));clearTimeout(timer);
 writeFileSync(root+'/'+file,stdout);writeFileSync(root+'/'+file+'.stderr',stderr);
 const r={name,command,args,capMs,started,finished:new Date().toISOString(),...ended,error:expired?'ETIMEDOUT':error,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sources,runnerSha256:hash(import.meta.filename),lockSha256:hash(root+'/Cargo.lock')};results.push(r);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(r));if(r.status!==0||r.error)throw Error('failed '+name);return r;
}
if(phase==='build'){
 mkdirSync(bin,{recursive:true});for(const mode of ['debug','release'])for(const algorithm of ['baseline','diagonal']){
  const r=await run(mode+'-'+algorithm,'cargo',['build','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',target,'--no-default-features','--features',algorithm,...(mode==='release'?['--release']:[])],240000),binary=bin+'/'+mode+'-'+algorithm;
  if(existsSync(binary))throw Error('existing binary');copyFileSync(target+'/'+mode+'/ruffini_hyper_square_root_public',binary);r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
}else if(phase==='check'){
 for(const mode of ['release','debug'])for(const algorithm of ['baseline','diagonal']){const binary=bin+'/'+mode+'-'+algorithm,r=await run(mode+'-'+algorithm,binary,['check'],mode==='debug'?1200000:300000);r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');}
}else if(phase==='bench'){
 const families=[[1,8,'terminal'],[2,8,'terminal'],[4,8,'terminal'],[8,32,'terminal'],[1,8,'sampled'],[4,32,'sampled']];
 for(const seed of [17,42,149])for(const[degree,bits,layout]of families)for(const lifetime of ['fresh','reused'])for(let position=0;position<3;position++){
  const algorithm=['baseline','control','diagonal'][(position+seed)%3],c=[degree,bits,layout,lifetime,seed],binary=bin+'/release-'+(algorithm==='control'?'baseline':algorithm),r=await run(c.join('-')+'-'+algorithm,'taskset',['-c','6',binary,'bench',...c.map(String),'0.5','6'],240000);r.algorithm=algorithm;r.position=position;r.binary=binary;r.binarySha256=hash(binary);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
}else throw Error('unknown phase');validate();
