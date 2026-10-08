import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,pilot=resolve(root,'..'),ws=resolve(root,'../../../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const phase=process.argv[2],manifest=root+'/'+phase+'-runs.json';
const snapshot=JSON.parse(readFileSync(pilot+'/snapshot.json','utf8'));
const validate=()=>{for(const[p,h]of snapshot.files)if(hash(pilot+'/snapshot/'+p)!==h)throw Error('source drift '+p);if(hash(pilot+'/baseline.rs')!==snapshot.baselineSha256)throw Error('baseline drift');};
validate();if(existsSync(manifest))throw Error('refusing to overwrite '+manifest);
const source=['main.rs','Cargo.toml','../main.rs','../baseline.rs'].map(p=>[p,hash(root+'/'+p)]);
const results=[];
function run(name,command,args,cap=240000){
 const file=phase+'-'+name+'.log';if(existsSync(root+'/'+file))throw Error('existing log '+file);
 const started=new Date().toISOString();
 const r=spawnSync(command,args,{encoding:'utf8',timeout:cap,killSignal:'SIGKILL',maxBuffer:16*1024*1024,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'}});
 writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const record={name,command,args,cap,started,finished:new Date().toISOString(),status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),source};
 if(phase!=='build')record.binarySha256=hash(root+'/binaries/'+(phase==='memory'?'memory':'release'));
 results.push(record);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify({name,status:r.status,error:r.error?.code??null,elapsedMs:Date.parse(record.finished)-Date.parse(started)}));
 if(r.error||r.status!==0)throw Error('failed '+name);return record;
}
if(phase==='build'){
 mkdirSync(root+'/binaries',{recursive:true});
 for(const mode of ['release','memory']){
  const r=run(mode,'cargo',['build','--offline','--release','--manifest-path',root+'/Cargo.toml','--target-dir',target,...(mode==='memory'?['--features','live-allocations']:[])]);
  const binary=root+'/binaries/'+mode;if(existsSync(binary))throw Error('refusing to replace binary');copyFileSync(target+'/release/ruffini_hyper_polynomial_lifetime',binary);
  r.binary=binary;r.binarySha256=hash(binary);r.lockSha256=hash(root+'/Cargo.lock');writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
}else if(phase==='memory'){
 for(const n of [16,32,64,128])for(const bits of [32,512,2048])for(const density of ['dense','sparse','fraction'])for(const lifetime of ['fresh','reused']) {
  const c=[n,n,bits,density,lifetime];run(c.join('-'),root+'/binaries/memory',['memory',...c.map(String)]);
 }
}else if(phase==='stable'){
 const cases=[[16,16,32,'dense'],[32,32,512,'dense'],[64,64,32,'dense'],[64,64,2048,'dense'],[128,128,2048,'dense'],[64,64,2048,'sparse'],[64,64,2048,'fraction'],[64,33,2048,'dense'],[64,1,2048,'dense'],[64,31,2048,'dense']];
 for(const seed of [137,149,163])for(const c of cases)for(const lifetime of ['fresh','reused']) {
  const a=[...c,lifetime,seed,'0.25','6'];run(a.slice(0,6).join('-'),'taskset',['-c','6',root+'/binaries/release','bench',...a.map(String)]);
 }
}else{throw Error('unknown phase');}
validate();for(const[p,h]of source)if(hash(root+'/'+p)!==h)throw Error('harness drift '+p);
writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
