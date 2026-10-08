import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,ws=resolve(root,'../../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const phase=process.argv[2];
const snapshot=JSON.parse(readFileSync(root+'/snapshot.json','utf8'));
const validate=()=>{for(const[p,h]of snapshot.files)if(hash(root+'/snapshot/'+p)!==h)throw Error('frozen source drift '+p);if(hash(root+'/baseline.rs')!==snapshot.baselineSha256)throw Error('baseline drift');};
validate();
const manifest=root+'/'+phase+'-runs.json';
if(existsSync(manifest))throw Error('refusing to overwrite '+manifest);
const results=[];
const run=(name,command,args,cap=180000)=>{
 const file=phase+'-'+name+'.log';if(existsSync(root+'/'+file))throw Error('refusing to overwrite '+file);
 const started=new Date().toISOString();
 const r=spawnSync(command,args,{encoding:'utf8',timeout:cap,killSignal:'SIGKILL',maxBuffer:16*1024*1024,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'}});
 writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const result={name,command,args,cap,started,finished:new Date().toISOString(),status:r.status,signal:r.signal,error:r.error?.code??null,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),harnessSha256:hash(root+'/main.rs'),baselineSha256:hash(root+'/baseline.rs')};
 results.push(result);writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(r.error||r.status!==0)throw Error('failed '+name);return result;
};
if(phase==='build') {
 mkdirSync(root+'/binaries',{recursive:true});
 for(const mode of ['debug','release','memory']) {
  const args=['build','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',target,...(mode==='debug'?[]:['--release']),...(mode==='memory'?['--features','allocation-profile']:[])];
  const record=run(mode,'cargo',args,240000);
  const original=target+'/'+(mode==='debug'?'debug':'release')+'/ruffini_hyper_polynomial_pilot',binary=root+'/binaries/'+mode;
  if(existsSync(binary))throw Error('refusing to replace binary');copyFileSync(original,binary);
  record.binary=binary;record.binarySha256=hash(binary);record.lockSha256=hash(root+'/Cargo.lock');
  writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
 }
} else if(phase==='check') {
 for(const mode of ['debug','release']) {const binary=root+'/binaries/'+mode;const r=run(mode,binary,['check'],240000);r.binarySha256=hash(binary);}
} else if(phase==='explore') {
 const cases=[];
 for(const n of [16,32,64,128])for(const bits of [32,512,2048])for(const density of ['dense','sparse'])cases.push([n,n,bits,density]);
 for(const n of [32,64])for(const bits of [512,2048])cases.push([n,n,bits,'fraction']);
 for(const m of [1,8,31,33])cases.push([64,m,2048,'dense']);
 for(const c of cases){const r=run(c.join('-'),'taskset',['-c','6',root+'/binaries/release','bench',...c.map(String),'0.1','3'],180000);r.binarySha256=hash(root+'/binaries/release');}
} else if(phase==='memory') {
 for(const n of [16,32,64,128])for(const bits of [32,512,2048])for(const density of ['dense','sparse','fraction']) {
  const c=[n,n,bits,density];const r=run(c.join('-'),root+'/binaries/memory',['memory',...c.map(String)]);r.binarySha256=hash(root+'/binaries/memory');
 }
} else {throw Error('unknown phase');}
validate();writeFileSync(manifest,JSON.stringify(results,null,2)+'\n');
