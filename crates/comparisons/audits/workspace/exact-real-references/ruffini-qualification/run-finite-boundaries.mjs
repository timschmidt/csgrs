import {readFileSync,writeFileSync,readdirSync,mkdirSync,existsSync} from 'node:fs';
import {spawn,spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM'),out=build+'/finite-boundaries-v1';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const dependencies=[];
for(const[file,folder]of [['native-build-manifest.json','classes'],['matrix-build-manifest.json','matrix'],['polynomial-build-manifest.json','polynomial']]){
 const m=JSON.parse(readFileSync(root+'/'+file,'utf8'));dependencies.push([file,hash(root+'/'+file)]);
 for(const[p,h]of m.sources)assert.equal(hash(p),h,p);
 for(const[p,h]of m.classes)assert.equal(hash(build+'/'+folder+'/'+p),h,p);
 if(m.dependencies)for(const[name,h]of Object.entries(m.dependencies))assert.equal(hash(build+'/deps/'+name),h,name);
}
const cp=out+':'+build+'/polynomial:'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const source=root+'/FiniteBoundaries.java';
const command=process.argv[2],attempt=process.argv[3];assert(/^[a-z0-9-]+$/.test(attempt??''));
const fileRoot='finite-boundaries-'+attempt;
const manifest=root+'/'+fileRoot+'-runs.json';assert(!existsSync(manifest),'preserve prior evidence');
const records=[];
function record(name,command,args,r,started,capMs){
 const file=fileRoot+'-'+name+'.log';writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const data={name,command,args,started,finished:new Date().toISOString(),capMs,status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sourceSha256:hash(source),runnerSha256:hash(import.meta.filename),dependencies};
 records.push(data);writeFileSync(manifest,JSON.stringify(records,null,2)+'\n');console.log(JSON.stringify({name,status:r.status,error:data.error,signal:r.signal,finished:data.finished}));return data;
}
if(command==='build'){
 assert(!existsSync(out),'use a new output directory instead of overwriting classes');mkdirSync(out,{recursive:true});
 const args=['--release','16','-encoding','UTF-8','-cp',cp,'-d',out,source],started=new Date().toISOString();
 const r=spawnSync('javac',args,{encoding:'utf8',timeout:60000});record('build','javac',args,r,started,60000);
 assert.equal(r.error,undefined);assert.equal(r.status,0,r.stderr);
 const classes=readdirSync(out,{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(out+'/'+p)]);
 writeFileSync(root+'/finite-boundaries-build-manifest.json',JSON.stringify({source,sourceSha256:hash(source),classes,dependencies,release:16,buildRecord:manifest},null,2)+'\n');
}else if(command==='run'){
 const compiled=JSON.parse(readFileSync(root+'/finite-boundaries-build-manifest.json','utf8'));assert.equal(hash(source),compiled.sourceSha256);
 for(const[p,h]of compiled.classes)assert.equal(hash(out+'/'+p),h,p);
 const tests=[['extensions'],['binary'],['big-small'],...[3,5,7,13,17,41,97].map(p=>['odd',String(p)]),...[3,5,7].map(p=>['berlekamp',String(p)]),...[30,31,32,35].map(s=>['high',String(s)])];
 for(const mode of ['jit','interpreter'])for(const test of tests){
  const args=['-ea','-Xmx256m','-Xss512k','-Djava.util.concurrent.ForkJoinPool.common.parallelism=1',...(mode==='interpreter'?['-Xint']:[]),'-cp',cp,'FiniteBoundaries',...test];
  const capMs=test[0]==='odd'?12000:60000,started=new Date().toISOString();let stdout='',stderr='';
  const child=spawn('java',args,{stdio:['ignore','pipe','pipe']});console.log(JSON.stringify({mode,test,pid:child.pid,started,capMs}));
  let error=null,timedOut=false;const timer=setTimeout(()=>{timedOut=true;child.kill('SIGKILL');},capMs);
  child.stdout.on('data',chunk=>stdout+=chunk);child.stderr.on('data',chunk=>stderr+=chunk);child.on('error',e=>error=e);
  const r=await new Promise(resolve=>child.on('close',(status,signal)=>resolve({status,signal,stdout,stderr,error:error??(timedOut?{code:'ETIMEDOUT'}:null)})));clearTimeout(timer);
  record(mode+'-'+test.join('-'),'java',args,r,started,capMs);
  assert(!error,'process launch failed');assert(stdout.length+stderr.length<16*1024*1024,'unexpected output volume');
  if(!timedOut)assert.equal(r.status,0,stderr);
 }
 assert.equal(hash(source),compiled.sourceSha256);for(const[p,h]of compiled.classes)assert.equal(hash(out+'/'+p),h,p);
}else throw Error('expected build or run');
