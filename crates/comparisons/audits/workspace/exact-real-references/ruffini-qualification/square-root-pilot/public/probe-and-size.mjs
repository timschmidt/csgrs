import {readFileSync,writeFileSync,existsSync,statSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/probe-size-runs.json'))throw Error('refusing to overwrite evidence');
const builds=JSON.parse(readFileSync(root+'/build-runs.json','utf8')),results=[];
for(const algorithm of ['baseline','diagonal'])for(const mode of ['probe','size']){
 const build=builds.find(r=>r.name===(mode==='probe'?'debug':'release')+'-'+algorithm);assert.equal(hash(build.binary),build.binarySha256);
 const command=mode==='probe'?'gdb':'size',args=mode==='probe'?['--nx','--nh','-q','-batch','-x',root+'/'+algorithm+'-probe.gdb',build.binary]:['-A','-d',build.binary];
 const started=new Date().toISOString(),r=spawnSync(command,args,{encoding:'utf8',timeout:60000,env:{...process.env,TMPDIR:'/home/tim/Documents/GitHub/workspace/.audit-square-root.C42NO5BC'}}),file=mode+'-'+algorithm+'.log';
 writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const record={algorithm,mode,command,args,started,finished:new Date().toISOString(),status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),binary:build.binary,binarySha256:hash(build.binary),binaryFileBytes:statSync(build.binary).size,runnerSha256:hash(import.meta.filename)};
 if(mode==='probe')record.scriptSha256=hash(root+'/'+algorithm+'-probe.gdb');results.push(record);writeFileSync(root+'/probe-size-runs.json',JSON.stringify(results,null,2)+'\n');
 assert.equal(r.status,0,r.stderr);assert.equal(r.error,undefined);if(mode==='probe'){assert(r.stdout.includes('PASS\tpublic-probe'));assert(r.stdout.includes('AUDIT_SQUARE_ROOT_HIT'));}
 console.log(JSON.stringify(record));
}
