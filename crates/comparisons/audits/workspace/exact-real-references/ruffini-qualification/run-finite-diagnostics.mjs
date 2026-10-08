import {readFileSync,writeFileSync,readdirSync,mkdirSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM'),out=build+'/finite-diagnostics-v1';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex'),source=root+'/FiniteDiagnostics.java';
assert(!existsSync(out));mkdirSync(out);
const cp=out+':'+build+'/finite-boundaries-v1:'+build+'/polynomial:'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const dependencies=[];for(const name of ['native-build-manifest.json','matrix-build-manifest.json','polynomial-build-manifest.json','finite-boundaries-build-manifest.json'])dependencies.push([name,hash(root+'/'+name)]);
const records=[];
for(const mode of ['build','jit','interpreter']){
 const command=mode==='build'?'javac':'java';const args=mode==='build'?['--release','16','-encoding','UTF-8','-cp',cp,'-d',out,source]:['-ea','-Xmx256m','-Xss512k',...(mode==='interpreter'?['-Xint']:[]),'-cp',cp,'FiniteDiagnostics'];
 const started=new Date().toISOString(),r=spawnSync(command,args,{encoding:'utf8',timeout:60000,killSignal:'SIGKILL'});
 const file='finite-diagnostics-'+mode+'.log';assert(!existsSync(root+'/'+file));writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const record={mode,command,args,started,finished:new Date().toISOString(),status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr'),sourceSha256:hash(source),runnerSha256:hash(import.meta.filename),dependencies};records.push(record);writeFileSync(root+'/finite-diagnostics-runs.json',JSON.stringify(records,null,2)+'\n');console.log(JSON.stringify({mode,status:r.status,error:record.error}));
 assert.equal(r.error,undefined);assert.equal(r.status,0,r.stderr);
}
const classes=readdirSync(out,{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(out+'/'+p)]);
writeFileSync(root+'/finite-diagnostics-build-manifest.json',JSON.stringify({source,sourceSha256:hash(source),classes,dependencies},null,2)+'\n');
