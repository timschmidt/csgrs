import {readFileSync,writeFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const manifest=JSON.parse(readFileSync(root+'/native-build-manifest.json','utf8'));
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
for(const [p,h] of manifest.sources) if(hash(p)!==h) throw Error('source changed: '+p);
for(const [p,h] of manifest.classes) if(hash(build+'/classes/'+p)!==h) throw Error('class changed: '+p);
const results=[];
for(const mode of ['jit','interpreter']) for(const test of ['corpus','signed-search','integer-boundaries','equality','format','cache','negative-reciprocal']) {
 const args=['-Xmx256m','-Xss512k','-ea',...(mode==='interpreter'?['-Xint']:[]),'-cp',build+'/classes:'+build+'/deps/*','RuffiniBoundary',test,...(test==='corpus'?[root+'/rational-corpus.tsv']:[])];
 const timeout=test==='negative-reciprocal'?2000:30000;
 const start=performance.now(),r=spawnSync('java',args,{encoding:'utf8',timeout,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 writeFileSync(root+'/'+mode+'-'+test+'.tsv',r.stdout??'');
 writeFileSync(root+'/'+mode+'-'+test+'.stderr',r.stderr??'');
 const result={mode,test,args,status:r.status,signal:r.signal,error:r.error?.code??null,elapsedMs:performance.now()-start,stdoutSha256:hash(root+'/'+mode+'-'+test+'.tsv')};
 results.push(result); console.log(JSON.stringify(result));
 writeFileSync(root+'/boundary-runs.json',JSON.stringify(results,null,2)+'\n');
 if(r.error&&r.error.code!=='ETIMEDOUT') throw r.error;
 if(test!=='negative-reciprocal'&&r.status!==0) throw Error('unexpected boundary process failure');
}
