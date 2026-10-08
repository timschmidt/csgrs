import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync,readdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,ws=resolve(root,'../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const sourcePaths=['Cargo.toml','Cargo.lock',...readdirSync(ws+'/hyperreal/src',{recursive:true}).filter(p=>p.endsWith('.rs')).map(p=>'src/'+p)].sort();
const snapshot=sourcePaths.map(p=>[p,hash(ws+'/hyperreal/'+p)]);
writeFileSync(root+'/hyper-source-snapshot.json',JSON.stringify(snapshot,null,2)+'\n');
const results=[];
for(const mode of ['debug','release']) {
 const buildArgs=['build','--offline','--manifest-path',root+'/Cargo.toml','--target-dir',target,...(mode==='release'?['--release']:[])];
 const built=spawnSync('cargo',buildArgs,{encoding:'utf8',timeout:120000,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'},maxBuffer:8*1024*1024});
 writeFileSync(root+'/hyper-'+mode+'-build.log',(built.stdout??'')+(built.stderr??''));
 if(built.error||built.status!==0) throw Error('Hyper control build failed: '+built.error?.message);
 const bin=target+'/'+mode+'/ruffini_hyper_boundary';
 const r=spawnSync(bin,[root+'/rational-corpus.tsv'],{encoding:'utf8',timeout:60000,maxBuffer:8*1024*1024});
 writeFileSync(root+'/hyper-'+mode+'.tsv',r.stdout??'');writeFileSync(root+'/hyper-'+mode+'.stderr',r.stderr??'');
 const result={mode,sourceSha256:hash(root+'/hyper_boundary.rs'),binary:bin,binarySha256:hash(bin),status:r.status,error:r.error?.message??null,signal:r.signal,outputSha256:hash(root+'/hyper-'+mode+'.tsv')};
 results.push(result);writeFileSync(root+'/hyper-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(r.error||r.status!==0)throw Error('Hyper controls failed');
}
for(const [p,h] of snapshot)if(hash(ws+'/hyperreal/'+p)!==h)throw Error('Hyper source changed during qualification: '+p);
