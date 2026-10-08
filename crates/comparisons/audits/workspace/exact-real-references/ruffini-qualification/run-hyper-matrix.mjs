import {readFileSync,writeFileSync,readdirSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,ws=resolve(root,'../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const paths=[];
for(const crate of ['hyperreal','hyperlattice','hyperlimit','hypersolve'])for(const p of ['Cargo.toml',...readdirSync(ws+'/'+crate+'/src',{recursive:true}).filter(p=>p.endsWith('.rs')).map(p=>'src/'+p)])paths.push(crate+'/'+p);
const snapshot=paths.sort().map(p=>[p,hash(ws+'/'+p)]);
writeFileSync(root+'/hyper-matrix-source-snapshot.json',JSON.stringify(snapshot,null,2)+'\n');
const results=[];
for(const mode of ['debug','release']) {
 const oldLog=root+'/hyper-matrix-'+mode+'-build.log';
 if(existsSync(oldLog)&&readFileSync(oldLog,'utf8').includes('error['))writeFileSync(root+'/hyper-matrix-'+mode+'-build-failure-'+Date.now()+'.log',readFileSync(oldLog));
 const args=['build','--offline','--manifest-path',root+'/matrix-hyper/Cargo.toml','--target-dir',target,...(mode==='release'?['--release']:[])];
 const built=spawnSync('cargo',args,{encoding:'utf8',timeout:120000,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'},maxBuffer:8*1024*1024});
 writeFileSync(root+'/hyper-matrix-'+mode+'-build.log',(built.stdout??'')+(built.stderr??'')+'\n'+JSON.stringify({status:built.status,error:built.error?.message??null})+'\n');
 if(built.error||built.status!==0)throw Error('Hyper matrix build failed');
 const bin=target+'/'+mode+'/ruffini_matrix_hyper';
 const ran=spawnSync(bin,[],{encoding:'utf8',timeout:60000,maxBuffer:8*1024*1024});
 const file='hyper-matrix-'+mode+'.log';writeFileSync(root+'/'+file,ran.stdout??'');writeFileSync(root+'/'+file+'.stderr',ran.stderr??'');
 const result={mode,binary:bin,binarySha256:hash(bin),sourceSha256:hash(root+'/matrix-hyper/controls.rs'),status:ran.status,error:ran.error?.message??null,signal:ran.signal,file,sha256:hash(root+'/'+file)};
 results.push(result);writeFileSync(root+'/hyper-matrix-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(ran.error||ran.status!==0)throw Error('Hyper matrix control failure');
}
for(const[p,h]of snapshot)if(hash(ws+'/'+p)!==h)throw Error('concurrent source change: '+p);
