import {readFileSync,writeFileSync,readdirSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,ws=resolve(root,'../..'),target=ws+'/.audit-targets/ireal-derivative-18555';
if(existsSync(root+'/hyper-polynomial-runs.json'))throw Error('refusing to overwrite Hyper polynomial evidence');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const paths=[];for(const crate of ['hyperreal','hyperlattice','hyperlimit','hypersolve'])for(const p of ['Cargo.toml',...readdirSync(ws+'/'+crate+'/src',{recursive:true}).filter(p=>p.endsWith('.rs')).map(p=>'src/'+p)])paths.push(crate+'/'+p);
const snapshot=paths.sort().map(p=>[p,hash(ws+'/'+p)]);writeFileSync(root+'/hyper-polynomial-source-snapshot.json',JSON.stringify(snapshot,null,2)+'\n');
const results=[];
for(const mode of ['debug','release']) {
 const args=['build','--offline','--manifest-path',root+'/polynomial-hyper/Cargo.toml','--target-dir',target,...(mode==='release'?['--release']:[])];
 const r=spawnSync('cargo',args,{encoding:'utf8',timeout:120000,env:{...process.env,TMPDIR:ws+'/.audit-coefficient-tmp.XBlc5e',CCACHE_DISABLE:'1'},maxBuffer:8*1024*1024});
 const log=root+'/hyper-polynomial-'+mode+'-build.log';if(existsSync(log))writeFileSync(root+'/hyper-polynomial-'+mode+'-build-attempt-'+Date.now()+'.log',readFileSync(log));
 writeFileSync(log,(r.stdout??'')+(r.stderr??'')+'\n'+JSON.stringify({status:r.status,error:r.error?.message??null})+'\n');
 if(r.error||r.status!==0)throw Error('Hyper polynomial build failed');
 const binary=target+'/'+mode+'/ruffini_polynomial_hyper';const ran=spawnSync(binary,[],{encoding:'utf8',timeout:120000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 const file='hyper-polynomial-'+mode+'.log';writeFileSync(root+'/'+file,ran.stdout??'');writeFileSync(root+'/'+file+'.stderr',ran.stderr??'');
 const result={mode,binary,binarySha256:hash(binary),sourceSha256:hash(root+'/polynomial-hyper/controls.rs'),status:ran.status,error:ran.error?.code??null,signal:ran.signal,file,sha256:hash(root+'/'+file)};results.push(result);writeFileSync(root+'/hyper-polynomial-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(result));
 if(ran.error||ran.status!==0)throw Error('Hyper polynomial control failure');
}
for(const[p,h]of snapshot)if(hash(ws+'/'+p)!==h)throw Error('source changed during Hyper polynomial qualification: '+p);
