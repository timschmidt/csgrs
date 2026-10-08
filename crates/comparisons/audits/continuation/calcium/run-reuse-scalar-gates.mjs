// Distinct captured gates against the immutable weak-cache trial.
import { spawn } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here=dirname(fileURLToPath(import.meta.url)), cwd=resolve(here,'../../..');
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
  'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
const jobs=[];
for(const profile of ['debug','release']) for(const [name,manifest,args] of [
  ['state','root-exp-reuse-qualification',['state']],
  ['public','root-exp-reuse-probe',[]],
  ['tiny','root-exp-reuse-public-trial',[]],
]) jobs.push([`root-exp-reuse-${name}-${profile}`,['run','--offline','--locked',
  ...(profile==='release'?['--release']:[]),'--manifest-path',`audits/continuation/calcium/${manifest}/Cargo.toml`,
  ...(args.length?['--',...args]:[])]]);
for(const [tag,args] of jobs) {
  const code=await new Promise((ok,fail)=>{
    const child=spawn(process.execPath,[resolve(here,'capture.mjs'),tag,cwd,'env',...env,'cargo',...args],{cwd,stdio:'inherit'});
    child.on('error',fail); child.on('close',ok);
  });
  if(code!==0)throw Error(`${tag} failed; inspect its preserved capture before retry`);
}
console.log('Completed all six additional scalar gates.');
