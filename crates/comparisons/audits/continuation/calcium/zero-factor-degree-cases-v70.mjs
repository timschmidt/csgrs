import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
function source(roots,scale,point){
 let p=[BigInt(scale)];for(const a of roots){const out=Array(p.length+1).fill(0n);for(let i=0;i<p.length;i++){out[i]-=BigInt(a)*p[i];out[i+1]+=p[i];}p=out;}
 return{polynomial:p.map(n=>n+'/1'),point:point+'/1'};
}
export function degreeCases(){
 const cases=[];const add=(family,lp,rp,a,b,ls,rs)=>cases.push({id:cases.length,op:3,family,
  left:source(lp,ls,a),right:source(rp,rs,b)});
 for(const ls of [-3,2])for(const rs of [-2,3]){
  add('shared-oversized',[0,...Array(8).fill(1)],[0,...Array(8).fill(1)],1,1,ls,rs);
  add('admitted-degree-nine',[0,1,2],[0,3,4,5],1,3,ls,rs);
  add('still-over-cap',[0,1,2,6],[0,3,4,5],1,3,ls,rs);
  add('zero-guard',[0,1],[0,1],1,0,ls,rs);
  for(let m=1;m<=3;m++)for(let k=1;k<=2;k++)for(const a of [-2,2])
   add('nonzero-orientation',[a,...[3,-4].slice(0,m-1)],[...Array(k).fill(0),-1],a,-1,ls,rs);
 }
 // Actually equal original source vectors exercise the shared GCD shortcut.
 for(const scale of [-5,3])add('equal-shared-oversized',[0,...Array(8).fill(1)],[0,...Array(8).fill(1)],1,1,scale,scale);
 return cases;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const cases=degreeCases();writeFileSync('zero-factor-degree-input-v70.json',JSON.stringify(cases)+'\n',{flag:'wx'});
 console.log(JSON.stringify({cases:cases.length,rows:cases.length*2}));
}
