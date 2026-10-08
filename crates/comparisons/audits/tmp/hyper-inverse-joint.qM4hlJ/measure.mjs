import {execFileSync} from 'node:child_process';
const root='/tmp/hyper-inverse-joint.qM4hlJ';
const variant=process.argv[2]??'joint';
const cases=[[1,'dense'],[2,'dense'],[4,'dense'],[8,'dense'],[12,'dense'],[8,'cauchy'],[8,'diagonal'],[8,'permutation']].map(([n,kind])=>['quadratic',n,kind]).concat([[1,'dense'],[2,'dense'],[4,'dense'],[8,'dense'],[12,'dense'],[8,'cauchy']].map(([n,kind])=>['affine',n,kind]),[1,2,4,8].map(n=>['standard',n,'dense']));
const invoke=(suite,method,n,kind,mode)=>{
  const binary=suite==='quadratic'&&method==='baseline'?'/tmp/hyper-krawczyk-ab.WlCBFy/fixed':`${root}/${suite}-${method==='joint'?variant:method}`;
  const args=suite==='quadratic'?[mode==='alloc'?'public-alloc':'public',String(n),kind,'fixed','1']:[mode,String(n),kind];
  return execFileSync('taskset',['-c','6',binary,...args],{encoding:'utf8'}).trim();
};
for(const [suite,n,kind] of cases) {
  if(suite==='standard'&&!['standard','all'].includes(process.argv[3]))continue;
  if(process.argv[3]==='standard'&&suite!=='standard')continue;
  if(process.argv[3]==='focused'&&(kind!=='dense'||(suite==='affine'?n>4:![1,4,8].includes(n))))continue;
  for(let block=0;block<21;block++) {
    const order=block%2?['joint','baseline','baseline','joint']:['baseline','joint','joint','baseline'];
    for(let phase=0;phase<order.length;phase++) {
      const method=order[phase];
      const out=invoke(suite,method,n,kind,'bench');
      const fields=out.split(',');
      if(fields.length!==5||fields[0]!=='0')throw Error(out);
      console.log(JSON.stringify({suite,variant,block,phase,method,n,kind,calls:Number(fields[3]),ns:Number(fields[4])}));
    }
  }
  for(const method of ['baseline','joint'])console.log(JSON.stringify({record:'alloc',suite,variant,method,...JSON.parse(invoke(suite,method,n,kind,'alloc'))}));
  process.stderr.write(`completed ${suite} ${n} ${kind}\n`);
}
