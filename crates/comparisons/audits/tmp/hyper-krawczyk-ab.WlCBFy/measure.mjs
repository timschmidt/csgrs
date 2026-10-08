import {execFileSync} from 'node:child_process';
const root='/tmp/hyper-krawczyk-ab.WlCBFy';
const configs=[[1,'dense'],[2,'dense'],[4,'dense'],[8,'dense'],[12,'dense'],[8,'cauchy']];
const invoke=(method,args)=>execFileSync('taskset',['-c','6',`${root}/${method}`,...args],{encoding:'utf8'}).trim();
for(const [n,kind] of configs) {
  for(let block=0;block<21;block++) {
    const order=block%2?['fixed','baseline','baseline','fixed']:['baseline','fixed','fixed','baseline'];
    for(let phase=0;phase<order.length;phase++) {
      const method=order[phase];
      const out=invoke(method,['public',String(n),kind,method==='baseline'?'legacy':'fixed','1']);
      const fields=out.split(',');
      if(fields.length!==5||fields[0]!=='0')throw Error(out);
      console.log(JSON.stringify({block,phase,method,n,kind,calls:Number(fields[3]),ns:Number(fields[4])}));
    }
  }
  for(const method of ['baseline','fixed']) {
    const out=invoke(method,['public-alloc',String(n),kind,method==='baseline'?'legacy':'fixed']);
    console.log(JSON.stringify({record:'alloc',method,...JSON.parse(out)}));
  }
  process.stderr.write(`completed ${n} ${kind}\n`);
}
