import {execFileSync} from 'node:child_process';
const root='/tmp/hyper-bernstein-scale.xDkXjE';
const variant=process.argv[2]??'candidate';
const binary=process.argv[3]==='batch'?'batch':'public';
const cases=binary==='batch'?[[16,'batch',8]]:[[1,'outside',8],[2,'spread',8],[4,'spread',8],[8,'spread',16],[16,'spread',16],[2,'cluster',32],[4,'cluster',32],[8,'cluster',32],[2,'repeated',16],[4,'repeated',16],[2,'endpoint',8],[4,'endpoint',8],[8,'endpoint',16],[2,'positive',24],[4,'positive',24],[4,'cluster',0]];
for(const [degree,kind,depth] of cases) {
  if(process.argv[3]==='short'&&!((degree===1&&kind==='outside')||(degree===2&&['spread','cluster','endpoint','positive'].includes(kind))))continue;
  for(let block=0;block<21;block++) {
    const order=block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
    for(let phase=0;phase<4;phase++) {
      const method=order[phase];
      const out=execFileSync('taskset',['-c','6',`${root}/${binary}-${method==='candidate'?variant:method}`,'bench',String(degree),kind,String(depth),'1'],{encoding:'utf8'}).trim();
      const fields=out.split(',');
      if(fields.length!==6||fields[0]!=='0')throw Error(out);
      console.log(JSON.stringify({variant,block,phase,method,degree,kind,depth,calls:Number(fields[4]),ns:Number(fields[5])}));
    }
  }
  for(const method of ['baseline','candidate']) {
    const out=execFileSync(`${root}/${binary}-${method==='candidate'?variant:method}`,['alloc',String(degree),kind,String(depth)],{encoding:'utf8'}).trim();
    console.log(JSON.stringify({record:'alloc',variant,method,...JSON.parse(out)}));
  }
  process.stderr.write(`completed ${degree} ${kind} ${depth}\n`);
}
