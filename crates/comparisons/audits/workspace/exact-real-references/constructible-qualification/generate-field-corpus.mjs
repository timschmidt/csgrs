import {writeFileSync,existsSync} from 'node:fs';
import {resolve} from 'node:path';
const destination=resolve(import.meta.dirname,'field-corpus.tsv');
if(existsSync(destination))throw Error('refusing to overwrite frozen corpus');
const q=(n,d=1)=>`q ${n} ${d}`,add=(a,b)=>`+ ${a} ${b}`,mul=(a,b)=>`* ${a} ${b}`,div=(a,b)=>`/ ${a} ${b}`,neg=a=>`n ${a}`,sqrt=a=>`s ${a}`;
const rows=[];
const emit=(group,label,order,a,b)=>rows.push([group,label,order,a,b].join('\t'));
function linear(a,b,r,d=1){return add(q(a,d),mul(q(b,d),sqrt(r)));}
function square(a,b,r,d=1){return add(add(q(a*a,d*d),mul(q(b*b,d*d),r)),mul(q(2*a*b,d*d),sqrt(r)));}
function signSurd(a,b,r){
  if(b===0)return Math.sign(a);if(a===0)return Math.sign(b);
  if(Math.sign(a)===Math.sign(b))return Math.sign(a);
  const norm=a*a-b*b*r;return norm===0?0:norm>0?Math.sign(a):Math.sign(b);
}
for(const r of [2,3,5,7])for(const d of [1,2])for(let a=-3;a<=3;a++)for(let b=-3;b<=3;b++){
  const x=linear(a,b,q(r),d),z=square(a,b,q(r),d),sign=signSurd(a,b,r),label=`${r}-${d}-${a}-${b}`;
  emit('quadratic',`root-${label}`,'EQ',sqrt(z),sign<0?neg(x):x);
  emit('quadratic',`sign-${label}`,['LT','EQ','GT'][sign+1],x,q(0));
  if(sign!==0){
    const norm=a*a-b*b*r;
    const rationalized=div(linear(a*d,-b*d,q(r)),q(norm));
    emit('quadratic',`inverse-${label}`,'EQ',div(q(1),x),rationalized);
  }
}
for(let depth=1;depth<=5;depth++){
  let r=q(2);for(let k=1;k<depth;k++)r=add(q(2),sqrt(r));
  for(let a=-3;a<=3;a++)for(const b of [-1,1]){
    const x=linear(a,b,r),z=square(a,b,r);
    // Every sqrt(r) is strictly between 1 and 2, so this sign is independent
    // of the donor and of any finite floating approximation.
    const sign=a>=2?1:a<=-2?-1:b;
    emit(`tower-${depth}`,`expanded-${a}-${b}`,'EQ',sqrt(z),sign<0?neg(x):x);
    const norm=add(q(a*a),neg(mul(q(b*b),r)));
    emit(`tower-${depth}`,`inverse-${a}-${b}`,'EQ',div(q(1),x),div(linear(a,-b,r),norm));
  }
}
for(const [a,b] of [[2,3],[2,5],[3,7],[5,7]])for(const sa of [-1,1])for(const sb of [-1,1]){
  const x=add(mul(q(sa),sqrt(q(a))),mul(q(sb),sqrt(q(b))));
  const square=add(q(a+b),mul(q(2*sa*sb),sqrt(q(a*b))));
  const sign=sa===sb?sa:(a>b?sa:sb);
  emit('independent','surd-'+[a,b,sa,sb].join('-'),'EQ',sqrt(square),sign<0?neg(x):x);
}
writeFileSync(destination,'group\tlabel\torder\tlhs\trhs\n'+rows.join('\n')+'\n');
console.log(JSON.stringify({rows:rows.length,groups:Object.fromEntries([...new Set(rows.map(s=>s.split('\t')[0]))].map(g=>[g,rows.filter(s=>s.startsWith(g+'\t')).length]))}));
