import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';

const base=fileURLToPath(new URL('../calcium/results/',import.meta.url));
const gcd=(a,b)=>{a=a<0n?-a:a;while(b){[a,b]=[b,a%b];}return a;};
const q=(n,d=1n)=>{assert(d>0n);const g=gcd(n,d);return[n/g,d/g];};
const read=v=>{assert(Array.isArray(v)&&v.length===2);assert(v.every(x=>typeof x==='string'&&/^-?\d+$/.test(x)));return q(BigInt(v[0]),BigInt(v[1]));};
const add=([a,b],[c,d])=>q(a*d+c*b,b*d),neg=([a,b])=>[-a,b],sub=(a,b)=>add(a,neg(b));
const mul=([a,b],[c,d])=>q(a*c,b*d),cmp=([a,b],[c,d])=>a*d<c*b?-1:a*d>c*b?1:0;
const abs=a=>a[0]<0n?neg(a):a;

function enclosure(row,name,radicand){
 const [l,h]=row[name].map(read),offset=q(BigInt(row.offset));assert(cmp(l,h)<=0);
 let lo=sub(l,offset),hi=sub(h,offset);
 if(row.sign<0)[lo,hi]=[neg(hi),neg(lo)];
 assert(lo[0]>=0n);assert(cmp(mul(lo,lo),radicand)<=0);assert(cmp(mul(hi,hi),radicand)>=0);
 // Loose sanity guard on enclosure width, independent of Hyper's rational scale.
 assert(cmp(sub(h,l),q(8n,1n<<BigInt(-row.precision)))<=0);
 return[l,h];
}

export function checkRows(rows){
 assert.equal(rows.length,49);assert.deepEqual(rows.at(-1),{terminal:true,rows:48});
 let count=0,unknown=0,equal=0,known=0,nonwinningBorrowedSelections=0;
 for(const [k,eq]of [[0,false],[32,false],[128,false],[2056,false],[4096,false],[0,true]])for(const sign of [-1,1])for(const offset of [0,7])for(const swap of [false,true]){
  const r=rows[count];assert.deepEqual([r.row,r.k,r.equal,r.sign,r.offset,r.swap],[count,k,eq,sign,offset,swap]);
  assert.equal(r.precision,-(Math.max(k,32)+64));
  const small=q(2n),large=add(small,q(eq?0n:1n,1n<<BigInt(k)));
  const min=enclosure(r,'min',sign>0?small:large),max=enclosure(r,'max',sign>0?large:small);
  if(!eq)assert(cmp(min[1],max[0])<0,'fine enclosures must separate unequal extrema');
  const trueOrder=eq?'Equal':((sign>0)!==swap?'Less':'Greater');
  if(k>=2056){assert.equal(r.ordering,null);assert.equal(r.certificate,'Unknown { min_precision: -2048 }');unknown++;nonwinningBorrowedSelections++;}
  else {assert.equal(r.ordering,trueOrder);assert(r.certificate.startsWith('Known {'));eq?equal++:known++;}
  assert.equal(r.borrowed_min_left,r.ordering!=='Greater');
  assert.equal(r.borrowed_max_left,r.ordering!=='Less');
  count++;
 }
 return{rows:count,enclosures:count*2,knownUnequal:known,knownEqual:equal,unknown,nonwinningBorrowedSelections};
}

function selectionModel(){
 let pairs=0,branches=0;
 // Mathematical checks of the source's overlap contract, NOT donor execution.
 for(let x=-32;x<=32;x++)for(let y=-32;y<=32;y++)for(let n=0;n<=8;n++){
  const a=q(BigInt(x),8n),b=q(BigInt(y),8n),eps=q(1n,1n<<BigInt(n)),max=cmp(a,b)<0?b:a;
  const left=cmp(sub(b,eps),a)<0,right=cmp(sub(a,eps),b)<0;
  assert(left||right);
  for(const [guard,value]of [[left,a],[right,b]])if(guard){assert(cmp(abs(sub(value,max)),eps)<=0);branches++;}
  pairs++;
 }
 return{kind:'independent rational overlap model, not Coq or Haskell execution',pairs,admissibleBranches:branches};
}

export function checkHyper(){
 const a=readFileSync(base+'coq-aern-run-debug-v85.stdout'),b=readFileSync(base+'coq-aern-run-release-v85.stdout');
 assert(a.length>0);assert(a.equals(b));assert(a.toString().endsWith('\n'));
 const rows=a.toString().trimEnd().split('\n').map(JSON.parse),result=checkRows(rows);
 const changes=[
  r=>r.pop(),r=>r[0].row++,r=>r[0].sign=1,r=>r[24].ordering='Equal',
  r=>r[0].borrowed_max_left=false,r=>r[24].certificate='Known {}',
  r=>[r[4].min,r[4].max]=[r[4].max,r[4].min],
  r=>r[0].min[0][1]='0',r=>r[8].precision++,r=>r[48].rows--,
  r=>r[40].equal=false,r=>r[39].max=[['0','1'],['1','1']],
 ];
 for(const mutate of changes){const altered=structuredClone(rows);mutate(altered);assert.throws(()=>checkRows(altered));}
 return{status:'verified',...result,matchingBytes:a.length,corruptionControls:changes.length,selectionModel:selectionModel()};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkHyper()));
