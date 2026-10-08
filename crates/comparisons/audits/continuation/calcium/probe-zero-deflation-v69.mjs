import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {oracle} from './power-sums-polynomial-oracle.mjs';
import {root,rootCount,image,q,qc,evaluate,integers} from './power-rational-oracle-v69.mjs';
const read=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function zeroDeflationProbe(){
 const old=read('results/point-demand-public.stdout').filter(r=>r.report?.status==='Undecided'),
  wide=read('results/power-wide-baseline-run-v69.stdout').filter(r=>r.policy===0&&r.report?.status==='Undecided');
 assert.equal(old.length,40);assert.equal(wide.length,30);
 const records=[],polynomials=new Map();let totalChecks=0;
 const check=(condition)=>{assert(condition);totalChecks++;};
 for(const [corpus,rows]of [['earlier',old],['wide',wide]])for(const row of rows){
  const left=root(row.left),right=root(row.right),lp=integers(left.p),rp=integers(right.p);
  check(row.report.operation==='Divide'&&row.report.message==='could not construct binary resultant polynomial exactly'&&row.report.root===null);
  check(rootCount(left.p,left.lo,left.hi)===1&&rootCount(right.p,right.lo,right.hi)===1);
  // Where there is no exact witness, the authored lower endpoint is not a root;
  // closed-interval counting therefore agrees with the public half-open owner.
  for(const r of [left,right])check(r.exact!==null||evaluate(r.p,r.lo)[0]!==0n);
  check(qc(right.lo,q(0))>0||qc(right.hi,q(0))<0);
  const zeroOrder=p=>{let k=0;while(k<p.length&&p[k]===0n)k++;return k;};
  const leftZeroOrder=zeroOrder(lp),rightZeroOrder=zeroOrder(rp);check(leftZeroOrder>0&&rightZeroOrder>0);
  const deflated=rp.slice(rightZeroOrder);check(deflated.length>=2&&deflated[0]!==0n);
  check((lp.length-1)*(rp.length-1)<=9);
  check(JSON.stringify([...Array(rightZeroOrder).fill(0n),...deflated].map(String))===JSON.stringify(rp.map(String)));
  const key=[lp,rp].map(p=>p.join(',')).join(';');
  if(!polynomials.has(key)){
   const before=oracle(lp,rp,3).poly,after=oracle(lp,deflated,3).poly;
   check(before===null&&after!==null);polynomials.set(key,{before,after});
  }
  const proposed=polynomials.get(key).after.map(n=>q(BigInt(n))),bounds=image([left.lo,left.hi],[right.lo,right.hi],3),
   imageRoots=rootCount(proposed,bounds[0],bounds[1]);check(imageRoots===1);
  // Removing x^k cannot change the divisor's roots within this nonzero interval.
  check(rootCount(deflated.map(n=>q(n)),right.lo,right.hi)===1);
  records.push({corpus,key:corpus==='earlier'?[row.type,row.i,row.j,row.op,row.scale,row.case??null]:row.id,
   leftDegree:lp.length-1,rightDegree:rp.length-1,leftZeroOrder,rightZeroOrder,
   deflatedRightDegree:deflated.length-1,quotientImage:bounds.map(v=>v.join('/')),imageRoots,
   polynomial:polynomials.get(key).after});
 }
 return{checkpoint:69,status:'certified-corpus-opportunity',records:records.length,earlier:40,wide:30,
  distinctNormalizedCarrierPairs:polynomials.size,totalChecks,proposals:records,
  proof:'For a selected nonzero divisor beta, Q(beta)=beta^k R(beta)=0 implies R(beta)=0. Both authored divisor intervals exclude zero exactly; removing this unused carrier factor preserves their selected roots. Independent nonzero deflated resultants have exactly one root in every tested quotient image.',
  limits:'Independent mathematical opportunity, not an implemented or retained Hyper change, a general completeness theorem, or a performance result. Repeated scales/carriers are not independent bugs. Preserve STRICT nonzero evidence, original source validation and proof replay; audit degree admission and shared-carrier square-free shortcuts before implementing.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(zeroDeflationProbe()));
