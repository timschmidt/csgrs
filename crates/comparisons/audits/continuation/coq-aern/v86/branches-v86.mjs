import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
const gcd=(a,b)=>{a=a<0n?-a:a;while(b)[a,b]=[b,a%b];return a;};
const q=(n,d=1n)=>{assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];};
const add=([a,b],[c,d])=>q(a*d+c*b,b*d),neg=([a,b])=>[-a,b],sub=(a,b)=>add(a,neg(b));
const mul=([a,b],[c,d])=>q(a*c,b*d),div=([a,b],[c,d])=>q(a*d,b*c),abs=a=>a[0]<0n?neg(a):a;
const square=([a,b])=>[sub(mul(a,a),mul(b,b)),mul(q(2n),mul(a,b))];
const serial=a=>a.map(r=>r.map(String)),same=(a,b)=>assert.deepEqual(a,b);
const admissible=(a,b,c,d)=>[
 ['a-positive',a[0]>0n,()=>[c,div(b,mul(q(2n),c))]],
 ['a-negative',a[0]<0n,()=>[div(b,mul(q(2n),d)),d]],
 ['b-positive',b[0]>0n,()=>[c,div(b,mul(q(2n),c))]],
 ['b-negative',b[0]<0n,()=>[c,neg(d)]],
];

let inputs=0,nonzeroInputs=0,branches=0,ambiguousOutputs=0;const branchCounts={};const trace=createHash('sha256');
for(let u=-16;u<=16;u++)for(let v=-16;v<=16;v++){
 const witness=[q(BigInt(u),16n),q(BigInt(v),16n)],z=square(witness),[a,b]=z;
 const norm=add(mul(witness[0],witness[0]),mul(witness[1],witness[1])),c=abs(witness[0]),d=abs(witness[1]);
 // Exact identities establish the nonnegative component radicals used by
 // the donor formulas; this is not execution of AERN2's sqrt or select.
 same(mul(norm,norm),add(mul(a,a),mul(b,b)));
 same(mul(c,c),div(add(norm,a),q(2n)));same(mul(d,d),div(sub(norm,a),q(2n)));
 const outputs=new Set();let allowed=0;
 for(const [branch,guard,run]of admissible(a,b,c,d))if(guard){
  const w=run();same(square(w),z);allowed++;branches++;branchCounts[branch]=(branchCounts[branch]??0)+1;
  outputs.add(JSON.stringify(serial(w)));trace.update(JSON.stringify({witness:serial(witness),z:serial(z),branch,w:serial(w)})+'\n');
 }
 if(u!==0||v!==0){assert(allowed>0);nonzeroInputs++;}else assert.equal(allowed,0);
 assert(outputs.size<=2);if(outputs.size===2)ambiguousOutputs++;inputs++;
}
const z=[q(-3n),q(-4n)],c=q(1n),d=q(2n),choices=admissible(...z,c,d).filter(([,guard])=>guard).map(([name,,run])=>({name,root:run()}));
assert.deepEqual(choices.map(x=>({name:x.name,root:serial(x.root)})),[
 {name:'a-negative',root:[['-1','1'],['2','1']]},
 {name:'b-negative',root:[['1','1'],['-2','1']]},
]);
for(const {root}of choices)same(square(root),z);
const controls=[
 ()=>same(square([c,d]),z),
 ()=>same(square([choices[0].root[0],choices[1].root[1]]),z),
 ()=>same(square([c,div(z[1],c)]),z),
 ()=>same(square([d,neg(c)]),z),
 ()=>div(q(1n),q(0n)),
];
for(const fail of controls)assert.throws(fail,{name:'AssertionError'});
console.log(JSON.stringify({status:'verified-rational-branch-model',inputs,nonzeroInputs,branches,branchCounts,ambiguousOutputs,
 negativeQuadrantExample:choices.map(x=>({branch:x.name,root:serial(x.root)})),corruptionControls:controls.length,sha256:trace.digest('hex'),
 limits:'Every admissible source-derived formula on a finite rational-root grid; not Haskell execution, a principal-root contract, or a repair of admitted proofs.'}));
