import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8');
export function checkControls() {
 const native=read('results/nfloat-controls-native.stdout');
 assert.equal(native,read('results/nfloat-controls-memcheck.stdout'));
 const actual=native.trimEnd().split('\n');
 assert.equal(actual.shift(),'suite,limbs,direction,pattern,a,b,c,d,status,checked,ok,round_preserved');
 const summary=JSON.parse(actual.pop()),expected=[],groups={};
 function row(suite,n,d,p,a,b,c,e,checked=1) {
  expected.push([suite,n,d,p,a,b,c,e,0,checked,1,1].join(','));
  const g=groups[suite]??={rows:0,comparisons:0};g.rows++;g.comparisons+=checked;
 }
 const shapes=[[0,0,0],[2,0,3],[1,1,1],[2,3,4],[4,2,3],[8,17,5],[17,8,17],[3,3,3],[8,8,8]];
 for(let n=1;n<=66;n++)for(let d=0;d<2;d++) {
  for(let p=0;p<8;p++)for(let op=0;op<10;op++)for(let alias=0;alias<3;alias++)row('scalar',n,d,p,op,alias,0,0);
  for(let p=0;p<8;p++)for(let kind=0;kind<5;kind++)row('conversion',n,d,p,kind,0,0,0);
  if([1,2,3,4,5,8,16,32,66].includes(n)) {
   for(const len of [0,1,2,3,8,17])for(let p=0;p<4;p++)for(let init=0;init<2;init++)
    for(let sub=0;sub<2;sub++)for(let rev=0;rev<2;rev++)for(let alias=0;alias<2;alias++)
     row('dot',n,d,p,len,init,sub,2*rev+alias);
   for(const [shape,[m,,p]]of shapes.entries())for(let pattern=0;pattern<3;pattern++)
    for(let alias=0;alias<(shape>=7?3:1);alias++)row('matrix',n,d,pattern,shape,alias,0,0,m*p);
  }
 }
 assert.deepEqual(actual,expected);
 assert.deepEqual(summary,{suite:'nfloat-controls',limb_bits:64,rows:44574,comparisons:74922,equalities:5698,failures:0});
 assert.deepEqual(groups,{scalar:{rows:31680,comparisons:31680},conversion:{rows:5280,comparisons:5280},
  dot:{rows:6912,comparisons:6912},matrix:{rows:702,comparisons:31050}});
 const mem=read('results/nfloat-controls-memcheck.stderr');
 assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/595,961 allocs, 595,961 frees, 299,063,144 bytes allocated/);
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 const names=['dyadic_dot_stack_accumulator_handles_wide_products_alignment_and_borrow',
  'dyadic_dot_stack_accumulator_preserves_arbitrary_precision_fallback',
  'dyadic_dot_word_accumulator_handles_wide_denominators_and_falls_back'].map(n=>'rational::arithmetic::tests::'+n);
 for(const profile of ['debug','release']) {
  const out=read('results/nfloat-hyper-dyadic-'+profile+'.stdout');
  assert.deepEqual([...out.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]),names);
  assert.match(out,/3 passed; 0 failed; 0 ignored; 0 measured; 691 filtered out/);
 }
 return {summary,groups,memory:{allocations:595961,requestedBytes:299063144,errors:0,liveBytes:0},
  hyperExistingTestsPerProfile:3};
}
if(process.argv.includes('--nfloat-controls-summary'))console.log(JSON.stringify(checkControls()));
