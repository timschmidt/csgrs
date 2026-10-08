import {spawnSync} from 'node:child_process';
const root='/tmp/aern-quadratic-fix.fsxgfP';
const cases=['uni_square','uni_factored','uni_wide','uni_pi','uni_cancellation','uni_scaled_cancellation',
  'uni_bad_degree','multi_square','multi_cross','multi_dense8','multi_dense32','multi_zero_terms',
  'multi_pi','multi_bad_degree','multi_bad_monomial','multi_cancellation','analyze_uni16','analyze_multi16'];
function run(side,name,iterations,mode='time') {
  const result=spawnSync('taskset',['-c','6',`${root}/bench-${side}`,name,mode,String(iterations)],
    {encoding:'utf8',timeout:30000});
  if(result.status!==0) throw Error(`${side} ${name}: ${result.stderr}`);
  return mode==='result'?result.stdout.trim():JSON.parse(result.stdout);
}
for(const name of cases) {
  const results=['baseline','candidate'].map(side=>({side,result:run(side,name,1,'result')}));
  console.log(JSON.stringify({kind:'results',case:name,results}));
  const trial=['baseline','candidate'].map(side=>run(side,name,100));
  const ns=Math.max(...trial.map(r=>r.ns/r.iterations));
  const iterations=Math.max(1,Math.min(10000000,Math.round(30_000_000/ns)));
  console.log(JSON.stringify({kind:'calibration',case:name,iterations,trial}));
  for(let block=0;block<15;block++) {
    const sides=block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
    console.log(JSON.stringify({kind:'pair',case:name,block,samples:sides.map(side=>({side,...run(side,name,iterations)}))}));
  }
  for(const side of ['baseline','candidate'])
    console.log(JSON.stringify({kind:'allocation',case:name,side,...run(side,name,100,'alloc')}));
}
