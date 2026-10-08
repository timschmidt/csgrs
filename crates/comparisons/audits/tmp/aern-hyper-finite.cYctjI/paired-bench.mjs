import {spawnSync} from 'node:child_process';
const root = '/tmp/aern-hyper-finite.cYctjI';
const variant = process.argv[2] || 'candidate';
if (!['candidate','outlined','bounded'].includes(variant)) throw Error('unknown variant');
const cases = ['exact_word','exact_wide','exact_subnormal','inexact_word','inexact_wide',
  'non_dyadic','non_dyadic_wide','near_max_below','near_max_above','near_max_negative',
  'near_max_deep','overflow','unsupported_subnormal','exact_max'];
function run(side, name, iterations, mode='time') {
  const directory = side === 'candidate' ? variant : side;
  const r = spawnSync('taskset', ['-c','6',`${root}/bench-${directory}/target/release/aern-finite-bench`,
    name, mode, String(iterations)], {encoding:'utf8',timeout:30000});
  if(r.status!==0) throw Error(`${side} ${name}: ${r.stderr}`);
  return JSON.parse(r.stdout);
}
for(const name of cases) {
  const trials = ['baseline','candidate'].map(side=>run(side,name,1000));
  const ns = Math.max(...trials.map(r=>r.ns/r.iterations));
  const iterations = Math.max(1,Math.min(10000000,Math.round(50_000_000/ns)));
  console.log(JSON.stringify({kind:'calibration',case:name,iterations,trials}));
  for(let block=0;block<15;block++) {
    const sides=block%2 ? ['candidate','baseline','baseline','candidate'] : ['baseline','candidate','candidate','baseline'];
    const samples=sides.map(side=>({side,...run(side,name,iterations)}));
    console.log(JSON.stringify({kind:'pair',case:name,block,samples}));
  }
  for(const side of ['baseline','candidate'])
    console.log(JSON.stringify({kind:'allocation',case:name,side,...run(side,name,100,'alloc')}));
}
