import {spawnSync} from 'node:child_process';
const root = '/tmp/aern-hyper-exp.PdU63J';
const candidate = process.argv[2] || 'candidate';
if(!['candidate','sign'].includes(candidate)) throw Error('invalid variant');
const cases = [
  ['tiny',1,1000000,-128,'cold'], ['small',1,4,-128,'cold'],
  ['half',1,2,-128,'cold'], ['seven_halves',7,2,-128,'cold'],
  ['negative',-7,2,-128,'cold'], ['large',10000,1,-128,'cold'],
  ['large_negative',-10000,1,-128,'cold'],
  ['cached',7,2,-128,'warm'], ['constructor',7,2,-128,'constructor'],
  ['coarse_small',1,4,1,'cold'], ['coarse_negative',-32,1,1,'cold'],
  ['coarse_repair',3,2,1,'cold'],
];
function run(side, c, iterations, mode=c[4]) {
  const directory = side==='candidate' ? candidate : side;
  const r = spawnSync('taskset',['-c','6',`${root}/expm1-bench-${directory}/target/release/aern-expm1-bench`,
    ...c.slice(1,4).map(String),mode,String(iterations)], {encoding:'utf8',timeout:30000});
  if(r.status!==0) throw Error(`${side} ${c[0]}: ${r.stderr}`);
  return JSON.parse(r.stdout);
}
for(const c of cases) {
  const trials = ['baseline','candidate'].map(side=>run(side,c,100));
  const perIteration = Math.max(...trials.map(r=>r.ns/r.iterations));
  const iterations = Math.max(1,Math.min(2000000,Math.round(50_000_000/perIteration)));
  console.log(JSON.stringify({kind:'calibration',case:c[0],iterations,trials}));
  for(let block=0;block<15;block++) {
    const sides=block%2 ? ['candidate','baseline','baseline','candidate'] : ['baseline','candidate','candidate','baseline'];
    const samples=sides.map(side=>({side,...run(side,c,iterations)}));
    console.log(JSON.stringify({kind:'pair',case:c[0],block,samples}));
  }
  if(c[4]==='cold') for(const side of ['baseline','candidate']) {
    console.log(JSON.stringify({kind:'allocation',case:c[0],side,...run(side,c,100,'alloc')}));
  }
}
