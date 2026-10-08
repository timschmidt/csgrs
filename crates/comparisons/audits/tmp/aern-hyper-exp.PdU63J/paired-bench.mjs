import {spawnSync} from 'node:child_process';
const root = '/tmp/aern-hyper-exp.PdU63J';
const candidate = process.argv[2] || 'candidate';
if(!['candidate','demand','demand-cached'].includes(candidate)) throw Error('invalid candidate');
const cases = [
  ['small',1,4,-128,'cold'], ['half',1,2,-128,'cold'],
  ['seven_fifths',7,5,-128,'cold'], ['seven_halves',7,2,-128,'cold'],
  ['fifteen_halves',15,2,-128,'cold'], ['negative',-7,2,-128,'cold'],
  ['integer_two',2,1,-128,'cold'], ['integer_128',128,1,-128,'cold'],
  ['integer_257',257,1,-128,'cold'], ['negative_32',-32,1,-128,'cold'],
  ['cached',7,2,-128,'warm'], ['constructor',7,2,-128,'constructor'],
  ['coarse_repair',3,2,1,'cold'],
];
function run(side, c, iterations, mode=c[4]) {
  const directory = side==='candidate' ? candidate : side;
  const r = spawnSync('taskset',['-c','6',`${root}/bench-${directory}/target/release/aern-exp-bench`,
    ...c.slice(1,4).map(String),mode,String(iterations)], {encoding:'utf8',timeout:30000});
  if(r.status!==0) throw Error(`${side} ${c[0]}: ${r.stderr}`);
  return JSON.parse(r.stdout);
}
for(const c of cases) {
  const trials = ['baseline','candidate'].map(side=>run(side,c,1000));
  const perIteration = Math.max(...trials.map(r=>r.ns/r.iterations));
  const iterations = Math.max(1,Math.min(2000000,Math.round(50_000_000/perIteration)));
  process.stdout.write(JSON.stringify({kind:'calibration',case:c[0],iterations,trials})+'\n');
  for(let block=0;block<15;block++) {
    const sides=block%2 ? ['candidate','baseline','baseline','candidate'] : ['baseline','candidate','candidate','baseline'];
    const samples=sides.map(side=>({side,...run(side,c,iterations)}));
    process.stdout.write(JSON.stringify({kind:'pair',case:c[0],block,samples})+'\n');
  }
  if(c[4]==='cold') for(const side of ['baseline','candidate']) {
    process.stdout.write(JSON.stringify({kind:'allocation',case:c[0],side,...run(side,c,100,'alloc')})+'\n');
  }
}
