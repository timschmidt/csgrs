import assert from 'node:assert/strict';
import {demandBindings} from './point-demand-bindings.mjs';
import {sha} from './point-demand-sources.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const b=demandBindings();
// The build's --lib tests are a distinct 447-test gate, not the full solver.
for(const profile of ['debug','release'])await captured('point-demand-solver-full-'+profile,'point-demand-candidate/hypersolve','env',
 [...cargoEnv,'cargo','test','--locked','--offline',...(profile==='release'?['--release']:[]),'--all-features']);
for(const mode of ['public','approx','extended']){
 const f=b.artifacts.find(f=>f.variant==='demand'&&f.mode===mode);
 await captured('point-demand-'+mode,'.',f.path,[]);
}
await captured('point-demand-public-check','.','node',['check-point-demand-public.mjs']);
for(const mode of ['extended','approx']){
 const f=b.artifacts.find(f=>f.variant==='demand'&&f.mode===mode);
 await captured('point-demand-memory-'+mode,'.','valgrind',[
  '--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',f.path]);
 assert.equal(sha('results/point-demand-memory-'+mode+'.stdout'),sha('results/point-demand-'+mode+'.stdout'));
}
demandBindings();console.log(JSON.stringify({status:'qualification-executions-completed',public:'public,approx,extended',
 limits:'Read full memory summaries before making memory claims. Full solver tests are separate from the earlier library-only gates. No CPU/WASM/consumer or retention decision implied.'}));
