import {resolve} from 'node:path';
import {demandBindings} from './point-demand-bindings.mjs';
demandBindings();
const files=[];
for(const variant of ['baseline','eager','demand']){
 const tree=variant==='eager'?'point-image-qualified-candidate':'e-plan-qualified-candidate';
 const solver=variant==='demand'?'point-demand-candidate':tree;
 const s=`[package]\nname = "calcium-point-cold-${variant}"\nversion = "0.0.0"\nedition = "2024"\n[workspace]\n[dependencies]\nhyperreal = { path = "../${tree}/hyperreal", features = ["serde"] }\nhyperlimit = { path = "../${tree}/hyperlimit" }\nhypersolve = { path = "../${solver}/hypersolve" }\nserde_json = "1.0"\n\n[[bin]]\nname = "point-cold-${variant}"\npath = "../point-cold-native.rs"\n\n[lib]\nname = "point_cold_${variant}"\npath = "../point-cold-platform.rs"\ncrate-type = ["cdylib"]\n`;
 files.push(['point-cold-'+variant+'/Cargo.toml',s]);
}
console.log('*** Begin Patch\n'+files.map(([p,s])=>'*** Add File: '+resolve(p)+'\n'+s.trimEnd().split('\n').map(l=>'+'+l).join('\n')).join('\n')+'\n*** End Patch');
