import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {coldBindings} from './point-cold-protocol.mjs';
coldBindings();
const source=readFileSync('point-history-work.rs','utf8');
const begin=source.indexOf('struct HistoryWork {'),end=source.indexOf('fn benchmark_main(');assert(begin>0&&end>begin);
const files={'point-wasm-work.rs':source.slice(begin,end)};
for(const variant of ['baseline','eager','demand']){
 const tree=variant==='eager'?'point-image-qualified-candidate':'e-plan-qualified-candidate';
 const solver=variant==='demand'?'point-demand-candidate':tree;
 files['point-wasm-'+variant+'/Cargo.toml']=`[package]
name = "calcium-point-wasm-${variant}"
version = "0.0.0"
edition = "2024"
[workspace]
[dependencies]
hyperreal = { path = "../${tree}/hyperreal", features = ["serde"] }
hyperlimit = { path = "../${tree}/hyperlimit" }
hypersolve = { path = "../${solver}/hypersolve" }
serde_json = "1.0"

[lib]
name = "point_wasm_${variant}"
path = "../point-wasm-platform.rs"
crate-type = ["cdylib"]
`;
}
console.log('*** Begin Patch');for(const[p,text]of Object.entries(files)){
 console.log('*** Add File: '+process.cwd()+'/'+p);console.log(text.trimEnd().split('\n').map(l=>'+'+l).join('\n'));
}console.log('*** End Patch');
