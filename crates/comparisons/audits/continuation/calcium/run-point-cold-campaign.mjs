import assert from 'node:assert/strict';
import {captured} from './point-qualified-capture.mjs';
import {coldBindings,variants} from './point-cold-protocol.mjs';
const before=coldBindings();
for(const v of variants)await captured('point-cold-native-'+v,'.','node',['run-point-cold-native.mjs',v]);
for(const v of variants)await captured('point-cold-wasm-'+v,'.','node',['--expose-gc','run-point-cold-wasm.mjs',v]);
await captured('point-cold-capacity','.','df',['-B1','/tmp','.']);
assert.deepEqual(coldBindings(),before);console.log(JSON.stringify({status:'captured',variants:3,platforms:2,groups:6912,queries:62208}));
