import {checkWasmQualification} from './check-point-wasm-costs.mjs';
const {qualification:q}=checkWasmQualification();
console.log(JSON.stringify({status:q.status,observations:q.observations,independentChecks:q.totalChecks,groups:q.groups,
 rowsSha256:q.rowsSha256,binariesSha256:q.binariesSha256,runtime:q.runtime}));
