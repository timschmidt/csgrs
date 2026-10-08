import {captured} from './point-qualified-capture.mjs';
await captured('point-demand-cost-environment','.','node',['point-image-cost-environment.mjs']);
await captured('point-demand-cost-cpu','.','node',['run-point-demand-costs.mjs','cpu']);
await captured('point-demand-cost-allocation','.','node',['run-point-demand-costs.mjs','allocation']);
await captured('point-demand-cost-environment-after','.','node',['point-image-cost-environment.mjs']);
