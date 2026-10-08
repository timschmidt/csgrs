import {captured} from './point-qualified-capture.mjs';
await captured('point-history-cost-environment','.','node',['point-image-cost-environment.mjs']);
await captured('point-history-cost-cpu','.','node',['run-point-history-costs.mjs','cpu']);
await captured('point-history-cost-allocation','.','node',['run-point-history-costs.mjs','allocation']);
await captured('point-history-cost-environment-after','.','node',['point-image-cost-environment.mjs']);
