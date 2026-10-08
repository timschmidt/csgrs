import assert from 'node:assert/strict';
import {json} from './point-demand-sources.mjs';
import {reanalysePointStatistics} from './reanalyse-point-statistics-v60.mjs';
const recomputed=await reanalysePointStatistics();assert.deepEqual(recomputed,json('point-statistics-v60-analysis.json'));
console.log(JSON.stringify({status:'pass',rawRows:recomputed.totalRows,comparisons:recomputed.totalComparisons,
 tests:recomputed.tests,campaigns:recomputed.campaigns.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts})),
 inventoryMatches:recomputed.inventory.count,limits:recomputed.limits}));
