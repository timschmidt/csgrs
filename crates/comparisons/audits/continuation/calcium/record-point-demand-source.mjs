import {writeFileSync} from 'node:fs';
import {demandSources} from './point-demand-sources.mjs';
const binding=demandSources(false);
writeFileSync('point-demand-source-binding.json',JSON.stringify(binding,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({status:'source-bound',root:binding.root,files:Object.keys(binding.sources).length,changed:binding.changed}));
