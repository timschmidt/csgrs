import {execFileSync} from 'node:child_process';
const base='/tmp/cdar-audit.HQMgJs/';
for(const work of ['square','multiply','add']) {
  for(const policy of ['raw','limited']) {
    for(let block=0;block<3;block++) {
      for(const variant of ['master','mbound','mbound','master']) {
        const out=execFileSync('taskset',['-c','6',base+'storage-'+variant,
          work,policy,'128','+RTS','-T','-M512m','-RTS'],
          {encoding:'utf8',timeout:15000});
        console.log(JSON.stringify({variant,block,...JSON.parse(out)}));
      }
    }
  }
}
