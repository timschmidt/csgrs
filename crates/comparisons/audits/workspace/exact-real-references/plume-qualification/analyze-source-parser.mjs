import {readFileSync, writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir = import.meta.dirname;
const hash = data => createHash('sha256').update(data).digest('hex');
const rows = file => readFileSync(resolve(dir, file), 'utf8').trimEnd().split('\n').slice(1).map(x => x.split('\t'));
const coverage = new Map(rows('../PLUME_READ_COVERAGE.tsv').map(([path, range, status]) => [path, {range, status}]));
const inventory = rows('../PLUME_FILE_INVENTORY.tsv');
let textFiles = 0, textLines = 0, binaryFiles = 0;
for (const [path, kind, bytes, lines, sha] of inventory) {
  const original = readFileSync(resolve(dir, '../Plume', path));
  if (original.length !== +bytes || hash(original) !== sha) throw Error(`Changed original ${path}`);
  const read = coverage.get(path);
  if (kind === 'text') {
    if (read?.range !== `1-${lines}` || !read.status.startsWith('complete')) throw Error(`Incomplete ${path}`);
    ++textFiles; textLines += +lines;
  } else ++binaryFiles;
  if (path.startsWith('cgi/')) {
    const copy = readFileSync(resolve(dir, '../../.audit-plume-cgi.Dwlx5B/compat', path.slice(4)));
    const expected = path === 'cgi/Alex.hs' ? Buffer.from(original.toString()
      .replace('import Array\n', 'import Data.Array\n')
      .replace('load_dfa al df dmp = map f (recover_dfa dmp)', 'load_dfa al df dmp = fmap f (recover_dfa dmp)')) : original;
    if (!copy.equals(expected)) throw Error(`Unexpected CGI bridge ${path}`);
  }
}
if (inventory.length !== 174 || textFiles !== 170 || textLines !== 24049 || binaryFiles !== 4) throw Error('Inventory count changed');
const probeSha = hash(readFileSync(resolve(dir, 'ParserProbe.hs')));
if (probeSha !== 'bebcb06c959fca82e06743fdb09639646d3fa2709f8eb0206d4f8172f8b11e77') throw Error('Parser probe changed');
const expected = 'lexer\t33938\tpass\nparser\t88\tpass\n';
for (const mode of ['O0', 'O2']) {
  const output = readFileSync(resolve(dir, `cgi-parser-${mode}.log`), 'utf8');
  if (output !== expected) throw Error(`Unexpected ${mode} result`);
}
const summary = {originalFiles: inventory.length, textFilesRead: textFiles, textLinesRead: textLines,
  binaryFilesIdentifiedNotExecuted: binaryFiles, cgiFilesOnlyAlexBridged: 57, parserProbeSha256: probeSha,
  parserPerBuild: {lexerChecks: 33938, parserChecks: 88, failures: 0}, O0O2Identical: true,
  scope: 'Source-container coverage and offline syntax checks only; report/functional audit remains open.'};
writeFileSync(resolve(dir, 'source-parser-summary.json'), JSON.stringify(summary, null, 2) + '\n');
console.log(JSON.stringify(summary, null, 2));
