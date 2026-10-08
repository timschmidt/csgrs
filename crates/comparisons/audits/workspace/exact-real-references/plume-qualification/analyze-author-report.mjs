import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = import.meta.dirname;
const data = name => readFileSync(resolve(dir, name));
const sha = name => createHash('sha256').update(data(name)).digest('hex');
assert.equal(sha('../Plume/author-report.pdf'), 'a671bb8c7ac6ef65a74e3ed47d3112d13afe8688aea87f4366874c200b264218');
assert.equal(sha('../Plume/author-report.txt'), '65e49cad5a4e3530429276313f2def4f2d8ade04d4673e5c77e641513c859fab');
assert.equal(sha('../Plume/report.txt'), '1e08410b19402f9baaebdac0e5e10005be4de58032f06e2fb4349de5ecb24e8e');
assert.equal(sha('../Plume/index.html'), 'ad5323616b0515bafb6bf8351a5365c5748aa8ad19b54d3c10999751774c818e');
const lines = name => data(name).toString().split('\n');
const primary = lines('../Plume/report.txt'), alternate = lines('../Plume/author-report.txt');
assert.equal(primary.length, alternate.length);
assert.equal(alternate.length - 1, 4871);
const changed = primary.flatMap((line, i) => line === alternate[i] ? [] : [i + 1]);
assert.deepEqual(changed, [564, 642, 643, 644, 646, 2370, 3323, 4693, 4694, 4696]);
// The five difference hunks were read and their corresponding PDF pages
// independently viewed. Byte comparison does not imply visual read credit.
const visualPages = [20, 21, 22, 65, 77, 78, 79, 80, 90, 104, 129];

// Finite geometric identity underlying the exact Leibniz remainder integral:
// (1+t^2) * sum(k=0..N-1, (-t^2)^k) = 1-(-t^2)^N.
// Check after clearing denominators; no floating or donor arithmetic involved.
let geometricChecks = 0;
for (let n = 1n; n <= 32n; n++) {
  for (let a = 0n; a <= 32n; a++) {
    const b = 32n;
    let sum = 0n;
    for (let k = 0n; k < n; k++) {
      sum += (k % 2n ? -1n : 1n) * a ** (2n * k) * b ** (2n * (n - 1n - k));
    }
    assert.equal((b*b + a*a) * sum, b ** (2n*n) - (n % 2n ? -1n : 1n) * a ** (2n*n));
    assert(b*b <= b*b + a*a && b*b + a*a <= 2n*b*b);
    geometricChecks++;
  }
}
// Integration gives |pi - 4*S_N| = 4*integral_0^1 t^(2N)/(1+t^2) dt,
// strictly between 2/(2N+1) and 4/(2N+1). N=10^50 cannot yield 100 digits.
const n = 10n ** 50n;
assert(2n * 10n ** 100n > 2n*n + 1n);

const depth = leaves => leaves <= 1 ? 0 : 1 + Math.max(depth(Math.floor(leaves / 2)), depth(Math.ceil(leaves / 2)));
let depthChecks = 0;
for (let leaves = 1; leaves <= 512; leaves++) {
  const d = depth(leaves);
  assert(BigInt(leaves) <= 1n << BigInt(d));
  if (d) assert(BigInt(leaves) > 1n << BigInt(d - 1));
  depthChecks++;
}
assert.equal(depth(16), 4);
assert.equal(Math.ceil(Math.log(16)), 3); // Far from an integer-boundary tie.

// Inventory the generated report-section links visible in the already-read
// landing page. Do not silently count the other HTML projections as read.
const html = data('../Plume/index.html').toString();
assert(html.includes('Converted with LaTeX2HTML'));
const sections = [...html.matchAll(/HREF="(node(\d+)\.html)#([^"]+)"/g)].map(match => ({
  path: match[1], number: Number(match[2]), anchor: match[3],
  indexLine: html.slice(0, match.index).split('\n').length,
}));
assert.equal(sections.length, 147);
assert.deepEqual(sections.map(section => section.number), Array.from({ length: 147 }, (_, i) => i + 1));
const readNodes = new Set([91, 97, 147]);
const inventory = ['path\tindex_line\tanchor\tclassification\tread_status', ...sections.map(section => [
  section.path, section.indexLine, section.anchor, 'generated report HTML projection',
  readNodes.has(section.number) ? 'independently read; see report coverage' : 'indexed only; no independent read credit',
].join('\t'))].join('\n') + '\n';
writeFileSync(resolve(dir, '../PLUME_HTML_LINK_INVENTORY.tsv'), inventory);
console.log(JSON.stringify({
  alternateTextLinesIndependentlyRead: 4871,
  extractedLinesDiffering: changed,
  alternatePdfPagesVisuallyRead: visualPages,
  exactGeometricIdentityChecks: geometricChecks,
  balancedDepthChecks: depthChecks,
  leibniz100DigitClaimRejected: true,
  generatedHtmlSectionsIndexed: sections.length,
  generatedHtmlSectionsIndependentlyRead: readNodes.size,
  limitations: [
    'Text and visual read credit are human-review assertions recorded in the coverage ledger, not proved by a hash check.',
    'The 144 remaining generated HTML section bodies and 122 other alternate PDF pages are not credited as independently read.',
    'Use the existing generated-artifact scope convention: primary report is fully read and visually checked; alternate text and all five extraction-difference pages are checked.',
    'Finite identity/depth checks corroborate the written elementary proof; there is no 10^50-term execution or new performance result.',
  ],
}, null, 2));
