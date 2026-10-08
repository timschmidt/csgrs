import {readFileSync, writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';

const root = import.meta.dirname, dir = root+'/abstractions-audit/';
const sha = b => createHash('sha256').update(b).digest('hex');
const read = p => readFileSync(p, 'utf8');
const meta = JSON.parse(read(dir+'metadata.json'));
assert.equal(sha(readFileSync(meta.source)), meta.sha256);
assert.equal(sha(readFileSync(root+'/inspect-abstractions-v2.mjs')), meta.scriptSha256);
for (const [name, hash] of Object.entries(meta.artifacts)) assert.equal(sha(readFileSync(dir+name)), hash, name);
function reconstruct(text) {
    const lines = [], columns = [];
    for (const row of text.trimEnd().split('\n')) {
        const m = /^L(\d+) C(\d+)-(\d+)\t([\s\S]*)$/.exec(row); assert(m);
        const l = +m[1]-1, start = +m[2], end = +m[3], s = m[4];
        assert.equal(start, (columns[l] ?? 0)+1);
        assert.equal(s.length, end-start+1);
        lines[l] = (lines[l] ?? '')+s; columns[l] = end;
    }
    assert.equal(lines.filter(x => x !== undefined).length, lines.length);
    return lines.join('\n');
}
assert.equal(reconstruct(read(dir+'source-chunks.txt')), read(meta.source));
assert.equal(reconstruct(read(dir+'inner-chunks.txt')), read(dir+'inner.svg'));
const entities = {lt:'<', gt:'>', quot:'"', amp:'&', apos:"'"};
const xml = s => s.replace(/&(#\d+|lt|gt|quot|amp|apos);/g,
    (_, k) => k.startsWith('#') ? String.fromCodePoint(+k.slice(1)) : entities[k]);
const model = read(dir+'decoded-model.xml');
const raw = /\bplantUmlData="([^"]*)"/.exec(model); assert(raw);
const data = JSON.parse(xml(raw[1])); assert.deepEqual(Object.keys(data).sort(), ['data','format']);
assert.equal(data.format, 'svg');
const uml = data.data;
const comment = read(dir+'plantuml-comment.txt').replace(/\r\n/g, '\n');
const commentedUML = comment.slice(comment.indexOf('@startuml'), comment.indexOf('@enduml')+7)
    .replace(/<\|- -/g, '<|--');
assert.equal(commentedUML, uml);
const old = 'dk.jonaslindstrom.math.algebra.abstractions.';
const short = s => { assert(s.startsWith(old)); return s.slice(old.length); };
const classes = [...uml.matchAll(/^interface (\S+) \{/gm)].map(m => short(m[1]));
const edges = [...uml.matchAll(/^(\S+) <\|-- (\S+)$/gm)].map(m => [short(m[1]),short(m[2])].join('<-')).sort();
assert.equal(classes.length, 14); assert.equal(new Set(classes).size, 14);
assert.equal(edges.length, 16); assert.equal(new Set(edges).size, 16);
const svg = read(dir+'inner.svg');
const svgEdges = [...svg.matchAll(/id="([^"<>]+)&lt;-([^"<>]+)"/g)]
    .map(m => [short(m[1]), short(m[2])].join('<-')).sort();
assert.deepEqual(svgEdges, edges);
const labels = [...svg.matchAll(/<text\b[^>]*>([^<]*)<\/text>/g)].map(m => xml(m[1]));
const methods = [...uml.matchAll(/^~ (.+)$/gm)].map(m => m[1]);
assert.equal(methods.length, 15);
assert.deepEqual(labels.slice().sort(), [old.slice(0,-1), ...classes, ...methods].sort());
const currentEdges = [], currentSources = {};
for (const name of [...classes, 'OrderedSet']) {
    const path = root+'/../Ruffini/common/src/main/java/dk/jonaslindstrom/ruffini/common/abstractions/'+name+'.java';
    const s = read(path); currentSources[name] = sha(readFileSync(path));
    const declaration = /public interface ([^{]+)\{/.exec(s); assert(declaration);
    const i = declaration[1].lastIndexOf(' extends ');
    if (i >= 0) {
        const parents = declaration[1].slice(i+9).replace(/<[^<>]*>/g,'').split(',').map(s=>s.trim());
        for (const parent of parents) currentEdges.push(parent+'<-'+name);
    }
}
assert.deepEqual(currentEdges.sort(), [...edges,'Set<-OrderedSet'].sort());
const png = readFileSync(dir+'inner.png'); assert(png.subarray(0,8).equals(Buffer.from([137,80,78,71,13,10,26,10])));
assert.equal(png.readUInt32BE(16), 554); assert.equal(png.readUInt32BE(20), 926);
const result = {
    scriptSha256:sha(readFileSync(import.meta.filename)), originalSHA256:meta.sha256,
    completeChunkReconstruction:true, twoImagePayloadsIdentical:meta.modelImageIdentical,
    modelAndCommentUMLIdenticalAfterDocumentedCommentEscaping:true,
    diagramInterfaces:classes, diagramEdges:edges, diagramMethodCount:methods.length,
    renderedLabelsMatchUML:true, currentEdgesMatchWithOneAdditionalOrderedSet:true,
    currentSources, renderedPNG:{sha256:sha(png),bytes:png.length,width:554,height:926},
    scope:'Artifact, UML, and inheritance consistency only. Full decoded SVG/envelopes read by auditor; no claim of manually reading opaque base64/deflate characters or of proving Java implementations from UML.'
};
writeFileSync(dir+'verification.json', JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result,null,2));
