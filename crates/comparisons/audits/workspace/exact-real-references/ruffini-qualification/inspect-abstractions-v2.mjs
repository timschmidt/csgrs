import {readFileSync, writeFileSync, existsSync, mkdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {inflateRawSync} from 'node:zlib';
import assert from 'node:assert/strict';

const root = import.meta.dirname;
const path = root + '/../Ruffini/abstractions.svg';
const sha = b => createHash('sha256').update(b).digest('hex');
const utf8 = b => { const s = b.toString('utf8'); assert(Buffer.from(s).equals(b)); return s; };
const base64 = s => { const b = Buffer.from(s, 'base64'); assert.equal(b.toString('base64'), s); return b; };
const bytes = readFileSync(path), source = utf8(bytes);
assert.equal(sha(bytes), '770dec9140a009a8436024ed8181d911c5023cbf9f8042f45a6b9751aa2bbe2c');
const lines = source.split('\n');
if (source.endsWith('\n')) lines.pop();
assert.equal(lines.length, 12);
const entities = {lt:'<', gt:'>', quot:'"', amp:'&', apos:"'"};
const xml = s => s.replace(/&(lt|gt|quot|amp|apos);/g, (_, name) => entities[name]);
const attribute = /\bcontent="([^"]*)"/.exec(source); assert(attribute);
const embedded = xml(attribute[1]);
const match = /<diagram\b[^>]*>([^<]*)<\/diagram>/.exec(embedded); assert(match);
const encoded = base64(match[1]);
const inflated = utf8(inflateRawSync(encoded, {maxOutputLength:16*1024*1024}));
const decoded = decodeURIComponent(inflated); assert(decoded.startsWith('<mxGraphModel'));
const hrefs = [...source.matchAll(/(?:xlink:)?href="([^"]*)"/g)].map(m => m[1]);
assert.equal(hrefs.length, 1);
const prefix = 'data:image/svg+xml;base64,'; assert(hrefs[0].startsWith(prefix));
const svgBytes = base64(hrefs[0].slice(prefix.length)), svg = utf8(svgBytes);
const nested = [...decoded.matchAll(/image=data:image\/svg\+xml,([^;"]+);/g)];
assert.equal(nested.length, 1);
assert(base64(nested[0][1]).equals(svgBytes), 'model and outer SVG images must be identical');
const sourceEnvelope = source.replace(attribute[1], '[embedded mxfile: decoded separately]')
    .replace(hrefs[0], '[embedded SVG: decoded separately]');
const mxfileEnvelope = embedded.replace(match[1], '[deflated URI-encoded model: decoded separately]');
const modelEnvelope = decoded.replace(nested[0][1], '[embedded SVG: identical to outer image]');
const active = s => /(?:<script\b|\bon\w+\s*=|<!ENTITY|<foreignObject\b)/i.test(s);
assert(!active(source) && !active(svg) && !active(decoded));
const nestedHrefs = [...svg.matchAll(/(?:xlink:)?href="([^"]*)"/g)].map(m => m[1]);
assert.equal(nestedHrefs.length, 0);
const umlComments = [...svg.matchAll(/<!--([\s\S]*?)-->/g)].filter(m => m[1].includes('@startuml'));
assert.equal(umlComments.length, 1);
const comment = umlComments[0]; assert(comment[1].includes('@enduml'));
function chunks(text, width=160) {
    return text.split('\n').flatMap((s, l) =>
        Array.from({length:Math.max(1, Math.ceil(s.length/width))}, (_, c) =>
            'L'+(l+1)+' C'+(c*width+1)+'-'+Math.min((c+1)*width,s.length)+'\t'+s.slice(c*width,(c+1)*width)))
        .join('\n')+'\n';
}
const meta = {
    source:path, sha256:sha(bytes), bytes:bytes.length, physicalLines:lines.length,
    lineCharacters:lines.map(s=>s.length), sourceChunks:chunks(source).trimEnd().split('\n').length,
    chunkCharacters:160, embeddedXMLCharacters:embedded.length,
    compressedPayloadBytes:encoded.length, inflatedURICharacters:inflated.length,
    decodedXMLCharacters:decoded.length, decodedXMLSha256:sha(decoded),
    innerSVGBytes:svgBytes.length, innerSVGSha256:sha(svgBytes), modelImageIdentical:true,
    innerSVGPhysicalLines:svg.split('\n').length,
    innerSVGPresentationChunks:chunks(svg).trimEnd().split('\n').length,
    activeMarkup:false, nestedHrefs, plantUMLCommentCharacters:comment[1].length
};
if (process.argv[2] === 'meta') console.log(JSON.stringify(meta, null, 2));
else if (process.argv[2] === 'prepare') {
    const out = root+'/abstractions-audit'; assert(!existsSync(out), 'preserve prepared evidence'); mkdirSync(out);
    const files = {
        'source-chunks.txt':chunks(source), 'embedded-mxfile.xml':embedded, 'decoded-model.xml':decoded,
        'envelopes.txt':sourceEnvelope+'\n'+mxfileEnvelope+'\n'+modelEnvelope+'\n',
        'inner.svg':svgBytes, 'inner-chunks.txt':chunks(svg), 'plantuml-comment.txt':comment[1]
    };
    for (const [name, data] of Object.entries(files)) writeFileSync(out+'/'+name, data);
    meta.scriptSha256 = sha(readFileSync(import.meta.filename));
    meta.artifacts = Object.fromEntries(Object.entries(files).map(([name, data]) => [name, sha(data)]));
    writeFileSync(out+'/metadata.json', JSON.stringify(meta, null, 2)+'\n');
    console.log(JSON.stringify(meta, null, 2));
} else throw Error('expected meta or prepare');
