import {readdirSync, readFileSync, existsSync, readlinkSync} from 'node:fs';
import {createHash} from 'node:crypto';
const roots = ['/tmp/hypercurve-dyadic-bernstein-control.a5ZKq2', '/tmp/hypercurve-affine-offset-control.JVwgaq'];
const hash = data => createHash('sha256').update(data).digest('hex');
function catalog(root, relative = '', out = new Map()) {
    for (const entry of readdirSync(root + '/' + relative, {withFileTypes: true})) {
        if (['target', '.git'].includes(entry.name)) continue;
        const path = relative ? relative + '/' + entry.name : entry.name;
        if (entry.isDirectory()) catalog(root, path, out);
        else if (entry.isSymbolicLink()) out.set(path, Buffer.from(readlinkSync(root + '/' + path)));
        else if (entry.isFile()) out.set(path, readFileSync(root + '/' + path));
    }
    return out;
}
const differences = [];
for (const crate of ['hyperreal', 'hypersolve', 'hypercurve', 'hyperlimit', 'hyperlattice', 'hypertri', 'hypermesh']) {
    if (!roots.every(root => existsSync(root + '/' + crate))) continue;
    const [left, right] = roots.map(root => catalog(root + '/' + crate));
    for (const file of new Set([...left.keys(), ...right.keys()])) {
        const a = left.get(file), b = right.get(file);
        if (a && b && a.equals(b)) continue;
        differences.push({file: crate + '/' + file, baseline_sha256: a && hash(a), candidate_sha256: b && hash(b)});
    }
}
console.log(JSON.stringify(differences, null, 2));
