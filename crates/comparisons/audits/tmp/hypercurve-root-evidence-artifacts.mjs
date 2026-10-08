import assert from 'node:assert/strict';
import {readFileSync, statSync} from 'node:fs';
import {createHash} from 'node:crypto';

const hash = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const draft = JSON.parse(readFileSync('/tmp/hypercurve-root-evidence-checkpoint-draft.json', 'utf8'));
const concurrent = Object.fromEntries(Object.entries(draft.concurrent_hypersolve_work.sha256).map(([path, expected]) => {
    const actual = hash(`/home/tim/Documents/GitHub/workspace/hypersolve/${path}`);
    assert.equal(actual, expected, path);
    return [path, actual];
}));
const dist = '/tmp/hypercurve-pages-root-evidence.xrVrhY';
const assets = ['index.html', 'hypercurve_ui.js', 'hypercurve_ui_bg.wasm'].map(name => ({
    name, bytes: statSync(`${dist}/${name}`).size, sha256: hash(`${dist}/${name}`),
}));
const report = JSON.parse(readFileSync('/tmp/hypercurve-root-evidence-pages-startup.json', 'utf8'));
assert.equal(report.passed, true);
const screenshotSha256 = hash(report.screenshot);
console.log(JSON.stringify({
    concurrent, assets, report, screenshot_sha256: screenshotSha256,
    previous_screenshot_equal: screenshotSha256 === '70c7e48325ca9476f30710cc805d6fbed8db6ef2c6f975204b5a854c99db2cb1',
    browser_script_sha256: hash('/tmp/hypercurve-root-evidence-pages-startup.mjs'),
    hyperreal_conversion_sha256: hash('/home/tim/Documents/GitHub/workspace/hyperreal/src/rational/arithmetic/queries_conversion.rs'),
}, null, 2));
