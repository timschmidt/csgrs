// Preserve a command's complete output without mistaking truncation for success.
import { spawn } from 'node:child_process';
import { createWriteStream, mkdirSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { finished } from 'node:stream/promises';

const [tag, cwd, command, ...args] = process.argv.slice(2);
if (!/^[a-z0-9-]+$/.test(tag ?? '') || !cwd || !command)
  throw Error('Usage: capture.mjs TAG CWD COMMAND [ARG ...]');
const results = resolve(dirname(fileURLToPath(import.meta.url)), 'results');
mkdirSync(results, { recursive: true });
const started = new Date().toISOString();
const start = performance.now();
const out = createWriteStream(resolve(results, `${tag}.stdout`), { flags: 'wx' });
const err = createWriteStream(resolve(results, `${tag}.stderr`), { flags: 'wx' });
// Refuse collisions before spawning a command that might modify build outputs.
await Promise.all([new Promise((ok, fail) => out.once('open', ok).once('error', fail)),
  new Promise((ok, fail) => err.once('open', ok).once('error', fail))]);
const child = spawn(command, args, { cwd: resolve(cwd), stdio: ['ignore', 'pipe', 'pipe'] });
child.stdout.pipe(out);
child.stderr.pipe(err);
console.log(JSON.stringify({ tag, pid: child.pid, started, cwd: resolve(cwd), command, args }));
const termination = await new Promise(resolve => {
  child.on('error', error => resolve({ code: null, signal: null, error: String(error) }));
  child.on('close', (code, signal) => resolve({ code, signal }));
});
await Promise.all([finished(out), finished(err)]);
const result = { tag, started, finished: new Date().toISOString(),
  elapsedSeconds: (performance.now() - start) / 1000,
  cwd: resolve(cwd), command, args, ...termination };
writeFileSync(resolve(results, `${tag}.json`), JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify(result));
process.exitCode = termination.code === 0 ? 0 : 1;
