import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, writeFile } from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';

const dist = process.argv[2];
assert.match(dist, /^\/tmp\/hypercurve-pages-dist\.[A-Za-z0-9]+$/);
const requests = [];
const exceptions = [];
const failures = [];
const consoleErrors = [];
const allowed = new Map([
  ['index.html', 'text/html'],
  ['hypercurve_ui.js', 'text/javascript'],
  ['hypercurve_ui_bg.wasm', 'application/wasm'],
]);
const server = http.createServer(async (request, response) => {
  const pathname = new URL(request.url, 'http://localhost').pathname;
  const name = pathname === '/hypercurve/' ? 'index.html' : pathname.slice('/hypercurve/'.length);
  if (pathname === '/favicon.ico') {
    response.writeHead(204).end();
    return;
  }
  if (!pathname.startsWith('/hypercurve/') || !allowed.has(name)) {
    requests.push({ pathname, status: 404 });
    response.writeHead(404).end();
    return;
  }
  try {
    const content = await readFile(path.join(dist, name));
    requests.push({ pathname, status: 200, bytes: content.length });
    response.writeHead(200, { 'Content-Type': allowed.get(name) }).end(content);
  } catch (error) {
    response.writeHead(500).end(String(error));
  }
});
let browser;
let socket;
let nextId = 0;
const pending = new Map();
const deadline = setTimeout(() => browser?.kill('SIGTERM'), 60000);
try {
  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', resolve);
  });
  const profile = await mkdtemp('/tmp/hypercurve-browser-profile.');
  browser = spawn('/usr/bin/chromium-browser', [
    '--headless=new', '--no-sandbox', '--no-first-run', '--no-default-browser-check',
    '--disable-background-networking', '--disable-component-update', '--disable-sync',
    '--disable-dev-shm-usage', '--use-angle=swiftshader', '--enable-unsafe-swiftshader',
    '--window-size=1100,760', '--remote-debugging-port=0',
    '--remote-debugging-address=127.0.0.1', `--user-data-dir=${profile}`, 'about:blank',
  ], { stdio: ['ignore', 'ignore', 'pipe'] });
  const endpoint = await new Promise((resolve, reject) => {
    let stderr = '';
    browser.once('error', reject);
    browser.once('exit', code => reject(new Error(`Chromium exited ${code}: ${stderr}`)));
    browser.stderr.on('data', chunk => {
      stderr += chunk;
      const found = stderr.match(/DevTools listening on (ws:\/\/\S+)/);
      if (found) resolve(new URL(found[1]));
    });
  });
  const pages = await (await fetch(`http://${endpoint.host}/json/list`)).json();
  const page = pages.find(page => page.type === 'page');
  assert.ok(page);
  socket = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve, reject) => {
    socket.addEventListener('open', resolve, { once: true });
    socket.addEventListener('error', reject, { once: true });
  });
  socket.addEventListener('message', event => {
    const message = JSON.parse(event.data);
    if (message.id) {
      const promise = pending.get(message.id);
      if (promise) {
        pending.delete(message.id);
        clearTimeout(promise.timer);
        if (message.error) promise.reject(new Error(JSON.stringify(message.error)));
        else promise.resolve(message.result);
      }
    } else if (message.method === 'Runtime.exceptionThrown') {
      exceptions.push(message.params.exceptionDetails);
    } else if (message.method === 'Network.loadingFailed') {
      failures.push(message.params);
    } else if (message.method === 'Runtime.consoleAPICalled' && message.params.type === 'error') {
      consoleErrors.push(message.params.args.map(arg => arg.value ?? arg.description));
    }
  });
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const id = ++nextId;
    const timer = setTimeout(() => {
      pending.delete(id);
      reject(new Error(`CDP timeout: ${method}`));
    }, 10000);
    pending.set(id, { resolve, reject, timer });
    socket.send(JSON.stringify({ id, method, params }));
  });
  await send('Runtime.enable');
  await send('Page.enable');
  await send('Network.enable');
  await send('Page.navigate', { url: `http://127.0.0.1:${server.address().port}/hypercurve/` });
  let state;
  const started = Date.now();
  while (Date.now() - started < 45000) {
    const result = await send('Runtime.evaluate', {
      expression: `({ready: !document.getElementById('loading_text') && !!document.getElementById('the_canvas_id'), title: document.title, pathname: location.pathname, baseURI: document.baseURI, width: document.querySelector('canvas')?.width, height: document.querySelector('canvas')?.height})`,
      returnByValue: true,
    });
    state = result.result.value;
    if (state?.ready && state.width > 0 && state.height > 0) break;
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  assert.ok(state?.ready && state.width > 0 && state.height > 0, JSON.stringify({ state, exceptions, consoleErrors }));
  await new Promise(resolve => setTimeout(resolve, 1500));
  const screenshot = await send('Page.captureScreenshot', { format: 'png' });
  await writeFile('/tmp/hypercurve-quotient-projection-api-pages-startup.png', Buffer.from(screenshot.data, 'base64'));
  assert.equal(state.pathname, '/hypercurve/');
  assert.equal(exceptions.length, 0, JSON.stringify(exceptions));
  assert.equal(failures.length, 0, JSON.stringify(failures));
  assert.equal(consoleErrors.length, 0, JSON.stringify(consoleErrors));
  assert.ok(requests.some(request => request.pathname.endsWith('.wasm') && request.status === 200));
  assert.ok(requests.every(request => request.status === 200));
  const report = { passed: true, dist, profile, state, requests, exceptions, failures, consoleErrors, screenshot: '/tmp/hypercurve-quotient-projection-api-pages-startup.png' };
  await writeFile('/tmp/hypercurve-quotient-projection-api-pages-startup.json', JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify(report));
} finally {
  clearTimeout(deadline);
  for (const promise of pending.values()) clearTimeout(promise.timer);
  socket?.close();
  browser?.kill('SIGTERM');
  server.closeAllConnections();
  await new Promise(resolve => server.close(resolve));
}
