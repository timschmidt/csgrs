// Optional real-browser execution of the same independent corpus. No npm packages.
import { spawn } from "node:child_process";
import { createServer } from "node:http";
import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

const [modulePath, corpusPath, mode] = process.argv.slice(2);
if (!modulePath || !corpusPath) throw new Error("Usage: node browser.mjs MODULE.wasm CORPUS.json");
const files = new Map([
  ["/", ["text/html", Buffer.from("<!doctype html><title>IEEE interval conformance</title>")]],
  ["/module.wasm", ["application/wasm", await fs.readFile(modulePath)]],
  ["/corpus.json", ["application/json", await fs.readFile(corpusPath)]],
  ["/check.mjs", ["text/javascript", await fs.readFile(fileURLToPath(new URL("check.mjs", import.meta.url)))]],
  ["/instantiate.mjs", ["text/javascript", await fs.readFile(fileURLToPath(new URL("instantiate.mjs", import.meta.url)))]],
]);
const server = createServer((request, response) => {
  const item = files.get(request.url);
  response.writeHead(item ? 200 : 404, { "Content-Type": item?.[0] ?? "text/plain" });
  response.end(item?.[1] ?? "Not found");
});
await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
const origin = `http://127.0.0.1:${server.address().port}`;
const profile = await fs.mkdtemp(path.join(os.tmpdir(), "interval-wasm-browser-"));
const browser = spawn(process.env.CHROMIUM ?? "chromium", [
  "--headless", "--no-sandbox", "--disable-gpu", "--disable-background-networking",
  "--no-first-run", "--remote-debugging-port=0", `--user-data-dir=${profile}`, "about:blank",
], { stdio: ["ignore", "ignore", "pipe"] });
let socket;
const deadline = setTimeout(() => browser.kill("SIGTERM"), 60000);
try {
  const address = await new Promise((resolve, reject) => {
    let output = "";
    browser.stderr.on("data", chunk => {
      output += chunk;
      const match = output.match(/DevTools listening on (ws:\/\/\S+)/);
      if (match) resolve(match[1]);
    });
    browser.once("error", reject);
    browser.once("exit", code => reject(new Error(`Browser exited before connection: ${code}`)));
  });
  socket = new WebSocket(address);
  await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
  const requests = new Map();
  let next = 1;
  socket.onmessage = event => {
    const message = JSON.parse(event.data);
    const request = requests.get(message.id);
    if (!request) return;
    requests.delete(message.id);
    if (message.error) request.reject(new Error(JSON.stringify(message.error)));
    else request.resolve(message.result);
  };
  socket.onclose = () => {
    for (const request of requests.values()) request.reject(new Error("Browser disconnected"));
    requests.clear();
  };
  const call = (method, params = {}, sessionId) => new Promise((resolve, reject) => {
    const id = next++;
    requests.set(id, { resolve, reject });
    socket.send(JSON.stringify({ id, method, params, sessionId }));
  });
  const version = await call("Browser.getVersion");
  const { targetId } = await call("Target.createTarget", { url: origin });
  const { sessionId } = await call("Target.attachToTarget", { targetId, flatten: true });
  let ready = false;
  for (let attempt = 0; attempt < 100 && !ready; attempt++) {
    const result = await call("Runtime.evaluate", {
      expression: `location.origin === ${JSON.stringify(origin)} && document.readyState === 'complete'`,
      returnByValue: true,
    }, sessionId);
    ready = result.result?.value === true;
    if (!ready) await new Promise(resolve => setTimeout(resolve, 50));
  }
  if (!ready) throw new Error("Local conformance page did not finish navigation");
  const result = await call("Runtime.evaluate", {
    expression: `(async () => {
      const api = await import('/check.mjs');
      return api[${JSON.stringify(mode === "--egress" ? "verifyEgress" : mode === "--lattice" ? "verifyLattice" : "verify")}](new Uint8Array(await (await fetch('/module.wasm')).arrayBuffer()), await (await fetch('/corpus.json')).json());
    })()`,
    awaitPromise: true,
    returnByValue: true,
  }, sessionId);
  if (result.exceptionDetails || !result.result?.value) throw new Error(JSON.stringify(result));
  console.log(JSON.stringify({ ...result.result.value, browser: version }, null, 2));
} finally {
  clearTimeout(deadline);
  socket?.close();
  if (browser.exitCode === null && browser.signalCode === null) {
    browser.kill("SIGTERM");
    await new Promise(resolve => browser.once("exit", resolve));
  }
  await new Promise(resolve => server.close(resolve));
  // Chromium helpers may finish profile writes just after the parent exits.
  await fs.rm(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 });
}
