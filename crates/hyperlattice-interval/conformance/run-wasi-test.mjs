import { readFile } from "node:fs/promises";
import { WASI } from "node:wasi";

const argv = process.argv.slice(2);
const env = {};
if (argv[0] === "--env") {
  const [, name, value] = argv.splice(0, 3);
  if (!/^[A-Z_][A-Z_0-9]*$/.test(name ?? "") || value === undefined) {
    throw new Error("Expected --env NAME VALUE before the WASI executable");
  }
  env[name] = value;
}
const preopens = {};
if (argv[0] === "--replay-dir") {
  const [, directory] = argv.splice(0, 2);
  if (!directory) throw new Error("Expected a replay fixture directory");
  preopens["/replay"] = directory;
}
const [binary, ...args] = argv;
const wasi = new WASI({
  version: "preview1",
  args: [binary, ...args],
  env,
  preopens,
  returnOnExit: true,
});
const { instance } = await WebAssembly.instantiate(await readFile(binary), {
  wasi_snapshot_preview1: wasi.wasiImport,
});
process.exitCode = wasi.start(instance);
