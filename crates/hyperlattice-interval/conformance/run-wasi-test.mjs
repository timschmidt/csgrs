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
const [binary, ...args] = argv;
const wasi = new WASI({
  version: "preview1",
  args: [binary, ...args],
  env,
  returnOnExit: true,
});
const { instance } = await WebAssembly.instantiate(await readFile(binary), {
  wasi_snapshot_preview1: wasi.wasiImport,
});
process.exitCode = wasi.start(instance);
