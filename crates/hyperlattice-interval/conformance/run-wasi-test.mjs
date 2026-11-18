import { readFile } from "node:fs/promises";
import { WASI } from "node:wasi";

const [binary, ...args] = process.argv.slice(2);
const wasi = new WASI({
  version: "preview1",
  args: [binary, ...args],
  returnOnExit: true,
});
const { instance } = await WebAssembly.instantiate(await readFile(binary), {
  wasi_snapshot_preview1: wasi.wasiImport,
});
process.exitCode = wasi.start(instance);
