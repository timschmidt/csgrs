// Run one WASI Preview 1 command module with a deliberately constructed guest argv,
// environment and optional replay-directory preopen. Prefix order is one --env
// NAME VALUE, then one --replay-dir DIRECTORY, then BINARY and guest arguments.
// Prefixes are consumed once and only at the front; remaining tokens are forwarded.
// Guest proc_exit becomes this process's exit code. Read/instantiate/start errors
// propagate, and this launcher supplies no timeout or trap recovery.
import { readFile } from "node:fs/promises";
import { WASI } from "node:wasi";

// Mutable launcher arguments, excluding Node and this script's path.
const argv = process.argv.slice(2);
// Guest variables start empty; the host environment is not copied into this object.
const env = {};
if (argv[0] === "--env") {
  const [, name, value] = argv.splice(0, 3);
  if (!/^[A-Z_][A-Z_0-9]*$/.test(name ?? "") || value === undefined) {
    throw new Error("Expected --env NAME VALUE before the WASI executable");
  }
  env[name] = value;
}
// No filesystem preopens by default; the optional host directory appears as /replay.
const preopens = {};
if (argv[0] === "--replay-dir") {
  const [, directory] = argv.splice(0, 2);
  if (!directory) throw new Error("Expected a replay fixture directory");
  preopens["/replay"] = directory;
}
// argv[0] in the guest is the supplied binary path; no separate existence/arity check.
const [binary, ...args] = argv;
// Return from proc_exit so its status can be assigned after the WASI command ends.
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
