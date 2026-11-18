// Bare modules and the no_std IEEE kernel must be import-free. Only standard
// library clients on Preview 1 need host functions.
export async function instantiate(module, wasiPreview1 = false, hostFunctions = []) {
  const imports = WebAssembly.Module.imports(module);
  for (const entry of imports) {
    if (!wasiPreview1 || entry.module !== "wasi_snapshot_preview1" ||
        entry.kind !== "function" || !hostFunctions.includes(entry.name)) {
      throw new Error(`Unexpected module import: ${JSON.stringify(entry)}`);
    }
  }
  if (!wasiPreview1) return WebAssembly.instantiate(module);
  const { WASI } = await import("node:wasi");
  const wasi = new WASI({ version: "preview1", args: [], env: {}, preopens: {} });
  const instance = await WebAssembly.instantiate(module, { wasi_snapshot_preview1: wasi.wasiImport });
  wasi.initialize(instance);
  return instance;
}

export const stdFunctions = ["environ_get", "environ_sizes_get", "fd_write", "proc_exit"];
