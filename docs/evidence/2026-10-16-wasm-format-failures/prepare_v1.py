"""Prepare a frozen source snapshot after reproducing current public failures."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'docs/evidence/2026-10-16-wasm-format-failures'
WORK=ROOT/'target/properties/wasm-format-failures'
SOURCE=WORK/'candidate-source-v1'
OUT=WORK/'preparation-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2)+'\n')


def replace(text,old,new):
    assert text.count(old)==1,old
    return text.replace(old,new)


baseline=json.loads((WORK/'baseline-v1/result.json').read_text())
assert baseline['complete'] and baseline['public_tests']=={'passed':1,'failed':11,'total':12}
assert baseline['sources_and_artifacts_unchanged']
with gzip.open(WORK/'baseline-v1/source-hashes.json.gz','rt') as stream:
    original=json.load(stream)
assert all(sha(ROOT/path)==digest for path,digest in original.items())
SOURCE.mkdir(exist_ok=False)
OUT.mkdir(exist_ok=False)
for name,digest in original.items():
    target=SOURCE/name
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/name,target)
    assert sha(target)==digest
changed={name:(SOURCE/name).read_text() for name in ['src/wasm/mod.rs','src/wasm/mesh_js.rs','scripts/test-wasm-bindings.sh','tests/wasm/export_projection.cjs']}
p=SOURCE/'src/wasm/mod.rs'
p.write_text(replace(p.read_text(),'pub mod curve_js;\n','pub mod curve_js;\nmod io_error_js;\n'))
p=SOURCE/'src/wasm/mesh_js.rs'
s=p.read_text()
s=replace(s,'    geometry_context,\n','    geometry_context,\n    io_error_js::io_error,\n')
s=replace(s,'    #[wasm_bindgen(js_name = fromArrays)]',
'''    /// Import finite coordinate triples and in-range index triples.
    ///
    /// Invalid buffers throw `GeometryError` with code `invalid_input` before
    /// constructing a native mesh. Topology is validated by consuming operations.
    #[wasm_bindgen(js_name = fromArrays)]''')
s=replace(s,'''            return Err(JsValue::from_str(
                "positions and indices must contain complete triples",
            ));''','''            return Err(geometry_exception(
                "positions and indices must contain complete triples",
                crate::errors::ValidationErrorKind::InvalidInput,
            ));''')
s=replace(s,'            .ok_or_else(|| JsValue::from_str("positions must be finite"))?;',
'''            .ok_or_else(|| {
                geometry_exception(
                    "positions must be finite",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;''')
s=replace(s,'            .ok_or_else(|| JsValue::from_str("an index is outside the position array"))?;',
'''            .ok_or_else(|| {
                geometry_exception(
                    "an index is outside the position array",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;''')
s=replace(s,'    #[wasm_bindgen(js_name = toSTLBinary)]',
'''    /// Import binary or ASCII STL through strict native closed-PWN validation.
    ///
    /// Canonical empty geometry is accepted. Coordinates parse as binary32 and
    /// are promoted exactly; validation certifies that parsed geometry, not the
    /// original exact construction or analytic source. Manifoldness is a separate
    /// claim. Failure throws `GeometryIoError`; native validation remains in its
    /// typed `cause`, and no mesh is returned. The byte buffer is not retained.
    #[wasm_bindgen(js_name = fromSTL)]
    pub fn from_stl(data: &[u8]) -> Result<Self, JsValue> {
        crate::io::stl::from_stl(data)
            .map(Into::into)
            .map_err(|error| io_error(error, "STL"))
    }

    /// Serialize binary32 STL, rejecting collapsed projected triangles.
    ///
    /// Failure throws an ordinary `GeometryIoError` object with a native `code`,
    /// `format`, and variant fields. This replaces the former string exception;
    /// inspect `message` for text and `code` for the machine-readable variant.
    #[wasm_bindgen(js_name = toSTLBinary)]''')
s=replace(s,'        crate::io::stl::to_stl_binary(&self.inner, "mesh").map_err(js_error)',
            '        crate::io::stl::to_stl_binary(&self.inner, "mesh")\n            .map_err(|error| io_error(error, "STL"))')
s=replace(s,'    #[wasm_bindgen(js_name = toSTLASCII)]',
'''    /// Serialize exact decimal spellings of the finite binary64 projection.
    ///
    /// Failure throws `GeometryIoError` with queryable native fields rather than
    /// a string. Reimport subsequently quantizes coordinates to binary32.
    #[wasm_bindgen(js_name = toSTLASCII)]''')
s=replace(s,'        crate::io::stl::to_stl_ascii(&self.inner, "mesh").map_err(js_error)',
            '        crate::io::stl::to_stl_ascii(&self.inner, "mesh")\n            .map_err(|error| io_error(error, "STL"))')
p.write_text(s)
p=SOURCE/'scripts/test-wasm-bindings.sh'
s=p.read_text()
s=replace(s,'node tests/wasm/export_projection.cjs "${wasm_bindings_dir}/csgrs.js"\n',
'''node tests/wasm/export_projection.cjs "${wasm_bindings_dir}/csgrs.js"
node tests/wasm/io_contracts.cjs "${wasm_bindings_dir}/csgrs.js"
''')
s+='''
# Compile and execute the native format pipeline through generated declarations.
cp tests/wasm/io_contracts.ts "${wasm_bindings_dir}/io_contracts.ts"
"${tsc_bin}" --strict --target ES2020 --lib ES2020,DOM,ESNext.Disposable \\
    --module Node16 --moduleResolution Node16 --outDir "${wasm_bindings_dir}" \\
    "${wasm_bindings_dir}/io_contracts.ts"
node "${wasm_bindings_dir}/io_contracts.js"
'''
p.write_text(s)
p=SOURCE/'tests/wasm/export_projection.cjs'
p.write_text(replace(p.read_text(),'''  // The JS API has no STL importer. Decode the format independently with its
  // binary32 coordinate quantization, then use the public mesh constructor.''',
'''  // Keep an independent decoder as an oracle alongside native import tests.
  // Apply binary32 quantization, then use the public mesh constructor.'''))
new={'src/wasm/io_error_js.rs':'io_error_js_v1.rs',
     'tests/wasm/io_contracts.cjs':'io_contracts_v1.cjs',
     'tests/wasm/io_contracts.ts':'io_contracts_v1.ts'}
for name,prototype in new.items():
    assert not (SOURCE/name).exists()
    shutil.copyfile(E/prototype,SOURCE/name)
subprocess.run(['rustfmt','--edition','2024','--config-path',str(SOURCE/'.rustfmt.toml'),
                *[str(SOURCE/name) for name in ['src/wasm/mod.rs','src/wasm/mesh_js.rs','src/wasm/io_error_js.rs']]],check=True)
assert all(sha(SOURCE/name)==sha(E/prototype) for name,prototype in new.items())
inputs=original|{name:sha(SOURCE/name) for name in [*changed,*new]}
with gzip.open(OUT/'source-hashes.json.gz','wt') as stream:
    json.dump(inputs,stream,sort_keys=True)
(OUT/'production.patch').write_text(''.join(''.join(difflib.unified_diff(before.splitlines(keepends=True),
    (SOURCE/name).read_text().splitlines(keepends=True),fromfile='a/'+name,tofile='b/'+name)) for name,before in changed.items()))
assert all(sha(ROOT/path)==digest for path,digest in original.items())
dump(OUT/'result.json',{'complete':True,'runner_sha256':sha(__file__),
    'base_commit':'3d335db43eac00d003bc2fb0ed1beb2a9c19d79b','source':str(SOURCE),
    'source_hashes_sha256':sha(OUT/'source-hashes.json.gz'),
    'implementation_paths':[*changed,*new],
    'prototype_sha256':{name:sha(E/name) for name in new.values()},
    'main_sources_unchanged':True,'native_parser_and_validation_unchanged':True})
print('Prepared seven-path candidate from reproduced public failures.',flush=True)
