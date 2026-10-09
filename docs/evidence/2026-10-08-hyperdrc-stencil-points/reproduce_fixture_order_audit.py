"""Reproduce both optional-marker variants on the full archived CPArti board.

Usage: python3 reproduce_fixture_order_audit.py /tmp/stencil-cost /tmp/order-audit
This runs after cost trials. It also proves the relevant production source files
are byte-identical to the pre-stencil revision and compares the recorded logs.
"""
import hashlib
import json
import pathlib
import re
import runpy
import subprocess
import sys
import zipfile

record = pathlib.Path(__file__).resolve().parent
cost, out = map(lambda p: pathlib.Path(p).resolve(), sys.argv[1:3])
out.mkdir(parents=True, exist_ok=False)
base = json.loads((record / "baseline.json").read_text())["production_commit"]
unchanged = {}
for name in ["crates/hyperdrc/src/checks/layer.rs", "crates/hyperdrc/src/kicad/model.rs"]:
    before = subprocess.check_output(["git", "show", base + ":" + name])
    current = pathlib.Path(name).read_bytes()
    assert current == before
    unchanged[name] = hashlib.sha256(current).hexdigest()
sources = json.loads((record / "validation/sources.json").read_text())["source_hashes"]
verify = lambda: all(hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest() == digest for name,digest in sources.items())
assert verify()
archive = pathlib.Path("crates/hyperdrc/docs/CPArti FPGA dev board.zip")
board = out / "CPArti FPGA dev board.kicad_pcb"
with zipfile.ZipFile(archive) as package:
    board.write_bytes(package.read(board.name))
artifacts = [json.loads(line) for line in (cost / "library-build.jsonl").read_text().splitlines()]
binary = out / "order-witness"
command = ["rustc", "--edition=2024", "-O", str(record / "fixture_order_witness.rs"), "-L", "dependency=target/release/deps"]
for library in ["hyperdrc", "hyperlimit", "serde_json"]:
    rlib = next(f for a in artifacts if a.get("reason")=="compiler-artifact" and a["target"]["name"]==library for f in a["filenames"] if f.endswith(".rlib"))
    command += ["--extern", library+"="+rlib]
command += ["-o",str(binary)]
with (out / "build.log").open("w") as log:
    subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
run = [str(binary),str(board)]
with (out / "witness.json").open("w") as stdout, (out / "run.log").open("w") as stderr:
    subprocess.run(run,stdout=stdout,stderr=stderr,check=True)
witness = json.loads((out / "witness.json").read_text())
original_argv = sys.argv
try:
    sys.argv = [str(record / "audit_fixture_reports.py"),str(cost)]
    context = runpy.run_path(sys.argv[0])
finally:
    sys.argv = original_argv


def remove_duplicate_layer(text):
    parts = re.split(r"(?=^layer-sanity |^geometry-uncertainty |^minimum-copper-neck-width |^drill-spacing )",text,flags=re.M)
    selected = [part for part in parts if "layers appear to contain duplicate geometry" in part]
    assert len(selected)==1
    block = selected[0]
    names = re.findall(r"\.kicad_pcb:(In1\.Cu|B\.Cu)",block.splitlines()[0])
    points = json.loads(next(line.removeprefix("   locations: ") for line in block.splitlines() if line.startswith("   locations: ")))
    matching = next(row for row in witness["input_order_variants"] if row["layers"]==names)
    assert matching["locations"]==points
    return "".join(part for part in parts if part != block), {"layers":names,"locations":points}


before, before_variant = remove_duplicate_layer(context["old_findings"])
after, after_variant = remove_duplicate_layer(context["new_findings"])
assert before==after
assert verify()
manifest = dict(
    base_revision=base, unchanged_production_files=unchanged,
    source_hashes=sources, source_unchanged=True,
    archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    board_sha256=hashlib.sha256(board.read_bytes()).hexdigest(),
    build_command=command, run_command=run,
    witness_source_sha256=hashlib.sha256((record / "fixture_order_witness.rs").read_bytes()).hexdigest(),
    previous_variant=before_variant,candidate_variant=after_variant,
    all_other_rendered_findings_unchanged_after_path_id_row_normalization=True,
    duplicate_geometry_unchanged=True,
    explanation="BoardModel::copper_layers iterates a HashMap; duplicate-layer checks retain input order and take up to eight optional markers from the left layer's authored profiles. Reversing this full-board pair reproduces both logged layer-order/marker variants. Those production source files are unchanged from the base revision. Both native regions have area 10000 and empty symmetric difference; both orderings emit identical finding geometry/message. This establishes the observed variation, not deterministic whole-report ordering or ID identity.",
)
(out / "manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
print("Both full-board layer/marker variants reproduced; every other normalized finding is unchanged.")
