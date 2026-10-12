"""Trace direct references to changed diagnostic records without running workloads."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import runpy
import shutil
import struct
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
digest = lambda data: hashlib.sha256(data).hexdigest()


def main():
    out = E / "eager-bound-decoder-location-uses-v2277"
    protocol_path = E / "eager-bound-decoder-location-use-protocol-v2277.json"
    assert not out.exists() and not protocol_path.exists()
    review_path = E / "eager-bound-decoder-load-image-review-v2263.json"
    review = read(review_path)
    comparison = read(E / review["comparison_record"])
    assert sha(E / review["comparison_record"]) == review["comparison_record_sha256"]
    comparison_protocol = read(E / "eager-bound-decoder-binary-comparison-protocol-v2262.json")
    sources = json.loads(gzip.decompress((E / comparison_protocol["source_record"]).read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / name) == value for name, value in sources.items())
    assert all(sha(ROOT / name) == value for name, value in guard.items())
    parser_path = E / review["parser"]
    assert sha(parser_path) == review["parser_sha256"]
    elf = runpy.run_path(str(parser_path))["elf"]
    objdump = Path(shutil.which("objdump"))
    tool_hash = sha(objdump)
    baseline = {r["family"]: r for r in comparison_protocol["build_commands"]}
    candidates = {r["family"]: r for r in comparison["results"]}
    protocol_path.write_text(json.dumps({
        "status": "Frozen before the seven-family diagnostic-reference audit",
        "source_record_sha256": review["source_record_sha256"],
        "runner_sha256": sha(Path(__file__)),
        "load_image_review": review_path.name, "load_image_review_sha256": sha(review_path),
        "objdump": str(objdump), "objdump_sha256": tool_hash,
        "known_before_execution": "A preliminary atan disassembly in /tmp identified direct record-address loads before panic_fmt and unwrap_failed calls. This audit checks all seven probes and preserves the complete disassemblies.",
        "checks": [
            "Verify frozen whole-binary hashes and identical executable section payloads before using one disassembly per pair.",
            "Collect every objdump RIP-relative reference into each changed 24-byte record; require an address load of the record base into the expected panic-location argument register.",
            "Follow each reference through a short straight-line instruction sequence to a call; resolve its GOT slot through the unchanged RELATIVE relocation and function symbol, requiring panic_fmt or unwrap_failed.",
            "Reject intervening branches or writes to the location register and reject data relocations pointing into the changed records.",
            "Archive source excerpts for the exact-sign decoding, rational constants and fraction constructor. Do not infer whole-program dataflow or absence of possible panics from reference scanning.",
        ],
        "cost_qualification": "None. The frozen full-file identity condition is still false; no timing receipt is reused and no runtime is retained by this audit.",
        "limits": "This is compiler-output/source inspection, not formal noninterference, exhaustive indirect-pointer analysis, a cost measurement or an absolute resource guarantee. Panic diagnostics may show different line numbers. All earlier failed gates remain failed.",
    }, indent=2) + "\n")
    out.mkdir()
    rows = []
    for family in review["families"]:
        name = family["family"]
        old_path = Path(baseline[name]["baseline_binary"])
        new_path = Path(candidates[name]["binary"])
        assert sha(old_path) == family["binary_sha256"]["baseline"]
        assert sha(new_path) == family["binary_sha256"]["candidate"]
        sections, functions = elf(old_path.read_bytes())
        other, _ = elf(new_path.read_bytes())
        for section, record in sections.items():
            if record["header"][2] & 4:
                assert record["payload"] == other[section]["payload"]
        relocations = {r[0]: r for r in struct.iter_unpack("<QQq", sections[".rela.dyn"]["payload"])}
        assert sections[".rela.dyn"]["payload"] == other[".rela.dyn"]["payload"]
        records = {r["address"]: r for r in family["source_location_records"]}
        within_record = lambda address: next((a for a in records if a <= address < a + 24), None)
        data_refs = [r for r in relocations.values() if within_record(r[2]) is not None]
        assert not data_refs
        command = [str(objdump), "--demangle=rust", "--no-show-raw-insn", "-d", str(old_path)]
        result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=30)
        assert result.returncode == 0 and result.stderr == b""
        archive = out / f"{name}.objdump.stdout.gz"
        archive.write_bytes(gzip.compress(result.stdout, mtime=0))
        lines = result.stdout.decode().splitlines()
        instructions, function = [], None
        for line in lines:
            label = re.match(r"^([0-9a-f]+) <(.*)>:$", line)
            if label:
                function = label[2]
            instruction = re.match(r"^\s*([0-9a-f]+):\s+(\S+)(.*)$", line)
            if instruction:
                instructions.append((int(instruction[1], 16), instruction[2], instruction[3], function, line))
        refs = []
        for index, instruction in enumerate(instructions):
            address, opcode, operands, function, line = instruction
            target = re.search(r"#\s*([0-9a-f]+)\b", operands)
            if target is None or within_record(int(target[1], 16)) is None:
                continue
            location = int(target[1], 16)
            assert location in records and opcode == "lea"
            record = records[location]
            invalid_cache = "unreachable!" in record["source_text"]
            register = "%rdx" if invalid_cache else "%r8"
            aliases = {"%rdx", "%edx", "%dx", "%dl", "%dh"} if invalid_cache else {"%r8", "%r8d", "%r8w", "%r8b"}
            assert operands.split("#")[0].strip().endswith("," + register)
            path = [line]
            for following in instructions[index + 1:index + 9]:
                _, operation, arguments, owner, raw = following
                assert owner == function
                path.append(raw)
                if operation == "call":
                    slot = re.search(r"#\s*([0-9a-f]+)\b", arguments)
                    assert slot is not None and arguments.strip().startswith("*")
                    relocation = relocations[int(slot[1], 16)]
                    assert relocation[1] == 8
                    symbols = [f["name"] for f in functions if f["address"] == relocation[2]]
                    expected = "4core9panicking9panic_fmt" if invalid_cache else "4core6result13unwrap_failed"
                    assert len(symbols) == 1 and expected in symbols[0]
                    break
                assert operation in {"mov", "lea"}
                destination = arguments.split("#")[0].strip().rsplit(",", 1)[-1]
                assert destination not in aliases
            else:
                raise AssertionError("No resolved diagnostic call after record reference")
            refs.append({"address": address, "function": function, "record_address": location,
                "baseline_line": record["baseline_line"], "candidate_line": record["candidate_line"],
                "location_register": register, "call_symbol": symbols[0], "call_relocation": relocation,
                "straight_line_instructions": path})
        assert {r["record_address"] for r in refs} == set(records)
        assert sha(old_path) == family["binary_sha256"]["baseline"]
        assert sha(new_path) == family["binary_sha256"]["candidate"]
        rows.append({"family": name, "binary_sha256": family["binary_sha256"],
            "command": command, "exit_code": result.returncode, "stdout_sha256": digest(result.stdout),
            "stdout_archive": archive.name, "stdout_archive_sha256": sha(archive),
            "stderr_sha256": digest(result.stderr), "direct_reference_count": len(refs),
            "direct_references": refs, "data_relocations_into_records": data_refs})
    excerpts = []
    for name, start, end in [
        ("crates/hyperreal/src/computable/node/representation.rs", 673, 706),
        ("crates/hyperreal/src/computable/node/representation.rs", 789, 808),
        ("crates/hyperreal/src/rational/arithmetic/construction.rs", 307, 350),
    ]:
        source = SOURCE / name
        excerpts.append({"path": name, "sha256": sha(source), "first_line": start, "last_line": end,
            "lines": source.read_text().splitlines()[start - 1:end]})
    assert len(rows) == 7 and sha(objdump) == tool_hash
    assert all(sha(SOURCE / name) == value for name, value in sources.items())
    assert all(sha(ROOT / name) == value for name, value in guard.items())
    (out / "summary.json").write_text(json.dumps({
        "status": "Every detected direct reference supplies diagnostic location metadata to a panic routine",
        "protocol": protocol_path.name, "protocol_sha256": sha(protocol_path),
        "source_record_sha256": review["source_record_sha256"],
        "families": rows, "source_excerpts": excerpts,
        "scope": "All seven complete disassemblies were parsed; direct RIP-relative references into the six records were traced to resolved panic routines. Only the archived instruction sequences and source excerpts support the use classification. This is not exhaustive dynamic dataflow or a whole-program equivalence theorem.",
        "diagnostic_change": "Panic reports at these six source statements may use line numbers increased by fourteen. The statements, columns, filenames, call instructions and call-target relocations are unchanged.",
        "cost_receipts_reused": 0, "workload_processes": 0, "runtime_retained": False,
        "source_files_verified": len(sources), "main_files_verified": len(guard),
    }, indent=2) + "\n")
    print(json.dumps({"families": len(rows), "direct_references": sum(r["direct_reference_count"] for r in rows),
                      "diagnostic_targets_only": True, "workload_processes": 0}))


if __name__ == "__main__":
    main()
