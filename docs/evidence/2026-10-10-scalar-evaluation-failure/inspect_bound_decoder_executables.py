"""Archive static ELF and disassembly differences without executing the probes."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
digest = lambda data: hashlib.sha256(data).hexdigest()


def elf(data):
    assert data[:6] == b"\x7fELF\x02\x01"
    offset = struct.unpack_from("<Q", data, 40)[0]
    width, count, names_index = struct.unpack_from("<HHH", data, 58)
    assert width == 64 and names_index < count
    raw = [struct.unpack_from("<IIQQQQIIQQ", data, offset + i * width) for i in range(count)]
    names = raw[names_index]
    strings = data[names[4]:names[4] + names[5]]
    sections = {}
    for row in raw:
        name = strings[row[0]:].split(b"\0", 1)[0].decode()
        payload = data[row[4]:row[4] + row[5]] if row[1] != 8 else b""
        assert row[1] == 8 or len(payload) == row[5]
        assert name not in sections
        sections[name] = {"header": list(row), "payload": payload}
    symbols = sections[".symtab"]["header"]
    names = raw[symbols[6]]
    strings = data[names[4]:names[4] + names[5]]
    functions = []
    assert symbols[9] == 24 and symbols[5] % 24 == 0
    for position in range(symbols[4], symbols[4] + symbols[5], 24):
        name, info, other, section, value, length = struct.unpack_from("<IBBHQQ", data, position)
        if info & 15 == 2 and length:
            functions.append({"address": value, "size": length,
                "name": strings[name:].split(b"\0", 1)[0].decode()})
    return sections, functions


def main():
    output = E / "bound-decoder-static-differences-v2256"
    assert not output.exists()
    comparison_path = E / "bound-decoder-binary-comparison-v2181/summary.json"
    comparison = read(comparison_path)
    protocol_path = E / comparison["protocol"]
    assert sha(protocol_path) == comparison["protocol_sha256"]
    protocol = read(protocol_path)
    baseline = {row["family"]: row for row in protocol["build_commands"]}
    restoration_path = E / "decoder-retained-restoration-review-v2255.json"
    restoration = read(restoration_path)
    sources = json.loads(gzip.decompress((E / restoration["source_record"]).read_bytes()))
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    tool = Path(shutil.which("objdump"))
    tool_hash = sha(tool)
    output.mkdir()
    rows, commands = [], []
    for item in comparison["results"]:
        family = item["family"]
        paths = {"baseline": Path(baseline[family]["baseline_binary"]), "candidate": Path(item["binary"])}
        data = {which: path.read_bytes() for which, path in paths.items()}
        assert digest(data["baseline"]) == baseline[family]["baseline_binary_sha256"] == item["baseline_binary_sha256"]
        assert digest(data["candidate"]) == item["binary_sha256"]
        parsed = {which: elf(value) for which, value in data.items()}
        old, new = parsed["baseline"][0], parsed["candidate"][0]
        assert old.keys() == new.keys()
        sections = []
        for name in old:
            before, after = old[name], new[name]
            a, b = before["payload"], after["payload"]
            sections.append({"name": name, "baseline_header": before["header"], "candidate_header": after["header"],
                "baseline_sha256": digest(a), "candidate_sha256": digest(b), "identical_payload": a == b,
                "different_byte_positions": sum(x != y for x, y in zip(a, b)) + abs(len(a) - len(b)),
                "allocated": bool(before["header"][2] & 2), "executable": bool(before["header"][2] & 4)})
        text_before, text_after = old[".text"], new[".text"]
        assert text_before["header"][3:6] == text_after["header"][3:6]
        address = text_before["header"][3]
        offsets = {address + i for i, (a, b) in enumerate(zip(text_before["payload"], text_after["payload"])) if a != b}
        assert len(offsets) == 98
        changed = {}
        for which, (_, functions) in parsed.items():
            selected, accounted = [], set()
            for function in functions:
                start, size = function["address"], function["size"]
                hits = sorted(i for i in offsets if start <= i < start + size)
                if not hits:
                    continue
                accounted.update(hits)
                selected.append(function | {"different_instruction_byte_addresses": hits})
                name = f"{family}-{which}-function-{len(selected)}"
                command = [str(tool), "--demangle=rust", "-d", f"--start-address={start}",
                           f"--stop-address={start + size}", str(paths[which])]
                result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=30)
                (output / (name + ".stdout.log")).write_bytes(result.stdout)
                (output / (name + ".stderr.log")).write_bytes(result.stderr)
                commands.append({"name": name, "command": command, "exit_code": result.returncode})
                assert result.returncode == 0 and result.stderr == b""
            assert accounted == offsets and len(selected) == 4
            changed[which] = selected
        assert changed["baseline"] == changed["candidate"]
        rows.append({"family": family, "binaries": {k: str(p) for k, p in paths.items()},
            "binary_sha256": {k: digest(v) for k, v in data.items()},
            "binary_bytes": {k: len(v) for k, v in data.items()}, "sections": sections,
            "changed_functions": changed["baseline"], "all_changed_text_bytes_accounted_for": True})
        assert all(sha(path) == digest(data[which]) for which, path in paths.items())
    assert len(rows) == 7 and sha(tool) == tool_hash
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    (output / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    record = {"status": "Static comparison of the archived retained and rejected decoder executables",
        "runner_sha256": sha(Path(__file__)), "objdump": str(tool), "objdump_sha256": tool_hash,
        "binary_comparison": str(comparison_path.relative_to(E)), "binary_comparison_sha256": sha(comparison_path),
        "restoration_record": restoration_path.name, "restoration_record_sha256": sha(restoration_path),
        "current_source_record_sha256": restoration["source_record_sha256"],
        "rejected_source_record_sha256": comparison["source_record_sha256"], "families": rows,
        "findings": [
            "All seven complete executables retain their byte length. Each has 98 changed .text byte positions, all contained in certify_required_domain, publish_bound, cheap_bound and cheap_bound_shallow. The corresponding functions retain their addresses and sizes within each pair.",
            "The four complete function disassemblies from both executables in every family are archived. In publish_bound, the optional magnitude decode changes shift/mask scheduling: the baseline shifts the payload before extracting presence, while the candidate masks presence first. This is a local code-generation observation, not a measured causal explanation.",
            "Exception-table, relocation-data, build-id and symbol/string-table payloads also differ. Equal executable sizes or equal function boundaries do not imply identical programs or permission to reuse the failed cost gate.",
        ],
        "next_experiment": "Evaluate the magnitude payload before selecting Some/None, matching the original eager then_some argument evaluation while keeping the verifier-supported branch. First check the actual decoder proof and generated code; freeze any new candidate and its unchanged correctness/resource gates before runtime measurements.",
        "new_workload_processes": 0, "new_formal_units": 0, "runtime_retained": False,
        "limits": "No probe workload was executed, no timings were repeated and no equivalence theorem is claimed. The original baseline CPU failure and four unresolved timing increases remain failed/incomplete evidence. The restored v1836 source and all property/target statuses are unchanged.",
        "source_files_verified": len(sources), "main_files_verified": len(main_guard),
        "artifacts_sha256": {p.name: sha(p) for p in sorted(output.iterdir()) if p.is_file()}}
    (output / "summary.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"families": 7, "changed_text_bytes_per_family": 98,
                      "changed_functions_per_family": 4, "new_workload_processes": 0}))


if __name__ == "__main__":
    main()
