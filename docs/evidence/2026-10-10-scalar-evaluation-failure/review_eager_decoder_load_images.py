"""Identify every changed loaded byte in the seven eager-decoder executables."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import runpy
import struct

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
digest = lambda data: hashlib.sha256(data).hexdigest()


def program_headers(data):
    offset = struct.unpack_from("<Q", data, 32)[0]
    width, count = struct.unpack_from("<HH", data, 54)
    assert width == 56
    return [list(struct.unpack_from("<IIQQQQQQ", data, offset + i * width)) for i in range(count)]


def normalized_symbols(sections):
    strings = sections[".strtab"]["payload"]
    rows = []
    for index, info, other, section, value, size in struct.iter_unpack("<IBBHQQ", sections[".symtab"]["payload"]):
        name = strings[index:].split(b"\0", 1)[0]
        name = re.sub(rb"\.llvm\.\d+$", b".llvm.MODULE", name).decode()
        rows.append([name, info, other, section, value, size])
    return rows


def referenced_text(sections, address, length):
    matches = []
    for row in sections.values():
        start = row["header"][3]
        if row["header"][2] & 2 and start <= address and address + length <= start + len(row["payload"]):
            matches.append(row["payload"][address - start:address - start + length].decode())
    assert len(matches) == 1
    return matches[0]


def main():
    output = E / "eager-bound-decoder-load-image-review-v2263.json"
    assert not output.exists()
    parser_path = E / "inspect_bound_decoder_executables.py"
    elf = runpy.run_path(str(parser_path))["elf"]
    protocol_path = E / "eager-bound-decoder-binary-comparison-protocol-v2262.json"
    protocol = read(protocol_path)
    comparison_path = E / "eager-bound-decoder-binary-comparison-v2262/summary.json"
    comparison = read(comparison_path)
    assert comparison["protocol_sha256"] == sha(protocol_path)
    expected = comparison["source_record_sha256"]
    source_record = E / protocol["source_record"]
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    baselines = {r["family"]: r for r in protocol["build_commands"]}
    name = "crates/hyperreal/src/computable/node/representation.rs"
    retained_source = E / "eager-bound-decoder-baseline-v2257/retained" / name
    old_lines = retained_source.read_text().splitlines()
    new_lines = (SOURCE / name).read_text().splitlines()
    rows = []
    for item in comparison["results"]:
        paths = {"baseline": Path(baselines[item["family"]]["baseline_binary"]), "candidate": Path(item["binary"])}
        data = {which: path.read_bytes() for which, path in paths.items()}
        assert digest(data["baseline"]) == item["baseline_binary_sha256"]
        assert digest(data["candidate"]) == item["binary_sha256"]
        a, b = [elf(data[which])[0] for which in ["baseline", "candidate"]]
        assert a.keys() == b.keys()
        headers = {which: program_headers(value) for which, value in data.items()}
        assert headers["baseline"] == headers["candidate"]
        changed_sections = [n for n in a if a[n]["payload"] != b[n]["payload"]]
        assert set(changed_sections) == {".note.gnu.build-id", ".data.rel.ro", ".symtab", ".strtab"}
        symbols_a, symbols_b = normalized_symbols(a), normalized_symbols(b)
        assert symbols_a == symbols_b
        normalize = lambda value: re.sub(rb"\.llvm\.\d+$", b".llvm.MODULE", value)
        assert {normalize(s) for s in a[".strtab"]["payload"].split(b"\0")} == {normalize(s) for s in b[".strtab"]["payload"].split(b"\0")}
        relocations = {offset: (info, addend) for offset, info, addend in struct.iter_unpack("<QQq", a[".rela.dyn"]["payload"])}
        before, after = a[".data.rel.ro"], b[".data.rel.ro"]
        assert before["header"] == after["header"]
        differences = [i for i, (x, y) in enumerate(zip(before["payload"], after["payload"])) if x != y]
        assert len(differences) == 6
        records, covered = [], set()
        for position in differences:
            cell = position // 8 * 8
            start = cell - 16
            old = struct.unpack_from("<QQII", before["payload"], start)
            new = struct.unpack_from("<QQII", after["payload"], start)
            assert old[:2] == new[:2] and old[3] == new[3] and new[2] == old[2] + 14
            location = before["header"][3] + start
            relocation = relocations[location]
            assert relocation[0] == 8 and old[0] == 0
            filename = referenced_text(a, relocation[1], old[1])
            assert filename == name
            assert old_lines[old[2] - 1] == new_lines[new[2] - 1]
            records.append({"address": location, "filename": filename, "baseline_line": old[2],
                "candidate_line": new[2], "column": old[3], "source_text": old_lines[old[2] - 1],
                "relative_relocation": list(relocation), "line_field_file_offset": before["header"][4] + cell})
            covered.update(range(before["header"][4] + cell, before["header"][4] + cell + 4))
        note = a[".note.gnu.build-id"]
        assert note["header"] == b[".note.gnu.build-id"]["header"]
        allowed = covered | set(range(note["header"][4], note["header"][4] + len(note["payload"]))) | set(range(40, 48))
        segments = []
        for header in headers["baseline"]:
            if header[0] != 1:
                continue
            offset, size = header[2], header[5]
            old, new = data["baseline"][offset:offset + size], data["candidate"][offset:offset + size]
            changed = [offset + i for i, (x, y) in enumerate(zip(old, new)) if x != y]
            assert all(i in allowed for i in changed)
            if header[1] & 1:
                assert old == new
            segments.append({"program_header": header, "baseline_sha256": digest(old), "candidate_sha256": digest(new),
                "changed_file_offsets": changed, "executable": bool(header[1] & 1)})
        executable_sections = {n: digest(r["payload"]) for n, r in a.items() if r["header"][2] & 4}
        assert executable_sections == {n: digest(r["payload"]) for n, r in b.items() if r["header"][2] & 4}
        rows.append({"family": item["family"], "binary_sha256": {k: digest(v) for k, v in data.items()},
            "binary_size_delta": len(data["candidate"]) - len(data["baseline"]),
            "changed_sections": changed_sections, "identical_executable_sections_sha256": executable_sections,
            "load_segments": segments, "source_location_records": records,
            "symbol_count": len(symbols_a), "normalized_symbol_rows_identical": True,
            "symbol_normalization": "Only trailing .llvm.<decimal-module-id> suffixes and name-table offsets",
            "section_table_offsets": {k: struct.unpack_from("<Q", v, 40)[0] for k, v in data.items()}})
        assert all(sha(path) == digest(data[which]) for which, path in paths.items())
    assert len(rows) == 7
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    output.write_text(json.dumps({"status": "Instruction equality with explicitly identified metadata differences; full executable identity still fails",
        "source_record_sha256": expected, "comparison_record": str(comparison_path.relative_to(E)),
        "comparison_record_sha256": sha(comparison_path), "parser": parser_path.name, "parser_sha256": sha(parser_path),
        "runner_sha256": sha(Path(__file__)), "retained_source_sha256": sha(retained_source),
        "candidate_source_sha256": sha(SOURCE / name), "families": rows,
        "findings": [
            "All executable sections and complete executable load segments are byte-identical in all seven probes. The lazy candidate's 98 changed instruction bytes are absent.",
            "Program headers are identical. Changed loaded bytes are confined to the ELF section-table-offset field, build-id note and six relocated source-location-shaped records. Each record points to the same source filename and unchanged statement/column, with its line increased by fourteen. These statements are five rational-constant unwraps and the invalid exact-sign-cache panic.",
            "Symbol rows match after removing only private LLVM module-id suffixes and decoding name-table offsets; normalized string-name sets also match. Complete file sizes fall by 24 or 32 bytes. No executable was rewritten or normalized for execution.",
        ],
        "qualification": {"full_executable_identity": False, "reused_cost_receipts": 0,
            "new_workload_processes": 0, "cost_gate_passed": False, "runtime_retained": False},
        "limits": "This is static byte and source-location evidence, not a whole-program equivalence theorem or a fresh timing/resource pass. The exact-identity condition in the frozen protocol remains unsatisfied. The earlier baseline CPU failure and unresolved timing increases are preserved. Any further cost/admission method needs a separately frozen protocol; no fixture, cap or failed result is silently replaced.",
        "source_files_verified": len(sources), "main_files_verified": len(main_guard)}, indent=2) + "\n")
    print(json.dumps({"families": 7, "executable_load_segments_identical": True,
                      "source_location_records_per_family": 6, "cost_receipts_reused": 0}))


if __name__ == "__main__":
    main()
