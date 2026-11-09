"""Experimental native corpus runner; all source-fidelity claims remain open."""
import concurrent.futures
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("reference_collector", HERE / "reference.py")
COLLECTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COLLECTOR)
MANIFEST = None
MANIFEST_PATH = None
REFERENCE_RESULTS = None
BUILD_RECEIPT = None
WORKER = None
OUT = None
CASE_SECONDS = 300
WORKERS = 2


digest = COLLECTOR.digest
save = COLLECTOR.save


def classify(receipt, source):
    if not receipt.exists():
        return {"status": "native_receipt_missing"}
    try:
        value = json.loads(receipt.read_text())
        if (type(value["protocol_version"]) is not int or value["protocol_version"] != 1 or value["policy"] != "STRICT"
                or value["source"] != str(source) or not isinstance(value["result"], dict)):
            raise ValueError("invalid protocol envelope")
        result = value["result"]
        if not isinstance(result.get("native", {}), dict):
            raise ValueError("invalid native observations")
        comparison = result.get("comparison")
        if isinstance(comparison, dict) and comparison.get("status") in (
                "represented_sets_equal", "represented_sets_different"):
            if value.get("curve_principal_certainty") != "Certified":
                raise ValueError("missing or noncertified operation receipt")
            dimension = comparison.get("dimension")
            if type(dimension) is not int or dimension not in (2, 3):
                raise ValueError("missing comparison dimension")
            if dimension == 3 and any(comparison.get(key) != "Certified" for key in (
                    "principal_certainty", "continuation_certainty", "continuation_identity_certainty")):
                raise ValueError("missing or noncertified mesh operation receipt")
            if "reference_import_certainty" in comparison and comparison["reference_import_certainty"] != "Certified":
                raise ValueError("noncertified import receipt")
            if comparison.get("continuation_identity_empty") is not True:
                return {"status": "native_continuation_identity_failed", "receipt": value}
            status = comparison["status"]
        elif isinstance(comparison, dict) and comparison.get("status") == "failed":
            status = "comparison_failed"
        elif result.get("status") == "failed":
            status = "native_or_import_failed"
        elif result.get("status") in ("native_failed", "native_mixed_dimensions",
                                       "native_observed_without_reference"):
            status = result["status"]
        else:
            raise ValueError("unknown or incomplete result protocol")
        return {"status": status, "receipt": value,
                "native_diagnostics_present": bool(result.get("native", {}).get("diagnostics")),
                "source_conformance": "unestablished; emitted-model comparison only"}
    except (ValueError, KeyError, TypeError) as error:
        return {"status": "native_receipt_invalid", "error": repr(error)}


def invoke(source, reference, folder, seconds=None, worker=None):
    seconds = CASE_SECONDS if seconds is None else seconds
    worker = WORKER if worker is None else worker
    receipt = folder / "receipt.json"
    log = folder / "native.log"
    command = [str(worker), str(source), str(reference) if reference else "-", str(receipt)]
    started = time.time()
    bounded = False
    with log.open("wb") as stream:
        proc = subprocess.Popen(COLLECTOR.process_command(command), cwd=source.parent,
                                env=dict(os.environ, **COLLECTOR.ENVIRONMENT),
                                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        with COLLECTOR.LOCK:
            COLLECTOR.ACTIVE.add(proc)
        save(folder / "process.json", {"pid": proc.pid, "started": started,
                                       "command": command, "launch_command": COLLECTOR.process_command(command),
                                       "external_address_space_bytes": COLLECTOR.ADDRESS_SPACE_BYTES,
                                       "cwd": str(source.parent)})
        try:
            proc.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            bounded = True
            COLLECTOR.terminate(proc)
        finally:
            with COLLECTOR.LOCK:
                COLLECTOR.ACTIVE.discard(proc)
    if bounded:
        result = {"status": "native_external_containment"}
    elif proc.returncode != 0:
        result = {"status": "native_process_failed"}
    else:
        result = classify(receipt, source)
    result.update(exit_code=proc.returncode, external_containment_reached=bounded,
                  external_address_space_bytes=COLLECTOR.ADDRESS_SPACE_BYTES,
                  elapsed_seconds=time.time() - started, log_sha256=digest(log))
    if receipt.exists():
        result["receipt_sha256"] = digest(receipt)
    phase = receipt.with_suffix(".phase.json")
    if phase.exists():
        result["last_phase_receipt"] = phase.read_text()
    return result


def case(item, reference_row):
    folder = OUT / item["path"]
    folder.mkdir(parents=True, exist_ok=False)
    row = dict(item, reference_status=reference_row["audited_status"],
               reference_diagnostics_present=reference_row.get("diagnostics_present"))
    source = Path(MANIFEST["root"]) / item["path"]
    try:
        if digest(source) != item["sha256"]:
            raise ValueError("source differs from original pinned manifest")
        if COLLECTOR.STOP.is_set():
            raise RuntimeError("campaign cancelled before invocation")
        reference = reference_row.get("reference") if reference_row["audited_status"] == "reference_emitted_model" else None
        if reference_row["audited_status"] == "reference_reported_empty":
            reference = folder / "reference.empty"
            reference.write_text("D03_EXPLICITLY_ADOPTED_EMPTY_REFERENCE_MODEL_V1\n")
        if reference:
            reference = Path(reference).resolve()
            reference_hash = digest(reference)
            if reference_row["audited_status"] == "reference_emitted_model":
                expected = [attempt["output_sha256"] for attempt in reference_row["attempts"]
                            if attempt.get("output") == str(reference)]
                if expected != [reference_hash]:
                    raise ValueError("reference bytes differ from producer receipt")
            row.update(adopted_reference=str(reference), adopted_reference_sha256=reference_hash)
        row.update(invoke(source, reference, folder))
        if reference:
            row["reference_unchanged"] = digest(reference) == reference_hash
            if not row["reference_unchanged"]:
                row["observed_status"] = row["status"]
                row["status"] = "reference_changed"
        row["source_unchanged"] = digest(source) == item["sha256"]
        if not row["source_unchanged"]:
            row["observed_status"] = row["status"]
            row["status"] = "source_changed"
    except Exception as error:
        row.update(status="collector_failure", error=repr(error))
    save(folder / "result.json", row)
    return row


def main():
    OUT.mkdir(exist_ok=False)
    raw = json.loads(REFERENCE_RESULTS.read_text())["rows"]
    references = {x["path"]: dict(x, audited_status=x["status"]) for x in raw}
    if set(references) != {x["path"] for x in MANIFEST["files"]} or len(raw) != len(references):
        raise ValueError("every native case requires its original reference outcome")
    if any(references[x["path"]]["sha256"] != x["sha256"] for x in MANIFEST["files"]):
        raise ValueError("native and reference source hashes differ")
    signal.signal(signal.SIGTERM, COLLECTOR.stop)
    signal.signal(signal.SIGINT, COLLECTOR.stop)
    save(OUT / "start.json", {"pid": os.getpid(), "started": time.time(),
         "manifest_sha256": digest(MANIFEST_PATH), "worker_sha256": digest(WORKER),
         "declared_build_receipt": str(BUILD_RECEIPT) if BUILD_RECEIPT else None,
         "declared_build_receipt_sha256": digest(BUILD_RECEIPT) if BUILD_RECEIPT else None,
         "worker_source_attribution": "Not inferred from runtime source; inspect the supplied build receipt and its validation separately",
         "workers": WORKERS, "external_case_wall_seconds": CASE_SECONDS,
         "external_address_space_bytes_per_process": COLLECTOR.ADDRESS_SPACE_BYTES,
         "heap_or_total_work_bound": None,
         "qualification": "Exact represented-model comparison only. Diagnostics and construction evidence remain qualified; source conformance is unestablished."})
    rows = []
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
            pending = [pool.submit(case, item, references[item["path"]]) for item in MANIFEST["files"]]
            for future in concurrent.futures.as_completed(pending):
                row = future.result()
                rows.append(row)
                with (OUT / "progress.jsonl").open("a") as stream:
                    stream.write(json.dumps(row) + "\n")
                print(json.dumps({"completed": len(rows), "path": row["path"],
                                  "status": row["status"]}), flush=True)
        rows.sort(key=lambda item: item["path"])
        save(OUT / "results.json", {"count": len(rows), "rows": rows,
                                   "cancelled": COLLECTOR.STOP.is_set()})
    finally:
        COLLECTOR.cleanup()


if __name__ == "__main__":
    raise SystemExit("Use run.py to configure the pinned corpus and tool paths")
