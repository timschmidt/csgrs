"""Experimental complete-corpus reference collection. No geometric success claim.

Every pinned source is attempted. Containment is external, never kernel exhaustion.
Runs use the original source directory so relative includes/imports keep meaning.
"""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

HERE = Path(__file__).resolve().parent
MANIFEST = None
MANIFEST_PATH = None
OUT = None
BACKEND = None
CASE_SECONDS = 300
WORKERS = 2
ADDRESS_SPACE_BYTES = 4 * 1024 * 1024 * 1024
ENVIRONMENT = {}
ACTIVE = set()
LOCK = threading.Lock()
STOP = threading.Event()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def process_command(command):
    # Exec-side setup is safe when collectors launch from worker threads.
    # The hard limit is inherited by descendants, independently per process.
    limit = ADDRESS_SPACE_BYTES
    if not isinstance(limit, int) or isinstance(limit, bool) or limit <= 0:
        raise ValueError("a positive external address-space limit is required")
    return ["/usr/bin/prlimit", f"--as={limit}:{limit}", "--",
            "/usr/bin/time", "-v", *command]


def terminate(proc):
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        except ProcessLookupError:
            proc.wait()


def attempt(source, folder, suffix, remaining):
    output = folder / ("reference." + suffix)
    summary = folder / (suffix + "-summary.json")
    log = folder / (suffix + ".log")
    command = [str(BACKEND), "--backend=Manifold", "--summary", "all",
               "--summary-file", str(summary), "-o", str(output), str(source)]
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", **ENVIRONMENT)
    started = time.time()
    bounded = False
    with log.open("wb") as stream:
        proc = subprocess.Popen(process_command(command),
                                cwd=source.parent, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)
        with LOCK:
            ACTIVE.add(proc)
        save(folder / (suffix + "-process.json"), {"pid": proc.pid, "started": started,
             "command": command, "launch_command": process_command(command),
             "external_address_space_bytes": ADDRESS_SPACE_BYTES, "cwd": str(source.parent)})
        try:
            proc.wait(timeout=max(0.01, remaining))
        except subprocess.TimeoutExpired:
            bounded = True
            terminate(proc)
        finally:
            with LOCK:
                ACTIVE.discard(proc)
    text = log.read_text(errors="replace")
    result = {"command": command, "exit_code": proc.returncode,
              "external_address_space_bytes": ADDRESS_SPACE_BYTES,
              "elapsed_seconds": time.time() - started,
              "external_containment_reached": bounded, "log_sha256": digest(log),
              "output": str(output) if output.exists() else None,
              "output_sha256": digest(output) if output.exists() else None,
              "summary": str(summary) if summary.exists() else None,
              "diagnostics_present": any(marker in text for marker in
                                         ("WARNING:", "ERROR:", "DEPRECATED:"))}
    save(folder / (suffix + "-result.json"), result)
    return result, text


def case(item):
    source = Path(MANIFEST["root"]) / item["path"]
    folder = OUT / item["path"]
    folder.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    row = dict(item, attempts=[])
    try:
        if digest(source) != item["sha256"]:
            raise ValueError("source hash differs from pinned manifest")
        if STOP.is_set():
            raise RuntimeError("campaign cancelled before attempt")
        obj, text = attempt(source, folder, "obj", CASE_SECONDS)
        row["attempts"].append(obj)
        if obj["external_containment_reached"]:
            row["status"] = "reference_external_containment"
        elif obj["exit_code"] == 0 and obj["output"]:
            row.update(status="reference_emitted_model", reference=obj["output"], dimension=3)
        elif "Current top level object is not a 3D object." in text.splitlines():
            svg, text = attempt(source, folder, "svg", CASE_SECONDS - (time.monotonic() - start))
            row["attempts"].append(svg)
            if svg["external_containment_reached"]:
                row["status"] = "reference_external_containment"
            elif svg["exit_code"] == 0 and svg["output"]:
                row.update(status="reference_emitted_model", reference=svg["output"], dimension=2)
            elif "Current top level object is empty." in text.splitlines():
                row["status"] = "reference_reported_empty"
            else:
                row["status"] = "reference_export_failed"
        elif "Current top level object is empty." in text.splitlines():
            # This is a pinned-producer observation, NOT independent proof that
            # the source denotes empty geometry. Diagnostics remain attached.
            row["status"] = "reference_reported_empty"
        else:
            row["status"] = "reference_export_failed"
        row["source_unchanged"] = digest(source) == item["sha256"]
        if not row["source_unchanged"]:
            row["observed_status"] = row["status"]
            row["status"] = "source_changed"
        row["diagnostics_present"] = any(x["diagnostics_present"] for x in row["attempts"])
    except Exception as error:
        row.update(status="collector_failure", error=repr(error))
    row["elapsed_seconds"] = time.monotonic() - start
    save(folder / "result.json", row)
    return row


def cleanup():
    with LOCK:
        processes = list(ACTIVE)
    for proc in processes:
        terminate(proc)


def stop(_signum, _frame):
    STOP.set()
    cleanup()


def main():
    OUT.mkdir(exist_ok=False)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    save(OUT / "start.json", {"pid": os.getpid(), "started": time.time(),
         "manifest_sha256": digest(MANIFEST_PATH), "workers": WORKERS,
         "external_case_wall_seconds": CASE_SECONDS,
         "external_address_space_bytes_per_process": ADDRESS_SPACE_BYTES,
         "heap_or_total_work_bound": None,
         "qualification": "Collection only; every reference export is a qualified emitted model. No source-equivalence or geometric-completion claim."})
    rows = []
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
            pending = [pool.submit(case, item) for item in MANIFEST["files"]]
            for future in concurrent.futures.as_completed(pending):
                row = future.result()
                rows.append(row)
                with (OUT / "progress.jsonl").open("a") as stream:
                    stream.write(json.dumps(row) + "\n")
                print(json.dumps({"completed": len(rows), "path": row["path"],
                                  "status": row["status"]}), flush=True)
        rows.sort(key=lambda item: item["path"])
        save(OUT / "results.json", {"count": len(rows), "rows": rows, "cancelled": STOP.is_set()})
    finally:
        cleanup()


if __name__ == "__main__":
    raise SystemExit("Use run.py to configure the pinned corpus and tool paths")
