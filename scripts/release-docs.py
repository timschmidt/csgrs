#!/usr/bin/env python3
"""Build local workspace docs with the release's declared docs.rs options.

This checks the current metadata keys, not docs.rs's hosted target/toolchain
matrix. Unhandled keys fail instead of silently weakening the qualification.
"""

import argparse
import json
import os
import subprocess


SUPPORTED = {"all-features", "no-default-features", "features", "rustdoc-args"}


def documentation_plan(metadata, names):
    packages = {package["name"]: package for package in metadata["packages"]}
    missing = set(names) - packages.keys()
    if missing:
        raise ValueError(f"documentation packages missing: {sorted(missing)}")
    plan = []
    for name in names:
        docs = ((packages[name].get("metadata") or {}).get("docs") or {}).get("rs") or {}
        unsupported = docs.keys() - SUPPORTED
        if unsupported:
            raise ValueError(f"{name}: unsupported docs.rs keys: {sorted(unsupported)}")
        args = ["-p", name]
        for key in ("all-features", "no-default-features"):
            value = docs.get(key, False)
            if not isinstance(value, bool):
                raise ValueError(f"{name}: {key} must be a boolean")
            if value:
                args.append("--" + key)
        for key in ("features", "rustdoc-args"):
            value = docs.get(key, [])
            if not isinstance(value, list) or any(
                not isinstance(item, str) or not item or "\x00" in item or "\x1f" in item
                for item in value
            ):
                raise ValueError(f"{name}: {key} must be an array of nonempty arguments")
        if docs.get("features"):
            args += ["--features", ",".join(docs["features"])]
        # Encoded flags preserve whitespace within individual manifest arguments.
        # Override both ambient forms, just as the release owns its warning gate.
        flags = [*docs.get("rustdoc-args", []), "-D", "warnings"]
        plan.append((args, flags))
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="")
    parser.add_argument("--target-dir", required=True)
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("packages", nargs="+")
    options = parser.parse_args()
    cargo = ["cargo"] + ([options.toolchain] if options.toolchain else [])
    config = [arg for value in options.config for arg in ("--config", value)]
    metadata = json.loads(subprocess.check_output(
        cargo + ["metadata", "--locked", "--no-deps", "--format-version", "1", *config]
    ))
    # Validate the entire requested set before building any of it.
    plan = documentation_plan(metadata, options.packages)
    for args, flags in plan:
        command = cargo + ["doc", "--locked", "--no-deps", *config, *args]
        print(json.dumps({"command": command, "rustdoc_args": flags}), flush=True)
        env = dict(os.environ, CARGO_TARGET_DIR=options.target_dir,
                   RUSTDOCFLAGS="", CARGO_ENCODED_RUSTDOCFLAGS="\x1f".join(flags))
        subprocess.run(command, env=env, check=True)
    print(f"local documentation configurations passed: {len(plan)}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error)) from error
