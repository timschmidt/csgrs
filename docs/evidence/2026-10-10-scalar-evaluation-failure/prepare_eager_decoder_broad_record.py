"""Bind the existing broad-result parser to the eager decoder's unchanged commands."""
import ast
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    source = E / "record_bound_decoder_validation.py"
    target = E / "record_eager_decoder_validation.py"
    record = E / "eager-bound-decoder-broad-recorder-preparation-v2276.json"
    assert not target.exists() and not record.exists()
    replacements = {
        "typed-api-prototype-v2169-bound-decoder-verus": "typed-api-prototype-v2260-eager-bound-decoder-verus",
        "bound-decoder-candidate-validation-v2180.json": "eager-bound-decoder-candidate-validation-v2276.json",
        "bound-decoder-broad-validation-v2180.json": "eager-bound-decoder-broad-validation-v2276.json",
    }
    for old, new, suffix in [(2172, 2268, "scalar-tests"), (2173, 2269, "clippy"),
        (2176, 2259, "included-fmt"), (2177, 2273, "stack-tests"),
        (2178, 2274, "default-tests"), (2179, 2275, "debug-tests"), (2180, 2276, "minimal-clippy")]:
        replacements[f"typed-api-prototype-v{old}-bound-decoder-{suffix}"] = f"typed-api-prototype-v{new}-eager-bound-decoder-{suffix}"
    text = source.read_text()
    assert all(s in text for s in replacements)
    pattern = "|".join(re.escape(s) for s in sorted(replacements, key=len, reverse=True))
    text = re.sub(pattern, lambda match: replacements[match[0]], text)
    ast.parse(text)
    target.write_text(text)
    record.write_text(json.dumps({"predecessor": source.name, "predecessor_sha256": sha(source),
        "runner": target.name, "runner_sha256": sha(target), "literal_replacements": replacements,
        "scope": "Same terminal-result, test-count, child-count, inherited-warning, ignored-case and strict-Clippy checks; only phase and output identities differ. The nine current-source commands comprise three early executions and six remaining executions. This parser does not run tests or decide cost/retention."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "checks_unchanged": True}))


if __name__ == "__main__":
    main()
