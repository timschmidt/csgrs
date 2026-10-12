"""Fork the retained runtime-result parser with only phase and record identities changed."""
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
old = E / "record_domain_predicate_validation.py"
new = E / "record_bound_merge_validation.py"
record = E / "bound-merge-validation-recorder-fork-v1849.json"
assert not new.exists() and not record.exists()
original = old.read_text()
mapping = {
    "typed-api-prototype-v1761-domain-predicate-verus": "typed-api-prototype-v1838-bound-merge-verus",
    "domain-predicate-candidate-validation-v1772.json": "bound-merge-candidate-validation-v1849.json",
    "domain-predicate-broad-validation-v1772.json": "bound-merge-broad-validation-v1849.json",
    "typed-api-prototype-v1699-domain-bound-stack-tests": "typed-api-prototype-v1769-domain-predicate-stack-tests",
}
for version, new_version, suffix in [
    (1764, 1841, "scalar-tests"), (1765, 1842, "clippy"),
    (1768, 1845, "included-fmt"), (1769, 1846, "stack-tests"),
    (1770, 1847, "default-tests"), (1771, 1848, "debug-tests"),
    (1772, 1849, "minimal-clippy"),
]:
    mapping[f"typed-api-prototype-v{version}-domain-predicate-{suffix}"] = f"typed-api-prototype-v{new_version}-bound-merge-{suffix}"
assert all(before in original for before in mapping)
pattern = "|".join(re.escape(value) for value in sorted(mapping, key=len, reverse=True))
updated = re.sub(pattern, lambda match: mapping[match[0]], original)
compile(updated, str(new), "exec")
new.write_text(updated)
record.write_text(json.dumps({"predecessor": old.name, "predecessor_sha256": sha(old),
    "fork": new.name, "fork_sha256": sha(new), "literal_replacements": mapping,
    "scope": "Keep the prior Cargo/nested-test parser, expected test counts, ignored-name comparison, inherited-warning comparison and strict Clippy/format checks unchanged."}, indent=2) + "\n")
print(json.dumps({"fork": new.name, "phase_and_record_replacements": len(mapping)}))
