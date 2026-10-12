"""Fork the captured validation workflow for the typed-word extension."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "validate_digit_scan_v2371.py"
    target = E / "validate_word_scan_v2384.py"
    record = E / "word-scan-validation-preparation-v2384.json"
    assert not target.exists() and not record.exists()
    text = parent.read_text()
    replacements = [
        ("Run digit-scan guards, acceptance and formatting against the final source.",
         "Run typed-word guards, acceptance and formatting against the final source."),
        ("a83cff31d79bb8bbbb5100829d14a5683ca081410433e95b32d22460a062a730",
         "67e3d14b5d7a319778d56b701498e97225c16134eca65bb14a7586c727f2fb5a"),
        ("typed-api-prototype-v2367-digit-scan-final-verus", "typed-api-prototype-v2380-word-scan-verus"),
        ("run_digit_scan_proof_checks_v2368.py", "run_word_scan_proof_checks_v2381.py"),
        ("digit-scan-validation-protocol-v2371.json", "word-scan-validation-protocol-v2384.json"),
        ("digit-scan-validation-v2371.json", "word-scan-validation-v2384.json"),
        ("typed-api-prototype-v2368-digit-scan-rejections", "typed-api-prototype-v2381-word-scan-rejections"),
        ("typed-api-prototype-v2369-digit-scan-acceptance", "typed-api-prototype-v2382-word-scan-acceptance"),
        ("typed-api-prototype-v2370-digit-scan-workspace-fmt", "typed-api-prototype-v2383-word-scan-workspace-fmt"),
        ("typed-api-prototype-v2371-digit-scan-included-fmt", "typed-api-prototype-v2384-word-scan-included-fmt"),
        ("typed-api-prototype-v2355-trailing-metadata-workspace-fmt", "typed-api-prototype-v2370-digit-scan-workspace-fmt"),
        ("typed-api-prototype-v2356-trailing-metadata-included-fmt", "typed-api-prototype-v2371-digit-scan-included-fmt"),
        ('extra = ["crates/hyperreal/verification/digit_scan_model.rs", "crates/hyperreal/verification/lib.rs"]',
         'extra = ["crates/hyperreal/verification/word_scan_model.rs"]'),
        ('["verified"] == 716', '["verified"] == 736'),
        ('["verified_model_theorems"]) == 160', '["verified_model_theorems"]) == 178'),
        ('"new_negative_controls": 19', '"new_negative_controls": 20'),
        ('["rejected_invocations"] == 19', '["rejected_invocations"] == 20'),
        ('["verus_invocations"] == 20', '["verus_invocations"] == 21'),
        ("The nineteen-control suite", "The twenty-control suite"),
        ("Prior 267 guards/five baselines", "Prior 286 guards/six baselines"),
        ("Fresh digit guards", "Fresh word guards"),
    ]
    for old, new in replacements:
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    assert text.count("test_digit_scan_rejections.py") == 2
    text = text.replace("test_digit_scan_rejections.py", "test_word_scan_rejections.py")
    ast.parse(text)
    target.write_text(text)
    record.write_text(json.dumps({"status": "Frozen before typed-word validation",
        "preparation_sha256": sha(Path(__file__)), "parent_sha256": sha(parent), "runner_sha256": sha(target),
        "new_negative_controls": 20, "positive_baselines": 1, "format_inputs": 112,
        "scope": "Only phase/source identities, the permitted new guard module, expected counts and the added format file change. Actual invocation capture and rejection criteria stay unchanged. All commands execute sequentially with no SOURCE edits."}, indent=2) + "\n")
    print(json.dumps({"frozen_word_validation": True, "new_negative_controls": 20}))


if __name__ == "__main__":
    main()
