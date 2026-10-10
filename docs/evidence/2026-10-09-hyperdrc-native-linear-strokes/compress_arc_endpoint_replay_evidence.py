"""Losslessly archive large replay logs and the redundant incremental trial table."""
import gzip
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
digest = lambda data: hashlib.sha256(data).hexdigest()
groups = {
    'arc-endpoint-production-replay': ('lossless-log-compression.json', [f'{variant}-reports-{index}.log' for variant in ['baseline', 'candidate'] for index in [1, 2]]),
    'arc-endpoint-broad-followup': ('lossless-trial-compression.json', ['additional-trials.json']),
}
for directory, (index, names) in groups.items():
    records = []
    for name in names:
        original = EVIDENCE / directory / name
        compressed = original.with_name(name + '.gz')
        raw = original.read_bytes()
        assert not compressed.exists()
        compressed.write_bytes(gzip.compress(raw, mtime=0))
        assert gzip.decompress(compressed.read_bytes()) == raw
        records.append({'original': name, 'original_bytes': len(raw), 'original_sha256': digest(raw), 'compressed': compressed.name, 'compressed_bytes': compressed.stat().st_size, 'compressed_sha256': digest(compressed.read_bytes()), 'lossless_byte_verification': True})
        original.unlink()
    (EVIDENCE / directory / index).write_text(json.dumps(records, indent=2) + '\n')
