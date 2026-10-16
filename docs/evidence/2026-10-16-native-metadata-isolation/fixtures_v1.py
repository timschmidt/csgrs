"""Require complete unchanged archives in both isolated source copies."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

E = Path(__file__).resolve().parent
assert shutil.which('unzip')
preparation = json.loads((E / 'preparation-v1.json').read_text())
records = []
for variant, source in preparation['sources'].items():
    for name, entry, expected, size in [
        ('CPArti FPGA dev board.zip', 'CPArti FPGA dev board.kicad_pcb',
         'f57195393ff7a938af0a9651781ba13536d248cbb43f13a41739e2a97cf99e35', 8177693),
        ('HVP109A.zip', 'HVP109A.kicad_pcb',
         '986b7e26b60c83c97a851c403fb90a927c9018d30ad6b94c7bba5925aa99e28a', 5611517),
    ]:
        path = Path(source) / 'crates/hyperdrc/docs' / name
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == expected, path
        with zipfile.ZipFile(path) as archive:
            assert archive.testzip() is None
            data = archive.read(entry)
            assert len(data) == size
            records.append({'variant': variant, 'archive': str(path), 'sha256': digest,
                            'entry': entry, 'bytes': len(data),
                            'entry_sha256': hashlib.sha256(data).hexdigest(),
                            'crc32': archive.getinfo(entry).CRC})
print(json.dumps({'fixtures': records, 'unzip': shutil.which('unzip'),
                  'hardware': subprocess.check_output(['lscpu'], text=True)}))
