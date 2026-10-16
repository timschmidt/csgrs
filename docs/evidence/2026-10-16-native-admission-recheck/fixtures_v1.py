"""Require the full archives, including both unmodified board entries."""
import hashlib,json,shutil,subprocess,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
assert shutil.which('unzip')
records=[]
for name, entry, expected in [
 ('CPArti FPGA dev board.zip','CPArti FPGA dev board.kicad_pcb','f57195393ff7a938af0a9651781ba13536d248cbb43f13a41739e2a97cf99e35'),
 ('HVP109A.zip','HVP109A.kicad_pcb','986b7e26b60c83c97a851c403fb90a927c9018d30ad6b94c7bba5925aa99e28a')]:
 p=ROOT/'crates/hyperdrc/docs'/name
 digest=hashlib.sha256(p.read_bytes()).hexdigest()
 assert digest==expected,(p,digest)
 with zipfile.ZipFile(p) as archive:
  assert archive.testzip() is None
  data=archive.read(entry)
  records.append({'archive':str(p),'sha256':digest,'entry':entry,'bytes':len(data),'entry_sha256':hashlib.sha256(data).hexdigest(),'crc32':archive.getinfo(entry).CRC})
print(json.dumps({'fixtures':records,'unzip':shutil.which('unzip'),'hardware':subprocess.check_output(['lscpu'],text=True)}))
