"""Preserve the compile-only v3 attempt and fix iteration over Arc-backed triangles."""
import json,shutil
from pathlib import Path
w=Path(__file__).resolve().parent
shutil.copytree(w/'prototype-v3',w/'prototype-v4')
p=w/'prototype-v4/tests/containment_winding.rs'
s=p.read_text();assert s.count('for triangle in &output.triangles {')==1
p.write_text(s.replace('for triangle in &output.triangles {','for triangle in output.triangles.iter() {'))
s=(w/'baseline_v5.py').read_text().replace("OUT=WORK/'baseline-v5'","OUT=WORK/'baseline-v6'").replace('prototype-v3','prototype-v4')
(w/'baseline_v6.py').write_text(s)
(w/'baseline-v5/harness-failure.json').write_text(json.dumps({'complete':False,'reason':'Rust E0277: Arc slice was not iterated via iter(); no tests ran.','correction':'prototype-v4 changes only iteration syntax; fresh baseline-v6 retains all assertions.'},indent=2)+'\n')
