"""Prepare exact-scope commit helpers; do not stage until the checkpoint passes."""
from pathlib import Path
W=Path(__file__).resolve().parent
old=W.parent/'containment-winding-git-v1'
out=W.parent/'containment-bounds-git-v1'
out.mkdir(exist_ok=False)
for name in ['prepare_v1.py','stage_v1.py','commit_v1.py']:
    s=(old/name).read_text().replace('containment-winding','containment-bounds')
    s=s.replace('containment_winding_admitted','containment_bounds_admitted')
    s=s.replace('Correct nonzero-winding containment and cancelled flat sheets',
        'Use certified supporting bounds for strict containment')
    s=s.replace('while admitting exercised native nonzero-winding containment',
        'while admitting the certified supporting-plane proof')
    (out/name).write_text(s)
print('Prepared exact-scope commit helpers without modifying the index',flush=True)
