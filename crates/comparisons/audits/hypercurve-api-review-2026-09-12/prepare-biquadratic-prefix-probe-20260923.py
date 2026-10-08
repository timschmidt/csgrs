from pathlib import Path
import re

A=Path(__file__).resolve().parent
source=(A/'selected-chamfer-scalar-shared-20260923.rs').read_text()
lines=[]
for line in source.splitlines():
    lines.append(line)
    match=re.match(r'    let (v\d+) = ',line)
    if match:
        name=match.group(1)
        lines.append(f'    if {name}.quadratic_tower_sign().is_none() {{ println!("first unresolved prefix: {name}"); return; }}')
p=A/'biquadratic-prefix-probe-20260923.rs'
assert not p.exists()
p.write_text('\n'.join(lines)+'\n')
print('Prepared bounded first-prefix scalar diagnostic.')
