"""Complete the unexecuted v3 probes against concrete public APIs."""
from pathlib import Path
work=Path(__file__).resolve().parent
p=work/'prototype-v3/tests/containment_winding.rs'
s=p.read_text().replace('use csgrs::{GeometryCertainty','use csgrs::solid::SolidExt;\nuse csgrs::{GeometryCertainty')
s=s.replace('solid::union_with_context(&output, &source, &context)','output.try_union_with_context(&source, &context)')
s=s.replace('triangle.0.map(|i| &output.positions[i])','triangle.indices().map(|i| &output.positions[i])')
s=s.replace('let coords = |p: &Point3| [&p.x, &p.y, &p.z];','fn coords(p: &Point3) -> [&Real; 3] { [&p.x, &p.y, &p.z] }')
p.write_text(s)
s=(work/'baseline_v3.py').read_text().replace("OUT=WORK/'baseline-v3'","OUT=WORK/'baseline-v4'").replace("'prototype-v2'","'prototype-v3'").replace("'running 12 tests'","'running 15 tests'")
(work/'baseline_v4.py').write_text(s)
