"""Propagate the constructor Result migration without changing fixture inputs."""
from pathlib import Path
import re
names='arrow|trapezoid|star|teardrop|egg|squircle|pie_slice|heart|supershape'
paths=[Path('examples/extrude.rs'),Path('examples/readme_renders.rs'),Path('tests/adversarial_stress.rs'),Path('benchmarks/rust/feature_pipeline.rs'),Path('benchmarks/rust/profile_primitives.rs'),Path('fuzz/fuzz_targets/fuzz_curve_shape_catalog.rs'),Path('src/curve/native.rs')]
for p in paths:
 s=p.read_text(); offset=s.index('#[cfg(test)]') if p.name=='native.rs' else 0
 pattern=re.compile(r'\b('+names+r')\s*\(') if p.name=='native.rs' else re.compile(r'\bcurve::('+names+r')\s*\(')
 changes=[]
 for m in pattern.finditer(s,offset):
  depth=1;index=m.end(); quoted=False;escaped=False
  while depth:
   c=s[index]
   if quoted:
    if escaped:escaped=False
    elif c=='\\':escaped=True
    elif c=='"':quoted=False
   elif c=='"':quoted=True
   elif c=='(':depth+=1
   elif c==')':depth-=1
   index+=1
  if p.name=='native.rs' and (tail:=re.match(r'\s*\.is_empty\(\)',s[index:])):
   changes.append((index,index+tail.end(),'.is_err()'))
  elif 'fuzz' in p.parts:
   changes.append((m.start(),index,'match '+s[m.start():index]+' { Ok(region) => region, Err(_) => return, }'))
  else:
   changes.append((index,index,'.expect("valid sampled profile fixture")'))
 for a,b,value in reversed(changes):s=s[:a]+value+s[b:]
 p.write_text(s)
 print(p,len(changes))
p=Path('src/wasm/curve_js.rs');s=p.read_text()
for name,args,next_marker in [
 ('star',[('points','usize'),('outer_radius','f64'),('inner_radius','f64')],'    pub fn teardrop('),
 ('teardrop',[('width','f64'),('length','f64'),('segments','usize')],'    pub fn egg('),
 ('egg',[('width','f64'),('length','f64'),('segments','usize')],'    pub fn ring('),
 ('heart',[('width','f64'),('height','f64'),('segments','usize')],'    pub fn crescent('),
]:
 start=s.index('    pub fn '+name+'(');end=s.index(next_marker,start)
 signature=', '.join(f'{n}: {t}' for n,t in args)
 values=[]
 for n,t in args:
  label={'outer_radius':'outerRadius','inner_radius':'innerRadius'}.get(n,n)
  values.append(n if t=='usize' else f'crate::wasm::mesh_js::geometry_scalar({n}, "{label}")?')
 s=s[:start]+f'''    pub fn {name}({signature}) -> Result<Self, JsValue> {{
        curve::{name}({', '.join(values)})
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }}

'''+s[end:]
p.write_text(s)
