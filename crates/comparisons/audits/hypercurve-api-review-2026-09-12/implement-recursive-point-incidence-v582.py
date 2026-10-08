from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent
base=json.loads((A/'ordered-field-gcd-20260928-v581-sources.json').read_text())
p=W/'hypercurve/src/bezier_offset.rs';s=p.read_text();assert hashlib.sha256(p.read_bytes()).hexdigest()==base['hypercurve/src/bezier_offset.rs']
# Migrate the existing finite root consumers directly to the common domain.
replacements=[]
for name in ['recursive_projective_polynomial_parameters','recursive_projective_polynomial_parameters_with_crossing']:
 pos=0
 while True:
  start=s.find(name+'(',pos)
  if start<0:break
  pos=start+len(name)+1
  if s[max(0,start-3):start]=='fn ':continue
  depth=1;quote=False;escape=False;args=[];at=pos;i=pos
  while depth:
   c=s[i]
   if quote:
    if escape:escape=False
    elif c=='\\':escape=True
    elif c=='"':quote=False
   elif c=='"':quote=True
   elif c in '([{':depth+=1
   elif c in ')]}':depth-=1
   elif c==',' and depth==1:args.append((at,i));at=i+1
   i+=1
  if s[at:i-1].strip():args.append((at,i-1))
  a,b=args[-2];arg=s[a:b];stripped=arg.strip();offset=arg.index(stripped)
  replacements.append((a+offset,a+offset+len(stripped),'SelectedThirdAxisDomain2::Finite('+stripped+')'))
assert len(replacements)==16
for a,b,value in sorted(replacements,reverse=True):s=s[:a]+value+s[b:]
a=s.index('fn recursive_projective_polynomial_parameters_with_crossing(');b=s.index('/// Isolates every rational-curve parameter',a)
block=s[a:b]
assert block.count('    range: &CurveParameterRange2,')==2
block=block.replace('    range: &CurveParameterRange2,',"    domain: SelectedThirdAxisDomain2<'_>,")
block=block.replace('    let domain = CurveParameterDomain2::new(range, None);\n    let unit_domain = range == &CurveParameterRange2::unit();','    let unit_domain = matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit());')
block=block.replace('domain.contains_finite_parameter(', 'domain.contains_parameter(')
a2=block.index('    let (_, bounds) = match domain.finite_envelope(policy)? {');b2=block.index('    let Some((base, projection))',a2)
local=block[a2:b2].replace('domain.finite_envelope(policy)?','CurveParameterDomain2::new(range, None).finite_envelope(policy)?')
block=block[:a2]+'    if let SelectedThirdAxisDomain2::Finite(range) = domain {\n'+''.join('    '+line+'\n' for line in local.rstrip().splitlines())+'    }\n\n'+block[b2:]
block=block.replace('SelectedThirdAxisDomain2::Finite(range),','domain,')
block=block.replace('(range == &CurveParameterRange2::unit())','(matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit()))')
s=s[:a]+block+s[b:]
s=s.replace('/// Isolates every root in a finite original-parameter range over a retained','/// Isolates every root in an original-parameter domain over a retained',1)
# Share source-domain nonzero proofs and exact membership across finite/ray consumers.
marker="impl SelectedThirdAxisDomain2<'_> {\n"
assert s.count(marker)==1
methods=(A/'selected-domain-methods-v582.rs').read_text()
s=s.replace(marker,marker+methods+'\n')
a=s.index('    fn certify_source_frame_in_domain(');b=s.index('    fn certify_finite_source(',a)
block=s[a:b];start=block.index('            let nonzero = |coefficients: &[Real]| {');end=block.index('            let source = self.source_power_basis()?;',start)
block=block[:start]+block[end:];block=block.replace('match nonzero(weight)?','match domain.polynomial_is_nonzero(weight, policy)?').replace('match nonzero(&speed_squared)?','match domain.polynomial_is_nonzero(&speed_squared, policy)?')
s=s[:a]+block+s[b:]
# One borrowed point-query owner now also consumes arbitrary retained point fields.
s=s.replace('BezierParallelAlgebraicPointQuery2','BezierParallelPointQuery2')
marker="impl BezierParallelPointQuery2<'_> {\n";assert s.count(marker)==1
s=s.replace(marker,marker+(A/'recursive-point-query-v582.rs').read_text()+'\n')
old='                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))\n                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))\n                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {\n                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))\n                }'
new='                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))\n                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))\n                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {\n                    let frame = match frame()? {\n                        Classification::Decided(frame) => frame,\n                        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),\n                    };\n                    BezierParallelPointQuery2 { parallel: self, range, frame: frame.as_deref() }\n                        .visit_recursive_point_parameters(point, incident, domain, policy, visitor)\n                }'
assert s.count(old)==1;s=s.replace(old,new)
p.write_text(s)
p=W/'hypercurve/src/curve_fillet.rs';assert hashlib.sha256(p.read_bytes()).hexdigest()==base['hypercurve/src/curve_fillet.rs'];p.write_text(p.read_text()+(A/'stationary-recursive-point-constraint-v575.rs').read_text())
print('Applied recursive point incidence and all 16 root-domain callers')
