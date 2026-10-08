from pathlib import Path
import hashlib,json,re
A=Path(__file__).resolve().parent;W=A.parent;C=A/'topology-hint-removal-candidate-v770';assert not C.exists()
base=json.loads((A/'compound-fill-v766-sources.json').read_text())
for name,sha in base.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
sources={};migrations={}
def arguments(text,start):
 args=[];stack=[];begin=start;i=start
 while i<len(text):
  if text.startswith('//',i):i=text.index('\n',i);continue
  if text.startswith('/*',i):i=text.index('*/',i+2)+2;continue
  c=text[i]
  if c=='"':
   i+=1
   while text[i]!='"':i+=2 if text[i]=='\\' else 1
  elif c in '([{':stack.append(c)
  elif c in ')]}':
   if not stack:
    assert c==')';last=text[begin:i].strip()
    if last:args.append(last)
    return args,i+1
   assert '([{'.index(stack.pop())==')]}'.index(c)
  elif c==','and not stack:args.append(text[begin:i].strip());begin=i+1
  i+=1
 raise ValueError('unterminated call')
def change_calls(text,pattern,drop,expected_count=None,rename=None):
 matches=list(re.finditer(pattern,text))
 if expected_count is not None:assert len(matches)==expected_count,(pattern,len(matches),expected_count)
 for match in reversed(matches):
  args,end=arguments(text,match.end());assert max(drop)<len(args)
  name=match.group()if rename is None else rename+'('
  text=text[:match.start()]+name+','.join(arg for i,arg in enumerate(args)if i not in drop)+')'+text[end:]
 return text,len(matches)
for name in base:
 if not name.endswith('.rs'):continue
 s=(W/name).read_text();pattern=r'::try_from_boundary_paths_with_loop_topology\('
 if re.search(pattern,s):
  s,count=change_calls(s,pattern,{3},rename='::try_from_boundary_paths_with_loop_semantics');sources[name]=s;migrations[name]=count
assert sum(migrations.values())==55,sum(migrations.values())
name='hypercurve/src/bezier_region.rs';s=sources[name]
a=s.index('    /// Constructs a curved region with explicit loop roles, fill rules, and\n');b=s.index('    pub(crate) fn try_from_boundary_paths_with_loop_semantics_raw(',a);s=s[:a]+s[b:]
s,count=change_calls(s,r'::try_from_boundary_paths_with_loop_semantics_raw\(',{4},expected_count=2)
s=s.replace('        certified_filled_sides: Option<Vec<bool>>,\n','')
old='''        if let Some(filled_sides) = certified_filled_sides {
            region = region
                .with_certified_filled_side_is_left(filled_sides)
                .map_err(curve_region_promotion_error)?;
        }
''';assert s.count(old)==1;s=s.replace(old,'')
s=s.replace('pub enum CurveBoundaryInteriorSide2 {','pub(crate) enum CurveBoundaryInteriorSide2 {',1)
old='''    /// Construction returns its regularized boundary, with material on the left.
    pub fn try_from_boundary_paths_with_loop_semantics('''
new='''    /// The arrangement certifies the interior side from exact winding;
    /// callers do not supply orientation hints. Construction returns the
    /// regularized boundary, with material on the left.
    pub fn try_from_boundary_paths_with_loop_semantics('''
assert s.count(old)==1;s=s.replace(old,new);sources[name]=s
name='hypercurve/src/curve_region_boolean.rs';s=sources[name];s,_=change_calls(s,r'::try_from_boundary_paths_with_loop_semantics_raw\(',{4},expected_count=1);sources[name]=s
name='hypercurve/src/lib.rs';s=(W/name).read_text();s=s.replace('    BezierBoundaryLoop2, CurveBoundaryInteriorSide2, CurveRegion2,','    BezierBoundaryLoop2, CurveRegion2,',1);s=s.replace('pub use bezier_region::{','pub(crate) use bezier_region::CurveBoundaryInteriorSide2;\npub use bezier_region::{',1);sources[name]=s
for name in ['hypercurve/benches/bezier_region.rs','hypercurve/benches/curve_path.rs','hypercurve/tests/hypercurve_curve_intersection.rs','hypercurve/tests/hypercurve_curve_region_boolean.rs']:
 s=sources[name];s,_=change_calls(s,r'\bpath_region\(',{1})
 if '/tests/'in name:s,_=change_calls(s,r'\bboolean_paths\(',{3,4})
 sources[name]=s
name='hypercurve/benches/curve_region_boolean_batch.rs';s=sources[name];s,_=change_calls(s,r'\bclipped_region\(',{2});s,_=change_calls(s,r'\bpromote\(',{1},expected_count=2);old='let promote = |path: &CurvePath2, interior_side| {';assert old in s;s=s.replace(old,'let promote = |path: &CurvePath2| {');sources[name]=s
name='hypercurve/tests/hypercurve_curve_intersection.rs';s=sources[name]
old='''for (second, second_side) in [
        (&same, CurveBoundaryInteriorSide2::Left),
        (&reversed, CurveBoundaryInteriorSide2::Right),
    ]'''
# Formatting indentation of existing loops can differ; retain every path.
s,count=re.subn(r'for \(second, second_side\) in \[\s*\(&same, CurveBoundaryInteriorSide2::Left\),\s*\(&reversed, CurveBoundaryInteriorSide2::Right\),\s*\]', 'for second in [&same, &reversed]',s);assert count==1,count;sources[name]=s
name='hypercurve/tests/hypercurve_curve_region_boolean.rs';s=sources[name]
s,count=re.subn(r'for \(start, end, side\) in \[\s*\(Real::zero\(\), q\(1, 4\), CurveBoundaryInteriorSide2::Right\),\s*\(q\(3, 4\), Real::one\(\), CurveBoundaryInteriorSide2::Left\),\s*\]', 'for (start, end) in [(Real::zero(), q(1, 4)), (q(3, 4), Real::one())]',s);assert count==1,count
for variable,left,right in [('cubic','cubic','reversed_cubic'),('wide_path','reparameterized','reversed_reparameterized'),('wide_path','second','second_reversed')]:
 pattern=rf'for \({variable}, interior_side\) in \[\s*\(&{left}, CurveBoundaryInteriorSide2::Left\),\s*\(&{right}, CurveBoundaryInteriorSide2::Right\),\s*\]'
 s,count=re.subn(pattern,f'for {variable} in [&{left}, &{right}]',s);assert count==1,(variable,count)
sources[name]=s
name='hypercurve/tests/hypercurve_curve_region_promotion.rs';s=sources[name]
s,count=re.subn(r'let interior_side = if reverse \{(.*?)\s*CurveBoundaryInteriorSide2::Right\s*\} else \{\s*CurveBoundaryInteriorSide2::Left\s*\};',r'if reverse {\1\n        }',s,flags=re.S);assert count==3,count;sources[name]=s
for name,s in list(sources.items()):
 if name.startswith('hypercurve/src/'):continue
 s=re.sub(r'\bCurveBoundaryInteriorSide2,\s*','',s)
 assert 'CurveBoundaryInteriorSide2'not in s,name
 assert 'interior_side'not in s,name
 sources[name]=s
name='hypercurve/README.md';s=(W/name).read_text();old="loop's filled membership by its supplied role.";assert s.count(old)==1
s=s.replace(old,old+' Interior sides are certified by the\narrangement from winding; authoring does not require orientation hints.');sources[name]=s
for name,s in sources.items():
 assert 'try_from_boundary_paths_with_loop_topology'not in s,name
 assert s!=(W/name).read_text();p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
(A/'topology-hint-removal-base-v770.json').write_text(json.dumps({n:base[n]for n in sources},indent=2)+'\n')
(A/'topology-hint-removal-migrations-v770.json').write_text(json.dumps(migrations,indent=2)+'\n')
print('Prepared',len(sources),'paths;',sum(migrations.values()),'direct topology API migrations;private exact-operation certificates retained')
