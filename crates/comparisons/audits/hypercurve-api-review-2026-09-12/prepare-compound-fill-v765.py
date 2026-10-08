from pathlib import Path
import hashlib,json,re
A=Path(__file__).resolve().parent;W=A.parent;C=A/'compound-fill-candidate-v765';assert not C.exists()
base=json.loads((A/'region-admission-v763-sources.json').read_text())
for name,sha in base.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
sources={};migrations={}
# The source arguments contain Rust expressions; honor their delimiter nesting.
def comma_after_argument(s,start):
 stack=[];i=start
 while i<len(s):
  if s.startswith('//',i):i=s.index('\n',i);continue
  if s.startswith('/*',i):i=s.index('*/',i+2)+2;continue
  c=s[i]
  if c=='"':
   i+=1
   while s[i]!='"':i+=2 if s[i]=='\\' else 1
  elif c in '([{':stack.append(c)
  elif c in ')]}':
   assert stack and '([{'.index(stack.pop())==')]}'.index(c),(s[start:i],c)
  elif c==',' and not stack:return i
  i+=1
 raise ValueError('argument not terminated')
for name in base:
 if not name.endswith('.rs'):continue
 s=(W/name).read_text();matches=list(re.finditer(r'::try_from_boundary_paths\(',s))
 if not matches:continue
 positions=[comma_after_argument(s,m.end())+1 for m in matches]
 rule='crate::FillRule::EvenOdd' if name.startswith('hypercurve/src/')else'hypercurve::FillRule::EvenOdd'
 for p in reversed(positions):s=s[:p]+' '+rule+','+s[p:]
 sources[name]=s;migrations[name]=len(matches)
assert sum(migrations.values())==104
name='hypercurve/src/bezier_region.rs';s=sources[name]
a=s.index('    /// Constructs a top-level exact curved region from closed boundary paths.');b=s.index('    /// Authored loops are construction inputs.',a)
s=s[:a]+'''    /// Constructs the exact regularized fill of closed boundary paths.
    ///
    /// The fill rule applies to the sum of signed winding across all paths.
    /// Thus equally oriented overlapping paths add under `NonZero`, while
    /// opposite traversals cancel. `EvenOdd` selects odd total winding.
    /// Reuses each path's cached boundary, including retained generated curves;
    /// construction removes canceled seams and orients material on the left.
    pub fn try_from_boundary_paths(
        paths: &[CurvePath2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::regularize_boundary_paths_raw(paths, fill_rule, attempt)
                .map_err(|error| error.with_operation(CurveOperation2::Construction))
        })
    }

'''+s[b:]
# Geometry produces winding evidence; construction selects the filled faces.
renames={'classify_point_from_boundary_side_ray_with_windings':'loop_windings_from_boundary_side_ray','classify_algebraic_point_from_boundary_side_ray_with_windings':'algebraic_loop_windings_from_boundary_side_ray'}
for old,new in renames.items():s=s.replace(old,new)
a=s.index('    pub(crate) fn loop_windings_from_boundary_side_ray(');b=s.index('    pub(crate) fn region_location_from_loop_windings(',a)
part=s[a:b];assert part.count('CurveResult<Classification<(Vec<i32>, RegionPointLocation)>>')==2
part=part.replace('CurveResult<Classification<(Vec<i32>, RegionPointLocation)>>','CurveResult<Classification<Vec<i32>>>')
validation='''        if self
            .data
            .certified_loop_roles
            .as_ref()
            .is_some_and(|roles| roles.len() != self.data.boundary_loops.len())
            || self
                .data
                .certified_loop_fill_rules
                .as_ref()
                .is_some_and(|rules| rules.len() != self.data.boundary_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region loop semantics are inconsistent with boundary loops".into(),
            ));
        }
'''
assert part.count(validation)==2;part=part.replace(validation,'')
old='''        let location = self.region_location_from_loop_windings(&windings)?;
        Ok(Classification::Decided((windings, location)))'''
assert part.count(old)==2;part=part.replace(old,'        Ok(Classification::Decided(windings))')
s=s[:a]+part+s[b:]
# Preserve both the geometric winding and independent filled-side test oracles.
a=s.index('                        let result = region\n                            .loop_windings_from_boundary_side_ray(')
b=s.index('                        assert_eq!(',a)
s=s[:b]+'''                        let result = result.map(|windings| {
                            let location = region.region_location_from_loop_windings(&windings).unwrap();
                            (windings, location)
                        });
'''+s[b:]
sources[name]=s
name='hypercurve/src/curve_region_boolean.rs';s=sources[name]
s=s.replace('    strict_line_image_only: OnceLock<bool>,','''    // Authored compound fills select from global signed winding. This rule
    // belongs to unary construction and is absent from published regions.
    regularization_fill_rule: Option<FillRule>,
    strict_line_image_only: OnceLock<bool>,''',1)
# Include directly constructed test contexts, with the same default semantics.
s,count=re.subn(r'(?m)^(\s*)strict_line_image_only: OnceLock::new\(\),',r'\1regularization_fill_rule: None,\n\1strict_line_image_only: OnceLock::new(),',s);assert count==17,count
pos=s.index('    /// Collects exact contacts and overlaps between regularized region boundaries.')
s=s[:pos]+'''    /// Applies an authored compound fill before publishing a normalized set.
    /// No raw-region regularization cache can be reused under another fill rule.
    pub(crate) fn regularize_boundary_paths_raw(
        paths: &[crate::CurvePath2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let raw = Self::try_from_boundary_paths_raw(paths, policy)?;
        let mut context = CurveRegionBooleanContext::try_new_unary(&raw, policy)?;
        context.data.regularization_fill_rule = Some(fill_rule);
        context.build_regularized_region()
    }

'''+s[pos:]
# Every side/face selection consumes the same winding-rule authority.
s,count=re.subn(r'self\s*\.data\s*\.first\s*\.region_location_from_loop_windings\(([^)]+)\)',r'self.location_from_windings(self.data.first, \1)',s);assert count==8,count
s,count=re.subn(r'self\s*\.region_for_carrier\(carrier_index\)\s*\.region_location_from_loop_windings\(([^)]+)\)',r'self.location_from_windings(self.region_for_carrier(carrier_index), \1)',s);assert count==1,count
for old,new in renames.items():s=s.replace(old,new)
for start,end in [('    fn algebraic_fragment_side_classification(','    fn fragment_side_classification('),('    fn fragment_side_classification_with_reference_tangent(','    fn region_for_carrier(')]:
 a=s.index(start);b=s.index(end,a);part=s[a:b]
 old='                Classification::Decided(classification) => return Ok(classification),'
 assert part.count(old)==1
 part=part.replace(old,'''                Classification::Decided(windings) => {
                    let location = self.location_from_windings(
                        self.region_for_carrier(carrier_index), &windings,
                    ).map_err(|cause| self.invalid(carrier_index, cause))?;
                    return Ok((windings, location));
                }''')
 s=s[:a]+part+s[b:]
pos=s.index('    fn region_for_carrier(')
s=s[:pos]+'''    fn location_from_windings(
        &self,
        region: &CurveRegion2,
        windings: &[i32],
    ) -> CurveResult<RegionPointLocation> {
        let Some(fill_rule) = self.data.regularization_fill_rule else {
            return region.region_location_from_loop_windings(windings);
        };
        if windings.len() != region.boundary_loops().len() {
            return Err(CurveError::Topology(
                "compound winding vector is inconsistent with boundary loops".into(),
            ));
        }
        let winding = windings.iter().try_fold(0_i64, |sum, &value| {
            sum.checked_add(i64::from(value)).ok_or_else(|| {
                CurveError::Topology("compound winding overflowed i64".into())
            })
        })?;
        let inside = match fill_rule {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        Ok(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

'''+s[pos:]
# The existing algebraic ray regression still asserts winding and membership.
a=s.index('                            raw.algebraic_loop_windings_from_boundary_side_ray(')
b=s.index('                    Classification::Decided((vec![if reversed',a)
part=s[a:b];old='                        .unwrap(),\n';assert part.count(old)==1
part=part.replace(old,'''                        .unwrap().map(|windings| {
                            let location = raw.region_location_from_loop_windings(&windings).unwrap();
                            (windings, location)
                        }),
''');s=s[:a]+part+s[b:];sources[name]=s
name='hypercurve/src/svg.rs';s=(W/name).read_text();a=s.index('fn region_from_paths(');b=s.index('\nfn geometry_from_paths(',a)
s=s[:a]+'''fn region_from_paths(paths: &[CurvePath2], fill_rule: FillRule) -> SvgResult<CurveRegion2> {
    CurveRegion2::try_from_boundary_paths(paths, fill_rule, &CurveContext::STRICT)
        .map(CurveOutcome::into_value)
        .map_err(svg_geometry_error)
}
'''+s[b:];sources[name]=s
name='hypercurve/README.md';s=(W/name).read_text();old='regions classify their regularized boundary, including holes and nested islands.';assert s.count(old)==1
s=s.replace(old,old+'''

`CurveRegion2::try_from_boundary_paths(paths, fill_rule, policy)` applies one
fill rule to the total signed winding of all closed paths, then publishes the
regularized set. `NonZero` preserves equally oriented overlaps and cancels
opposite winding; `EvenOdd` selects odd winding. SVG compound fills use this
same admission path. Explicit material/hole constructors instead combine each
loop's filled membership by its supplied role.''');sources[name]=s
for name,s in sources.items():
 assert s!=(W/name).read_text();p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
(A/'compound-fill-base-v765.json').write_text(json.dumps({n:base[n]for n in sources},indent=2)+'\n')
(A/'compound-fill-migrations-v765.json').write_text(json.dumps(migrations,indent=2)+'\n')
print('Prepared',len(sources),'paths;',sum(migrations.values()),'existing calls migrated directly')
