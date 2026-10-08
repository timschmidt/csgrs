from pathlib import Path
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve/src/bezier_offset.rs')
s=p.read_text()
def change(old,new,count=1):
    global s
    assert s.count(old)==count,(old[:120],s.count(old),count)
    s=s.replace(old,new)
def scope(start,end,fn):
    global s
    a=s.index(start); b=s.index(end,a+len(start))
    s=s[:a]+fn(s[a:b])+s[b:]

# Every authored expression is constructed with its own immutable radicand.
needle='BezierRecursiveQuadraticParallelExpression2 {'
positions=[]
start=0
while (a:=s.find(needle,start))>=0:
    start=a+len(needle)
    if s[max(0,a-7):a].endswith(('struct ','impl ')):
        continue
    depth=1; b=start
    while depth:
        if s[b]=='{': depth+=1
        elif s[b]=='}': depth-=1
        b+=1
    body=s[start:b-1]
    parts=[]; part=0; stack=[]
    for i,c in enumerate(body):
        if c in '([{': stack.append(c)
        elif c in ')]}': stack.pop()
        elif c==',' and not stack:
            if body[part:i].strip(): parts.append(body[part:i].strip())
            part=i+1
    if body[part:].strip(): parts.append(body[part:].strip())
    assert len(parts)==2,parts
    values={}
    for part in parts:
        name,colon,value=part.partition(':')
        values[name.strip()]=value.strip() if colon else name.strip()
    assert set(values)=={'rational','radical'},values
    line=s.count('\n',0,a)+1
    if line<40000 or 78000<line<79000:
        speed='speed_squared.clone()'
    elif 62000<line<64000:
        speed='self.circle.speed_squared.clone()'
    elif 114000<line<115000:
        speed='speed_squared.into()' if len([x for x in positions if 114000<x[3]<115000])==0 else 'candidate_speed_squared.into()'
    elif line>150000:
        speed='system.circle.speed_squared.clone()'
    else:
        raise AssertionError(line)
    replacement=f"BezierRecursiveQuadraticParallelExpression2::new({values['rational']}, {values['radical']}, {speed})"
    positions.append((a,b,replacement,line))
assert len(positions)==16,len(positions)
for a,b,replacement,_ in reversed(positions): s=s[:a]+replacement+s[b:]

# Shared circle radicand is one for a rational target, otherwise its actual S.
change('''            let speed_squared = add(
                &multiply(&tangent_x, &tangent_x)?,
                &multiply(&tangent_y, &tangent_y)?,
            )?;
            let normal_x''','''            let speed_squared: Arc<[_]> = if unit_target_speed {
                real(&[Real::one()])?
            } else {
                add(
                    &multiply(&tangent_x, &tangent_x)?,
                    &multiply(&tangent_y, &tangent_y)?,
                )?
            }.into();
            let normal_x''')
change('''                if unit_target_speed {
                    real(&[Real::one()])?
                } else {
                    speed_squared
                },
                weight,''','''                weight,''')
scope('    fn recursive_selected_radial_target_system(', '    fn represented_center_parallel_system(', lambda x:x.replace('            speed_squared,\n','').replace('                speed_squared,\n',''))
# Build the circle once before optional global projection, and use its norm.
change('''            let (base, projection) = if project_incidence {
                let projected_coefficients = if unit_target_speed {
                    add(&circle_rational, &circle_radical)?
                } else {
                    subtract(
                        &multiply(&circle_rational, &circle_rational)?,
                        &multiply(&multiply(&circle_radical, &circle_radical)?, &speed_squared)?,
                    )?
                };''','''            let circle = BezierRecursiveQuadraticParallelExpression2::new(
                circle_rational, circle_radical, speed_squared.clone(),
            );
            let (base, projection) = if project_incidence {
                let projected_coefficients = if unit_target_speed {
                    add(&circle.rational, &circle.radical)?
                } else {
                    circle.squared_magnitude_difference()?.to_vec()
                };''')
change('''                BezierRecursiveQuadraticParallelExpression2::new(circle_rational, circle_radical, speed_squared.clone()),''','''                circle,''')

# Chord predicates share the same radicand; the incidence owns system access.
change('''            let speed_squared = if zero_distance {
                real(&[Real::one()])?
            } else {
                add(
                    &multiply(&tangent_x, &tangent_x)?,
                    &multiply(&tangent_y, &tangent_y)?,
                )?
            };''','''            let speed_squared: Arc<[_]> = if zero_distance {
                real(&[Real::one()])?
            } else {
                add(
                    &multiply(&tangent_x, &tangent_x)?,
                    &multiply(&tangent_y, &tangent_y)?,
                )?
            }.into();''')
scope('    fn recursive_projective_parallel_system_with_frame(', '    /// Replays one target domain against', lambda x:x.replace('            speed_squared,\n','').replace('                speed_squared,\n',''))

# Fixed-distance incidence must bind the candidate speed, not the center speed.
a=s.index('    fn recursive_fixed_distance_system(')
inc=s.index('        let Some(incidence) = (|| {',a)
source=s.index('        let source = self.source_power_basis()?;',inc)
projection=s.index('        let Some(projected_coefficients) = (|| {',source)
block=s[source:projection]
s=s[:inc]+block+s[inc:source]+s[projection:]
a=s.index('        let Some(projected_coefficients) = (|| {',inc)
b=s.index('        let field_base = field.base_and_extension_path().0;',a)
s=s[:a]+'''        let Some(projected_coefficients) = incidence.squared_magnitude_difference() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((base, projection)) =
            recursive_quadratic_polynomial_projection(projected_coefficients.to_vec())
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
'''+s[b:]
scope('    fn recursive_fixed_distance_system(', '    /// Solves fixed-distance candidates from one recursive center', lambda x:x.replace('                    speed_squared,\n','').replace('                speed_squared: candidate_speed_squared,\n',''))

scope('impl BezierRecursiveFixedDistanceSystem2 {','impl BezierRecursiveProjectiveChordParallelIntervalSystem2 {',lambda x:x.replace('self.speed_squared','self.incidence.speed_squared'))
scope('impl BezierRecursiveProjectiveChordParallelSystem2 {','fn recursive_quadratic_polynomial_interval(',lambda x:x.replace('self.speed_squared','self.incidence.speed_squared'))
scope('impl BezierRecursiveSelectedRadialParallelSystem2 {','impl BezierRecursiveCircleFrame2 {',lambda x:x.replace('self.speed_squared','self.circle.speed_squared'))

# Norms/signs and interval replay derive S from the bound expression.
s=s.replace('.squared_magnitude_difference(&self.incidence.speed_squared)', '.squared_magnitude_difference()')
s=s.replace('.squared_magnitude_difference(&self.circle.speed_squared)', '.squared_magnitude_difference()')
s=s.replace('.squared_magnitude_difference(&system.speed_squared)', '.squared_magnitude_difference()')
s=s.replace('.sign_with_positive_speed(&self.incidence.speed_squared, policy, sign)', '.sign_with_positive_speed(policy, sign)')
s=s.replace('.sign_with_positive_speed(&self.circle.speed_squared, policy,', '.sign_with_positive_speed(policy,')
change('''                    expression.sign_with_positive_speed(
                        &system.speed_squared,
                        policy,
                        polynomial_sign,
                    )''','''                    expression.sign_with_positive_speed(policy, polynomial_sign)''')
change('''        self.incidence
            .squared_magnitude_difference()
    }''','''        self.incidence.squared_magnitude_difference().map(<[_]>::to_vec)
    }''')
change('''            expression.squared_magnitude_difference()
        }''','''            expression.squared_magnitude_difference().map(<[_]>::to_vec)
        }''')
change('''[(&self.source_weight, false), (&self.incidence.speed_squared, true)]''','''[(&self.source_weight[..], false), (&self.incidence.speed_squared[..], true)]''')
change('''recursive_quadratic_polynomial_is_identically_zero(&norm, policy)?''','''recursive_quadratic_polynomial_is_identically_zero(norm, policy)?''')
# Remove the redundant radicand argument from both expression interval helpers.
for name in ['recursive_quadratic_parallel_expression_interval','recursive_quadratic_parallel_expression_transverse_root']:
    old=f'''fn {name}(\n    expression: &BezierRecursiveQuadraticParallelExpression2,\n    speed_squared: &[BezierRecursiveQuadraticValue2],'''
    change(old,f'''fn {name}(\n    expression: &BezierRecursiveQuadraticParallelExpression2,''')
scope('fn recursive_quadratic_parallel_expression_interval(', 'impl BezierRecursiveSelectedRadialParallelSystem2 {', lambda x:x.replace('            speed_squared,\n','            &expression.speed_squared,\n').replace('                speed_squared,\n',''))
# All callers pass the bound expression only.
import re
s,n=re.subn(r'(recursive_quadratic_parallel_expression_(?:interval|transverse_root)\(\n\s+(?:&self\.incidence|expression|difference),\n)\s+&(?:self\.(?:incidence|circle)|clipping_system)\.speed_squared,\n',r'\1',s)
assert n==4,n
# The nested helper call was previously using its parameter name.
s=s.replace('''                expression,
                &expression.speed_squared,
                unit_target_speed,''','''                expression,
                unit_target_speed,''')

p.write_text(s)
print('Migrated 16 expression constructors, three system owners, shared norms and interval callers')
