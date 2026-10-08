from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'finite-point-image-candidate-v787';B=A/'finite-point-image-base-v787.json';base=json.loads(B.read_text())
def edit(rel,fn):
 p=C/rel
 if not p.exists():
  data=(W/rel).read_bytes();base[rel]=hashlib.sha256(data).hexdigest();p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data);B.write_text(json.dumps(base,indent=2)+'\n')
 s=p.read_text();new=fn(s);assert new!=s,rel;p.write_text(new)
def sub(s,a,b):
 assert s.count(a)==1,(a[:100],s.count(a));return s.replace(a,b)
def general(s):
 s=sub(s,'let image = self.point_at_algebraic_parameter(refined, policy)?;','''let Classification::Decided(image) = self.point_at_algebraic_parameter(refined, policy)? else {
                        continue;
                    };''')
 a=s.index('            Ok(match image.status() {',s.index('pub(crate) fn exact_contact_point_evidence('));b=s.index('\n        }',a)
 s=s[:a]+'''            Ok(match image {
                Classification::Decided(image) => Some(CurvePoint2::from(image)),
                Classification::Uncertain(_) => None,
            })'''+s[b:];return s
edit('hypercurve/src/rational_bezier_general.rs',general)
def offset(s):
 for anchor,var,parameter,note in [('fn point_image_from_frame_scales(', 'Ok(match', 'self.data.parameter', 'The selected frame and nonzero source scale already certify this affine denominator.'),('"retained an exact analytic-parallel supporting-line contact"', 'let image = match', 'algebraic_parameter', 'The selected line parameter already certifies this affine denominator.')]:
  if anchor.startswith('fn '):a=s.index('        Ok(match image.status() {',s.index(anchor))
  else:a=s.rindex('        let image = match image.status() {',0,s.index(anchor))
  b=s.index('\n        }',a)+len('\n        }')
  part=s[a:b]
  part=part.replace('match image.status()', 'match image')
  part=part.replace('''            BezierAlgebraicImageStatus::Transformed
            | BezierAlgebraicImageStatus::RetainedRationalExpression => image,''','''            Classification::Decided(image) => image,''')
  part=part.replace('''            BezierAlgebraicImageStatus::InvalidParameterEvidence
            | BezierAlgebraicImageStatus::XImageFailed
            | BezierAlgebraicImageStatus::YImageFailed => {''',f'''            Classification::Uncertain(_) => {{
                // {note}''')
  part=part.replace('image.parameter().clone()',f'crate::bezier_algebraic_image::parameter_representation(&{parameter}, &policy.strict_counterpart())' if parameter=='self.data.parameter' else f'crate::bezier_algebraic_image::parameter_representation({parameter}, &policy.strict_counterpart())')
  s=s[:a]+part+s[b:]
 return s
edit('hypercurve/src/bezier_offset.rs',offset)
def endpoint(s):
 s=s.replace('CubicBezier2, CurveContext, CurveResult,','CubicBezier2, Classification, CurveContext, CurveResult,')
 s=s.replace('point: OnceLock<CurveResult<BezierEndpointPointImage2>>,','point: OnceLock<CurveResult<Classification<BezierEndpointPointImage2>>>,')
 # Only the eager rational factories need a new classification; polynomial
 # factories and trusted lazy constructors still return their existing value.
 for method in ['from_source_curve','rational_quadratic','rational']:
  a=s.index(f'    pub fn {method}(');b=s.index('\n    }',a)+len('\n    }');part=s[a:b];part=part.replace(') -> CurveResult<Self>',') -> CurveResult<Classification<Self>>')
  if method=='from_source_curve':
   part=part.replace('Self::quadratic(curve, parameter, policy),','Self::quadratic(curve, parameter, policy).map(Classification::Decided),').replace('Self::cubic(curve, parameter, policy),','Self::cubic(curve, parameter, policy).map(Classification::Decided),')
  else:
   pos=part.index('        let mut derivatives')
   part=part[:pos]+'''        let point = match curve.point_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
'''+part[pos:]
   part=part.replace('''point: BezierEndpointPointImage2::Rational(
                    curve.point_at_algebraic_parameter(parameter, policy)?,
                ),''','point: BezierEndpointPointImage2::Rational(point),')
   part=part.replace('Ok(Self {','Ok(Classification::Decided(Self {').replace('''        })
    }''','''        }))
    }''')
  s=s[:a]+part+s[b:]
 s=sub(s,'''    /// Returns the exact point image at the endpoint.
    pub fn point(&self) -> &BezierEndpointPointImage2 {
        self.try_point()
            .expect("certified private split endpoint point image must remain constructible")
    }

''','')
 s=s.replace('self.try_point()', 'self.point()').replace('other.try_point()', 'other.point()').replace('expected.try_point()', 'expected.point()')
 s=s.replace('self.point().is_ok_and(|point| point.is_exact())','self.point().is_ok_and(|point| matches!(point, Classification::Decided(point) if point.is_exact()))')
 s=s.replace('''    pub(crate) fn try_point(&self) -> CurveResult<&BezierEndpointPointImage2> {''','''    /// Returns a certified affine endpoint image, or the construction blocker.
    /// Lazy rational sources must prove a finite point before exposing one.
    pub fn point(&self) -> CurveResult<Classification<&BezierEndpointPointImage2>> {''')
 s=s.replace('BezierAlgebraicEndpointImageData::Materialized { point, .. } => Ok(point),','BezierAlgebraicEndpointImageData::Materialized { point, .. } => Ok(Classification::Decided(point)),')
 a=s.index('    pub fn point(&self)');b=s.index('    pub(crate) fn try_tangent',a);part=s[a:b]
 part=part.replace('.map(BezierEndpointPointImage2::Polynomial)', '.map(|image| Classification::Decided(BezierEndpointPointImage2::Polynomial(image)))')
 part=part.replace('.map(BezierEndpointPointImage2::Rational)', '.map(|image| image.map(BezierEndpointPointImage2::Rational))')
 part=part.replace('''.as_ref()
                .map_err(Clone::clone),''','''.as_ref()
                .map(|image| match image {
                    Classification::Decided(image) => Classification::Decided(image),
                    Classification::Uncertain(reason) => Classification::Uncertain(*reason),
                })
                .map_err(Clone::clone),''')
 s=s[:a]+part+s[b:];return s
edit('hypercurve/src/bezier_split_endpoint.rs',endpoint)
def region(s):
 # Admission invariant checks cannot use an uncertain expected endpoint.
 old='''            let expected = crate::BezierAlgebraicEndpointImage2::from_source_curve(
                source_curve,
                parameter,
                policy,
            )?;'''
 new=old.replace('let expected =','let Classification::Decided(expected) =').replace(')?;', ''')? else {
                return Err(CurveError::Topology("retained algebraic endpoint is not a certified finite source point".into()));
            };''');s=sub(s,old,new)
 old='''            BezierParameter2::Exact(_) => Ok(None),
            BezierParameter2::Algebraic(parameter) => {
                Ok(Some(BezierAlgebraicEndpointImage2::from_source_curve(
                    &offset_subcurve,
                    parameter,
                    policy,
                )?))
            }'''
 new='''            BezierParameter2::Exact(_) => Ok(Classification::Decided(None)),
            BezierParameter2::Algebraic(parameter) => {
                BezierAlgebraicEndpointImage2::from_source_curve(
                    &offset_subcurve, parameter, policy,
                ).map(|image| image.map(Some))
            }''';s=sub(s,old,new)
 for end in ['start','end']:
  s=sub(s,f'    let {end}_image = endpoint_image({end})?;',f'''    let {end}_image = match endpoint_image({end})? {{
        Classification::Decided(image) => image,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }};''')
 for family in ['RationalQuadratic','Rational']:
  old=f'''        BezierSubcurve2::{family}(curve) => rational_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        ),'''
  new=f'''        BezierSubcurve2::{family}(curve) => match curve.point_at_algebraic_parameter(parameter, policy)? {{
            Classification::Decided(image) => rational_image_coordinate_order(&image, use_x, origin_coordinate, policy),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }},''';s=sub(s,old,new)
 old='''            match exact_point_from_image(image.point(), Some(policy)) {'''
 new='''            let point = match image.point()? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match exact_point_from_image(point, Some(policy)) {''';s=sub(s,old,new)
 return s
edit('hypercurve/src/bezier_region.rs',region)
def split(s):
 s=sub(s,'''            let expected =
                BezierAlgebraicEndpointImage2::from_source_curve(source_curve, parameter, policy)?;''','''            let Classification::Decided(expected) =
                BezierAlgebraicEndpointImage2::from_source_curve(source_curve, parameter, policy)? else {
                    return Err(CurveError::Topology("algebraic split endpoint is not a certified finite source point".into()));
                };''')
 s=s.replace('G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<BezierAlgebraicEndpointImage2>,','G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<Classification<BezierAlgebraicEndpointImage2>>,')
 old='''    let endpoint_images = boundaries
        .iter()
        .map(|boundary| endpoint_image_for(boundary, &mut endpoint_image))
        .collect::<CurveResult<Vec<_>>>()?;''';new='''    let mut endpoint_images = Vec::with_capacity(boundaries.len());
    for boundary in &boundaries {
        match endpoint_image_for(boundary, &mut endpoint_image)? {
            Classification::Decided(image) => endpoint_images.push(image),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }''';s=sub(s,old,new)
 s=sub(s,') -> CurveResult<Option<BezierAlgebraicEndpointImage2>>',') -> CurveResult<Classification<Option<BezierAlgebraicEndpointImage2>>>')
 s=sub(s,'''        BezierParameter2::Exact(_) => Ok(None),
        BezierParameter2::Algebraic(parameter) => Ok(Some(endpoint_image(parameter)?)),''','''        BezierParameter2::Exact(_) => Ok(Classification::Decided(None)),
        BezierParameter2::Algebraic(parameter) => endpoint_image(parameter).map(|image| image.map(Some)),''')
 for method in ['quadratic','cubic']:
  old=f'|parameter| BezierAlgebraicEndpointImage2::{method}(self, parameter, policy),'
  new=f'|parameter| BezierAlgebraicEndpointImage2::{method}(self, parameter, policy).map(Classification::Decided),';s=sub(s,old,new)
 old='''                BezierAlgebraicEndpointImage2::from_source_curve_first_order(
                    self, parameter, policy,
                )''';s=sub(s,old,old+'.map(Classification::Decided)')
 return s
edit('hypercurve/src/bezier_split.rs',split)
def arrangement(s):
 old='''                let expected = BezierAlgebraicEndpointImage2::from_source_curve(
                    source_curve,
                    parameter,
                    policy,
                )?;''';new=old.replace('let expected =','let Classification::Decided(expected) =').replace(')?;', ''')? else {
                    return Err(CurveError::Topology("algebraic arrangement endpoint is not a certified finite source point".into()));
                };''');s=sub(s,old,new)
 old='''        let Ok(point_image) = image.try_point() else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };''';new='''        let point_image = match image.point() {
            Ok(Classification::Decided(image)) => image,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
        };''';s=sub(s,old,new)
 s=s.replace('lazy.try_point()', 'lazy.point()').replace('eager.try_point()', 'eager.point()');return s
edit('hypercurve/src/bezier_arrangement.rs',arrangement)
print('Migrated production point factories, rational endpoint constructors, splitting and admission callers; tests and measure consumers pending')
