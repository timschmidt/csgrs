from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'point-definition-candidate-v790';rel='hypercurve/src/bezier_algebraic_image.rs';data=(W/rel).read_bytes();assert not C.exists();p=C/rel;p.parent.mkdir(parents=True);p.write_bytes(data);(A/'point-definition-base-v790.json').write_text(json.dumps({rel:hashlib.sha256(data).hexdigest()},indent=2)+'\n');s=data.decode()
a=s.index('/// Exact algebraic image of a rational quadratic Bezier affine point.');b=s.index('\n#[derive(Debug, PartialEq)]\nstruct RetainedRationalPointExpression',a)
s=s[:a]+'''/// Exact affine point of a rational Bezier at one selected algebraic parameter.
#[derive(Clone, Debug)]
pub struct RationalBezierAlgebraicPointImage2 {
    data: Arc<RationalBezierAlgebraicPointImageData>,
}

#[derive(Debug, PartialEq)]
struct RationalBezierAlgebraicPointImageData {
    parameter: AlgebraicRootRepresentation,
    definition: RationalPointDefinition,
}

// The enclosing Arc already allocates one immutable image. Keep its active
// definition inline instead of adding a separate allocation for coordinates.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
enum RationalPointDefinition {
    Coordinates {
        x: BezierAlgebraicRationalCoordinateImage,
        y: BezierAlgebraicRationalCoordinateImage,
    },
    Expression {
        expression: RetainedRationalPointExpression,
        message: &'static str,
    },
    Parametric(RetainedRationalPointParametricSource),
}
''' +s[b:]
a=s.index('    fn new(',s.index('impl RationalBezierAlgebraicPointImage2 {'));b=s.index('    #[inline(never)]\n    pub(crate) fn resolved(',a)
s=s[:a]+'''    fn from_coordinates(
        parameter: AlgebraicRootRepresentation,
        x: BezierAlgebraicRationalCoordinateImage,
        y: BezierAlgebraicRationalCoordinateImage,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter,
                definition: RationalPointDefinition::Coordinates { x, y },
            }),
        }
    }

    /// The owning geometry supplies the non-pole certificate for this exact
    /// expression when independent coordinate projection cannot finish.
    pub(crate) fn from_retained_expression(
        parameter: BezierAlgebraicParameter2,
        parameter_root: AlgebraicRootRepresentation,
        x_numerator: Vec<Real>,
        y_numerator: Vec<Real>,
        denominator: Vec<Real>,
        message: &'static str,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter: parameter_root,
                definition: RationalPointDefinition::Expression {
                    expression: RetainedRationalPointExpression {
                        parameter,
                        x_numerator,
                        y_numerator,
                        denominator,
                    },
                    message,
                },
            }),
        }
    }

    /// The caller owns the source's finite-domain proof at this parameter.
    /// Retain that source without forcing its Cartesian coordinate images.
    pub(crate) fn from_parametric_source(
        curve: RationalBezier2,
        parameter: BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter: parameter_representation(&parameter, policy),
                definition: RationalPointDefinition::Parametric(RetainedRationalPointParametricSource {
                    curve,
                    parameter,
                    resolved: OnceLock::new(),
                }),
            }),
        }
    }

    fn retained_expression(&self) -> Option<&RetainedRationalPointExpression> {
        match &self.data.definition {
            RationalPointDefinition::Expression { expression, .. } => Some(expression),
            _ => None,
        }
    }

    fn parametric_source(&self) -> Option<&RetainedRationalPointParametricSource> {
        match &self.data.definition {
            RationalPointDefinition::Parametric(source) => Some(source),
            _ => None,
        }
    }

''' +s[b:]
# Apply only inside the point implementation. Tangent diagnostic reports keep
# their existing construction-status representation.
a=s.index('impl RationalBezierAlgebraicPointImage2 {');b=s.index('impl RationalBezierAlgebraicPointPredicate2',a);part=s[a:b]
part=part.replace('&self.data.parametric_source','self.parametric_source()').replace('resolved.data.retained_expression.as_ref()', 'resolved.retained_expression()').replace('self.data.parametric_source.as_ref()', 'self.parametric_source()').replace('other.data.parametric_source.as_ref()', 'other.parametric_source()').replace('image.data.retained_expression.as_ref()', 'image.retained_expression()')
part=part.replace('''    /// Returns the final construction status.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        self.data.status
    }''','''    /// Reports whether coordinates are projected or retained by their exact source.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { .. } => BezierAlgebraicImageStatus::Transformed,
            RationalPointDefinition::Expression { .. } | RationalPointDefinition::Parametric(_) =>
                BezierAlgebraicImageStatus::RetainedRationalExpression,
        }
    }''')
part=part.replace('''        self.data.x.as_ref()''','''        match &self.data.definition {
            RationalPointDefinition::Coordinates { x, .. } => Some(x),
            _ => None,
        }''').replace('''        self.data.y.as_ref()''','''        match &self.data.definition {
            RationalPointDefinition::Coordinates { y, .. } => Some(y),
            _ => None,
        }''')
c=part.index('    pub fn retained_parameter(');d=part.index('\n    /// Returns the exact x numerator',c)
part=part[:c]+'''    pub fn retained_parameter(&self) -> Option<&BezierAlgebraicParameter2> {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { .. } => None,
            RationalPointDefinition::Expression { expression, .. } => Some(&expression.parameter),
            RationalPointDefinition::Parametric(source) => Some(&source.parameter),
        }
    }
''' +part[d:]
c=part.index('    pub fn retained_coordinate_polynomials(');d=part.index('\n    /// Materializes one exact linear projection',c)
part=part[:c]+'''    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        match &self.data.definition {
            RationalPointDefinition::Expression { expression, .. } => Some((
                expression.x_numerator.as_slice(),
                expression.y_numerator.as_slice(),
                expression.denominator.as_slice(),
            )),
            RationalPointDefinition::Parametric(source) => {
                let power_basis = source.curve.homogeneous_power_basis().ok()?;
                Some((
                    power_basis.x_numerator.as_slice(),
                    power_basis.y_numerator.as_slice(),
                    power_basis.weight.as_slice(),
                ))
            }
            RationalPointDefinition::Coordinates { x, y } =>
                (x.denominator_coefficients() == y.denominator_coefficients()).then_some((
                    x.numerator_coefficients(),
                    y.numerator_coefficients(),
                    x.denominator_coefficients(),
                )),
        }
    }
''' +part[d:]
part=part.replace('''    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.data.message.as_deref()
    }''','''    /// Describes a retained expression when coordinate projection is deferred.
    pub fn message(&self) -> Option<&str> {
        match &self.data.definition {
            RationalPointDefinition::Expression { message, .. } => Some(message),
            _ => None,
        }
    }''')
s=s[:a]+part+s[b:]
# Existing lazy-coordinate regression reads the source without resolving it.
s=s.replace('''                    .data
                    .parametric_source
                    .as_ref()''','''                    .parametric_source()''')
a=s.index('fn rational_point_image_with_parameter_representation(');b=s.index('pub(crate) fn rational_point_image_from_power_basis(',a);part=s[a:b];c=part.index('        RationalCoordinateImagePair::Transformed');part=part[:c]+'''        RationalCoordinateImagePair::Transformed { first: x, second: y } =>
            Ok(Classification::Decided(RationalBezierAlgebraicPointImage2::from_coordinates(parameter_root, x, y))),
        RationalCoordinateImagePair::Retained {
            first_numerator: x_numerator,
            second_numerator: y_numerator,
            denominator,
        } => Ok(Classification::Decided(RationalBezierAlgebraicPointImage2::from_retained_expression(
            parameter.clone(), parameter_root, x_numerator, y_numerator, denominator,
            "retained an exact non-pole Real-coefficient rational point expression",
        ))),
        RationalCoordinateImagePair::Failed { reason, .. } => Ok(Classification::Uncertain(reason)),
    }
}

''';s=s[:a]+part+s[b:];p.write_text(s)
print('Prepared one-file explicit point-definition candidate; production remains committed V788')
