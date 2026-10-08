#![allow(unused_imports)]
use hypercurve::*;
fn p(x:i64,y:i64)->Point2{Point2::new(Real::from(x),Real::from(y))}
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
trait IntoCertifiedClassification<T> {
    fn into_certified_classification(self) -> Classification<T>;
}

impl<T> IntoCertifiedClassification<T> for Classification<T> {
    fn into_certified_classification(self) -> Classification<T> {
        self
    }
}

impl<T> IntoCertifiedClassification<T> for CurveOutcome<Classification<T>> {
    fn into_certified_classification(self) -> Classification<T> {
        assert_eq!(self.certainty, CurveCertainty::Certified);
        self.value
    }
}

fn decided<T>(classification: impl IntoCertifiedClassification<T>) -> T {
    match classification.into_certified_classification() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected decided result, got {reason:?}"),
    }
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}
fn axis_aligned_algebraic_rectangle(policy: &CurveContext) -> CurveRegion2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            vec![-q(1, 2), Real::zero(), Real::one()],
            policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
    let horizontal = |height: Real| {
        RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), height.clone()),
                Point2::new(Real::one(), height),
            ],
            vec![Real::one(); 2],
        )
        .unwrap()
    };
    let bottom_right = CurvePoint2::from(
        horizontal(Real::zero())
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
    );
    let top_right = CurvePoint2::from(
        horizontal(Real::one())
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
    );
    let bottom_left = CurvePoint2::from(p(0, 0));
    let top_left = CurvePoint2::from(p(0, 1));
    let chord = |start, end| {
        BezierSplitFragment2::AlgebraicChord(decided(
            BezierAlgebraicChord2::try_new(start, end, policy).unwrap(),
        ))
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            chord(bottom_left.clone(), bottom_right.clone()),
            chord(bottom_right, top_right.clone()),
            chord(top_right, top_left.clone()),
            chord(top_left, bottom_left),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

fn independent_rounded_rectangle(distance: &Real, policy: &CurveContext) -> CurveRegion2 {
    let width=q(1,2).sqrt().unwrap();
    let centers=[p(0,0),Point2::new(width.clone(),Real::zero()),Point2::new(width.clone(),Real::one()),p(0,1)];
    let points=[Point2::new(Real::zero(),-distance),Point2::new(width.clone(),-distance),Point2::new(&width+distance,Real::zero()),Point2::new(&width+distance,Real::one()),Point2::new(width,Real::one()+distance),Point2::new(Real::zero(),Real::one()+distance),Point2::new(-distance,Real::one()),Point2::new(-distance,Real::zero())];
    let segments=(0..8).map(|i| if i%2==0 {Segment2::Line(LineSeg2::try_new(points[i].clone(),points[(i+1)%8].clone()).unwrap())} else {Segment2::Arc(CircularArc2::try_from_center(points[i].clone(),points[(i+1)%8].clone(),centers[((i+1)/2)%4].clone(),false).unwrap())}).collect();
    CurveRegion2::try_from_native_material_contours(vec![Contour2::try_new(segments).unwrap()],policy).unwrap().into_value()
}
fn main() {
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let distance=q(1,10);
        let source=axis_aligned_algebraic_rectangle(&policy);
        let rounded=source.offset(distance.clone(),&OffsetCornerStyle2::Round,&policy).unwrap();
        assert_eq!(rounded.certainty,CurveCertainty::Certified);
        let expected=independent_rounded_rectangle(&distance,&policy);
        let xor=rounded.value.boolean_region(&expected,BooleanOp::Xor,&policy);
        match xor {
            Ok(value)=>{println!("policy={policy:?} certainty={:?} empty={}",value.certainty,value.value.is_empty()); assert_eq!(value.certainty,CurveCertainty::Certified); assert!(value.value.is_empty());},
            Err(error)=>panic!("independent round rectangle comparison: {error:?}")
        }
    }
}
