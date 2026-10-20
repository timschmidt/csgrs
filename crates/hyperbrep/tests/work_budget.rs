use hyperbrep::{
    BudgetExceeded, Curve3, CurveParameterLocation, GeometryError, OperationContext, Point3, Real,
    STRICT_PREDICATES, WorkBudget, WorkKind, WorkUsage,
};

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn five_root_curve() -> Curve3 {
    // Bernstein form of the monic product with roots 1/8, 1/4, 1/2, 3/4, 7/8.
    let controls = [-105, 241, -381, 381, -241, 105]
        .map(|n| Point3::new(rational(n, 10240), Real::zero(), Real::zero()));
    Curve3::rational_bezier(controls.to_vec(), vec![Real::one(); 6]).unwrap()
}

#[test]
fn brep_parameter_location_retries_after_resource_exhaustion() {
    let curve = five_root_curve();
    let zero = OperationContext::strict(WorkBudget::ZERO);
    assert_eq!(zero.predicate_policy(), STRICT_PREDICATES);
    let denied = zero.evaluate(|| curve.parameters_of(&Point3::origin()));
    assert_eq!(
        denied.result.unwrap_err(),
        BudgetExceeded {
            kind: WorkKind::RootRefinement,
            limit: 0,
            used: 0,
            requested: 1,
        }
    );
    assert_eq!(denied.usage, WorkUsage::default());
    let enough = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 0,
    });
    let retry = enough.evaluate(|| curve.parameters_of(&Point3::origin()));
    assert!(retry.usage.refinements > 0);
    assert_eq!(retry.usage.promotions, 0);
    let CurveParameterLocation::Parameters(parameters) = retry.result.unwrap().unwrap() else {
        panic!("the complete curve has five exact preimages of the origin");
    };
    assert_eq!(parameters.len(), 5);
    for (parameter, expected) in parameters
        .iter()
        .zip([1, 2, 4, 6, 7].map(|n| rational(n, 8)))
    {
        assert_eq!(parameter.try_real(), Some(&expected));
    }
}

#[test]
fn nested_brep_queries_share_the_parent_allowance() {
    let curve = five_root_curve();
    let enough = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 0,
    });
    let single = enough.evaluate(|| curve.parameters_of(&Point3::origin()));
    single.result.unwrap().unwrap();
    assert!(single.usage.refinements > 0);
    let parent = OperationContext::strict(WorkBudget {
        refinements: single.usage.refinements,
        promotions: 0,
    });
    let attempted = parent.evaluate(|| {
        let first = enough.evaluate(|| curve.parameters_of(&Point3::origin()));
        first.result.unwrap().unwrap();
        assert_eq!(first.usage, single.usage);
        let second = enough.evaluate(|| curve.parameters_of(&Point3::origin()));
        assert!(second.result.is_err());
        // An ignored nested failure cannot publish success in the parent.
        42
    });
    assert_eq!(attempted.usage, single.usage);
    let denied = attempted.result.unwrap_err();
    assert_eq!(denied.kind, WorkKind::RootRefinement);
    assert_eq!(denied.limit, single.usage.refinements);
    assert_eq!(denied.used, single.usage.refinements);
    assert_eq!(denied.requested, 1);
}

#[test]
fn a_smaller_child_can_fail_without_exhausting_its_parent() {
    let curve = five_root_curve();
    let parent = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 0,
    });
    let attempted = parent.evaluate(|| {
        let child = OperationContext::strict(WorkBudget::ZERO)
            .evaluate(|| curve.parameters_of(&Point3::origin()));
        assert!(child.result.is_err());
        curve.parameters_of(&Point3::origin())
    });
    assert!(attempted.usage.refinements > 0);
    assert!(matches!(
        attempted.result.unwrap().unwrap(),
        CurveParameterLocation::Parameters(parameters) if parameters.len() == 5
    ));
    assert!(!hypersolve::work_budget::is_active());
}

#[test]
fn cheap_geometry_and_domain_errors_remain_distinct_from_resource_failure() {
    let zero = OperationContext::strict(WorkBudget::ZERO);
    let line = Curve3::line(
        Point3::origin(),
        Point3::new(Real::one(), Real::one(), Real::one()),
    )
    .unwrap();
    let bounds = line.bounds().unwrap();
    let warm = zero.evaluate(|| line.bounds());
    assert_eq!(warm.result.unwrap().unwrap(), bounds);
    assert_eq!(warm.usage, WorkUsage::default());
    let invalid = zero.evaluate(|| Curve3::line(Point3::origin(), Point3::origin()));
    assert_eq!(
        invalid.result.unwrap().unwrap_err(),
        GeometryError::DegenerateLine
    );
    assert_eq!(invalid.usage, WorkUsage::default());
}

#[test]
fn explicit_context_suspends_and_restores_an_ambient_approximation() {
    use hypercurve::{Curve2, CurveCertainty, LineSeg2, Point2};

    let curve = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let tiny: Real = "1e-2000".parse().unwrap();
    let parameter = Real::one() + (Real::pi() + &tiny + &tiny).exp().unwrap()
        - (Real::pi() + tiny).exp().unwrap();
    let zero = OperationContext::strict(WorkBudget::ZERO);
    let outer = hypercurve::provisional(|| {
        let attempted = zero.evaluate(|| curve.point_at(&parameter.clone().into()));
        assert!(attempted.result.unwrap().is_err());
        assert_eq!(attempted.usage, WorkUsage::default());
        curve.point_at(&parameter.into())
    });
    assert_eq!(outer.certainty(), CurveCertainty::Approximate512Consumed);
    assert!(outer.into_unverified().is_ok());
}

#[test]
fn unwinding_restores_scope_without_refunding_parent_work() {
    use hypersolve::work_budget::{consume, is_active};

    let parent = OperationContext::strict(WorkBudget {
        refinements: 2,
        promotions: 0,
    });
    let child = OperationContext::strict(WorkBudget {
        refinements: 1,
        promotions: 0,
    });
    let attempted = parent.evaluate(|| {
        let panic = std::panic::catch_unwind(|| {
            child.evaluate(|| {
                consume(WorkKind::RootRefinement, 1).unwrap();
                panic!("deliberate unwind after admitted work");
            })
        });
        assert!(panic.is_err());
        consume(WorkKind::RootRefinement, 1).unwrap();
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
    });
    assert_eq!(attempted.usage.refinements, 2);
    assert_eq!(attempted.result.unwrap_err().used, 2);
    assert!(!is_active());
    let fresh = child.evaluate(|| consume(WorkKind::RootRefinement, 1));
    fresh.result.unwrap().unwrap();
    assert_eq!(fresh.usage.refinements, 1);
}

#[test]
fn promotion_exhaustion_cannot_publish_a_partial_curve_intersection() {
    use hypercurve::{Curve2, Point2, RationalBezier2};

    let curve = |points: [(i32, i32); 4]| {
        Curve2::from(
            RationalBezier2::try_new(
                points
                    .into_iter()
                    .map(|(x, y)| Point2::from_values(x, y))
                    .collect(),
                [1, 2, 2, 1].map(Real::from).to_vec(),
            )
            .unwrap(),
        )
    };
    let first = curve([(0, 0), (0, 1), (1, 0), (1, 1)]);
    let second = curve([(0, 1), (0, 0), (1, 1), (1, 0)]);
    let limited = OperationContext::strict(WorkBudget {
        refinements: 4096,
        promotions: 0,
    });
    let denied = limited.evaluate(|| {
        let partial = first.intersect_curve(&second).unwrap();
        assert!(!partial.is_complete());
        partial
    });
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 0);
    let retry = OperationContext::strict(WorkBudget {
        refinements: 4096,
        promotions: 128,
    })
    .evaluate(|| first.intersect_curve(&second));
    assert!(retry.usage.promotions > 0);
    let intersection = retry.result.unwrap().unwrap();
    assert!(intersection.is_complete());
    assert!(intersection.blockers().is_empty());
    // Equal x parameters reduce the y equality to (2t-1)(7t²-7t+1)=0.
    assert_eq!(intersection.contacts().len(), 3);
}
