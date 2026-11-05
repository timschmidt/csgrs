use super::*;

fn fixture() -> Curve2 {
    let source = crate::QuadraticBezier2::new(
        crate::Point2::from_values(0, 0),
        crate::Point2::from_values(1, 0),
        crate::Point2::from_values(1, 1),
    );
    let path = CurvePath2::try_new(vec![source.into()]).unwrap();
    let options =
        BezierParallelVerificationOptions::try_new((Real::one() / Real::from(20)).unwrap(), 16)
            .unwrap();
    path.approximate_parallel_blend2d_certified((Real::one() / Real::from(10)).unwrap(), &options)
        .unwrap()
        .path()
        .curves()[0]
        .clone()
}

fn recipe(
    source_index: Option<usize>,
    start: Option<Real>,
    bound: Option<Real>,
    qualified: Option<bool>,
    depth_request: Option<usize>,
) -> Vec<u8> {
    let curve = fixture();
    let (receipt, _) = curve.parallel_fidelity_replay().unwrap();
    let mut writer = GraphWriter::new(1_048_576, 256);
    let construction = if qualified.is_some() || depth_request.is_some() {
        let old = &receipt.source.construction;
        let flag = OnceLock::new();
        flag.set(qualified.unwrap_or(*old.qualified.get().unwrap()))
            .unwrap();
        Arc::new(ParallelFidelityConstruction2 {
            path: old.path.clone(),
            distance: old.distance.clone(),
            max_verification_depth: depth_request.unwrap_or(old.max_verification_depth),
            qualified: flag,
        })
    } else {
        Arc::clone(&receipt.source.construction)
    };
    let source = writer.fidelity_construction(&construction).unwrap();
    let candidate = writer
        .native_curve(&Curve2::new(receipt.candidate.as_ref().unwrap().clone()))
        .unwrap();
    writer.begin(ROOT).unwrap();
    writer.writer.write_u32(source).unwrap();
    writer
        .writer
        .write_len(source_index.unwrap_or(receipt.source.index))
        .unwrap();
    writer.writer.write_u32(candidate).unwrap();
    writer
        .writer
        .write_real(&start.unwrap_or_else(|| receipt.source_range.start().clone()))
        .unwrap();
    writer
        .writer
        .write_real(receipt.source_range.end())
        .unwrap();
    writer.writer.write_bool(true).unwrap();
    writer
        .writer
        .write_real(&bound.unwrap_or_else(|| receipt.error_bound.clone()))
        .unwrap();
    let mut bytes = writer.count.to_be_bytes().to_vec();
    bytes.extend(writer.writer.bytes);
    bytes
}

fn read(bytes: &[u8], depth: usize, verification: usize) -> SvgResult<Vec<Definition>> {
    super::super::read_definitions(&mut ExactReader::new(bytes, 1_048_576), depth, verification)
}

#[test]
fn fidelity_replay_rejects_invalid_sources_ranges_and_error_bounds() {
    assert!(matches!(
        read(&recipe(Some(3), None, None, None, None), 256, 64),
        Err(SvgError::MalformedInput(_))
    ));
    assert!(matches!(
        read(
            &recipe(None, Some(Real::from(2)), None, None, None),
            256,
            64
        ),
        Err(SvgError::Exact(_))
    ));
    assert!(matches!(
        read(&recipe(None, None, Some(-Real::one()), None, None), 256, 64),
        Err(SvgError::Exact(_))
    ));
    let mut bytes = recipe(None, None, None, None, None);
    bytes[4] = 255;
    assert!(matches!(
        read(&bytes, 256, 64),
        Err(SvgError::Unsupported(_))
    ));
}

#[test]
fn fidelity_verification_depth_is_an_actual_replay_limit_and_retry_succeeds() {
    let bytes = recipe(None, None, None, None, Some(u32::MAX as usize));
    assert!(matches!(
        read(&bytes, 256, 1),
        Err(SvgError::SizeOverflow {
            limit: "parallel verification depth"
        })
    ));
    let definitions = read(&bytes, 256, 64).unwrap();
    let Definition::Curve(curve) = definitions.last().unwrap() else {
        panic!("checked curve")
    };
    let receipt = curve.parallel_fidelity().unwrap();
    assert_eq!(receipt.construction_depth_request(), u32::MAX as usize);
    assert!(receipt.verification_leaf_count() > 1);
    assert_eq!(
        receipt.construction_certainty(),
        crate::CurveCertainty::Certified
    );
}

#[test]
fn fidelity_dependency_depth_and_weaker_qualification_are_preserved() {
    let bytes = recipe(None, None, None, Some(true), None);
    assert!(matches!(
        read(&bytes, 1, 64),
        Err(SvgError::SizeOverflow {
            limit: "retained fidelity definition depth"
        })
    ));
    let definitions = read(&bytes, 256, 64).unwrap();
    let Definition::Curve(curve) = definitions.last().unwrap() else {
        panic!("checked curve")
    };
    // A strict recheck is still required. A weaker historical qualification is
    // retained rather than silently upgraded or used to authorize verification.
    assert_eq!(
        curve.parallel_fidelity().unwrap().construction_certainty(),
        crate::CurveCertainty::Approximate512Consumed
    );
}

#[test]
fn fidelity_replay_observes_cancellation_and_does_not_poison_retry() {
    use hypersolve::work_budget::{
        CancellationToken, WorkBudget, WorkKind, with_cancellable_budget,
    };
    let bytes = recipe(None, None, None, None, None);
    let token = CancellationToken::new();
    token.cancel();
    let mut blocked = false;
    let attempt = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
        blocked = matches!(read(&bytes,256,64),Err(SvgError::Exact(ExactCurveError::Blocked(blocker))) if blocker.reason()==crate::UncertaintyReason::BudgetExhausted);
    });
    assert!(blocked);
    assert_eq!(attempt.result.unwrap_err().kind, WorkKind::Cancellation);
    assert!(read(&bytes, 256, 64).is_ok());
}
