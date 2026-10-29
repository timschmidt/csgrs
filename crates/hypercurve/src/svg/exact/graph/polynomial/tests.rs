use super::*;
use hypersolve::bivariate_topology::BivariateTopologyError;

fn source() -> PolynomialCurveSet2 {
    let equation = hypersolve::BivariatePolynomial::new(vec![
        vec![Real::one(), Real::from(2)],
        vec![],
        vec![Real::from(-2)],
    ]);
    let domain = BivariateCurveDomain {
        first: [(-1).into(), 1.into()],
        second: [(-1).into(), 1.into()],
    };
    PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(&equation, &domain, BivariateTopologyConfig::default()).unwrap(),
        AlgebraicFiberRootIsolationConfig::default(),
    )
    .unwrap()
}

fn read(bytes: &[u8], count: u32) -> SvgResult<Vec<Definition>> {
    let mut input = count.to_be_bytes().to_vec();
    input.extend_from_slice(bytes);
    let mut reader = ExactReader::new(&input, 1_048_576);
    let definitions = read_definitions(&mut reader, 256)?;
    assert!(reader.is_empty());
    Ok(definitions)
}

#[test]
fn polynomial_records_rebuild_topology_and_check_every_reference() {
    let source = source();
    let curve = &source.curves()[0];
    let mut writer = GraphWriter::new(1_048_576, 256);
    let reference = writer.curve(curve).unwrap();
    let decoded = read(&writer.writer.bytes, writer.count).unwrap();
    let restored = curve_reference(&decoded, reference as usize).unwrap();
    assert!(restored.start().coincides_with(&curve.start()).unwrap());
    assert!(restored.end().coincides_with(&curve.end()).unwrap());
    assert_eq!(
        restored.polynomial_branch().unwrap().topology(),
        source.topology()
    );
    // This untransformed record has five u32 fields, a traversal byte,
    // an empty map collection, and an absent-transform byte after its tag.
    let offset = writer.writer.bytes.len() - 27;
    assert_eq!(writer.writer.bytes[offset], POLYNOMIAL_BRANCH);
    for (field, invalid) in [(1, u32::MAX), (5, 99), (9, 99), (13, 0), (17, 0)] {
        let mut bytes = writer.writer.bytes.clone();
        bytes[offset + field..offset + field + 4].copy_from_slice(&invalid.to_be_bytes());
        assert!(matches!(
            read(&bytes, writer.count),
            Err(SvgError::MalformedInput(_))
        ));
    }
}

#[test]
fn polynomial_replay_retains_typed_limits_and_allows_retry() {
    let source = source();
    let mut writer = GraphWriter::new(1_048_576, 256);
    writer
        .polynomial_source(source.curves()[0].polynomial_branch().unwrap())
        .unwrap();
    let mut limited = writer.writer.bytes.clone();
    // The seven declared construction/evaluation limits terminate this source.
    let config = limited.len() - 7 * 4;
    limited[config..config + 4].copy_from_slice(&0_u32.to_be_bytes());
    assert!(matches!(
        read(&limited, writer.count),
        Err(SvgError::PolynomialTopology(
            BivariateTopologyError::DegreeLimit
        ))
    ));
    let denied =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
            assert!(matches!(
                read(&writer.writer.bytes, writer.count),
                Err(SvgError::PolynomialTopology(
                    BivariateTopologyError::BudgetExhausted
                ))
            ));
        });
    assert!(denied.result.is_err());
    assert!(read(&writer.writer.bytes, writer.count).is_ok());
}

#[test]
fn unused_polynomial_sources_are_validated() {
    let mut writer = GraphWriter::new(1_048_576, 256);
    writer.begin(POLYNOMIAL_SOURCE).unwrap();
    writer
        .polynomial(&hypersolve::BivariatePolynomial::new(vec![vec![
            Real::zero(),
        ]]))
        .unwrap();
    for value in [-1, 1, -1, 1] {
        writer.writer.write_real(&Real::from(value)).unwrap();
    }
    for limit in [8, 128, 128, 128, 64, 128, 64] {
        writer.writer.write_len(limit).unwrap();
    }
    assert!(matches!(
        read(&writer.writer.bytes, writer.count),
        Err(SvgError::PolynomialTopology(
            BivariateTopologyError::ZeroPolynomial
        ))
    ));
}
