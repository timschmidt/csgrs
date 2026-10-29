#![cfg(feature = "svg-io")]

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::io::{
    IoError,
    svg::{export_svg, import_svg},
};
use hypercurve::{CurvePath2, CurveRegion2, PolynomialCurveSet2, Real};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, BivariateTopologyError,
    decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::fmt::Write;

fn unused_source(equation: &[&[i32]], domain: [i32; 4], max_degree: u32) -> String {
    // A version-5 compound graph with one source and no boundary roots.
    // Every source is still validated before the empty region is published.
    let mut bytes = b"HCP5".to_vec();
    bytes.extend_from_slice(&1_u32.to_be_bytes());
    bytes.push(29);
    bytes.extend_from_slice(&u32::try_from(equation.len()).unwrap().to_be_bytes());
    let write_real = |bytes: &mut Vec<u8>, value| {
        let value = Real::from(value).to_bytes();
        bytes.extend_from_slice(&u32::try_from(value.len()).unwrap().to_be_bytes());
        bytes.extend(value);
    };
    for row in equation {
        bytes.extend_from_slice(&u32::try_from(row.len()).unwrap().to_be_bytes());
        for value in *row {
            write_real(&mut bytes, *value);
        }
    }
    for value in domain {
        write_real(&mut bytes, value);
    }
    for value in [max_degree, 128, 128, 128, 64, 128, 64] {
        bytes.extend_from_slice(&value.to_be_bytes());
    }
    bytes.extend_from_slice(&0_u32.to_be_bytes());
    let mut extension = String::from("5:");
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="black" fill-rule="evenodd" data-hypercurve-path="{extension}" d=""/></svg>"#
    )
}

fn expect_source_error(
    document: &str,
    expected: BivariateTopologyError,
    kind: ValidationErrorKind,
) {
    let error = import_svg(document).unwrap_err();
    let IoError::Construction {
        format: "SVG",
        source,
    } = error
    else {
        panic!("typed polynomial reconstruction error required: {error:?}");
    };
    assert_eq!(source.kind(), kind);
    assert!(matches!(source, ValidationError::PolynomialTopology(error) if error==expected));
}

#[test]
fn svg_adapter_preserves_polynomial_failure_categories() {
    let parabola: &[&[i32]] = &[&[1, 2], &[], &[-2]];
    expect_source_error(
        &unused_source(parabola, [1, -1, -1, 1], 8),
        BivariateTopologyError::InvalidDomain,
        ValidationErrorKind::InvalidInput,
    );
    expect_source_error(
        &unused_source(&[&[0]], [-1, 1, -1, 1], 8),
        BivariateTopologyError::ZeroPolynomial,
        ValidationErrorKind::InvalidInput,
    );
    expect_source_error(
        &unused_source(&[&[0], &[1]], [-1, 1, -1, 1], 8),
        BivariateTopologyError::VerticalComponent,
        ValidationErrorKind::Unsupported,
    );
    expect_source_error(
        &unused_source(parabola, [-1, 1, -1, 1], 0),
        BivariateTopologyError::DegreeLimit,
        ValidationErrorKind::ResourceExhausted,
    );
    let valid = unused_source(parabola, [-1, 1, -1, 1], 8);
    let denied = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget::ZERO,
        || {
            expect_source_error(
                &valid,
                BivariateTopologyError::BudgetExhausted,
                ValidationErrorKind::ResourceExhausted,
            );
        },
    );
    assert!(denied.result.is_err());
    let (region, wires, paths) = import_svg(&valid).unwrap();
    assert!(region.is_empty() && wires.is_empty() && paths.is_empty());
}

#[test]
fn svg_adapter_retains_polynomial_sources_for_further_subdivision() {
    let equation =
        BivariatePolynomial::new(vec![vec![1.into(), 2.into()], vec![], vec![(-2).into()]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 1.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let source = PolynomialCurveSet2::from_topology(
        topology,
        AlgebraicFiberRootIsolationConfig::default(),
    )
    .unwrap();
    let path = CurvePath2::try_new(source.curves().to_vec()).unwrap();
    let text = export_svg(&CurveRegion2::empty(), &[], &[path]).unwrap();
    let (region, wires, paths) = import_svg(&text).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    let curve = &paths[0].curves()[0];
    assert_eq!(
        curve.polynomial_branch().unwrap().topology().equation(),
        &equation
    );
    let (left, right) = curve.split_at(Real::zero().into()).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    assert!(
        left.start()
            .coincides_with(&source.curves()[0].start())
            .unwrap()
    );
    assert!(right.end().coincides_with(&source.curves()[0].end()).unwrap());
    assert_eq!(export_svg(&region, &wires, &paths).unwrap(), text);
}
