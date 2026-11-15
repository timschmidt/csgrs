#![cfg(all(feature = "enclosure-program", feature = "hypervoxel-adapter"))]
use hyperinterval::DyadicBudget;
use hypersdf::{Sdf, SdfExpr, SdfProgram};
use hypersolve::expression_program::ExpressionBudget;
use hypervoxel::GridFrame;
use std::sync::Arc;
#[test]
fn native_source_classifies_a_complete_voxel_frame() {
    let mut graph = ExpressionBudget::new(10000, 1000000);
    let p = SdfProgram::compile(
        Arc::new(Sdf::new(SdfExpr::constant((-1).into()))),
        &mut graph,
    )
    .unwrap();
    let report = p
        .classify_voxel_grid(
            GridFrame::unit(0).unwrap(),
            Default::default(),
            &mut graph,
            &mut DyadicBudget::new(4096, 1_000_000_000),
        )
        .unwrap();
    assert_eq!(report.cells().len(), 1);
}

use hyperinterval::{DyadicInterval, IntervalError};
use hyperlattice::Matrix4;
use hyperlimit::Point3;
use hyperreal::{Rational, Real};
use hypersdf::{
    SdfCellLocation as Location, SdfProgramError, SdfVoxelCellGrid, SdfVoxelEnclosureBatch,
    SdfVoxelEnclosureError as Error, SdfVoxelEnclosureOptions as Options, SdfVoxelLengthUnit,
    SdfVoxelOccupancy,
};
use hypersolve::{expression_program::ExpressionError, work_budget::CancellationToken};
use hypervoxel::{LengthUnit, MaterialRegionId, OccupancyState, VoxelCell};
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn program(expr: SdfExpr) -> SdfProgram {
    SdfProgram::compile(Arc::new(Sdf::new(expr)), &mut graph()).unwrap()
}
fn options() -> Options {
    Options {
        precision: -24,
        max_cells: 512,
    }
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn report(p: &SdfProgram, f: GridFrame) -> SdfVoxelEnclosureBatch {
    p.classify_voxel_grid(f, options(), &mut graph(), &mut arithmetic())
        .unwrap()
}
fn ratio(a: i64, b: u64) -> Real {
    Rational::fraction(a, b).unwrap().into()
}

#[test]
fn trigonometric_whole_cells_complete_where_legacy_is_unknown() {
    let p = program(SdfExpr::x().sin().add_expr(SdfExpr::y().cos()).complement());
    let f = GridFrame::new(
        [0.into(), 0.into(), 0.into()],
        [ratio(1, 8), ratio(1, 8), ratio(1, 8)],
        1,
        LengthUnit::Millimeter,
    )
    .unwrap();
    let legacy = p
        .source()
        .classify_voxel_grid(
            SdfVoxelCellGrid::new(
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(ratio(1, 8), ratio(1, 8), ratio(1, 8)),
                [2; 3],
            )
            .with_units(SdfVoxelLengthUnit::Millimeter),
        )
        .unwrap();
    assert!(
        legacy
            .cells
            .iter()
            .all(|c| c.occupancy == SdfVoxelOccupancy::Unknown)
    );
    let native = report(&p, f.clone());
    assert_eq!(native.frame(), &f);
    assert_eq!(native.options(), options());
    assert!(Arc::ptr_eq(native.source().source(), p.source()));
    // On [0,1/4], sin(x)>=0 and cos(y)>0, so -(sin(x)+cos(y))<0.
    assert_eq!(native.cells().len(), 8);
    for (i, row) in native.cells().iter().enumerate() {
        assert_eq!(
            row.address().xyz,
            [(i % 2) as u64, ((i / 2) % 2) as u64, (i / 4) as u64]
        );
        assert_eq!(
            row.enclosure().location(),
            Some(Location::ConservativeInside)
        );
        assert_eq!(row.enclosure().metric_status(), p.source().metric_status());
    }
    let intake = native
        .continuous_field_batch(MaterialRegionId(19), &mut graph())
        .unwrap();
    let grid = intake.materialize_exact_sparse_grid().unwrap();
    for row in native.cells() {
        assert_eq!(
            grid.get(row.address()).unwrap(),
            VoxelCell::material(MaterialRegionId(19))
        );
    }
    assert_eq!(grid.frame().units(), LengthUnit::Millimeter);
}

#[test]
fn literal_polynomial_ranges_and_reconstructed_address_bounds_agree() {
    let p = program(
        SdfExpr::x()
            .add_expr(SdfExpr::constant(2.into()).mul_expr(SdfExpr::y()))
            .sub_expr(SdfExpr::constant(3.into()).mul_expr(SdfExpr::z()))
            .sub_expr(SdfExpr::constant(20.into())),
    );
    let f = GridFrame::new(
        [(-2).into(), 3.into(), 5.into()],
        [1.into(), 2.into(), 3.into()],
        1,
        LengthUnit::Unitless,
    )
    .unwrap();
    let native = report(&p, f.clone());
    for row in native.cells() {
        let [x, y, z] = row.address().xyz.map(|v| v as i64);
        let lo = (-2 + x) + 2 * (3 + 2 * y) - 3 * (8 + 3 * z) - 20;
        let hi = (-1 + x) + 2 * (5 + 2 * y) - 3 * (5 + 3 * z) - 20;
        assert_eq!(row.enclosure().image(), &d(lo, hi, 0));
        assert_eq!(
            row.enclosure().cell(),
            &[
                d(x - 2, x - 1, 0),
                d(3 + 2 * y, 5 + 2 * y, 0),
                d(5 + 3 * z, 8 + 3 * z, 0)
            ]
        );
        let exact = row.address().bounds(&f).unwrap();
        assert_eq!(
            exact.min,
            [
                Real::from(x - 2),
                Real::from(3 + 2 * y),
                Real::from(5 + 3 * z)
            ]
        );
        assert_eq!(
            exact.max,
            [
                Real::from(x - 1),
                Real::from(5 + 2 * y),
                Real::from(8 + 3 * z)
            ]
        );
        let next = p
            .image(
                row.enclosure().cell().clone(),
                -24,
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap();
        assert_eq!(next.image(), row.enclosure().image());
    }
    let intake = native
        .continuous_field_batch(MaterialRegionId(2), &mut graph())
        .unwrap();
    assert_eq!(intake.materialize_exact_sparse_grid().unwrap().len(), 8);
}

#[test]
fn exact_zero_and_zero_overlap_have_different_storage_admission() {
    let f = GridFrame::new(
        core::array::from_fn(|_| (-1).into()),
        core::array::from_fn(|_| 1.into()),
        1,
        LengthUnit::Unitless,
    )
    .unwrap();
    let zero = report(&program(SdfExpr::constant(0.into())), f.clone());
    assert!(
        zero.cells()
            .iter()
            .all(|c| c.enclosure().location() == Some(Location::Boundary))
    );
    zero.continuous_field_batch(MaterialRegionId(1), &mut graph())
        .unwrap()
        .materialize_exact_sparse_grid()
        .unwrap();
    let crossing = report(&program(SdfExpr::x()), f);
    assert!(
        crossing
            .cells()
            .iter()
            .all(|c| c.enclosure().location() == Some(Location::Unknown))
    );
    let mut intake = crossing
        .continuous_field_batch(MaterialRegionId(1), &mut graph())
        .unwrap();
    assert!(
        intake
            .cells
            .iter()
            .all(|c| c.cell.occupancy == OccupancyState::Unknown)
    );
    assert!(intake.materialize_exact_sparse_grid().is_err());
    assert_eq!(intake.materialize_sparse_grid().unwrap().len(), 8);
    intake.cells[0].cell = VoxelCell::material(MaterialRegionId(99));
    assert_eq!(
        crossing.cells()[0].enclosure().location(),
        Some(Location::Unknown)
    );
    assert_eq!(
        crossing
            .continuous_field_batch(MaterialRegionId(1), &mut graph())
            .unwrap()
            .cells[0]
            .cell
            .occupancy,
        OccupancyState::Unknown
    );
}

#[test]
fn algebraic_frames_and_transformed_sources_retain_their_meaning() {
    let f = GridFrame::new(
        [Real::from(2).sqrt().unwrap(), 0.into(), 0.into()],
        [Real::from(3).sqrt().unwrap(), 1.into(), 1.into()],
        0,
        LengthUnit::Meter,
    )
    .unwrap();
    let p = program(SdfExpr::x());
    let native = report(&p, f.clone());
    assert_eq!(native.frame(), &f);
    let c = native.cells()[0].enclosure();
    assert_eq!(c.location(), Some(Location::ConservativeOutside));
    // sqrt(2)>1 and sqrt(2)+sqrt(3)<4; outward conversion remains within [1,4].
    assert!(
        c.cell()[0]
            .subset_of(&d(1, 4, 0), &mut arithmetic())
            .unwrap()
    );
    assert!(!c.cell()[0].is_singleton());
    let mut transform = Matrix4::identity();
    transform.0[0][0] = 2.into();
    transform.0[0][3] = 3.into();
    let affine = program(SdfExpr::x().affine_transform(transform).unwrap());
    let f = GridFrame::new(
        [(-1).into(), 0.into(), 0.into()],
        [ratio(1, 2), 1.into(), 1.into()],
        1,
        LengthUnit::Nanometer,
    )
    .unwrap();
    let native = report(&affine, f);
    for row in native.cells() {
        let x = row.address().xyz[0] as i64;
        assert_eq!(row.enclosure().image(), &d(-8 + x, -7 + x, -2));
        assert!(Arc::ptr_eq(
            row.enclosure().source().source(),
            affine.source()
        ));
    }
    native
        .continuous_field_batch(MaterialRegionId(7), &mut graph())
        .unwrap()
        .materialize_exact_sparse_grid()
        .unwrap();
}

#[test]
fn capacity_size_work_and_conversion_failures_leave_retry_independent() {
    let p = program(SdfExpr::x());
    let f = GridFrame::unit(1).unwrap();
    assert!(matches!(
        p.classify_voxel_grid(
            f.clone(),
            Options {
                max_cells: 7,
                ..options()
            },
            &mut graph(),
            &mut arithmetic()
        ),
        Err(Error::Capacity {
            limit: 7,
            required: 8
        })
    ));
    assert!(matches!(
        p.classify_voxel_grid(
            GridFrame::unit(21).unwrap(),
            options(),
            &mut graph(),
            &mut arithmetic()
        ),
        Err(Error::Capacity {
            required: 9_223_372_036_854_775_808,
            ..
        })
    ));
    assert!(matches!(
        p.classify_voxel_grid(
            f.clone(),
            options(),
            &mut ExpressionBudget::new(0, 1000000),
            &mut arithmetic()
        ),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    assert!(matches!(
        p.classify_voxel_grid(
            f.clone(),
            options(),
            &mut ExpressionBudget::new(10000, 0),
            &mut arithmetic()
        ),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(matches!(
        p.classify_voxel_grid(
            f.clone(),
            options(),
            &mut graph(),
            &mut DyadicBudget::new(0, 1000000)
        ),
        Err(Error::Coordinate {
            axis: 0,
            source: IntervalError::SizeExhausted,
            ..
        })
    ));
    assert!(matches!(
        p.classify_voxel_grid(
            f.clone(),
            options(),
            &mut graph(),
            &mut DyadicBudget::new(10000, 0)
        ),
        Err(Error::Coordinate {
            axis: 0,
            source: IntervalError::WorkExhausted,
            ..
        })
    ));
    assert_eq!(report(&p, f).cells().len(), 8);
}

#[test]
fn final_work_charge_is_required_for_complete_publication() {
    let p = program(SdfExpr::x());
    let f = GridFrame::unit(1).unwrap();
    let mut measured = graph();
    let mut scalar = arithmetic();
    let result = p
        .classify_voxel_grid(f.clone(), options(), &mut measured, &mut scalar)
        .unwrap();
    let work = 100_000_000 - measured.remaining_work();
    let arithmetic_work = 1_000_000_000_000 - scalar.remaining_work();
    let mut exact = ExpressionBudget::new(1_000_000, work);
    let mut exact_scalar = DyadicBudget::new(16384, arithmetic_work);
    p.classify_voxel_grid(f.clone(), options(), &mut exact, &mut exact_scalar)
        .unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(exact_scalar.remaining_work(), 0);
    let mut short = ExpressionBudget::new(1_000_000, work - 1);
    assert!(matches!(
        p.classify_voxel_grid(f.clone(), options(), &mut short, &mut arithmetic()),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert_eq!(short.remaining_work(), 0);
    let mut export_short = ExpressionBudget::new(10000, 16);
    assert!(matches!(
        result.continuous_field_batch(MaterialRegionId(3), &mut export_short),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert_eq!(export_short.remaining_work(), 0);
    let mut export_exact = ExpressionBudget::new(10000, 17);
    assert_eq!(
        result
            .continuous_field_batch(MaterialRegionId(3), &mut export_exact)
            .unwrap()
            .cells
            .len(),
        8
    );
    assert_eq!(export_exact.remaining_work(), 0);
    assert_eq!(report(&p, f).cells().len(), 8);
}

#[test]
fn cancellation_before_evaluation_and_between_evaluation_and_export() {
    let p = program(SdfExpr::constant((-1).into()));
    let f = GridFrame::unit(1).unwrap();
    let token = CancellationToken::new();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token.clone());
    token.cancel();
    assert!(matches!(
        p.classify_voxel_grid(f.clone(), options(), &mut cancelled, &mut arithmetic()),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    let result = report(&p, f.clone());
    let mut active = graph();
    let token = CancellationToken::new();
    active.cancellation = Some(token.clone());
    token.cancel();
    assert!(matches!(
        result.continuous_field_batch(MaterialRegionId(1), &mut active),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    result
        .continuous_field_batch(MaterialRegionId(1), &mut graph())
        .unwrap()
        .materialize_exact_sparse_grid()
        .unwrap();
    assert_eq!(report(&p, f).cells().len(), 8);
}

#[test]
fn fresh_process_reconstructs_native_source_before_storage() {
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "trigonometric_whole_cells_complete_where_legacy_is_unknown",
            "--nocapture",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("1 passed"));
}

#[test]
fn whole_domain_failure_aborts_instead_of_publishing_partial_occupancy() {
    let p = program(SdfExpr::x().sqrt());
    let f = GridFrame::new(
        [(-2).into(), 0.into(), 0.into()],
        [1.into(), 1.into(), 1.into()],
        1,
        LengthUnit::Unitless,
    )
    .unwrap();
    assert!(matches!(
        p.classify_voxel_grid(f, options(), &mut graph(), &mut arithmetic()),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::Enclosure {
                source: IntervalError::EmptyDomain,
                ..
            }
        )))
    ));
    let partial = GridFrame::new(
        [(-1).into(), 0.into(), 0.into()],
        [2.into(), 1.into(), 1.into()],
        0,
        LengthUnit::Unitless,
    )
    .unwrap();
    assert!(matches!(
        p.classify_voxel_grid(partial, options(), &mut graph(), &mut arithmetic()),
        Err(Error::Program(SdfProgramError::Expression(
            ExpressionError::DomainNotCertified { .. }
        )))
    ));
    let retry = report(&p, GridFrame::unit(1).unwrap());
    assert_eq!(retry.cells().len(), 8);
}

#[test]
fn independent_concurrent_attempts_keep_source_and_cancellation_separate() {
    let p = program(SdfExpr::constant((-1).into()));
    std::thread::scope(|scope| {
        let good = scope.spawn(|| report(&p, GridFrame::unit(1).unwrap()));
        let failed = scope.spawn(|| {
            let mut budget = graph();
            let token = CancellationToken::new();
            token.cancel();
            budget.cancellation = Some(token);
            p.classify_voxel_grid(
                GridFrame::unit(1).unwrap(),
                options(),
                &mut budget,
                &mut arithmetic(),
            )
        });
        let result = good.join().unwrap();
        assert!(matches!(
            failed.join().unwrap(),
            Err(Error::Program(SdfProgramError::Expression(
                ExpressionError::Cancelled
            )))
        ));
        assert!(Arc::ptr_eq(result.source().source(), p.source()));
        result
            .continuous_field_batch(MaterialRegionId(2), &mut graph())
            .unwrap()
            .materialize_exact_sparse_grid()
            .unwrap();
    });
}

#[cfg(target_pointer_width = "64")]
#[test]
fn impossible_record_buffer_is_a_typed_storage_failure() {
    let p = program(SdfExpr::constant(1.into()));
    let error = p
        .classify_voxel_grid(
            GridFrame::unit(21).unwrap(),
            Options {
                max_cells: usize::MAX,
                ..options()
            },
            &mut ExpressionBudget::new(usize::MAX, usize::MAX),
            &mut arithmetic(),
        )
        .unwrap_err();
    assert!(matches!(error, Error::Storage(_)));
    assert!(std::error::Error::source(&error).is_some());
    assert_eq!(report(&p, GridFrame::unit(0).unwrap()).cells().len(), 1);
}

#[test]
fn complete_outside_report_materializes_successful_empty_geometry() {
    let p = program(SdfExpr::constant(1.into()));
    let proof = report(&p, GridFrame::unit(1).unwrap());
    let intake = proof
        .continuous_field_batch(MaterialRegionId(8), &mut graph())
        .unwrap();
    let empty = intake.materialize_exact_sparse_grid().unwrap();
    assert!(empty.is_empty());
    for cell in proof.cells() {
        assert_eq!(empty.get(cell.address()).unwrap(), VoxelCell::empty());
    }
    let replay = report(&p, empty.frame().clone());
    assert!(
        replay
            .cells()
            .iter()
            .all(|cell| cell.enclosure().location() == Some(Location::ConservativeOutside))
    );
}
