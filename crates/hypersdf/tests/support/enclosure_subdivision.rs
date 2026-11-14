use super::*;
use hyperinterval::IntervalError;
use hyperlattice::{BoxN, IntervalLatticeError, Matrix4};
use hypersdf::{SdfSubdivision, SdfSubdivisionOptions, SdfUnresolvedCell};
use std::error::Error;
fn options(depth: u32, cells: usize) -> SdfSubdivisionOptions {
    SdfSubdivisionOptions {
        precision: -48,
        max_depth: depth,
        max_cells: cells,
    }
}
fn dependency_field() -> SdfProgram {
    program(logistic().offset((Real::from(3) / Real::from(8)).unwrap()))
}
fn dependency_cell() -> [DyadicInterval; 3] {
    [d(0, 1, 0), zero(), zero()]
}
fn verify_cover(cover: &SdfSubdivision) {
    for (index, cell) in cover.cells().iter().enumerate() {
        if let Some((axis, [left, right])) = cell.split() {
            assert!(left > index && right > left);
            assert!(cell.unresolved().is_none());
            let l = &cover.cells()[left];
            let r = &cover.cells()[right];
            assert_eq!(l.depth(), cell.depth() + 1);
            assert_eq!(r.depth(), cell.depth() + 1);
            for i in 0..3 {
                let p = &cell.enclosure().cell()[i];
                let a = &l.enclosure().cell()[i];
                let b = &r.enclosure().cell()[i];
                if i == axis {
                    assert_ne!(a, p);
                    assert_ne!(b, p);
                    assert_eq!(a.hull(b, &mut arithmetic()).unwrap(), *p);
                    assert_eq!(
                        a.intersection(b, &mut arithmetic()).unwrap(),
                        p.midpoint(&mut arithmetic()).unwrap()
                    );
                } else {
                    assert_eq!(a, p);
                    assert_eq!(b, p);
                }
            }
        } else {
            assert_eq!(
                cell.unresolved().is_some(),
                cell.enclosure().location() == Some(Location::Unknown)
            );
        }
        assert!(Arc::ptr_eq(
            cell.enclosure().source().source(),
            cover.cells()[0].enclosure().source().source()
        ));
    }
}
#[test]
fn complete_dependency_cover_has_independent_exact_leaves_and_reusable_evidence() {
    let p = dependency_field();
    let cover = p
        .subdivide(
            dependency_cell(),
            options(3, 11),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(cover.cells().len(), 11);
    assert_eq!(cover.leaves().count(), 6);
    assert_eq!(cover.options(), options(3, 11));
    assert_eq!(cover.cells()[0].enclosure().image(), &d(-3, 5, -3));
    assert_eq!(cover.location(), Some(Location::ConservativeInside));
    verify_cover(&cover);
    let expected = [(0, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8)];
    let actual = cover
        .leaves()
        .map(|(_, c)| c.enclosure().cell()[0].clone())
        .collect::<Vec<_>>();
    for (lo, hi) in expected {
        assert!(actual.contains(&d(lo, hi, -3)));
    }
    for (_, cell) in cover.leaves() {
        assert_eq!(
            cell.enclosure().location(),
            Some(Location::ConservativeInside)
        );
        let replay = p
            .image(
                cell.enclosure().cell().clone(),
                -48,
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap();
        assert_eq!(replay.image(), cell.enclosure().image());
        assert_eq!(replay.metric_status(), cell.enclosure().metric_status());
    }
}
#[test]
fn unresolved_resolution_and_closed_boundary_contacts_remain_explicit() {
    let p = program(SdfExpr::x());
    let coarse = p
        .subdivide(
            [d(-2, 2, 0), zero(), zero()],
            options(0, 1),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(coarse.location(), Some(Location::Unknown));
    assert_eq!(
        coarse.cells()[0].unresolved(),
        Some(SdfUnresolvedCell::DepthLimit)
    );
    let cover = p
        .subdivide(
            [d(-2, 2, 0), zero(), zero()],
            options(2, 7),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    verify_cover(&cover);
    assert_eq!(cover.cells().len(), 7);
    assert_eq!(cover.location(), Some(Location::Unknown));
    let leaves = cover.leaves().map(|(_, c)| c).collect::<Vec<_>>();
    for (lo, hi, location) in [
        (-2, -1, Location::ConservativeInside),
        (-1, 0, Location::Unknown),
        (0, 1, Location::Unknown),
        (1, 2, Location::ConservativeOutside),
    ] {
        let leaf = leaves
            .iter()
            .find(|c| c.enclosure().cell()[0] == d(lo, hi, 0))
            .unwrap();
        assert_eq!(leaf.enclosure().location(), Some(location));
    }
    assert_eq!(
        leaves
            .iter()
            .filter(|c| c.unresolved() == Some(SdfUnresolvedCell::DepthLimit))
            .count(),
        2
    );
    let tie = program(SdfExpr::x().add_expr(SdfExpr::y()))
        .subdivide(
            [d(-1, 1, 0), d(-1, 1, 0), zero()],
            options(1, 3),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(tie.cells()[0].split(), Some((0, [1, 2])));
    verify_cover(&tie);
}
#[test]
fn empty_zero_and_unresolved_singletons_do_not_trigger_nonprogress() {
    let zero_field = program(SdfExpr::constant(Real::zero()));
    let c = zero_field
        .subdivide(
            [d(-1, 1, 0), d(-2, 2, 0), zero()],
            options(u32::MAX, 1),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(c.cells().len(), 1);
    assert_eq!(c.location(), Some(Location::Boundary));
    assert_eq!(c.cells()[0].unresolved(), None);
    let empty = zero_field
        .subdivide(
            [DyadicInterval::empty(), d(-2, 2, 0), zero()],
            options(u32::MAX, 1),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(empty.cells().len(), 1);
    assert_eq!(empty.location(), None);
    assert!(empty.cells()[0].enclosure().image().is_empty());
    verify_cover(&empty);
    let p = program(SdfExpr::constant((Real::one() / Real::from(3)).unwrap()));
    let c = p
        .subdivide(
            [zero(), zero(), zero()],
            SdfSubdivisionOptions {
                precision: 0,
                ..options(u32::MAX, 1)
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(c.cells().len(), 1);
    assert_eq!(c.location(), Some(Location::Unknown));
    assert_eq!(
        c.cells()[0].unresolved(),
        Some(SdfUnresolvedCell::Singleton)
    );
    let exact = p
        .subdivide(
            [zero(), zero(), zero()],
            options(u32::MAX, 1),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(exact.location(), Some(Location::ConservativeOutside));
}
#[test]
fn cell_capacity_graph_work_and_live_records_are_actual_limits() {
    let p = dependency_field();
    assert!(matches!(
        p.subdivide(
            dependency_cell(),
            options(3, 10),
            &mut graph(),
            &mut arithmetic()
        ),
        Err(SdfProgramError::SubdivisionCapacity {
            limit: 10,
            required: 11
        })
    ));
    assert!(matches!(
        p.subdivide(
            [DyadicInterval::empty(), zero(), zero()],
            options(3, 0),
            &mut graph(),
            &mut arithmetic()
        ),
        Err(SdfProgramError::SubdivisionCapacity {
            limit: 0,
            required: 1
        })
    ));
    let mut b = graph();
    let start = b.remaining_work();
    let cover = p
        .subdivide(dependency_cell(), options(3, 11), &mut b, &mut arithmetic())
        .unwrap();
    let used = start - b.remaining_work();
    // 3 bindings+8 nodes+1 output, two 8-node evaluator arrays, 11 cell
    // records, three pending indices and six fixed working records = 48.
    assert_eq!(p.program().definition().nodes.len(), 8);
    assert!(
        p.subdivide(
            dependency_cell(),
            options(3, 11),
            &mut ExpressionBudget::new(48, used),
            &mut arithmetic()
        )
        .is_ok()
    );
    assert!(matches!(
        p.subdivide(
            dependency_cell(),
            options(3, 11),
            &mut ExpressionBudget::new(47, used),
            &mut arithmetic()
        ),
        Err(SdfProgramError::Expression(ExpressionError::SizeExhausted))
    ));
    assert!(matches!(
        p.subdivide(
            dependency_cell(),
            options(3, 11),
            &mut ExpressionBudget::new(48, used - 1),
            &mut arithmetic()
        ),
        Err(SdfProgramError::Expression(ExpressionError::WorkExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut b = graph();
    b.cancellation = Some(token);
    assert!(matches!(
        p.subdivide(dependency_cell(), options(3, 11), &mut b, &mut arithmetic()),
        Err(SdfProgramError::Expression(ExpressionError::Cancelled))
    ));
    let retry = p
        .subdivide(
            dependency_cell(),
            options(3, 11),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(retry.location(), cover.location());
    assert_eq!(retry.cells().len(), cover.cells().len());
}
#[test]
fn arithmetic_exhaustion_domain_failure_and_unrepresentable_split_stay_typed() {
    let p = dependency_field();
    let mut a = arithmetic();
    let start = a.remaining_work();
    p.subdivide(dependency_cell(), options(3, 11), &mut graph(), &mut a)
        .unwrap();
    let used = start - a.remaining_work();
    assert!(
        p.subdivide(
            dependency_cell(),
            options(3, 11),
            &mut graph(),
            &mut DyadicBudget::new(16384, used)
        )
        .is_ok()
    );
    assert!(matches!(
        p.subdivide(
            dependency_cell(),
            options(3, 11),
            &mut graph(),
            &mut DyadicBudget::new(16384, used - 1)
        ),
        Err(SdfProgramError::Expression(ExpressionError::Enclosure {
            source: IntervalError::WorkExhausted,
            ..
        }))
    ));
    let root = program(SdfExpr::x().sqrt());
    assert!(matches!(
        root.subdivide(
            [d(-1, 1, 0), zero(), zero()],
            options(8, 1023),
            &mut graph(),
            &mut arithmetic()
        ),
        Err(SdfProgramError::Expression(
            ExpressionError::DomainNotCertified { .. }
        ))
    ));
    let p = program(SdfExpr::constant((Real::one() / Real::from(3)).unwrap()));
    let err = p
        .subdivide(
            [d(0, 1, i32::MIN), zero(), zero()],
            SdfSubdivisionOptions {
                precision: 0,
                ..options(1, 3)
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap_err();
    assert!(err.source().is_some());
    assert!(matches!(
        err,
        SdfProgramError::Lattice(IntervalLatticeError::Scalar(IntervalError::Overflow))
    ));
    assert!(
        p.subdivide(
            [d(0, 1, i32::MIN + 1), zero(), zero()],
            SdfSubdivisionOptions {
                precision: 0,
                ..options(1, 3)
            },
            &mut graph(),
            &mut arithmetic()
        )
        .is_ok()
    );
}
#[test]
fn generated_affine_sphere_cover_preserves_the_source_and_set() {
    let mut matrix = Matrix4::identity();
    matrix.0[0][0] = 2.into();
    matrix.0[0][3] = 1.into();
    let p = program(
        SdfExpr::sphere(point(0, 0, 0), 1.into())
            .affine_transform(matrix)
            .unwrap(),
    );
    let cover = p
        .subdivide(
            [d(-3, 5, 0), zero(), zero()],
            options(4, 31),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    verify_cover(&cover);
    assert_eq!(cover.location(), Some(Location::Unknown));
    assert!(
        cover
            .leaves()
            .any(|(_, c)| c.enclosure().location() == Some(Location::ConservativeInside))
    );
    assert!(
        cover
            .leaves()
            .any(|(_, c)| c.enclosure().location() == Some(Location::ConservativeOutside))
    );
    for (_, c) in cover.leaves() {
        let cell = BoxN::new(c.enclosure().cell().clone()).unwrap();
        assert!(
            cell.subset_of(
                &BoxN::new([d(-3, 5, 0), zero(), zero()]).unwrap(),
                &mut arithmetic()
            )
            .unwrap()
        );
        assert_eq!(c.enclosure().metric_status(), p.source().metric_status());
    }
}

#[test]
fn three_dimensional_cover_splits_every_axis_and_matches_exact_octants() {
    let p = program(SdfExpr::x().add_expr(SdfExpr::y()).add_expr(SdfExpr::z()));
    let cover = p
        .subdivide(
            core::array::from_fn(|_| d(-1, 1, 0)),
            options(3, 15),
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    verify_cover(&cover);
    assert_eq!(cover.cells().len(), 15);
    assert_eq!(cover.leaves().count(), 8);
    for cell in cover.cells() {
        if let Some((axis, _)) = cell.split() {
            assert_eq!(axis, cell.depth() as usize);
        }
    }
    for x in -1..=0 {
        for y in -1..=0 {
            for z in -1..=0 {
                let expected = [d(x, x + 1, 0), d(y, y + 1, 0), d(z, z + 1, 0)];
                let (_, leaf) = cover
                    .leaves()
                    .find(|(_, cell)| cell.enclosure().cell() == &expected)
                    .unwrap();
                assert_eq!(leaf.enclosure().image(), &d(x + y + z, x + y + z + 3, 0));
                assert_eq!(leaf.unresolved(), Some(SdfUnresolvedCell::DepthLimit));
            }
        }
    }
}
#[test]
fn fresh_process_reconstructs_source_and_complete_cover_evidence() {
    const KEY: &str = "HYPERSDF_SUBDIVISION_REPLAY";
    if std::env::var(KEY).is_ok() {
        complete_dependency_cover_has_independent_exact_leaves_and_reusable_evidence();
        return;
    }
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "subdivision_acceptance::fresh_process_reconstructs_source_and_complete_cover_evidence",
            "--nocapture",
        ])
        .env(KEY, "1")
        .status()
        .unwrap();
    assert!(status.success());
}
