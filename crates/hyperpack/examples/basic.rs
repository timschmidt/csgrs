use hyperpack::{
    FeasibilityStatus, ItemId, Real, Rect2, SheetBin2, SheetItem2, maxrects_best_short_side_fit_2d,
};

fn main() -> hyperpack::PackResult<()> {
    let bin = SheetBin2::new(Rect2::new(Real::from(10), Real::from(10))?);
    let items = vec![
        SheetItem2::new(ItemId::new("a")?, Rect2::new(Real::from(4), Real::from(4))?),
        SheetItem2::new(ItemId::new("b")?, Rect2::new(Real::from(6), Real::from(3))?),
    ];

    let report = maxrects_best_short_side_fit_2d(&bin, &items)?;
    assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
    assert!(report.rejected.is_empty());
    Ok(())
}
