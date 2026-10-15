use csgrs::{
    GeometryContext, Real,
    solid::{self, SolidExt},
};

fn main() {
    let segments: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!([8, 16, 24, 32].contains(&segments));
    let body = solid::cuboid(12.into(), 12.into(), 4.into()).unwrap();
    let drill = solid::cylinder(2.into(), 6.into(), segments)
        .unwrap()
        .translated(6.into(), 6.into(), (-1).into())
        .unwrap();
    let cut = body
        .try_difference_with_context(&drill, &GeometryContext::STRICT)
        .unwrap();
    println!("segments={segments} initial={:?}", cut.certainty);
    match solid::empty().try_union_with_context(&cut.value, &GeometryContext::STRICT) {
        Ok(outcome) => println!(
            "union_empty={:?} positions={} triangles={}",
            outcome.certainty,
            outcome.value.positions.len(),
            outcome.value.triangles.len()
        ),
        Err(error) => println!("union_empty={error:?}"),
    }
    match cut
        .value
        .try_difference_with_context(&body, &GeometryContext::STRICT)
    {
        Ok(outcome) => println!(
            "difference_body={:?} positions={} triangles={}",
            outcome.certainty,
            outcome.value.positions.len(),
            outcome.value.triangles.len()
        ),
        Err(error) => println!("difference_body={error:?}"),
    }
    // Keep the original exact-coordinate type connected through this probe.
    assert_eq!(Real::from(12).exact_rational().unwrap().to_string(), "12");
}
