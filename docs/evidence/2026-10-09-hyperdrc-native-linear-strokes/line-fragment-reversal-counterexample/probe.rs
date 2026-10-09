use hypercurve::{BulgeVertex2, Contour2, ContourFragmentSet, ContourSplitMarkers, LineSeg2, LineSide, Point2, Real, Segment2, SegmentSplitMarker};
fn p(x: i32, y: i32) -> Point2 { Point2::new(Real::from(x), Real::from(y)) }
fn main() {
    let source = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0,0), Real::zero()),
        BulgeVertex2::new(p(6,0), Real::zero()),
        BulgeVertex2::new(p(6,6), Real::zero()),
        BulgeVertex2::new(p(0,6), Real::zero()),
    ]).unwrap();
    let mut rows = ContourSplitMarkers::with_contour_endpoints(&source).segments().to_vec();
    rows[0].insert(1, SegmentSplitMarker {segment_index:0, param: "0.5".parse().unwrap(), point: p(3,0)});
    let markers = ContourSplitMarkers::new(rows).unwrap();
    let fragments = ContourFragmentSet::from_split_markers(&source, &markers).unwrap();
    let Segment2::Line(first) = &fragments.fragments()[0].segment else { panic!("line") };
    let reversed = first.reversed();
    let rebuilt = LineSeg2::try_new(reversed.start().clone(), reversed.end().clone()).unwrap();
    let point = p(1,1);
    let forward = first.classify_point(&point).unwrap();
    let actual = reversed.classify_point(&point).unwrap();
    let rebuilt_side = rebuilt.classify_point(&point).unwrap();
    println!("forward={forward:?}, reversed={actual:?}, rebuilt={rebuilt_side:?}, independent_oracle=Right");
    assert_eq!(forward, LineSide::Left);
    assert_eq!(rebuilt_side, LineSide::Right);
    assert_eq!(actual, LineSide::Right);
}
