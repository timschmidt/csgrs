use hypercurve::{CircularArc2, Contour2, CurvePath2, CurveRegion2, LineSeg2, OffsetCap,
    OffsetCornerStyle2, Point2, PredicatePolicy, Real, Segment2, evaluate_under};
use std::time::Instant;

fn direct(line: &LineSeg2, cap: OffsetCap) -> Result<CurveRegion2, String> {
    let left = line.offset_left(Real::one()).map_err(|e| e.to_string())?;
    let right = line.offset_left(-Real::one()).map_err(|e| e.to_string())?;
    let edge = |start: Point2, end: Point2, center: Point2| -> Result<Segment2,String> {
        if cap == OffsetCap::Butt {
            LineSeg2::try_new(start,end).map(Segment2::Line).map_err(|e|e.to_string())
        } else {
            CircularArc2::try_from_center(start,end,center,false).map(Segment2::Arc).map_err(|e|e.to_string())
        }
    };
    let end = edge(right.end().clone(),left.end().clone(),line.end().clone())?;
    let start = edge(left.start().clone(),right.start().clone(),line.start().clone())?;
    let contour = Contour2::try_new(vec![right.into(),end,left.reversed().into(),start]).map_err(|e|e.to_string())?;
    CurveRegion2::try_from_native_material_contours(vec![contour]).map_err(|e|e.to_string())
}

fn main() {
    for [dx,dy] in [[2,0],[0,2],[3,4],[1,1],[1,2],[-1,1]] {
        for cap in [OffsetCap::Butt,OffsetCap::Round] {
            for method in ["stroke_path","direct_public_contour"] {
                let clock=Instant::now();
                let result=evaluate_under(PredicatePolicy::STRICT,|| -> Result<(),String> {
                    let line=LineSeg2::try_new(Point2::from_values(0,0),Point2::from_values(dx,dy)).unwrap();
                    let r=if method=="stroke_path" {
                        CurveRegion2::stroke_path(&CurvePath2::try_new(vec![line.into()]).unwrap(),Real::one(),&OffsetCornerStyle2::Round,cap).map_err(|e|e.to_string())?
                    } else {direct(&line,cap)?};
                    let area=r.filled_area().map_err(|e|e.to_string())?.ok_or("area unknown")?;
                    let mut expected=Real::from(2)*Real::from(dx*dx+dy*dy).sqrt().unwrap();
                    if cap==OffsetCap::Round {expected+=Real::pi();}
                    if hyperlimit::compare_reals(&area,&expected,PredicatePolicy::STRICT).value()!=Some(std::cmp::Ordering::Equal) {
                        return Err("analytic area not certified equal".into());
                    }
                    let _bounds=r.bounds().map_err(|e|e.to_string())?.ok_or("bounds unknown")?;
                    let rebuilt=CurveRegion2::try_from_boundary_paths(&r.boundary_paths().map_err(|e|e.to_string())?,hypercurve::FillRule::NonZero).map_err(|e|format!("rebuild: {e}"))?;
                    let area=rebuilt.filled_area().map_err(|e|format!("rebuilt area: {e}"))?.ok_or("rebuilt area unknown")?;
                    if hyperlimit::compare_reals(&area,&expected,PredicatePolicy::STRICT).value()!=Some(std::cmp::Ordering::Equal) {
                        return Err("rebuilt analytic area not certified equal".into());
                    }
                    Ok(())
                });
                println!("{dx},{dy} {cap:?} {method} {:?} {:?} elapsed_ns={}",result.certainty(),result.into_unverified(),clock.elapsed().as_nanos());
            }
        }
    }
}
