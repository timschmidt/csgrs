use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, AtomicUsize, Ordering::Relaxed};
use std::time::Instant;
struct Counting;
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicIsize = AtomicIsize::new(0);
static PEAK: AtomicIsize = AtomicIsize::new(0);
fn allocated(n: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(n, Relaxed);
    let live = LIVE.fetch_add(n as isize, Relaxed) + n as isize;
    PEAK.fetch_max(live, Relaxed);
}
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            allocated(layout.size());
        }
        p
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc_zeroed(layout) };
        if !p.is_null() {
            allocated(layout.size());
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        unsafe { System.dealloc(p, layout) };
        LIVE.fetch_sub(layout.size() as isize, Relaxed);
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, n: usize) -> *mut u8 {
        let q = unsafe { System.realloc(p, layout, n) };
        if !q.is_null() {
            LIVE.fetch_sub(layout.size() as isize, Relaxed);
            allocated(n);
        }
        q
    }
}
#[global_allocator]
static ALLOC: Counting = Counting;

use hypercurve::{CurvePath2, CurveRegion2, FillRule, LineSeg2, OffsetCap, OffsetCornerStyle2,
    Point2, PredicatePolicy, Real, RegionPointLocation, Segment2, evaluate_under};
use serde_json::{Value, json};

fn exact(value: &Real) -> Value {
    match serde_json::to_value(value) {
        Ok(value) => value,
        Err(error) => json!({"serialization_error":error.to_string()}),
    }
}
fn point(p: &Point2) -> Value {json!([exact(p.x()),exact(p.y())])}
fn native(region: &CurveRegion2, complete: &mut bool) -> Value {
    match region.native_contours_fast_path() {
        Ok(Some(view)) => { assert_eq!(view.material_contours().len(),1); assert!(view.hole_contours().is_empty()); json!({"material":view.material_contours().iter().map(|c| c.segments().iter().map(|s| match s {
            Segment2::Line(line) => json!({"line":[point(line.start()),point(line.end())]}),
            Segment2::Arc(arc) => json!({"arc":{"start":point(arc.start()),"end":point(arc.end()),"center":point(arc.center()),"radius_squared":exact(arc.radius_squared()),"clockwise":arc.is_clockwise()}}),
        }).collect::<Vec<_>>()).collect::<Vec<_>>(),"holes":view.hole_contours().len()}) },
        Ok(None) => {*complete=false;json!({"status":"unsupported_native_view"})},
        Err(error) => {*complete=false;json!({"error":error.to_string()})},
    }
}
fn equality(actual: &Real, expected: &Real, complete: &mut bool) -> Value {
    let comparison=hyperlimit::compare_reals(actual,expected,PredicatePolicy::STRICT).value();
    assert!(!matches!(comparison,Some(std::cmp::Ordering::Less|std::cmp::Ordering::Greater)),"contradictory exact measurement");
    *complete &= comparison==Some(std::cmp::Ordering::Equal);
    json!({"actual":exact(actual),"expected":exact(expected),"certified_equal":comparison.is_some()})
}
fn oracle(x:i64,y:i64,dx:i64,dy:i64,rnum:i64,rden:i64,round:bool) -> RegionPointLocation {
    // Coordinates x/2,y/2 and integer segment direction. All comparisons
    // below are exact integer arithmetic independent of the geometry kernel.
    let length2=dx*dx+dy*dy;let dot=x*dx+y*dy;let cross=x*dy-y*dx;
    let (left,right)=if round {
        if dot<=0 {((x*x+y*y)*rden,4*rnum)}
        else if dot>=2*length2 {(((x-2*dx).pow(2)+(y-2*dy).pow(2))*rden,4*rnum)}
        else {(cross*cross*rden,4*rnum*length2)}
    } else {
        if dot<0 || dot>2*length2 {return RegionPointLocation::Outside;}
        (cross*cross*rden,4*rnum*length2)
    };
    match left.cmp(&right) {
        std::cmp::Ordering::Greater=>RegionPointLocation::Outside,
        std::cmp::Ordering::Equal=>RegionPointLocation::Boundary,
        std::cmp::Ordering::Less if !round && (dot==0 || dot==2*length2)=>RegionPointLocation::Boundary,
        std::cmp::Ordering::Less=>RegionPointLocation::Inside,
    }
}
struct Observation { owners:Vec<CurveRegion2>, snapshot:Value, complete:bool }
fn perform(case:usize,round:bool) -> Observation {
    let (label,dx,dy,rnum,rden,translated)=match case {
        0=>("axis",2_i64,0_i64,1,1,false),
        1=>("rational_diagonal",3,4,1,1,false),
        2=>("surd_diagonal",1,1,1,1,false),
        3=>("symbolic_translation",1,2,1,1,true),
        4=>("symbolic_radius",1,1,3,1,true),
        5=>("small_rational_radius",-2,3,1,64,false),
        _=>panic!("unknown case"),
    };
    let center=if translated {Real::pi()} else {Real::zero()};
    let radius2=(Real::from(rnum)/Real::from(rden)).unwrap();let radius=radius2.clone().sqrt().unwrap();
    let length=Real::from(dx*dx+dy*dy).sqrt().unwrap();
    let start=Point2::new(center.clone(),Real::zero());
    let end=Point2::new(&center+Real::from(dx),Real::from(dy));
    let path=CurvePath2::try_new(vec![LineSeg2::try_new(start,end).unwrap().into()]).unwrap();
    let cap=if round {OffsetCap::Round} else {OffsetCap::Butt};
    let provisional=evaluate_under(PredicatePolicy::STRICT,||CurveRegion2::stroke_path(&path,radius.clone(),&OffsetCornerStyle2::Round,cap));
    let region=match provisional.certified().expect("STRICT receipt") {
        Ok(region)=>region,
        Err(error)=>return Observation{owners:vec![],snapshot:json!({"case":label,"cap":format!("{cap:?}"),"construction_error":error.to_string()}),complete:false},
    };
    let mut complete=true;let mut owners=vec![region];
    let rebuild=owners[0].boundary_paths().and_then(|p|CurveRegion2::try_from_boundary_paths(&p,FillRule::NonZero));
    let rebuild_error=match rebuild {Ok(r)=>{owners.push(r);None},Err(error)=>{complete=false;Some(error.to_string())}};
    let mut expected_area=Real::from(2)*&radius*&length;
    if round {expected_area+=Real::pi()*radius2;}
    let extent_x=if round {radius.clone()} else {(Real::from(dy.abs())*&radius/&length).unwrap()};
    let extent_y=if round {radius.clone()} else {(Real::from(dx.abs())*&radius/&length).unwrap()};
    let expected_bounds=[&center+Real::from(dx.min(0))-&extent_x,Real::from(dy.min(0))-&extent_y,
        &center+Real::from(dx.max(0))+&extent_x,Real::from(dy.max(0))+&extent_y];
    let snapshots=owners.iter().map(|r| {
        let area=match r.filled_area() {
            Ok(Some(area))=>equality(&area,&expected_area,&mut complete),
            Ok(None)=>{complete=false;json!({"area":"unavailable"})},
            Err(e)=>{complete=false;json!({"error":e.to_string()})},
        };
        let bounds=match r.bounds() {
            Ok(Some(b))=>json!([b.min_x(),b.min_y(),b.max_x(),b.max_y()].into_iter().zip(&expected_bounds).map(|(a,e)|equality(a,e,&mut complete)).collect::<Vec<_>>()),
            Ok(None)=>{complete=false;json!({"bounds":"unavailable"})},
            Err(e)=>{complete=false;json!({"error":e.to_string()})},
        };
        let mut queries=Vec::new();
        for x in (dx.min(0)-3)*2..=(dx.max(0)+3)*2 {
            for y in (dy.min(0)-3)*2..=(dy.max(0)+3)*2 {
                let expected=oracle(x,y,dx,dy,rnum,rden,round);
                let p=Point2::new(&center+(Real::from(x)/Real::from(2)).unwrap(),(Real::from(y)/Real::from(2)).unwrap());
                let actual=match r.classify_point(&p.into()) {
                    Ok(actual)=>{assert_eq!(actual,expected,"integer oracle disagrees at {x}/2,{y}/2");format!("{actual:?}")},
                    Err(error)=>{complete=false;format!("error: {error}")},
                };
                queries.push(json!([x,y,format!("{expected:?}"),actual]));
            }
        }
        json!({"area":area,"bounds":bounds,"native":native(r,&mut complete),"queries":queries})
    }).collect::<Vec<_>>();
    Observation{owners,snapshot:json!({"case":label,"cap":format!("{cap:?}"),"regions":snapshots,"rebuild_error":rebuild_error}),complete}
}
fn main() {
    let case:usize=std::env::args().nth(1).unwrap().parse().unwrap();
    let round=match std::env::args().nth(2).unwrap().as_str(){"round"=>true,"butt"=>false,_=>panic!("unknown cap")};
    let output=std::env::args().nth(3).unwrap();let mut rows=Vec::new();let mut snapshots=Vec::new();
    for phase in ["fresh_process","repeat_after_model_drop"] {
        let before_live=LIVE.load(Relaxed);PEAK.store(before_live,Relaxed);
        let before_calls=CALLS.load(Relaxed);let before_bytes=BYTES.load(Relaxed);let start=Instant::now();
        let observation=perform(case,round);
        let elapsed=start.elapsed().as_nanos();let calls=CALLS.load(Relaxed)-before_calls;let bytes=BYTES.load(Relaxed)-before_bytes;
        let retained=LIVE.load(Relaxed)-before_live;let peak=PEAK.load(Relaxed)-before_live;
        rows.push(json!({"phase":phase,"elapsed_ns":elapsed,"allocation_calls":calls,"allocated_bytes":bytes,
            "retained_requested_bytes_delta":retained,"peak_extra_requested_bytes":peak,"certified_completion":observation.complete,
            "retained_regions":observation.owners.len()}));
        snapshots.push(observation.snapshot);drop(observation.owners);
    }
    std::fs::write(output,serde_json::to_vec(&snapshots).unwrap()).unwrap();
    println!("{}",serde_json::to_string_pretty(&rows).unwrap());
}
