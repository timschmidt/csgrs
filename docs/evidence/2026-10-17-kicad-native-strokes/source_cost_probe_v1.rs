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

use hypercurve::{CurveRegion2, Point2, RegionPointLocation, Segment2};
use hyperdrc::kicad::{BoardModel, TrackLoadError, load_kicad_pcb};
use hyperdrc::{PcbRegion, PcbRegionExt, Scalar};
use hyperlimit::PredicatePolicy;
use serde_json::{Value, json};
use std::path::Path;

fn exact(x: &Scalar) -> Value { serde_json::to_value(x).unwrap() }
fn point(p: &Point2) -> Value { json!([exact(p.x()), exact(p.y())]) }
struct Case { label: &'static str, dx: i64, dy: i64, x: Scalar, scale: Scalar, empty: bool, two_disks: bool }
fn case(n: usize) -> Case {
    let (label, dx, dy, x, scale, empty, two_disks) = match n {
        0 => ("empty", 2, 0, "0", "1", true, false),
        1 => ("coincident_disk", 0, 0, "0", "1", false, false),
        2 => ("axis", 2, 0, "0", "1", false, false),
        3 => ("rational_diagonal", 3, 4, "0", "1", false, false),
        4 => ("surd_diagonal", 1, 1, "0", "1", false, false),
        5 => ("decimal_translation", 2, 0, "9007199254740992.1", "1", false, false),
        6 => ("subrange", 2, 0, "0", "1e-400", false, false),
        7 => ("superrange", 0, 2, "1e400", "1", false, false),
        8 => ("disjoint_disks", 0, 0, "0", "1", false, true),
        _ => panic!("unknown case"),
    };
    Case { label, dx, dy, x: x.parse().unwrap(), scale: scale.parse().unwrap(), empty, two_disks }
}
fn oracle(x: i64, y: i64, c: &Case) -> RegionPointLocation {
    use RegionPointLocation::{Boundary, Inside, Outside};
    if c.empty { return Outside; }
    if c.two_disks {
        let distance = (x*x+y*y).min((x-8)*(x-8)+y*y);
        return match distance.cmp(&4) { std::cmp::Ordering::Less => Inside,
            std::cmp::Ordering::Equal => Boundary, std::cmp::Ordering::Greater => Outside };
    }
    let length2 = c.dx*c.dx+c.dy*c.dy;
    let dot = x*c.dx+y*c.dy;
    let (left, right) = if length2 == 0 || dot <= 0 { (x*x+y*y, 4) }
        else if dot >= 2*length2 { ((x-2*c.dx).pow(2)+(y-2*c.dy).pow(2),4) }
        else { ((x*c.dy-y*c.dx).pow(2),4*length2) };
    match left.cmp(&right) { std::cmp::Ordering::Less => Inside,
        std::cmp::Ordering::Equal => Boundary, std::cmp::Ordering::Greater => Outside }
}
#[derive(Default)]
struct Facts { incorrect: usize, undecided: usize, errors: usize }
impl Facts { fn complete(&self) -> bool { self.incorrect+self.undecided+self.errors==0 } }
fn equality(actual: &Scalar, expected: &Scalar, facts: &mut Facts) -> Value {
    let result=hyperlimit::compare_reals(actual,expected,PredicatePolicy::STRICT).value();
    match result { Some(std::cmp::Ordering::Equal) => {}, Some(_) => facts.incorrect+=1, None => facts.undecided+=1 }
    json!({"actual":exact(actual),"expected":exact(expected),"strict_comparison":format!("{result:?}")})
}
fn native(region: &CurveRegion2, facts: &mut Facts) -> Value {
    match region.native_contours_fast_path() {
        Ok(Some(view)) => json!({"material":view.material_contours().iter().map(|c|
            c.segments().iter().map(|s|match s {
                Segment2::Line(l)=>json!({"line":[point(l.start()),point(l.end())]}),
                Segment2::Arc(a)=>json!({"arc":{"start":point(a.start()),"end":point(a.end()),
                    "center":point(a.center()),"radius_squared":exact(&a.radius_squared()),"clockwise":a.is_clockwise()}})
            }).collect::<Vec<_>>()).collect::<Vec<_>>(),"holes":view.hole_contours().len()}),
        Ok(None)=>{facts.errors+=1;json!({"unsupported_native_view":true})},
        Err(e)=>{facts.errors+=1;json!({"error":e.to_string()})},
    }
}
fn snapshot(region: &PcbRegion, c: &Case, facts: &mut Facts) -> Value {
    let factor=if c.two_disks {Scalar::from(2)}else{Scalar::one()};
    let expected_area=if c.empty {Scalar::zero()}else {
        (Scalar::from(2)*Scalar::from(c.dx*c.dx+c.dy*c.dy).sqrt().unwrap()+factor*Scalar::pi())*&c.scale*&c.scale
    };
    let area=match region.try_area() {Ok(a)=>equality(&a,&expected_area,facts),
        Err(e)=>{facts.errors+=1;json!({"error":e.to_string()})}};
    let native_region=match region.clone().try_into_native() {Ok(r)=>r,
        Err(e)=>{facts.errors+=1;return json!({"area":area,"handoff_error":e.to_string()});}};
    let expected_bounds=[&c.x-&c.scale,-c.scale.clone(),
        &c.x+Scalar::from(if c.two_disks{5}else{c.dx+1})*&c.scale,Scalar::from(c.dy+1)*&c.scale];
    let bounds=match native_region.bounds() {
        Ok(None) if c.empty=>Value::Null,
        Ok(Some(_)) if c.empty=>{facts.incorrect+=1;json!({"unexpected_nonempty_bounds":true})},
        Ok(Some(b))=>json!([b.min_x(),b.min_y(),b.max_x(),b.max_y()].into_iter().zip(&expected_bounds)
            .map(|(a,e)|equality(a,e,facts)).collect::<Vec<_>>()),
        Ok(None)=>{facts.incorrect+=1;json!({"missing_bounds":true})},
        Err(e)=>{facts.errors+=1;json!({"error":e.to_string()})},
    };
    let mut queries=Vec::new();
    for x in -4..=2*(if c.two_disks{4}else{c.dx})+4 {
        for y in -4..=2*c.dy+4 {
            let expected=oracle(x,y,c);
            let actual=region.classify_xy(&c.x+(Scalar::from(x)/Scalar::from(2)).unwrap()*&c.scale,
                (Scalar::from(y)/Scalar::from(2)).unwrap()*&c.scale);
            match &actual {Ok(a) if *a==expected=>{},Ok(_)=>facts.incorrect+=1,Err(_)=>facts.errors+=1}
            queries.push(json!([x,y,format!("{expected:?}"),match actual{Ok(a)=>format!("{a:?}"),Err(e)=>format!("error: {e}")} ]));
        }
    }
    let report=match region.try_to_multipolygon() {
        Ok(view)=>json!({"status":"finite_report","polygons":view.0.iter().map(|p|json!({
            "exterior":p.exterior().0.iter().map(|v|[v.x,v.y]).collect::<Vec<_>>(),
            "holes":p.interiors().iter().map(|r|r.0.iter().map(|v|[v.x,v.y]).collect::<Vec<_>>()).collect::<Vec<_>>()
        })).collect::<Vec<_>>()}),
        Err(e)=>json!({"status":"report_unavailable","operation":e.operation,"source":e.source,"detail":e.detail}),
    };
    json!({"area":area,"bounds":bounds,"native":native(&native_region,facts),"queries":queries,"report":report})
}
struct Observation { board: Option<BoardModel>, profiles: Vec<PcbRegion>, snapshot: Value, complete: bool, status: &'static str }
fn perform(path: &Path, n: usize) -> Observation {
    let c=case(n);
    let board=match load_kicad_pcb(path) {
        Ok(b)=>b,
        Err(e)=>return Observation{board:None,profiles:vec![],complete:false,status:"source_load_error",
            snapshot:json!({"case":c.label,"load_error":format!("{e:#}"),
                "typed_track_failure":e.downcast_ref::<TrackLoadError>().map(|t|format!("{:?}",t.failure))})},
    };
    let mut profiles=Vec::new(); let mut errors=Vec::new();
    match board.all_copper(){Ok(p)=>profiles.push(p),Err(e)=>errors.push(e.to_string())}
    match board.copper_layers(&[]){Ok(layers)=>profiles.extend(layers.into_iter().map(|(_,r)|r)),Err(e)=>errors.push(e.to_string())}
    let mut facts=Facts{errors:errors.len(),..Facts::default()};
    if board.copper.len()!=if c.empty{0}else if c.two_disks{2}else{1}{facts.incorrect+=1;}
    let snapshots=profiles.iter().map(|r|snapshot(r,&c,&mut facts)).collect::<Vec<_>>();
    let status=if facts.incorrect>0{"incorrect_source_geometry"}else if facts.errors>0{"geometry_error"}
        else if facts.undecided>0{"undecided"}else{"certified_completion"};
    let state=json!({"case":c.label,"source_feature_count":board.copper.len(),"profiles":snapshots,
        "aggregation_errors":errors,"incorrect_facts":facts.incorrect,"undecided_facts":facts.undecided,"geometry_errors":facts.errors});
    Observation{board:Some(board),profiles,snapshot:state,complete:facts.complete(),status}
}
fn main() {
    let n=std::env::args().nth(1).unwrap().parse().unwrap();
    let source=std::env::args().nth(2).unwrap();let output=std::env::args().nth(3).unwrap();
    let mut rows=Vec::new();let mut snapshots=Vec::new();
    for phase in ["fresh_process","repeat_after_owner_drop"] {
        let live=LIVE.load(Relaxed);PEAK.store(live,Relaxed);
        let calls=CALLS.load(Relaxed);let bytes=BYTES.load(Relaxed);let start=Instant::now();
        let observation=hypercurve::evaluate_under(PredicatePolicy::STRICT,||perform(Path::new(&source),n)).certified().unwrap();
        let elapsed=start.elapsed().as_nanos();let allocated_calls=CALLS.load(Relaxed)-calls;let allocated_bytes=BYTES.load(Relaxed)-bytes;
        let retained=LIVE.load(Relaxed)-live;let peak=PEAK.load(Relaxed)-live;
        rows.push(json!({"phase":phase,"elapsed_ns":elapsed,"allocation_calls":allocated_calls,"allocated_bytes":allocated_bytes,
            "retained_requested_bytes_delta":retained,"peak_extra_requested_bytes":peak,"certified_completion":observation.complete,
            "status":observation.status,"local_operation_receipt":"Certified","retained_profiles":observation.profiles.len()}));
        snapshots.push(observation.snapshot);drop(observation.profiles);drop(observation.board);
    }
    std::fs::write(output,serde_json::to_vec(&snapshots).unwrap()).unwrap();
    println!("{}",serde_json::to_string_pretty(&rows).unwrap());
}
