use hyperdrc::Scalar;
use hyperdrc::geometry::line_polygon;
use hyperdrc::kicad::load_kicad_pcb;
use serde_json::{Value, json};
use std::hint::black_box;
use std::time::Instant;

fn scalar(value: &str) -> Scalar { value.parse().unwrap() }
fn equal(a: &Scalar, b: &Scalar) -> bool {
    hyperlimit::compare_reals(a,b,hyperlimit::PredicatePolicy::STRICT).value() == Some(std::cmp::Ordering::Equal)
}
fn main() {
    let directory = std::path::PathBuf::from(std::env::args().nth(1).unwrap());
    std::fs::create_dir_all(&directory).unwrap();
    let min = f64::from_bits(1);
    let cases = [
        ("ordinary",[0.,0.],[2.,0.],0.5,Scalar::one()),
        ("large_center",[1e20,0.],[1e20,16384.],16384.,Scalar::from(268435456)),
        ("tiny_length",[0.,0.],[1e-200,0.],1e-200,Scalar::try_from(1e-200).unwrap().powi_i64(2).unwrap()),
        ("subnormal_width",[0.,0.],[2.,0.],min,Scalar::from(2)*Scalar::try_from(min).unwrap()),
        ("overflow_delta",[-f64::MAX,0.],[f64::MAX,0.],1.,Scalar::from(2)*Scalar::try_from(f64::MAX).unwrap()),
        ("overflow_norm",[0.,0.],[1e200,0.],2.,Scalar::from(2)*Scalar::try_from(1e200).unwrap()),
        ("rational_diagonal",[-1.,-1.],[2.,3.],0.25,scalar("1.25")),
        ("surd_diagonal",[0.,0.],[1.,1.],2.,Scalar::from(2)*Scalar::from(2).sqrt().unwrap()),
    ];
    let iterations=100;
    for (name,start,end,width,area) in cases {
        let now=Instant::now();
        for _ in 0..iterations { black_box(line_polygon(black_box(start),black_box(end),black_box(width))); }
        let elapsed=now.elapsed().as_nanos();
        let polygon=line_polygon(start,end,width);
        let witness=json!({"present":polygon.is_some(),"area_matches":polygon.as_ref().is_some_and(|p|p.try_area().is_ok_and(|a|equal(&a,&area))),"source_valid":polygon.as_ref().is_some_and(|p|p.try_exact_region().is_ok())});
        println!("{}",json!({"name":format!("butt_{name}"),"input_bits":start.into_iter().chain(end).chain([width]).map(f64::to_bits).collect::<Vec<_>>(),"iterations":iterations,"elapsed_ns":elapsed,"witness":witness}));
    }
    let cases = [
        ("round_caps","(start 0 0) (end 2 0) (width 2)",Some(Scalar::from(4)+Scalar::pi())),
        ("decimal","(start 9007199254740992.1 0) (end 9007199254740992.3 0) (width 0.2)",Some(scalar("0.04")+scalar("0.01")*Scalar::pi())),
        ("tiny","(start 0 0) (end 1e-400 0) (width 2e-400)",Some((Scalar::from(2)+Scalar::pi())*scalar("1e-800"))),
        ("huge","(start 1e400 0) (end 1e400 2) (width 2)",Some(Scalar::from(4)+Scalar::pi())),
        ("disk","(start 3 4) (end 3 4) (width 2)",Some(Scalar::pi())),
        ("missing_width","(start 0 0) (end 2 0)",None),
        ("invalid_width","(start 0 0) (end 2 0) (width bad)",None),
        ("negative_width","(start 0 0) (end 2 0) (width -1)",None),
        ("missing_endpoint","(start 0 0) (width 2)",None),
        ("extra_coordinate","(start 0 0 4) (end 2 0) (width 2)",None),
    ];
    for (name,fields,area) in cases {
        let input=format!("(kicad_pcb (segment (layer F.Cu) {fields}))");
        let path=directory.join(format!("{name}.kicad_pcb"));std::fs::write(&path,&input).unwrap();
        let now=Instant::now();
        for _ in 0..iterations { black_box(load_kicad_pcb(black_box(&path)).unwrap()); }
        let elapsed=now.elapsed().as_nanos();let board=load_kicad_pcb(&path).unwrap();
        let witness:Value=json!({"features":board.copper.len(),"matches_source":board.copper.first().is_some_and(|f|match &area {Some(area)=>f.region.try_area().is_ok_and(|a|equal(&a,area)),None=>f.region.try_area().is_err()})});
        println!("{}",json!({"name":format!("track_{name}"),"input":input,"iterations":iterations,"elapsed_ns":elapsed,"witness":witness}));
    }
}
