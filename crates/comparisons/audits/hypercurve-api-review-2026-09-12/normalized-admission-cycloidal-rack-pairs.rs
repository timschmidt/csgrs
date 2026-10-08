use hypercurve::{Contour2, CurveContext, CurveRegion2, Real};
use std::cmp::Ordering;
fn exact_ratio(n:usize,d:usize)->Option<Real>{(Real::from(n as u64)/Real::from(d as u64)).ok()}
fn real_cmp(a:&Real,b:&Real)->Option<Ordering>{
let a=hypercurve::CurvePoint2::from(hypercurve::Point2::new(a.clone(),Real::zero()));
let b=hypercurve::CurvePoint2::from(hypercurve::Point2::new(b.clone(),Real::zero()));
match a.compare_coordinate(&b,hypercurve::Axis2::X,&CurveContext::STRICT).unwrap().value{
hypercurve::Classification::Decided(order)=>Some(order),_=>None}
}
fn rack_points(teeth:usize,segments_per_flank:usize)->Vec<[Real;2]>{
let module=Real::one();let clearance=Real::zero();
    let two = Real::from(2_u8);
    let three = Real::from(3_u8);
    let four = Real::from(4_u8);
    let pitch = Real::pi() * module.clone();
    let half_pitch = (pitch.clone() / two.clone()).expect("two is nonzero");
    let quarter_pitch = (pitch.clone() / four.clone()).expect("four is nonzero");
    let dedendum =
        module.clone() * (Real::from(5_u8) / four).expect("four is nonzero") + clearance;
    let root = -dedendum.clone();
    let flank_sweep = (Real::pi() / three).expect("three is nonzero");
    let run_factor = two.clone() * (flank_sweep.clone() - flank_sweep.clone().sin());
    let face_run = module.clone() * run_factor.clone();
    let root_run = dedendum.clone() * run_factor;
    if real_cmp(&face_run, &quarter_pitch) != Some(Ordering::Less)
        || real_cmp(&root_run, &quarter_pitch) != Some(Ordering::Less)
    {
        panic!("invalid fixed rack fixture");
    }

    let Some(sample_capacity) = segments_per_flank.checked_add(1) else {
        panic!("invalid fixed rack fixture");
    };
    let mut unit_flank = Vec::with_capacity(sample_capacity);
    for sample in 0..=segments_per_flank {
        let Some(fraction) = exact_ratio(sample, segments_per_flank) else {
            panic!("invalid fixed rack fixture");
        };
        let theta = flank_sweep.clone() * fraction;
        unit_flank.push([
            two.clone() * (theta.clone() - theta.clone().sin()),
            two.clone() * (Real::one() - theta.cos()),
        ]);
    }

    let Some(tooth_capacity) = segments_per_flank
        .checked_mul(4)
        .and_then(|count| count.checked_add(4))
    else {
        panic!("invalid fixed rack fixture");
    };
    let mut tooth_points = Vec::with_capacity(tooth_capacity);
    tooth_points.push([-half_pitch.clone(), root.clone()]);
    tooth_points.extend(unit_flank.iter().rev().map(|[run, rise]| {
        [
            -quarter_pitch.clone() - dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            -quarter_pitch.clone() + module.clone() * run.clone(),
            module.clone() * rise.clone(),
        ]
    }));
    tooth_points.push([quarter_pitch.clone() - face_run, module.clone()]);
    tooth_points.extend(
        unit_flank[..segments_per_flank]
            .iter()
            .rev()
            .map(|[run, rise]| {
                [
                    quarter_pitch.clone() - module.clone() * run.clone(),
                    module.clone() * rise.clone(),
                ]
            }),
    );
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            quarter_pitch.clone() + dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.push([half_pitch.clone(), root.clone()]);

    let Some(top_capacity) = teeth
        .checked_mul(tooth_points.len().saturating_sub(1))
        .and_then(|count| count.checked_add(1))
    else {
        panic!("invalid fixed rack fixture");
    };
    let mut top = Vec::with_capacity(top_capacity);
    for tooth in 0..teeth {
        let offset = Real::from(tooth as u64) * pitch.clone();
        for (point_index, point) in tooth_points.iter().enumerate() {
            if tooth > 0 && point_index == 0 {
                continue;
            }
            top.push([offset.clone() + point[0].clone(), point[1].clone()]);
        }
    }

    let Some(point_capacity) = top.len().checked_add(2) else {
        panic!("invalid fixed rack fixture");
    };
    let mut points = Vec::with_capacity(point_capacity);
    let backing = root.clone() - module;
    let left = top.first().expect("validated rack has a left root")[0].clone();
    let right = top.last().expect("validated rack has a right root")[0].clone();
    points.push([left, backing.clone()]);
    points.push([right, backing]);
    points.extend(top.into_iter().rev());
    points
}
fn main(){
let points=rack_points(1,8);
for quadratic in [false,true]{
let curves:Vec<hypercurve::Curve2>=(0..points.len()).map(|i|{
let a=&points[i];let b=&points[(i+1)%points.len()];
let l=hypercurve::LineSeg2::try_new(hypercurve::Point2::new(a[0].clone(),a[1].clone()),hypercurve::Point2::new(b[0].clone(),b[1].clone())).unwrap();
if quadratic {hypercurve::QuadraticBezier2::from_line_segment(l).into()}else{l.into()}
}).collect();
let mut failed=0;
for i in 0..curves.len(){for j in i+1..curves.len(){
match curves[i].intersect_curve(&curves[j],&CurveContext::STRICT){
Ok(r) if r.value.is_complete()=>{},
r=>{println!("pair {quadratic} {i} {j} {r:?}");failed+=1;}
}
}}
println!("failed {quadratic} {failed}");
}
}
