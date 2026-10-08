use std::{hint::black_box, time::Instant};
use hypercurve::{Classification, CurveContext, Point2, RationalQuadraticBezier2, Real};
fn r(v:i32)->Real { Real::from(v) }
fn q(a:i32,b:i32)->Real { (r(a)/r(b)).unwrap() }
fn main() {
  let iterations=1000u32;
  for (weights_label,weights) in [("unequal",[r(1),r(2),r(3)]),("unit_radical",[r(1),q(1,2).sqrt().unwrap(),r(1)]),("unit_pi",[r(1),Real::pi(),r(1)])] {
    let conic=RationalQuadraticBezier2::try_new(Point2::from_values(0,0),Point2::from_values(2,4),Point2::from_values(6,0),weights[0].clone(),weights[1].clone(),weights[2].clone()).unwrap();
    for (parameter_label,t) in [("rational",q(1,2)),("radical",q(1,2).sqrt().unwrap()),("pi_inverse",(r(1)/Real::pi()).unwrap())] {
      for refine in [false,true] {
        let started=Instant::now();
        for _ in 0..iterations {
          let Classification::Decided(point)=black_box(conic.point_at(black_box(t.clone()),&CurveContext::STRICT)) else { panic!("finite conic evaluation did not certify"); };
          if refine {
            black_box(point.x().certified_dyadic_interval(-128).expect("finite x interval"));
            black_box(point.y().certified_dyadic_interval(-128).expect("finite y interval"));
          }
        }
        println!("{weights_label},{parameter_label},{refine},{iterations},{}",started.elapsed().as_nanos());
      }
    }
  }
}
