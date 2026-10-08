use hypercurve::{CurveContext, Point2, RationalBezier2, Real};

fn main() {
    let curve = RationalBezier2::try_new(
        vec![Point2::new(Real::zero(), Real::zero()), Point2::new(Real::one(), Real::from(2))],
        vec![Real::from(2), Real::from(3)],
    ).unwrap();
    let mut pass = 0;
    let mut incomplete = 0;
    for t in [Real::zero(), (Real::one()/Real::from(2)).unwrap(), Real::one()] {
        for order in [0,1,2,3,8,20,21,32,60,61,62,63,64,68,80] {
            match curve.derivatives_at(&t,order,&CurveContext::STRICT) {
                Err(e) => { println!("INCOMPLETE t={t} order={order}: {e:?}"); incomplete+=1; }
                Ok(ds) => {
                    assert_eq!(ds.len(),order);
                    // x(t)=3t/(2+t), y(t)=2*x(t). For k>=1,
                    // x^(k)(t)=6*(-1)^(k-1)*k!/(2+t)^(k+1).
                    let divisor=Real::from(2)+&t;
                    let mut expected=(Real::from(6)/(&divisor*&divisor)).unwrap();
                    for (i,d) in ds.iter().enumerate() {
                        if i>0 { expected=(-expected*Real::from((i+1) as u64)/&divisor).unwrap(); }
                        assert_eq!(d.dx(),&expected,"t={t} derivative={}",i+1);
                        assert_eq!(d.dy(),&(Real::from(2)*&expected),"t={t} derivative={}",i+1);
                    }
                    pass+=1;
                }
            }
        }
    }
    println!("TOTAL PASS {pass} INCOMPLETE {incomplete}");
}
