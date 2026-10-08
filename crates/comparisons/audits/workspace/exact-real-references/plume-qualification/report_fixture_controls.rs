use hyperreal::{Computable, Rational, Real};
use rug::{Integer, Rational as Q};
use std::fs;

fn q(n: i32, d: i32) -> Q { Q::from((n, d)) }
fn c(n: i32, d: i32) -> Computable { Computable::rational(q(n, d).to_string().parse().unwrap()) }
fn r(n: i32, d: i32) -> Real { Real::new(q(n, d).to_string().parse::<Rational>().unwrap()) }

// Independent GMP oracle and two Hyper layers evaluate the same source fixture.
// These are public constructors; exact folding is allowed and no raw-node claim
// is made. Split coefficients reproduce the worksheet's rational-control form.
fn polynomial<T: Clone>(x: T, y: T, split: bool, constant: fn(i32, i32) -> T,
    add: fn(T,T)->T, sub: fn(T,T)->T, mul: fn(T,T)->T, div: fn(T,T)->T) -> T {
    let x2 = mul(x.clone(), x.clone());
    let y2 = mul(y.clone(), y.clone());
    let y4 = mul(y2.clone(), y2.clone());
    let y6 = mul(y4.clone(), y2.clone());
    let y8 = mul(y4.clone(), y4.clone());
    let first = if split { add(constant(333,1), constant(3,4)) } else { constant(1335,4) };
    let second = if split { div(constant(11,1),constant(2,1)) } else { constant(11,2) };
    let inner = sub(sub(sub(mul(constant(11,1),mul(x2.clone(),y2)),y6.clone()),
        mul(constant(121,1),y4)),constant(2,1));
    add(add(add(mul(first,y6),mul(x2,inner)),mul(second,y8)),div(x,mul(constant(2,1),y)))
}

fn decimal(s: &str) -> Q {
    let (whole, fraction) = s.split_once('.').unwrap_or((s,""));
    let digits = format!("{whole}{fraction}");
    let mut denominator = Integer::from(1);
    for _ in fraction.bytes() { denominator *= 10; }
    Q::from((Integer::from_str_radix(&digits,10).unwrap(),denominator))
}

fn main() {
    let mut real_checks = 0;
    let mut approximation_checks = 0;
    for dx in -2..=2 { for dy in -2..=2 { for sx in [-1,1] { for sy in [-1,1] {
        let x = sx*(77617+dx); let y = sy*(33096+dy);
        for split in [false,true] {
            let expected = polynomial(q(x,1),q(y,1),split,q,|a,b|a+b,|a,b|a-b,|a,b|a*b,|a,b|a/b);
            let real = polynomial(r(x,1),r(y,1),split,r,|a,b|a+b,|a,b|a-b,|a,b|a*b,|a,b|(a/b).unwrap());
            assert_eq!(real, Real::new(expected.to_string().parse::<Rational>().unwrap()));
            real_checks += 1;
            let computable = polynomial(c(x,1),c(y,1),split,c,|a,b|a.add(b),
                |a,b|a.add(b.negate()),|a,b|a.multiply(b),|a,b|a.multiply(b.inverse()));
            for bits in [0,8,32,128,512,8] {
                let got = Q::from(Integer::from_str_radix(&computable.approx(-bits).to_string(),10).unwrap());
                let truth = Q::from(&expected*(Integer::from(1)<<bits));
                assert!((got-truth).abs()<=1, "x={x} y={y} split={split} bits={bits}");
                approximation_checks += 1;
            }
        }
    }}}}
    let expected = polynomial(q(77617,1),q(33096,1),false,q,|a,b|a+b,|a,b|a-b,|a,b|a*b,|a,b|a/b);
    assert_eq!(expected,q(-54767,66192));
    let donor = fs::read_to_string("exact-real-references/plume-qualification/solaris-cancellation-calc.log").unwrap();
    assert!(donor.contains("Exit status: 0"));
    let output = donor.lines().find_map(|line|line.strip_prefix("= ")).unwrap();
    let mut scale = Integer::from(1); for _ in 0..30 { scale*=10; }
    assert!((decimal(output)-&expected).abs()<=Q::from((1,scale)));
    println!("PASS cancellation: Real exact={real_checks}, Computable precision/history={approximation_checks}, donor 30-place fixture=1; exact={expected}");

    // The worksheet's archived function-maximum prefix has midpoint 1857/128
    // and radius 1/256. Exact inequalities qualify it without trusting Maple
    // plot data or floating point. max(30-1/x-60*x)=30-2*sqrt(60) at x=1/sqrt(60).
    let lo=q(129099,1_000_000); let hi=q(1291,10_000);
    assert!(Q::from(&lo*&lo)*60<1 && Q::from(&hi*&hi)*60>1);
    let center=q(1857,128); let radius=q(1,256);
    let lower_sqrt=(q(30,1)-Q::from(&center+&radius))/2;
    let upper_sqrt=(q(30,1)-Q::from(&center-&radius))/2;
    assert!(lower_sqrt>0 && Q::from(&lower_sqrt*&lower_sqrt)<=60);
    assert!(Q::from(&upper_sqrt*&upper_sqrt)>=60);
    println!("PASS archived maximum prefix: critical point inside positive interval; exact squared-endpoint enclosure");

    let mut identities=0;
    let mut printed_dyadic_failures=0;
    for a in -4..=4 { for b in -4..=4 { for x in -4..=4 { for y in -4..=4 {
        let (a,b,x,y)=(q(a,4),q(b,4),q(x,4),q(y,4));
        let want=Q::from(&a+&x)*Q::from(&b+&y)/4;
        let p: Q=(Q::from(&a*&b)+Q::from(&x*&y))/2;
        let correct_cross=(Q::from(&b*&x)+Q::from(&a*&y))/2;
        assert_eq!((p.clone()+correct_cross)/2,want); identities+=1;
        // Printed page45 repeats b*x in both cross terms. This is a rejected
        // report transcription, not a replay of the correct donor source.
        if (p+Q::from(&b*&x))/2!=want { printed_dyadic_failures+=1; }
    }}}}
    assert!(printed_dyadic_failures>0);
    // Page47 moves -4e to the wrong side. The corrected inequality and carry
    // invariant hold on every legal signed-digit tuple below.
    let mut carry=0; let mut printed_inequality_failures=0;
    for a in -1_i32..=1 { for b in -1_i32..=1 { for c in -1_i32..=1 {
        if (a+b+2*c).abs()>2 {continue;}
        for aa in -1..=1 {for bb in -1..=1 {
            let d=2*(a+b+2*c)+aa+bb;
            let e=if d>2 {1} else if d< -2 {-1} else {0};
            let cc=a+b+2*c-2*e;
            assert!((-2..=2).contains(&(aa+bb+2*cc)));
            assert!((-2+4*e..=2+4*e).contains(&d)); carry+=1;
            if !(-2-4*e..=2-4*e).contains(&d) {printed_inequality_failures+=1;}
        }}
    }}}
    assert!(printed_inequality_failures>0);
    println!("PASS exact report identity controls={identities}, carry controls={carry}; rejected printed dyadic cross-term failures={printed_dyadic_failures}, inequality failures={printed_inequality_failures}");
}
