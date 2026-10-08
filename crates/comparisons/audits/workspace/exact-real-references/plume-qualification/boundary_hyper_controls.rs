use hyperreal::{Computable, Rational};
use rug::{Integer, Rational as Q};

fn c(q: &Q) -> Computable { Computable::rational(q.to_string().parse::<Rational>().unwrap()) }
fn power2(e: i32) -> Q {
    if e >= 0 { Q::from(Integer::from(1) << e) }
    else { Q::from((1, Integer::from(1) << -e)) }
}
fn decimal(text: &str) -> Q {
    let (neg,body)=text.strip_prefix('-').map_or((false,text),|s|(true,s));
    let (a,b)=body.split_once('.').unwrap_or((body,""));
    assert!(!a.is_empty() && a.bytes().chain(b.bytes()).all(|x|x.is_ascii_digit()));
    let mut n=Integer::from_str_radix(&format!("{a}{b}"),10).unwrap();
    if neg {n = -n;}
    let mut d=Integer::from(1); for _ in b.bytes() {d*=10;}
    Q::from((n,d))
}
fn check_approx(x:&Computable,q:&Q,p:i32) {
    let actual=Q::from(Integer::from_str_radix(&x.approx(p).to_string(),10).unwrap());
    let want=Q::from(q / power2(p));
    assert!((actual-want).abs()<=1,"precision {p}, value {q}");
}
fn main() {
    let trace=std::env::var_os("PLUME_BOUNDARY_TRACE").is_some();
    let mut formats=0;
    let mut approximations=0;
    for a in -32..=32 {if trace {eprintln!("format a={a}");} for e in [-4,0,1,4] {for places in [0_usize,1,3,6] {
        let expected=Q::from((a,32))*power2(e);
        for history in [false,true] {
            let x=c(&expected);
            if history {let _=x.approx(-512); let _=x.approx(-2);}
            let output=format!("{x:.places$}");
            let bound=Q::from((1,10_u64.pow(places as u32)));
            assert!((decimal(&output)-&expected).abs()<=bound,"a={a} e={e} places={places} output={output}");
            formats+=1;
        }
    }}}
    if trace {eprintln!("format complete {formats}; constructing public zeros");}
    // Public constructors allow all normal simplifications. No opaque/raw-node
    // or proof of decidable equality for arbitrary computable reals is claimed.
    let third=c(&Q::from((1,3)));
    let radical=c(&Q::from(2)).sqrt();
    let zeros=[c(&Q::from(0)),third.clone().add(third.negate()),
        radical.clone().multiply(radical).add(c(&Q::from(-2)))];
    for shift in [-600,-501,-500,-499,-1,0,1,498,499,500,501,600] {
        if trace {eprintln!("shift={shift}");}
        let scale=power2(shift);
        for (index,z) in zeros.iter().enumerate() {
            if trace {eprintln!("zero index={index}");}
            let x=z.clone().multiply(c(&scale)).multiply(c(&Q::from((1,2))));
            for p in [0,-8,-64,-640,-2] {if trace {eprintln!("zero approx {p}");} check_approx(&x,&Q::from(0),p);approximations+=1;}
            for places in [0_usize,1,8] {if trace {eprintln!("zero format {places}");} assert_eq!(decimal(&format!("{x:.places$}")),0);formats+=1;}
        }
        for a in [-1,1] {
            if trace {eprintln!("nonzero a={a}");}
            let expected=scale.clone()*Q::from((a,4));
            let x=c(&Q::from((a,2))).multiply(c(&scale)).multiply(c(&Q::from((1,2))));
            for p in [0,-8,-64,-640,-2] {check_approx(&x,&expected,p);approximations+=1;}
        }
    }
    println!("PASS boundary Hyper controls: decimal/history={formats}, shifted-zero/nonzero precision/history={approximations}");
}
