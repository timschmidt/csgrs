use hyperreal::Real;

fn main() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let opaque_zero = sine.clone() * sine + cosine.clone() * cosine - Real::one();

    println!("definitely_zero={}", opaque_zero.definitely_zero());
    println!("f64={:?}", opaque_zero.to_f64_lossy());
    let sinc = opaque_zero.clone().sinc();
    let sinc_pi = opaque_zero.clone().sinc_pi();
    let cosc = opaque_zero.cosc();
    println!("sinc={sinc:?}");
    println!("sinc_pi={sinc_pi:?}");
    println!("cosc={cosc:?}");
    println!("sinc_decimal={:#.12}", sinc.unwrap());
    println!("sinc_pi_decimal={:#.12}", sinc_pi.unwrap());
    println!("cosc_decimal={:#.12}", cosc.unwrap());
}
