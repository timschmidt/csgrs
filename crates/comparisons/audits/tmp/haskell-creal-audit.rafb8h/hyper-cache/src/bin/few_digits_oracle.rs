use rug::{Float, Integer, float::Round};
use std::{collections::BTreeMap, env, fs};

fn operation(x: &mut Float, name: &str, round: Round) {
    match name {
        "exp" => { x.exp_round(round); }
        "sin" => { x.sin_round(round); }
        "cos" => { x.cos_round(round); }
        "atan" => { x.atan_round(round); }
        "sinh" => { x.sinh_round(round); }
        "cosh" => { x.cosh_round(round); }
        "asinh" => { x.asinh_round(round); }
        "erf" => { x.erf_round(round); }
        "ln" => { x.ln_round(round); }
        "sqrt" => { x.sqrt_round(round); }
        "asin" => { x.asin_round(round); }
        "acos" => { x.acos_round(round); }
        "atanh" => { x.atanh_round(round); }
        "acosh" => { x.acosh_round(round); }
        _ => panic!("unknown operation"),
    }
}

fn main() {
    let mut counts = BTreeMap::<String,(usize,usize)>::new();
    for line in fs::read_to_string(env::args().nth(1).unwrap()).unwrap().lines() {
        let fields: Vec<_> = line.split('\t').collect();
        assert_eq!(fields.len(), 6);
        let name = fields[0];
        let p = 1024;
        let integer = |s: &str| Integer::from_str_radix(s,10).unwrap();
        let input = Float::with_val(p,integer(fields[1])) / integer(fields[2]);
        let mut lo = input.clone();
        let mut hi = input;
        operation(&mut lo,name,Round::Down);
        operation(&mut hi,name,Round::Up);
        let numerator = Float::with_val(p,integer(fields[4]));
        let denominator = integer(fields[5]);
        let got_lo = Float::with_val_round(p,&numerator / &denominator,Round::Down).0;
        let got_hi = Float::with_val_round(p,&numerator / &denominator,Round::Up).0;
        let bits: i32 = fields[3].parse().unwrap();
        let tolerance = Float::with_val(p,1) >> bits;
        let lower_allowed = Float::with_val_round(p,&hi-&tolerance,Round::Up).0;
        let upper_allowed = Float::with_val_round(p,&lo+&tolerance,Round::Down).0;
        let ok = got_lo >= lower_allowed && got_hi <= upper_allowed;
        let entry=counts.entry(name.into()).or_default();
        entry.0 += 1;
        if !ok {
            entry.1 += 1;
            if entry.1<=2 {println!("FAIL {name} {}/{} bits={bits} got={} reference={}",fields[1],fields[2],got_lo.to_f64(),lo.to_f64());}
        }
    }
    for (name,(n,bad)) in counts { println!("{name}\tchecks={n}\tfailures={bad}"); }
}
