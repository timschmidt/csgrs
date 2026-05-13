use hyperreal::{
    Computable, MagnitudeBits, Rational, RealSign, RealStructuralFacts, ZeroKnowledge,
};
use num::BigInt;
use std::cmp::Ordering;

fn rational(n: i64, d: u64) -> Computable {
    Computable::rational(Rational::fraction(n, d).expect("valid nonzero denominator"))
}

fn huge_integer_pow10(exp: u32) -> Computable {
    Computable::rational(Rational::from_bigint(BigInt::from(10_u8).pow(exp)))
}

fn facts(value: &Computable) -> String {
    let RealStructuralFacts {
        sign,
        zero,
        exact_rational,
        magnitude,
    } = value.structural_facts();

    format!(
        "sign={}, zero={}, exact_rational={}, magnitude={}",
        sign_text(sign),
        zero_text(zero),
        exact_rational,
        magnitude_text(magnitude)
    )
}

fn sign_text(sign: Option<RealSign>) -> &'static str {
    match sign {
        Some(RealSign::Negative) => "negative",
        Some(RealSign::Zero) => "zero",
        Some(RealSign::Positive) => "positive",
        None => "unknown",
    }
}

fn zero_text(zero: ZeroKnowledge) -> &'static str {
    match zero {
        ZeroKnowledge::Zero => "zero",
        ZeroKnowledge::NonZero => "nonzero",
        ZeroKnowledge::Unknown => "unknown",
    }
}

fn magnitude_text(magnitude: Option<MagnitudeBits>) -> String {
    match magnitude {
        Some(MagnitudeBits { msd, exact_msd }) => format!("msd {msd} (exact={exact_msd})"),
        None => "unknown".to_owned(),
    }
}

fn ordering_text(ordering: Ordering) -> &'static str {
    match ordering {
        Ordering::Less => "less",
        Ordering::Equal => "equal within tolerance",
        Ordering::Greater => "greater",
    }
}

fn print_node(name: &str, value: &Computable) {
    println!("{name}:");
    println!("  facts: {}", facts(value));
    println!("  sign_until(-24): {:?}", value.sign_until(-24));
    println!("  approx(-12): {}", value.approx(-12));
    println!("  approx(-24): {}", value.approx(-24));
    println!();
}

fn print_facts_only(name: &str, value: &Computable) {
    println!("{name}:");
    println!("  facts: {}", facts(value));
    println!("  sign_until(-24): {:?}", value.sign_until(-24));
    println!();
}

fn print_refinement(name: &str, value: &Computable, precisions: &[i32]) {
    println!("{name} refinement:");
    for &precision in precisions {
        println!("  approx({precision:>3}) = {}", value.approx(precision));
    }
    println!("  decimal = {:.24}", value);
    println!();
}

fn print_stage_graphs() {
    println!("symbolic stage 1 - original expression graph:");
    println!(
        r#"```mermaid
flowchart TD
    phase["10^30*pi + 7/5"]
    tiny["2^-40"]
    phaseTiny["add tiny"]
    sinHuge["sin"]
    phaseCos["10^30*pi + 7/5"]
    cosHuge["cos"]
    sinSq["square"]
    cosSq["square"]
    norm["add: trig_norm"]
    sevenTenths["7/10"]
    atan["atan"]
    tan["tan"]
    threeFifths["3/5"]
    asin["asin"]
    sinAsin["sin"]
    numerator["add: numerator"]
    invNorm["inverse"]
    product["multiply"]
    root["sqrt root"]
    phase --> phaseTiny
    tiny --> phaseTiny
    phaseTiny --> sinHuge --> sinSq --> norm
    phaseCos --> cosHuge --> cosSq --> norm
    sevenTenths --> atan --> tan --> numerator
    threeFifths --> asin --> sinAsin --> numerator
    norm --> invNorm --> product
    numerator --> product --> root
    root:::root
    classDef root fill:#f7f3c6,stroke:#8a6d00,stroke-width:2px
```
"#
    );

    println!("symbolic stage 2 - inverse-function reductions:");
    println!(
        r#"```mermaid
flowchart TD
    sevenTenths["7/10"]
    atanTan["tan(atan(7/10))"]
    sevenReduced["7/10 retained"]
    threeFifths["3/5"]
    asinSin["sin(asin(3/5))"]
    threeReduced["3/5 retained"]
    numerator["add: numerator"]
    sevenTenths --> atanTan --> sevenReduced --> numerator
    threeFifths --> asinSin --> threeReduced --> numerator
    atanTan:::changed
    asinSin:::changed
    sevenReduced:::result
    threeReduced:::result
    classDef changed fill:#ffe3dc,stroke:#b5472f,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
```
"#
    );

    println!("numeric stage 3 - large argument reduction:");
    println!(
        r#"```mermaid
flowchart TD
    hugeSin["sin(10^30*pi + 7/5 + 2^-40)"]
    reducedSin["sin(7/5 + 2^-40)"]
    hugeCos["cos(10^30*pi + 7/5)"]
    reducedCos["cos(7/5)"]
    sinSq["square"]
    cosSq["square"]
    norm["add: trig_norm"]
    hugeSin --> reducedSin --> sinSq --> norm
    hugeCos --> reducedCos --> cosSq --> norm
    hugeSin:::changed
    hugeCos:::changed
    reducedSin:::result
    reducedCos:::result
    classDef changed fill:#ffe3dc,stroke:#b5472f,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
```
"#
    );

    println!("numeric stage 4 - refinement and cache:");
    println!(
        r#"```mermaid
flowchart LR
    root["sqrt root"]
    p8["approx(-8)"]
    p16["approx(-16)"]
    p32["approx(-32)"]
    p64["approx(-64)"]
    p80["approx(-80)"]
    cached["second approx(-80): cache hit"]
    root --> p8 --> p16 --> p32 --> p64 --> p80 --> cached
    p8:::changed
    p16:::changed
    p32:::changed
    p64:::changed
    p80:::result
    cached:::cache
    classDef changed fill:#e0f2fe,stroke:#0369a1,stroke-width:2px
    classDef result fill:#dcfce7,stroke:#15803d,stroke-width:2px
    classDef cache fill:#ede9fe,stroke:#6d28d9,stroke-width:2px
```
"#
    );
}

fn build_expression() -> (Computable, Computable, Computable, Computable, Computable) {
    let pi = Computable::pi();
    let huge = huge_integer_pow10(30);
    let residual = rational(7, 5);
    let tiny = rational(1, 1_u64 << 40);
    let three_fifths = rational(3, 5);
    let seven_tenths = rational(7, 10);

    let huge_pi = pi.clone().multiply(huge.clone());
    let phase = huge_pi.clone().add(residual.clone());
    let phase_plus_tiny = phase.clone().add(tiny);

    let sin_huge = phase_plus_tiny.sin();
    let cos_huge = phase.cos();
    let tan_atan = seven_tenths.atan().tan();
    let sin_asin = three_fifths.asin().sin();

    let trig_norm = sin_huge.square().add(cos_huge.square());
    let numerator = tan_atan.add(sin_asin);
    let product = numerator.clone().multiply(trig_norm.clone().inverse());
    let root = product.clone().sqrt();

    (trig_norm, numerator, product, root, residual)
}

fn main() {
    let pi = Computable::pi();
    let huge = huge_integer_pow10(30);
    let residual = rational(7, 5);
    let tiny = rational(1, 1_u64 << 40);
    let three_fifths = rational(3, 5);
    let seven_tenths = rational(7, 10);

    println!("# Computable refinement walk");
    println!();
    println!("Expression:");
    println!(
        "sqrt((tan(atan(7/10)) + sin(asin(3/5))) / \
         (sin(10^30*pi + 7/5 + 2^-40)^2 + cos(10^30*pi + 7/5)^2))"
    );
    println!();
    print_stage_graphs();

    let huge_pi = pi.clone().multiply(huge);
    let phase = huge_pi.add(residual.clone());
    let phase_plus_tiny = phase.clone().add(tiny.clone());

    print_node("residual 7/5", &residual);
    print_node("tiny 2^-40", &tiny);
    println!("phase = 10^30*pi + 7/5:");
    println!("  facts: {}", facts(&phase));
    println!("  zero_status: {:?}", phase.zero_status());
    println!("  sign_until(0): {:?}", phase.sign_until(0));
    println!();

    let (trig_norm, numerator, product, root, _) = build_expression();

    print_node("trig_norm", &trig_norm);
    print_node("numerator", &numerator);
    print_refinement("trig_norm", &trig_norm, &[-8, -16, -32, -64]);
    print_refinement("numerator", &numerator, &[-8, -16, -32, -64]);
    print_facts_only("product before sqrt", &product);
    print_facts_only("root", &root);
    let (_, _, _, final_root, _) = build_expression();
    let final_scaled = final_root.approx(-80);
    println!("final root:");
    println!("  approx(-80) = {final_scaled}");
    println!("  decimal = {:.24}", final_root);
    println!();

    let sin_huge = phase_plus_tiny.sin();
    let cos_huge = phase.cos();
    let reduced_sin = residual.clone().add(tiny).sin();
    let reduced_cos = residual.cos();

    println!("large-argument reduction checks:");
    println!(
        "  sin(10^30*pi + 7/5 + 2^-40) vs sin(7/5 + 2^-40): {}",
        ordering_text(sin_huge.compare_absolute(&reduced_sin, -64))
    );
    println!(
        "  cos(10^30*pi + 7/5) vs cos(7/5): {}",
        ordering_text(cos_huge.compare_absolute(&reduced_cos, -64))
    );
    println!();

    println!("inverse-function reduction checks:");
    println!(
        "  tan(atan(7/10)) vs 7/10: {}",
        ordering_text(
            seven_tenths
                .clone()
                .atan()
                .tan()
                .compare_absolute(&seven_tenths, -64)
        )
    );
    println!(
        "  sin(asin(3/5)) vs 3/5: {}",
        ordering_text(
            three_fifths
                .clone()
                .asin()
                .sin()
                .compare_absolute(&three_fifths, -64)
        )
    );
    println!();

    println!("cache demonstration:");
    let (_, _, _, cached_root, _) = build_expression();
    println!("  first approx(-80) = {}", cached_root.approx(-80));
    println!("  second approx(-80) = {}", cached_root.approx(-80));
}
