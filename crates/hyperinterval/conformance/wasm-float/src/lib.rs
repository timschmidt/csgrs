//! Allocation-free public-API consumer. Inputs and outputs cross WASM as bits.
#![no_std]

use hyperinterval::{DomainCoverage, F32Enclosure, F64Enclosure, IntervalError};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}

fn error_code(error: IntervalError) -> u64 {
    match error {
        IntervalError::NonFiniteEndpoint => 16,
        IntervalError::ReversedEndpoints => 17,
        IntervalError::Overflow => 18,
        IntervalError::ContainsZero => 19,
        IntervalError::EmptyDomain => 20,
        IntervalError::UnqualifiedEnvironment => 21,
        _ => panic!("unexpected primitive error"),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn qualification() -> u32 {
    u32::from(F32Enclosure::backend_supported())
        | (u32::from(F64Enclosure::backend_supported()) << 1)
}

macro_rules! query {
    ($function:ident, $properties:ident, $interval:ty, $float:ty, $bits:ty, $conversion:ident, $convert:ident) => {
        /// Exact set facts and width ordering. Bit 63 marks a typed error code.
        #[unsafe(no_mangle)]
        pub extern "C" fn $properties(
            flags: u32,
            a: $bits,
            b: $bits,
            c: $bits,
            d: $bits,
            point: $bits,
        ) -> u64 {
            assert!(flags <= 3);
            let result = (|| {
                let x = if flags & 1 == 0 {
                    <$interval>::new(<$float>::from_bits(a), <$float>::from_bits(b))?
                } else {
                    <$interval>::empty()
                };
                let y = if flags & 2 == 0 {
                    <$interval>::new(<$float>::from_bits(c), <$float>::from_bits(d))?
                } else {
                    <$interval>::empty()
                };
                let width = x.width().zip(y.width()).map(|(x, y)| x.cmp(&y));
                let facts = [
                    x.is_empty(),
                    x.is_singleton(),
                    x.contains_point(<$float>::from_bits(point))?,
                    x.subset_of(&y),
                    x.interior_subset_of(&y),
                    x.strictly_left_of(&y),
                    x.disjoint(&y),
                    x.overlaps(&y),
                    width == Some(core::cmp::Ordering::Less),
                    width == Some(core::cmp::Ordering::Equal),
                    width == Some(core::cmp::Ordering::Greater),
                    x.signs().contains_negative(),
                    x.signs().contains_zero(),
                    x.signs().contains_positive(),
                ];
                Ok::<_, IntervalError>(facts.iter().enumerate().fold(0, |bits, (i, value)| {
                    bits | (u64::from(*value) << i)
                }))
            })();
            match result {
                Ok(bits) => bits,
                Err(error) => (1 << 63) | error_code(error),
            }
        }

        /// Cross-format endpoints use the destination encoding.
        #[unsafe(no_mangle)]
        pub extern "C" fn $conversion(selector: u32, empty: u32, a: $bits, b: $bits) -> u64 {
            assert!(selector <= 2 && empty <= 1);
            let source = if empty == 1 {
                Ok(<$interval>::empty())
            } else {
                <$interval>::new(<$float>::from_bits(a), <$float>::from_bits(b))
            };
            let result = source.and_then(|source| source.$convert());
            if selector == 0 {
                return match result {
                    Ok(range) => u64::from(range.is_empty()),
                    Err(error) => error_code(error),
                };
            }
            let range = result.expect("endpoints require success");
            let (lo, hi) = range.bounds().expect("endpoints require nonempty success");
            u64::from(if selector == 1 { lo } else { hi }.to_bits())
        }

        /// Selector 0 returns status; 1/2 return complete endpoint bits.
        /// Endpoints of errors/empty results trap rather than fabricate a value.
        #[unsafe(no_mangle)]
        pub extern "C" fn $function(
            operation: u32,
            selector: u32,
            flags: u32,
            a: $bits,
            b: $bits,
            c: $bits,
            d: $bits,
            exponent: u32,
        ) -> u64 {
            assert!(selector <= 2 && flags <= 3);
            let result = (|| {
                let x = if flags & 1 == 0 {
                    <$interval>::new(<$float>::from_bits(a), <$float>::from_bits(b))?
                } else {
                    <$interval>::empty()
                };
                let mut domain = DomainCoverage::Complete;
                let range = if matches!(operation, 1..=4 | 10..=13 | 20) {
                    let y = if flags & 2 == 0 {
                        <$interval>::new(<$float>::from_bits(c), <$float>::from_bits(d))?
                    } else {
                        <$interval>::empty()
                    };
                    match operation {
                        1 => x.add(&y)?,
                        2 => x.sub(&y)?,
                        3 => x.mul(&y)?,
                        4 => x.div(&y)?,
                        10 => x.min_image(&y),
                        11 => x.max_image(&y),
                        12 => x.hull(&y),
                        13 => x.intersection(&y),
                        20 => x.add(&y)?.square()?.sub(&x)?,
                        _ => unreachable!(),
                    }
                } else {
                    match operation {
                        0 => x,
                        5 => x.square()?,
                        6 => {
                            let image = x.sqrt()?;
                            domain = image.domain;
                            image.range
                        }
                        7 => x.affine(<$float>::from_bits(c), <$float>::from_bits(d))?,
                        8 => x.neg(),
                        9 => x.abs(),
                        14 => x.reciprocal()?,
                        15 => x.midpoint_enclosure()?,
                        16 => x.radius_enclosure()?,
                        17 => match x.width() {
                            Some(width) => width.enclosure()?,
                            None => <$interval>::empty(),
                        },
                        18 => x.expanded()?,
                        19 => x.checked_pow(exponent)?,
                        _ => panic!("invalid operation"),
                    }
                };
                Ok::<_, IntervalError>((range, domain))
            })();
            if selector == 0 {
                return match result {
                    Err(error) => error_code(error),
                    Ok((range, domain)) => {
                        u64::from(range.is_empty())
                            + if domain == DomainCoverage::Partial { 2 } else { 0 }
                    }
                };
            }
            let (range, _) = result.expect("endpoints require success");
            let (lower, upper) = range.bounds().expect("endpoints require nonempty success");
            u64::from(if selector == 1 { lower } else { upper }.to_bits())
        }
    };
}

query!(
    query32,
    properties32,
    F32Enclosure,
    f32,
    u32,
    convert32,
    to_f64_enclosure
);
query!(
    query64,
    properties64,
    F64Enclosure,
    f64,
    u64,
    convert64,
    to_f32_enclosure
);
