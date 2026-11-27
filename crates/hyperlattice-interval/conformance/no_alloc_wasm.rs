//! Clean no_std cdylib consumer of fixed-grid interval and retained affine APIs.
//!
//! The fixed-wasm manifest links this source with only the fixed feature and no global
//! allocator. Seven C-ABI exports clamp/reduce arbitrary i32/u32 inputs into bounded
//! fixtures, assert complete native results and return integer digests for a host oracle.
//! The digests alone are not independent proofs of all bounds; assertions and the
//! host's corner/formula expectations supply complementary evidence.
//!
//! Unexpected errors/assertions enter a permanent spin in this panic handler. These
//! exports are conformance probes, not a general fallible geometry ABI. Hosts must
//! provide external containment if a regression hangs a call. A successful link and
//! finite no-growth runs do not establish a universal stack/work/memory bound.
#![no_std]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};

#[panic_handler]
/// Keep the allocator-free guest freestanding by spinning forever on panic.
/// Ignore diagnostic information and perform no formatting, allocation or unwinding.
/// This supplies no error status or recovery; the embedding runtime must stop a hang.
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
/// Clamp input to s in `[-256,256]`, retain t in `[s,s+1]`, and map it to
/// (2t+3,2t-4), then subtract the two coordinates. Assert the exact singleton seven
/// before returning seven. Composing retained images preserves the common t; boxing
/// the intermediate image would lose that relation. Unexpected failure spins.
pub extern "C" fn affine_correlation_probe(input: i32) -> i32 {
    /// Whole-unit i32 endpoints for the clamped, representable integer fixture.
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(-256, 256);
    let point = |x| I::point(x).unwrap();
    let duplicate = AffineMap::new(
        IntervalMatrix::new([[point(2)], [point(2)]]).unwrap(),
        IntervalVector::new([point(3), point(-4)]).unwrap(),
    )
    .unwrap();
    let difference = AffineMap::new(
        IntervalMatrix::new([[point(1), point(-1)]]).unwrap(),
        IntervalVector::new([point(0)]).unwrap(),
    )
    .unwrap();
    let source = IntervalVector::new([I::new(s, s + 1).unwrap()]).unwrap();
    let image = difference.image(duplicate.image(source));
    let bounds = image.enclosure(&mut ()).unwrap().lanes().unwrap()[0]
        .bounds()
        .unwrap();
    assert_eq!(bounds, (7, 7));
    bounds.0
}

#[unsafe(no_mangle)]
/// Enclose (2x-y+3,x+y-4) on `[s,s+1]` by `[-1,2]`, with input clamped to s.
/// Here s lies in `[-256,256]`. Assert all four bounds (2s+1,2s+6,s-5,s-1), then
/// return lo_x+3*hi_x+5*lo_y+7*hi_y = 20s-13. This Cartesian enclosure treats
/// source lanes independently; the bounded fixture fits the whole-unit i32 grid.
pub extern "C" fn interval_affine_probe(input: i32) -> i32 {
    /// Whole-unit i32 endpoints for the clamped, representable integer fixture.
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(-256, 256);
    let point = |x| I::point(x).unwrap();
    let a = IntervalMatrix::new([[point(2), point(-1)], [point(1), point(1)]]).unwrap();
    let x = IntervalVector::new([I::new(s, s + 1).unwrap(), I::new(-1, 2).unwrap()]).unwrap();
    let b = IntervalVector::new([point(3), point(-4)]).unwrap();
    let result = a.affine_enclosure(&x, &b, &mut ()).unwrap();
    let [(lo0, hi0), (lo1, hi1)] = result.into_lanes().unwrap().map(|i| i.bounds().unwrap());
    assert_eq!((lo0, hi0, lo1, hi1), (2 * s + 1, 2 * s + 6, s - 5, s - 1));
    lo0 + 3 * hi0 + 5 * lo1 + 7 * hi1
}

#[unsafe(no_mangle)]
/// Reduce depth modulo 33 and clamp input to s in `[-256,256]`.
/// Build a capacity-32 chain on `[s,s+1]` by `[-1,2]` using (x,y)->(x+y+1,y-1).
/// For n maps, y_n=y-n and x_n=x+n*y+n-n(n-1)/2. Assert the resulting bounds,
/// then return their weighted digest. Depth zero exercises the retained source.
/// The input reduction guarantees capacity admission; scalar work is not metered.
pub extern "C" fn affine_chain_probe(input: i32, depth: u32) -> i32 {
    /// Whole-unit i32 endpoints for the clamped, representable integer fixture.
    type I = FixedInterval<i32, 0>;
    let p = |n| I::point(n).unwrap();
    let s = input.clamp(-256, 256);
    let depth = (depth % 33) as i32;
    let source = IntervalVector::new([I::new(s, s + 1).unwrap(), I::new(-1, 2).unwrap()]).unwrap();
    let mut chain = AffineChain::<_, _, 2, 32>::new(source);
    for _ in 0..depth {
        chain
            .push(
                AffineMap::new(
                    IntervalMatrix::new([[p(1), p(1)], [p(0), p(1)]]).unwrap(),
                    IntervalVector::new([p(1), p(-1)]).unwrap(),
                )
                .unwrap(),
            )
            .unwrap();
    }
    let result = chain.enclosure(&mut ()).unwrap();
    let [(lo0, hi0), (lo1, hi1)] = result.into_lanes().unwrap().map(|i| i.bounds().unwrap());
    let offset = depth - depth * (depth - 1) / 2;
    assert_eq!(
        (lo0, hi0, lo1, hi1),
        (
            s - depth + offset,
            s + 1 + 2 * depth + offset,
            -1 - depth,
            2 - depth
        )
    );
    lo0 + 3 * hi0 + 5 * lo1 + 7 * hi1
}

#[unsafe(no_mangle)]
/// Clamp input to s in `[2,256]` and query a two-by-two interval matrix.
/// Its diagonal is (-s-2,-s-1), with off-diagonal intervals `[1,2]` and `[-1,1]`.
/// Check induced norms, signed logarithmic measures and norms of row/column
/// strict-dominance margins. Return the weighted norm/measure digest 50-44s.
/// The margin queries are consumed by subsequent norm queries; failure spins.
pub extern "C" fn interval_norm_probe(input: i32) -> i32 {
    /// Whole-unit i32 endpoints for the clamped, representable integer fixture.
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(2, 256);
    let p = |a, b| I::new(a, b).unwrap();
    let matrix =
        IntervalMatrix::new([[p(-s - 2, -s - 2), p(1, 2)], [p(-1, 1), p(-s - 1, -s - 1)]]).unwrap();
    let one = matrix.norm_one(&mut ()).unwrap().bounds().unwrap();
    let infinity = matrix.norm_infinity(&mut ()).unwrap().bounds().unwrap();
    let measure_one = matrix.measure_one(&mut ()).unwrap().bounds().unwrap();
    let measure_infinity = matrix.measure_infinity(&mut ()).unwrap().bounds().unwrap();
    let rows = matrix.row_dominance_margins(&mut ()).unwrap();
    let columns = matrix.column_dominance_margins(&mut ()).unwrap();
    assert_eq!(one, (s + 2, s + 3));
    assert_eq!(infinity, (s + 3, s + 4));
    assert_eq!(measure_one, (-s, 1 - s));
    assert_eq!(measure_infinity, (-s - 1, -s));
    assert_eq!(
        rows.norm_one(&mut ()).unwrap().bounds(),
        Some((2 * s, 2 * s + 2))
    );
    assert_eq!(
        columns.norm_infinity(&mut ()).unwrap().bounds(),
        Some((s + 1, s + 2))
    );
    one.0
        + 3 * one.1
        + 5 * infinity.0
        + 7 * infinity.1
        + 11 * measure_one.0
        + 13 * measure_one.1
        + 17 * measure_infinity.0
        + 19 * measure_infinity.1
}

#[unsafe(no_mangle)]
/// Clamp input to s in `[0,255]` and compare exact widths on the i64 2^-63 grid.
/// Raw widths are u64::MAX-s, 2s, u64::MAX-s and u64::MAX. Assert that the widest
/// width cannot project to an interval of the same signed endpoint family. The
/// first-axis tie rule selects zero when s=0 and three otherwise; return 1000*axis+s.
/// No rounded width projection supplies the selection decision.
pub extern "C" fn interval_width_probe(input: i32) -> i32 {
    /// Finest admitted i64 grid: raw numerators have spacing 2^-63; widths may need u64.
    type I = FixedInterval<i64, 63>;
    let s = i64::from(input.clamp(0, 255));
    let source = IntervalVector::new([
        I::new(i64::MIN, i64::MAX - s).unwrap(),
        I::new(-s, s).unwrap(),
        I::new(i64::MIN + s, i64::MAX).unwrap(),
        I::new(i64::MIN, i64::MAX).unwrap(),
    ])
    .unwrap();
    let widths = source.widths(&mut ()).unwrap().unwrap();
    assert_eq!(widths[0].numerator(), u64::MAX - s as u64);
    assert_eq!(widths[1].numerator(), 2 * s as u64);
    assert_eq!(widths[0], widths[2]);
    assert_eq!(widths[3].numerator(), u64::MAX);
    assert!(widths[3].to_interval().is_err());
    let axis = source.widest_axis(&mut ()).unwrap().unwrap();
    assert_eq!(axis, if s == 0 { 0 } else { 3 });
    axis as i32 * 1000 + s as i32
}

#[unsafe(no_mangle)]
/// Clamp input to s in `[-256,256]` and retain raw fixed-grid bounds before projection.
/// Adjacent numerators have half-grid centers/radii, so their enclosures round outward;
/// `[-7,9]` has exact raw center one and radius eight. Consume projected radii in a
/// norm query, reconstruct both vector and matrix bounds, and check that an extreme
/// radius projection fails while the retained full-range width still succeeds.
/// Return 2s+31 only after every assertion. These are raw 2^-63-grid numerators.
pub extern "C" fn midpoint_radius_probe(input: i32) -> i32 {
    /// Finest admitted i64 grid: raw numerators have spacing 2^-63; widths may need u64.
    type I = FixedInterval<i64, 63>;
    let s = i64::from(input.clamp(-256, 256));
    let a = I::new(s, s + 1).unwrap();
    let b = I::new(-s - 1, -s).unwrap();
    let c = I::new(-7, 9).unwrap();
    let source = IntervalVector::new([a, b, c]).unwrap();
    let form = source.midpoint_radius();
    let midpoint = form.midpoint_enclosure(&mut ()).unwrap();
    assert_eq!(midpoint.lanes().unwrap(), &[a, b, I::point(1).unwrap()]);
    let radius = form.radius_enclosure(&mut ()).unwrap();
    assert_eq!(
        radius.lanes().unwrap(),
        &[
            I::new(0, 1).unwrap(),
            I::new(0, 1).unwrap(),
            I::point(8).unwrap()
        ]
    );
    assert_eq!(radius.norm_one(&mut ()).unwrap().bounds(), Some((8, 10)));
    assert_eq!(form.into_intervals(), source);
    let matrix = IntervalMatrix::new([[a, b], [c, I::point(s).unwrap()]]).unwrap();
    let form = matrix.midpoint_radius();
    assert_eq!(
        form.midpoint_enclosure(&mut ()).unwrap().rows().unwrap()[1],
        [I::point(1).unwrap(), I::point(s).unwrap()]
    );
    assert_eq!(
        form.radius_enclosure(&mut ()).unwrap().rows().unwrap()[1],
        [I::point(8).unwrap(), I::point(0).unwrap()]
    );
    assert_eq!(form.into_intervals(), matrix);
    let extreme = IntervalVector::new([I::new(i64::MIN, i64::MAX).unwrap()])
        .unwrap()
        .midpoint_radius();
    assert!(extreme.radius_enclosure(&mut ()).is_err());
    assert_eq!(
        extreme.into_intervals().widths(&mut ()).unwrap().unwrap()[0].numerator(),
        u64::MAX
    );
    (2 * s + 31) as i32
}

#[unsafe(no_mangle)]
/// Clamp input to s in `[-256,256]`, form I-A and check the involution I-(I-A)=A.
/// For the varying diagonal `[-s,1-s]`, independently compute absolute extrema, then
/// assert both induced contraction-margin enclosures. Consume the remainder in a
/// box action and check its full output. Return a weighted margin digest; negative
/// margins remain valid results and are not evidence of a contraction certificate.
pub extern "C" fn contraction_probe(input: i32) -> i32 {
    /// Whole-unit i32 endpoints for the clamped, representable integer fixture.
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(-256, 256);
    let zero = I::point(0).unwrap();
    let matrix = IntervalMatrix::new([
        [I::new(s, s + 1).unwrap(), I::new(-1, 1).unwrap()],
        [zero, I::point(1).unwrap()],
    ])
    .unwrap();
    let remainder = matrix.identity_minus(&mut ()).unwrap();
    assert_eq!(remainder.rows().unwrap()[0][0].bounds(), Some((-s, 1 - s)));
    assert_eq!(remainder.identity_minus(&mut ()).unwrap(), matrix);
    let low = if s == 0 || s == 1 {
        0
    } else {
        s.abs().min((1 - s).abs())
    };
    let high = s.abs().max((1 - s).abs());
    let one = remainder.contraction_margin_one(&mut ()).unwrap();
    let infinity = remainder.contraction_margin_infinity(&mut ()).unwrap();
    assert_eq!(one.bounds(), Some((1 - high.max(1), 1 - low)));
    assert_eq!(infinity.bounds(), Some((-high, 1 - low)));
    let image = remainder
        .mul_vector(
            &IntervalVector::new([I::new(-1, 1).unwrap(); 2]).unwrap(),
            &mut (),
        )
        .unwrap();
    assert_eq!(
        image.lanes().unwrap()[0].bounds(),
        Some((-high - 1, high + 1))
    );
    assert_eq!(image.lanes().unwrap()[1], zero);
    one.bounds().unwrap().0 + 3 * infinity.bounds().unwrap().0 + 7 * infinity.bounds().unwrap().1
}
