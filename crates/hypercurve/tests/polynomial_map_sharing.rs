//! Retained map-prefix storage measured independently of native wire size.
use hypercurve::{BezierParameter2, CurvePoint2, CurvePointDefinition2, RationalMap2, Real};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, Ordering};

struct CountingAllocator;
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;
static LIVE_BYTES: AtomicIsize = AtomicIsize::new(0);
static LIVE_BLOCKS: AtomicIsize = AtomicIsize::new(0);
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            LIVE_BYTES.fetch_add(layout.size() as isize, Ordering::Relaxed);
            LIVE_BLOCKS.fetch_add(1, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        LIVE_BYTES.fetch_sub(layout.size() as isize, Ordering::Relaxed);
        LIVE_BLOCKS.fetch_sub(1, Ordering::Relaxed);
        unsafe { System.dealloc(p, layout) };
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let new = unsafe { System.realloc(p, layout, size) };
        if !new.is_null() {
            LIVE_BYTES.fetch_add(size as isize - layout.size() as isize, Ordering::Relaxed);
        }
        new
    }
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn source() -> CurvePoint2 {
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 0, 1], &[-1]]),
        parameter: BezierParameter2::Exact(2.into()).into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn translation() -> RationalMap2 {
    RationalMap2::new(
        [polynomial(&[&[1], &[1]]), polynomial(&[&[0, 1]])],
        polynomial(&[&[1]]),
    )
}
#[test]
fn retained_polynomial_map_prefix_allocations_grow_linearly() {
    let source = source();
    let map = translation();
    drop(source.rational_image(&map).unwrap());
    let mut samples = Vec::new();
    for count in [128_usize, 512] {
        let bytes = LIVE_BYTES.load(Ordering::Relaxed);
        let blocks = LIVE_BLOCKS.load(Ordering::Relaxed);
        let mut points = Vec::with_capacity(count + 1);
        points.push(source.clone());
        for i in 0..count {
            points.push(points[i].rational_image(&map).unwrap());
        }
        let retained_bytes = LIVE_BYTES.load(Ordering::Relaxed) - bytes;
        let retained_blocks = LIVE_BLOCKS.load(Ordering::Relaxed) - blocks;
        samples.push((count, retained_bytes, retained_blocks));
        let last = points.last().unwrap();
        assert_eq!(
            last.affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&Real::from((count + 2) as u64))
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            last.rational_form(polynomial(&[&[-2, 0, 0, 1]]), polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        std::thread::Builder::new()
            .stack_size(128 * 1024)
            .spawn(move || drop(points))
            .unwrap()
            .join()
            .unwrap();
    }
    #[cfg(feature = "svg")]
    for count in [4, 8, 16] {
        let mut point = source.clone();
        let mut curves = Vec::new();
        for _ in 0..count {
            let next = point.rational_image(&map).unwrap();
            curves.push(hypercurve::Curve2::try_line(point, next.clone()).unwrap());
            point = next;
        }
        let geometry = hypercurve::SvgGeometry2::new(
            hypercurve::CurveRegion2::empty(),
            vec![],
            vec![hypercurve::CurvePath2::try_new(curves).unwrap()],
        );
        let text = hypercurve::export_svg_document(&geometry).unwrap();
        assert_eq!(
            hypercurve::export_svg_document(&hypercurve::import_svg_document(&text).unwrap())
                .unwrap(),
            text
        );
        eprintln!(
            "retained map prefix wire: {count} spans, {} bytes",
            text.len()
        );
        if let Some(directory) = std::env::var_os("HYPERCURVE_MAP_SHARING_EVIDENCE") {
            std::fs::write(
                std::path::Path::new(&directory).join(format!("{count}.svg")),
                &text,
            )
            .unwrap();
        }
    }
    eprintln!("retained map prefixes (count, bytes, allocations): {samples:?}");
    assert!(samples[0].1 > 0 && samples[0].2 > 0);
    assert!(
        samples[1].1 <= 6 * samples[0].1,
        "four times as many retained prefixes must not require quadratic heap bytes: {samples:?}"
    );
    assert!(
        samples[1].2 <= 6 * samples[0].2,
        "retained allocation count must scale with the shared chain"
    );
}
