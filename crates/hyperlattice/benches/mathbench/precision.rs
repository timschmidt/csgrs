fn bench_precisions(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig_by_precision");
    let value = 1.2345678901234567_f64;

    for precision in [128_usize, 160, 192, 256] {
        let mut astro_ctx = astro_backend::Ctx::new(precision);
        let astro_value = astro_ctx.f(value);
        group.bench_with_input(
            BenchmarkId::new("astro/sin", precision),
            &precision,
            |b, _| b.iter(|| astro_ctx.sin(black_box(&astro_value))),
        );
    }

    for precision in [128_usize, 160, 192, 256] {
        let arp_ctx = arp_backend::Ctx::new(precision);
        let arp_value = arp_ctx.f(value);
        group.bench_with_input(
            BenchmarkId::new("arp/sin", precision),
            &precision,
            |b, _| b.iter(|| arp_ctx.sin(black_box(&arp_value))),
        );
    }

group.finish();
}
