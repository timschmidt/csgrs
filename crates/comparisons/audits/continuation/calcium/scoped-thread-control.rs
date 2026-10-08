// No Hyper, proof cache, GMP or MPFR: isolate the Memcheck scoped-thread record.
fn main() {
    for _ in 0..18 {
        let value = 1732usize;
        std::thread::scope(|scope| {
            let handle = scope.spawn(|| std::hint::black_box(value));
            assert_eq!(handle.join().unwrap(), value);
        });
    }
    println!("{{\"suite\":\"std-scoped-thread-control\",\"scopes\":18}}");
}
