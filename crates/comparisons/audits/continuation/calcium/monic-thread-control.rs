use std::sync::Barrier;
fn main() {
    let mut completed = 0;
    for _ in 0..81 {
        let barrier = Barrier::new(4);
        std::thread::scope(|scope| {
            let handles: Vec<_> = (0..4).map(|worker| {
                let barrier = &barrier;
                scope.spawn(move || { barrier.wait(); worker + 1 })
            }).collect();
            for handle in handles { completed += handle.join().unwrap(); }
        });
    }
    assert_eq!(completed, 810);
    println!("{{\"suite\":\"std-thread-only-control\",\"scopes\":81,\"workers_per_scope\":4,\"checksum\":{completed}}}");
}
