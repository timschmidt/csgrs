use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

#[repr(align(4096))]
struct Aligned([u64; 512]);

#[test]
fn process_allocator_preserves_alignment_zeroing_and_reallocation_contents() {
    let aligned = Box::new(Aligned([0x1357_2468_ffff_0001; 512]));
    assert_eq!((&*aligned as *const Aligned as usize) % 4096, 0);
    assert!(
        aligned
            .0
            .iter()
            .all(|value| *value == 0x1357_2468_ffff_0001)
    );
    let mut values = vec![0_u64; 4097];
    assert!(values.iter().all(|value| *value == 0));
    for (i, value) in values.iter_mut().enumerate() {
        *value = (i as u64).wrapping_mul(0x1_0000_0001);
    }
    values.reserve_exact(8193);
    values.extend((4097_u64..12290).map(|i| i.wrapping_mul(0x1_0000_0001)));
    values.shrink_to_fit();
    assert_eq!(values.len(), 12290);
    for (i, value) in values.iter().enumerate() {
        assert_eq!(*value, (i as u64).wrapping_mul(0x1_0000_0001));
    }
    values.truncate(17);
    values.shrink_to_fit();
    assert_eq!(
        values,
        (0_u64..17)
            .map(|i| i.wrapping_mul(0x1_0000_0001))
            .collect::<Vec<_>>()
    );
}

#[test]
fn process_allocator_preserves_cross_thread_ownership_and_destruction() {
    struct Owned {
        payload: Vec<u64>,
        drops: Arc<AtomicUsize>,
    }
    impl Drop for Owned {
        fn drop(&mut self) {
            self.drops.fetch_add(1, Ordering::SeqCst);
        }
    }
    let drops = Arc::new(AtomicUsize::new(0));
    let values = (0..8)
        .map(|worker| {
            Arc::new(Owned {
                payload: (0..4096_u64).map(|i| i + worker).collect(),
                drops: drops.clone(),
            })
        })
        .collect::<Vec<_>>();
    let weak = values.iter().map(Arc::downgrade).collect::<Vec<_>>();
    std::thread::scope(|scope| {
        for (worker, value) in values.into_iter().enumerate() {
            scope.spawn(move || {
                assert_eq!(value.payload.len(), 4096);
                for (i, actual) in value.payload.iter().enumerate() {
                    assert_eq!(*actual, i as u64 + worker as u64);
                }
                let aligned = Arc::new(Aligned([worker as u64; 512]));
                assert_eq!((Arc::as_ptr(&aligned) as usize) % 4096, 0);
                assert_eq!(aligned.0[511], worker as u64);
            });
        }
    });
    assert_eq!(drops.load(Ordering::SeqCst), 8);
    assert!(weak.into_iter().all(|value| value.upgrade().is_none()));
    assert_eq!(Arc::strong_count(&drops), 1);
}
