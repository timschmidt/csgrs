// Reuse opaque-atom comparisons without retaining expression graphs or changing
// the proof domain. Approximation variants and their arguments are immutable;
// refinement changes only facts/caches, which structural equality ignores.
// Weak keys keep allocations from being reused while an entry exists. A hit
// compares BOTH exact addresses, never just a hash, so collisions cannot prove
// equality. A false entry means structural inequality, not numerical inequality.
const EXP_ATOM_CACHE_SIZE: usize = 16;

struct ExpAtomEntry {
    a: std::sync::Weak<Node>,
    b: std::sync::Weak<Node>,
    equal: bool,
}

struct ExpAtomCache {
    entries: [Option<ExpAtomEntry>; EXP_ATOM_CACHE_SIZE],
}

impl ExpAtomCache {
    const fn new() -> Self {
        Self {
            entries: [const { None }; EXP_ATOM_CACHE_SIZE],
        }
    }

    fn slot(a: usize, b: usize) -> usize {
        let hash = a.wrapping_mul(0x9e37_79b9) ^ b.rotate_left(11);
        (hash ^ (hash >> 16)) & (EXP_ATOM_CACHE_SIZE - 1)
    }

    fn get(&self, a: &Computable, b: &Computable) -> Option<bool> {
        let ap = Arc::as_ptr(&a.internal) as usize;
        let bp = Arc::as_ptr(&b.internal) as usize;
        let entry = self.entries[Self::slot(ap, bp)].as_ref()?;
        (entry.a.as_ptr() as usize == ap && entry.b.as_ptr() as usize == bp)
            .then_some(entry.equal)
    }

    fn insert(&mut self, a: &Computable, b: &Computable, equal: bool) {
        let slot = Self::slot(
            Arc::as_ptr(&a.internal) as usize,
            Arc::as_ptr(&b.internal) as usize,
        );
        self.entries[slot] = Some(ExpAtomEntry {
            a: Arc::downgrade(&a.internal),
            b: Arc::downgrade(&b.internal),
            equal,
        });
    }
}

std::thread_local! {
    // At most 32 weakly retained Node allocations per thread, not their child
    // graphs or heap payloads. Eviction and thread exit release those weak refs.
    static EXP_ATOM_CACHE: std::cell::RefCell<ExpAtomCache> =
        const { std::cell::RefCell::new(ExpAtomCache::new()) };
}

fn exp_atom_structural_eq(a: &Computable, b: &Computable) -> bool {
    if Arc::ptr_eq(&a.internal, &b.internal) {
        return true;
    }
    let (a, b) = if (Arc::as_ptr(&a.internal) as usize) < (Arc::as_ptr(&b.internal) as usize) {
        (a, b)
    } else {
        (b, a)
    };
    if let Ok(Some(equal)) = EXP_ATOM_CACHE.try_with(|cache| {
        cache.try_borrow().ok().and_then(|cache| cache.get(a, b))
    }) {
        return equal;
    }
    // Do not hold a RefCell borrow while traversing a graph. Reentrant calls or
    // calls during TLS teardown must simply use the full uncached comparison.
    let equal = Computable::internal_structural_eq(a, b);
    let _ = EXP_ATOM_CACHE.try_with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache.insert(a, b, equal);
        }
    });
    equal
}
