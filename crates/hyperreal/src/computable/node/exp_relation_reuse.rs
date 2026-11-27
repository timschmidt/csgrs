/// Number of direct-mapped comparison slots per thread.
/// The slot mask requires a nonzero power of two. This caps entries and weak keys,
/// not comparison work or the total number of threads.
const EXP_ATOM_CACHE_SIZE: usize = 16;

/// One memoized structural-recognizer result for an ordered pair of immutable node allocations.
/// Weak keys keep the allocation addresses unavailable for reuse until eviction,
/// without retaining live node values or their child graphs. A false result is a
/// recognizer non-match, including unhandled forms, not numerical inequality.
struct ExpAtomEntry {
    /// First allocation key; callers canonicalize the pair by increasing address.
    a: std::sync::Weak<Node>,
    /// Second allocation key, checked in full on lookup rather than inferred from a slot hash.
    b: std::sync::Weak<Node>,
    /// Previously computed internal_structural_eq result, independent of approximation/fact caches.
    equal: bool,
}

/// Per-thread direct-mapped cache for positive and negative structural-recognizer results.
/// Collisions replace entries and cannot themselves prove a match; lookup compares
/// both exact addresses. This cache does not validate the real domains of its operands.
struct ExpAtomCache {
    /// Fixed slots, each holding at most two weak node keys; None denotes an empty slot.
    entries: [Option<ExpAtomEntry>; EXP_ATOM_CACHE_SIZE],
}

impl ExpAtomCache {
    /// Construct an empty cache without allocating node keys; usable in the constant TLS initializer.
    const fn new() -> Self {
        Self {
            entries: [const { None }; EXP_ATOM_CACHE_SIZE],
        }
    }

    /// Mix the ordered addresses with wrapping word arithmetic and mask to a slot index.
    /// The hash is a placement heuristic, never evidence of structural or numerical equality.
    fn slot(a: usize, b: usize) -> usize {
        let hash = a.wrapping_mul(0x9e37_79b9) ^ b.rotate_left(11);
        (hash ^ (hash >> 16)) & (EXP_ATOM_CACHE_SIZE - 1)
    }

    /// Look up exactly this ordered pair of live operands; Some(false) is a cached non-match.
    /// Check both addresses after hashing. Weak keys prevent address reuse, so matching
    /// live operands need no weak upgrade; a collision or empty slot returns None.
    /// This method does not sort the pair or mutate the cache.
    fn get(&self, a: &Computable, b: &Computable) -> Option<bool> {
        let ap = Arc::as_ptr(&a.internal) as usize;
        let bp = Arc::as_ptr(&b.internal) as usize;
        let entry = self.entries[Self::slot(ap, bp)].as_ref()?;
        (entry.a.as_ptr() as usize == ap && entry.b.as_ptr() as usize == bp).then_some(entry.equal)
    }

    /// Replace the pair hash slot with weak keys and the supplied recognizer result.
    /// The caller must provide the same canonical ordering as lookup and a valid
    /// result for these immutable nodes. Replacement releases the prior weak keys;
    /// this method does not perform the comparison or keep node payloads alive.
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
    /// Lazily available thread-local comparison cache with at most 32 weak node keys.
    /// Node payloads and child graphs are not strongly retained. Eviction and TLS
    /// destruction release keys; borrow conflicts or unavailable TLS cause best-effort bypass.
    static EXP_ATOM_CACHE: std::cell::RefCell<ExpAtomCache> =
        const { std::cell::RefCell::new(ExpAtomCache::new()) };
}

/// Compare atoms using pointer identity, a symmetric cache lookup, then the full structural recognizer.
/// Pointer identity returns immediately without adding weak keys. Other pairs are
/// ordered by address so both argument orders use the same slot. A full-address
/// hit reuses either true or false; neither hash collisions nor nearby values merge.
///
/// Release any cache borrow before traversing the graph. Borrow conflicts and TLS
/// teardown fall back to the uncached recognizer, and failed insertion is ignored.
/// Approximation/fact refinement cannot stale a hit because those fields are not
/// compared. The fallback may allocate traversal state and normalize rationals,
/// and has no total-work or cancellation limit supplied by this cache.
/// A false result is failure to recognize structural equality, not proof of unequal real values.
fn exp_atom_structural_eq(a: &Computable, b: &Computable) -> bool {
    if Arc::ptr_eq(&a.internal, &b.internal) {
        return true;
    }
    let (a, b) = if (Arc::as_ptr(&a.internal) as usize) < (Arc::as_ptr(&b.internal) as usize) {
        (a, b)
    } else {
        (b, a)
    };
    if let Ok(Some(equal)) =
        EXP_ATOM_CACHE.try_with(|cache| cache.try_borrow().ok().and_then(|cache| cache.get(a, b)))
    {
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
