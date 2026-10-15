//! Retain certified values without freezing transient failures.

use std::sync::OnceLock;

/// Computes outside the publication lock so a failed attempt leaves the cache
/// retryable. Concurrent cold readers may duplicate work, but only one success
/// is retained. A failed reader reuses a concurrently published success rather
/// than weakening that available certificate.
pub(crate) fn get_or_try_init<T, E>(
    cache: &OnceLock<T>,
    evaluate: impl FnOnce() -> Result<T, E>,
) -> Result<&T, E> {
    if let Some(value) = cache.get() {
        return Ok(value);
    }
    match evaluate() {
        Ok(value) => Ok(cache.get_or_init(|| value)),
        Err(error) => cache.get().ok_or(error),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Barrier;

    #[test]
    fn a_late_failure_reuses_the_published_certificate() {
        let cache = OnceLock::new();
        let started = Barrier::new(2);
        let published = Barrier::new(2);
        std::thread::scope(|scope| {
            let failing = scope.spawn(|| {
                get_or_try_init(&cache, || {
                    started.wait();
                    published.wait();
                    Err("predicate unresolved")
                })
            });
            started.wait();
            let certificate = get_or_try_init(&cache, || Ok::<_, &str>(42)).unwrap();
            published.wait();
            assert!(std::ptr::eq(failing.join().unwrap().unwrap(), certificate));
            assert!(std::ptr::eq(
                get_or_try_init::<_, &str>(&cache, || panic!("warm cache recomputed")).unwrap(),
                certificate,
            ));
        });
    }
}
