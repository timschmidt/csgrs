use crate::Rational;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

type Term = (Rational, Rational);

// Only rational data may be shared. Opaque Computable owners stay in the
// coefficient's ordinary Vec so query copies preserve graph-sharing decisions.
#[derive(Clone, Default)]
pub(super) struct RadicalTerms(Option<Arc<Vec<Term>>>);

impl RadicalTerms {
    pub(super) fn new() -> Self {
        Self(None)
    }

    pub(super) fn as_slice(&self) -> &[Term] {
        self.0.as_ref().map_or(&[], |terms| terms.as_slice())
    }

    fn writable(&mut self) -> &mut Vec<Term> {
        Arc::make_mut(self.0.get_or_insert_with(|| Arc::new(Vec::new())))
    }

    pub(super) fn push(&mut self, term: Term) {
        self.writable().push(term);
    }

    pub(super) fn remove(&mut self, index: usize) -> Term {
        self.writable().remove(index)
    }

    pub(super) fn retain(&mut self, predicate: impl FnMut(&Term) -> bool) {
        if let Some(terms) = &mut self.0 {
            Arc::make_mut(terms).retain(predicate);
        }
    }
}

impl From<Vec<Term>> for RadicalTerms {
    fn from(terms: Vec<Term>) -> Self {
        if terms.is_empty() {
            Self::new()
        } else {
            Self(Some(Arc::new(terms)))
        }
    }
}

impl Deref for RadicalTerms {
    type Target = [Term];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for RadicalTerms {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match &mut self.0 {
            Some(terms) => Arc::make_mut(terms).as_mut_slice(),
            None => &mut [],
        }
    }
}

impl IntoIterator for RadicalTerms {
    type Item = Term;
    type IntoIter = std::vec::IntoIter<Term>;

    fn into_iter(self) -> Self::IntoIter {
        self.0
            .map_or_else(Vec::new, Arc::unwrap_or_clone)
            .into_iter()
    }
}

impl<'a> IntoIterator for &'a RadicalTerms {
    type Item = &'a Term;
    type IntoIter = std::slice::Iter<'a, Term>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a> IntoIterator for &'a mut RadicalTerms {
    type Item = &'a mut Term;
    type IntoIter = std::slice::IterMut<'a, Term>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_unique_shared_and_detached_buffers_release_their_owners() {
        let mut empty = RadicalTerms::new();
        empty.retain(|_| panic!("empty storage has no entries"));
        assert_eq!((&mut empty).into_iter().count(), 0);
        assert!(empty.0.is_none());
        assert_eq!(empty.into_iter().count(), 0);

        let pair = (Rational::new(13), Rational::new(7));
        let unique = RadicalTerms::from(vec![pair.clone()]);
        let pointer = unique.as_ptr();
        let weak = Arc::downgrade(unique.0.as_ref().unwrap());
        let iterator = unique.into_iter();
        assert_eq!(iterator.as_slice().as_ptr(), pointer);
        assert!(weak.upgrade().is_none());
        assert_eq!(iterator.collect::<Vec<_>>(), vec![pair.clone()]);

        let owner = RadicalTerms::from(vec![pair.clone()]);
        let weak = Arc::downgrade(owner.0.as_ref().unwrap());
        let shared = owner.clone();
        assert_eq!(owner.as_ptr(), shared.as_ptr());
        let values = shared.into_iter().collect::<Vec<_>>();
        assert_eq!(values, vec![pair.clone()]);
        assert_eq!(owner.as_slice(), values.as_slice());
        drop(owner);
        assert!(weak.upgrade().is_none());

        let owner = RadicalTerms::from(vec![pair.clone()]);
        let weak = Arc::downgrade(owner.0.as_ref().unwrap());
        let mut detached = owner.clone();
        detached[0].1 = Rational::new(-3);
        assert_ne!(detached.as_ptr(), owner.as_ptr());
        assert_eq!(owner.as_slice(), [pair].as_slice());
        let detached_weak = Arc::downgrade(detached.0.as_ref().unwrap());
        detached.remove(0);
        detached.push((Rational::new(2), Rational::new(9)));
        detached.retain(|_| false);
        assert!(detached.is_empty());
        drop(detached);
        assert!(detached_weak.upgrade().is_none());
        assert!(weak.upgrade().is_some());
        drop(owner);
        assert!(weak.upgrade().is_none());
    }
}
