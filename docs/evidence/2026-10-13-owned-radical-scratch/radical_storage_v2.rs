use crate::Rational;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

type Term = (Rational, Rational);

#[derive(Clone)]
enum Storage {
    Owned(Vec<Term>),
    Retained(Arc<Vec<Term>>),
}

// Mutable scratch arrays keep ordinary Vec ownership. Only a read from a
// retained certificate shares a buffer, and that buffer owns only rationals.
#[derive(Clone)]
pub(super) struct RadicalTerms(Storage);

#[derive(Clone, Default)]
pub(super) struct ClosedRadicalTerms(Option<Arc<Vec<Term>>>);

impl Default for RadicalTerms {
    fn default() -> Self {
        Self::new()
    }
}

impl RadicalTerms {
    pub(super) fn new() -> Self {
        Self(Storage::Owned(Vec::new()))
    }

    pub(super) fn as_slice(&self) -> &[Term] {
        match &self.0 {
            Storage::Owned(terms) => terms,
            Storage::Retained(terms) => terms,
        }
    }

    fn into_vec(self) -> Vec<Term> {
        match self.0 {
            Storage::Owned(terms) => terms,
            Storage::Retained(terms) => Arc::unwrap_or_clone(terms),
        }
    }

    fn writable(&mut self) -> &mut Vec<Term> {
        if matches!(self.0, Storage::Retained(_)) {
            let terms = std::mem::take(self).into_vec();
            self.0 = Storage::Owned(terms);
        }
        // The only other variant was converted above under exclusive access.
        let Storage::Owned(terms) = &mut self.0 else {
            unreachable!("retained storage was converted to owned storage");
        };
        terms
    }

    pub(super) fn push(&mut self, term: Term) {
        self.writable().push(term);
    }

    pub(super) fn remove(&mut self, index: usize) -> Term {
        self.writable().remove(index)
    }

    pub(super) fn retain(&mut self, predicate: impl FnMut(&Term) -> bool) {
        self.writable().retain(predicate);
    }

    pub(super) fn retained(&self) -> ClosedRadicalTerms {
        if self.is_empty() {
            return ClosedRadicalTerms(None);
        }
        ClosedRadicalTerms(Some(match &self.0 {
            Storage::Owned(terms) => Arc::new(terms.clone()),
            Storage::Retained(terms) => terms.clone(),
        }))
    }
}

impl ClosedRadicalTerms {
    pub(super) fn to_terms(&self) -> RadicalTerms {
        match &self.0 {
            Some(terms) => RadicalTerms(Storage::Retained(terms.clone())),
            None => RadicalTerms::new(),
        }
    }
}

impl Deref for ClosedRadicalTerms {
    type Target = [Term];

    fn deref(&self) -> &Self::Target {
        self.0.as_ref().map_or(&[], |terms| terms.as_slice())
    }
}

impl From<Vec<Term>> for RadicalTerms {
    fn from(terms: Vec<Term>) -> Self {
        Self(Storage::Owned(terms))
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
        self.writable().as_mut_slice()
    }
}

impl IntoIterator for RadicalTerms {
    type Item = Term;
    type IntoIter = std::vec::IntoIter<Term>;

    fn into_iter(self) -> Self::IntoIter {
        self.into_vec().into_iter()
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
    fn publication_and_mutation_keep_scratch_arrays_owned() {
        let mut empty = RadicalTerms::new();
        empty.retain(|_| panic!("empty storage has no entries"));
        assert_eq!((&mut empty).into_iter().count(), 0);
        assert!(matches!(empty.0, Storage::Owned(_)));
        assert!(empty.retained().0.is_none());
        assert!(matches!(empty.retained().to_terms().0, Storage::Owned(_)));

        let pair = (Rational::new(13), Rational::new(7));
        let owner = RadicalTerms::from(vec![pair.clone()]);
        let copy = owner.clone();
        assert!(matches!(owner.0, Storage::Owned(_)));
        assert!(matches!(copy.0, Storage::Owned(_)));
        assert_ne!(owner.as_ptr(), copy.as_ptr());
        let closed = owner.retained();
        let weak = Arc::downgrade(closed.0.as_ref().unwrap());
        assert_ne!(owner.as_ptr(), closed.as_ptr());
        let mut first = closed.to_terms();
        let second = first.clone();
        assert!(matches!(first.0, Storage::Retained(_)));
        assert!(matches!(second.0, Storage::Retained(_)));
        assert_eq!(closed.as_ptr(), first.as_ptr());
        assert_eq!(first.as_ptr(), second.as_ptr());
        let republished = first.retained();
        assert_eq!(republished.as_ptr(), closed.as_ptr());
        first[0].1 = Rational::new(-3);
        assert!(matches!(first.0, Storage::Owned(_)));
        assert_ne!(first.as_ptr(), closed.as_ptr());
        first.remove(0);
        first.push((Rational::new(2), Rational::new(9)));
        first.retain(|_| false);
        assert!(first.is_empty());
        assert!(matches!(first.0, Storage::Owned(_)));
        assert_eq!(&*closed, [pair.clone()].as_slice());
        assert_eq!(second.as_slice(), [pair.clone()].as_slice());
        assert_eq!(owner.as_slice(), [pair.clone()].as_slice());
        assert_eq!(copy.as_slice(), [pair].as_slice());
        drop(closed);
        drop(republished);
        assert!(weak.upgrade().is_some());
        drop(second);
        assert!(weak.upgrade().is_none());
    }

    #[test]
    fn consuming_owned_and_retained_views_reuses_or_releases_buffers() {
        let pair = (Rational::new(13), Rational::new(7));
        let owned = RadicalTerms::from(vec![pair.clone()]);
        let pointer = owned.as_ptr();
        let iterator = owned.into_iter();
        assert_eq!(iterator.as_slice().as_ptr(), pointer);
        assert_eq!(iterator.collect::<Vec<_>>(), vec![pair.clone()]);

        let closed = RadicalTerms::from(vec![pair.clone()]).retained();
        let weak = Arc::downgrade(closed.0.as_ref().unwrap());
        let view = closed.to_terms();
        let pointer = view.as_ptr();
        drop(closed);
        let iterator = view.into_iter();
        assert_eq!(iterator.as_slice().as_ptr(), pointer);
        assert!(weak.upgrade().is_none());
        assert_eq!(iterator.collect::<Vec<_>>(), vec![pair.clone()]);

        let closed = RadicalTerms::from(vec![pair.clone()]).retained();
        let weak = Arc::downgrade(closed.0.as_ref().unwrap());
        let values = closed.to_terms().into_iter().collect::<Vec<_>>();
        assert_ne!(values.as_ptr(), closed.as_ptr());
        assert_eq!(values, vec![pair]);
        assert_eq!(values.as_slice(), &*closed);
        drop(closed);
        assert!(weak.upgrade().is_none());
    }
}
