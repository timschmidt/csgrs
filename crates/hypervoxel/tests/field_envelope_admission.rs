use hyperreal::Real;
use hypervoxel::{
    AggregateCertainty, CertifiedFieldInterval, CertifiedTensorInterval, CertifiedVectorInterval,
    FieldEnvelopeFacts, HypervoxelError,
};

fn interval(lower: i64, upper: i64) -> CertifiedFieldInterval {
    CertifiedFieldInterval {
        lower: lower.into(),
        upper: upper.into(),
    }
}

fn inverted_error() -> HypervoxelError {
    HypervoxelError::InvalidFieldEnvelope {
        reason: "inverted component interval",
    }
}

#[test]
fn reversed_vector_component_is_not_certified() {
    let vector = CertifiedVectorInterval {
        components: vec![interval(2, 1)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([&vector], []),
        Err(inverted_error())
    );
    let broad = CertifiedVectorInterval {
        components: vec![interval(0, 3)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([&broad, &vector], []),
        Err(inverted_error())
    );
}

#[test]
fn reversed_tensor_component_is_not_certified() {
    let tensor = CertifiedTensorInterval {
        rows: 1,
        cols: 1,
        components: vec![interval(2, 1)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([], [&tensor]),
        Err(inverted_error())
    );
    let broad = CertifiedTensorInterval {
        rows: 1,
        cols: 1,
        components: vec![interval(0, 3)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([], [&broad, &tensor]),
        Err(inverted_error())
    );
}

#[test]
fn missing_tensor_components_are_not_certified() {
    for (rows, cols, count) in [(2, 2, 1), (1, 1, 2), (0, 2, 1), (usize::MAX, 2, 0)] {
        let tensor = CertifiedTensorInterval {
            rows,
            cols,
            components: vec![interval(0, 1); count],
        };
        assert_eq!(
            FieldEnvelopeFacts::from_envelopes([], [&tensor]),
            Err(HypervoxelError::InvalidFieldEnvelope {
                reason: "tensor dimensions do not match component count"
            })
        );
    }
}

#[test]
fn incompatible_shapes_do_not_hide_invalid_components() {
    let valid = CertifiedVectorInterval {
        components: vec![interval(0, 3); 2],
    };
    let invalid = CertifiedVectorInterval {
        components: vec![interval(2, 1)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([&valid, &invalid], []),
        Err(inverted_error())
    );
    let valid = CertifiedTensorInterval {
        rows: 1,
        cols: 1,
        components: vec![interval(0, 3)],
    };
    let invalid = CertifiedTensorInterval {
        rows: 2,
        cols: 1,
        components: vec![interval(0, 3), interval(2, 1)],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([], [&valid, &invalid]),
        Err(inverted_error())
    );
}

#[test]
fn undecided_component_order_remains_explicit() {
    // The existing affine-enclosure regression uses this exact sine identity
    // outside the scalar normal form. Its finite enclosure does not prove order.
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(zero.certified_dyadic_interval(-16).is_some());
    let vector = CertifiedVectorInterval {
        components: vec![CertifiedFieldInterval {
            lower: zero,
            upper: Real::zero(),
        }],
    };
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([&vector], []),
        Err(HypervoxelError::UnknownScalarOrdering {
            field: "vector/tensor component order"
        })
    );
}

#[test]
fn independent_integer_hulls_remain_valid_subsequent_operands() {
    let mut cases = 0;
    for a in -2..=2 {
        for b in a..=2 {
            for c in -2..=2 {
                for d in c..=2 {
                    let left = CertifiedVectorInterval {
                        components: vec![interval(a, b), interval(c, d)],
                    };
                    let right = CertifiedVectorInterval {
                        components: vec![interval(c, d), interval(a, b)],
                    };
                    let expected = interval(a.min(c), b.max(d));
                    let facts = FieldEnvelopeFacts::from_envelopes([&left, &right], []).unwrap();
                    assert!(facts.certified_envelope_ready);
                    assert_eq!(facts.certainty, AggregateCertainty::Certified);
                    assert_eq!(
                        facts.vector_interval.as_ref().unwrap().components,
                        vec![expected.clone(); 2]
                    );
                    let merged = facts.vector_interval.as_ref().unwrap();
                    let next = FieldEnvelopeFacts::from_envelopes([merged, &left], []).unwrap();
                    assert_eq!(next.vector_interval, facts.vector_interval);
                    let tensor = CertifiedTensorInterval {
                        rows: 1,
                        cols: 2,
                        components: merged.components.clone(),
                    };
                    let tensor_facts = FieldEnvelopeFacts::from_envelopes([], [&tensor]).unwrap();
                    assert!(tensor_facts.certified_envelope_ready);
                    assert_eq!(tensor_facts.tensor_interval.as_ref().unwrap(), &tensor);
                    cases += 1;
                }
            }
        }
    }
    assert_eq!(cases, 225);
}

#[test]
fn zero_dimensional_envelopes_and_absent_envelopes_remain_distinct() {
    let absent = FieldEnvelopeFacts::from_envelopes([], []).unwrap();
    assert!(!absent.has_envelopes);
    assert!(!absent.certified_envelope_ready);
    assert_eq!(absent.certainty, AggregateCertainty::Unknown);
    let unit = CertifiedVectorInterval { components: vec![] };
    for (rows, cols) in [(0, 0), (0, usize::MAX), (3, 0)] {
        let tensor = CertifiedTensorInterval {
            rows,
            cols,
            components: vec![],
        };
        let facts = FieldEnvelopeFacts::from_envelopes([&unit], [&tensor]).unwrap();
        assert_eq!(facts.envelope_count, 2);
        assert!(facts.has_envelopes);
        assert!(facts.certified_envelope_ready);
        assert_eq!(facts.certainty, AggregateCertainty::Certified);
    }
}

#[test]
fn source_mutation_requires_new_validation_and_failure_preserves_input() {
    let mut vector = CertifiedVectorInterval {
        components: vec![interval(0, 1)],
    };
    assert!(
        FieldEnvelopeFacts::from_envelopes([&vector], [])
            .unwrap()
            .certified_envelope_ready
    );
    vector.components[0].lower = 2.into();
    let before = vector.clone();
    assert_eq!(
        FieldEnvelopeFacts::from_envelopes([&vector], []),
        Err(inverted_error())
    );
    assert_eq!(vector, before);
    vector.components[0].upper = 3.into();
    assert!(
        FieldEnvelopeFacts::from_envelopes([&vector], [])
            .unwrap()
            .certified_envelope_ready
    );
}
