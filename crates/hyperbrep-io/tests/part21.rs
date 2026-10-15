use hyperbrep_io::part21::{ErrorKind, NumberLimits, ReadLimits, Resource, ValueKind, parse};
use hyperreal::Real;

fn file(records: &str) -> String {
    format!(
        "ISO-10303-21;HEADER;FILE_DESCRIPTION(('Hyper test'),'2;1');FILE_NAME('','',(),(),'','','');FILE_SCHEMA(('TEST'));ENDSEC;DATA;{records}ENDSEC;END-ISO-10303-21;"
    )
}

#[test]
fn exact_numbers_retain_spelling_and_never_round_through_float() {
    let source =
        file("#1=NUMBERS(9007199254740993.0,-1.25E-3,+4.E3,18446744073709551616,1.E400,1.E-400);");
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    let record = exchange.entity(1).unwrap().components().next().unwrap();
    let numbers: Vec<_> = record
        .arguments()
        .map(|v| match v.kind() {
            ValueKind::Real(n) | ValueKind::Integer(n) => n,
            _ => panic!("numeric fixture"),
        })
        .collect();
    assert_eq!(
        numbers.iter().map(|n| n.lexeme()).collect::<Vec<_>>(),
        [
            "9007199254740993.0",
            "-1.25E-3",
            "+4.E3",
            "18446744073709551616",
            "1.E400",
            "1.E-400"
        ]
    );
    let limits = NumberLimits::default();
    assert_eq!(
        numbers[0].to_real(&limits).unwrap(),
        Real::from(9_007_199_254_740_993u64)
    );
    assert_eq!(
        numbers[1].to_real(&limits).unwrap(),
        (Real::from(-1) / Real::from(800)).unwrap()
    );
    assert_eq!(numbers[2].to_real(&limits).unwrap(), Real::from(4000));
    assert_eq!(
        numbers[3].to_real(&limits).unwrap(),
        Real::from(u64::MAX) + Real::one()
    );
    assert_eq!(
        numbers[4].to_real(&limits).unwrap() * numbers[5].to_real(&limits).unwrap(),
        Real::one()
    );
    assert_ne!(numbers[5].to_real(&limits).unwrap(), Real::zero());
}

#[test]
fn scalar_conversion_has_separate_inclusive_digit_and_exponent_limits() {
    let source = file("#1=A(12.50E-2,0.E99999999999999999999999999999999);");
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    let mut values = exchange
        .entity(1)
        .unwrap()
        .components()
        .next()
        .unwrap()
        .arguments();
    let ValueKind::Real(value) = values.next().unwrap().kind() else {
        panic!()
    };
    for (limits, resource) in [
        (
            NumberLimits {
                digits: 3,
                exponent: 2,
            },
            Resource::DecimalDigits,
        ),
        (
            NumberLimits {
                digits: 4,
                exponent: 1,
            },
            Resource::DecimalExponent,
        ),
    ] {
        assert!(
            matches!(value.to_real(&limits).unwrap_err().kind, ErrorKind::Limit { resource: r, .. } if r == resource)
        );
    }
    assert_eq!(
        value
            .to_real(&NumberLimits {
                digits: 4,
                exponent: 2
            })
            .unwrap(),
        (Real::one() / Real::from(8)).unwrap()
    );
    let ValueKind::Real(zero) = values.next().unwrap().kind() else {
        panic!()
    };
    assert!(matches!(
        zero.to_real(&NumberLimits::default()).unwrap_err().kind,
        ErrorKind::Limit {
            resource: Resource::DecimalExponent,
            ..
        }
    ));
    assert!(matches!(
        zero.to_real(&NumberLimits {
            digits: 1,
            exponent: usize::MAX
        })
        .unwrap_err()
        .kind,
        ErrorKind::Limit {
            resource: Resource::DecimalExponent,
            ..
        }
    ));
}

#[test]
fn complex_instances_forward_references_and_unknown_records_are_retained() {
    let source = file("#9=UNKNOWN_VENDOR_ENTITY(#2,#404);#2=(A()B(LENGTH_MEASURE(1.0),(),$,*));");
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    assert_eq!(
        exchange.entities().map(|e| e.id()).collect::<Vec<_>>(),
        [9, 2]
    );
    let second = exchange.entity(2).unwrap();
    assert!(second.is_complex());
    assert_eq!(
        second.components().map(|r| r.name()).collect::<Vec<_>>(),
        ["A", "B"]
    );
    let args: Vec<_> = second.components().nth(1).unwrap().arguments().collect();
    assert_eq!(args[0].kind(), ValueKind::Typed("LENGTH_MEASURE"));
    assert_eq!(args[0].children().count(), 1);
    assert_eq!(args[1].kind(), ValueKind::Aggregate);
    assert_eq!(args[1].children().count(), 0);
    assert_eq!(args[2].kind(), ValueKind::Omitted);
    assert_eq!(args[3].kind(), ValueKind::Derived);
    let missing: Vec<_> = exchange.unresolved_references().collect();
    assert_eq!(missing.len(), 1);
    assert_eq!((missing[0].source, missing[0].target), (Some(9), 404));
    assert_eq!(&source[missing[0].span.clone()], "#404");
    let output = exchange.to_step(10000).unwrap();
    assert!(output.contains("#9=UNKNOWN_VENDOR_ENTITY(#2,#404);"));
    assert!(output.contains("#2=(A()B(LENGTH_MEASURE(1.0),(),$,*));"));
    let replay = parse(output.as_bytes(), &ReadLimits::default()).unwrap();
    assert_eq!(replay.entity(2).unwrap().components().count(), 2);
}

#[test]
fn strings_binary_and_logical_spellings_survive_deterministic_writing() {
    let source = file(
        r#"/* before */ #001 = !CUSTOM('one''two','\X2\03C0\X0\','\X4\0001F600\X0\','π',"0","12F",.T.,.F.,.U.,.UNSPECIFIED.);"#,
    );
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    let record = exchange.entity(1).unwrap().components().next().unwrap();
    assert_eq!(record.name(), "!CUSTOM");
    let kinds: Vec<_> = record.arguments().map(|v| v.kind()).collect();
    assert_eq!(kinds[0], ValueKind::String("'one''two'"));
    assert_eq!(kinds[3], ValueKind::String("'π'"));
    assert_eq!(kinds[4], ValueKind::Binary("\"0\""));
    assert_eq!(kinds[8], ValueKind::Enumeration("U"));
    let written = exchange.to_step(10000).unwrap();
    assert!(written.contains("#001=!CUSTOM("));
    assert!(!written.contains("/* before */"));
    let replay = parse(written.as_bytes(), &ReadLimits::default()).unwrap();
    assert_eq!(
        replay
            .entity(1)
            .unwrap()
            .components()
            .next()
            .unwrap()
            .arguments()
            .map(|v| v.kind())
            .collect::<Vec<_>>(),
        kinds
    );
    assert_eq!(replay.to_step(10000).unwrap(), written);
    assert!(matches!(
        exchange.to_step(written.len() - 1).unwrap_err().kind,
        ErrorKind::Limit {
            resource: Resource::OutputBytes,
            ..
        }
    ));
    assert_eq!(exchange.to_step(written.len()).unwrap(), written);
    // A padding count removes leading zero bits, not arbitrary leading bits.
    // Check every possible first hexadecimal digit at every padding count.
    for padding in 0..=3 {
        for digit in 0..=15 {
            let source = file(&format!("#1=A(\"{padding}{digit:X}A\");"));
            assert_eq!(
                parse(source.as_bytes(), &ReadLimits::default()).is_ok(),
                digit < (16 >> padding),
                "padding {padding}, first digit {digit:X}"
            );
        }
    }
}

#[test]
fn sections_and_header_reference_ownership_are_explicit() {
    let source = "ISO-10303-21;HEADER;H(#8);ENDSEC;DATA('first',(#9));#8=A();ENDSEC;DATA;#9=B(#8);ENDSEC;END-ISO-10303-21;";
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    assert_eq!(exchange.sections().len(), 2);
    let first = exchange.sections().next().unwrap();
    assert_eq!(first.entities().map(|e| e.id()).collect::<Vec<_>>(), [8]);
    assert_eq!(first.arguments().unwrap().count(), 2);
    assert_eq!(
        exchange
            .sections()
            .nth(1)
            .unwrap()
            .arguments()
            .map(|a| a.count()),
        None
    );
    assert_eq!(
        exchange
            .references()
            .map(|r| (r.source, r.target))
            .collect::<Vec<_>>(),
        [(None, 8), (None, 9), (Some(9), 8)]
    );
    assert_eq!(exchange.unresolved_references().count(), 0);
}

#[test]
fn equal_ids_in_different_exchanges_cannot_cross_index_parameters() {
    let a = file("#1=A(1);");
    let b = file("#1=B(2,3);");
    let a = parse(a.as_bytes(), &ReadLimits::default()).unwrap();
    let b = parse(b.as_bytes(), &ReadLimits::default()).unwrap();
    let left = a.entity(1).unwrap().components().next().unwrap();
    let right = b.entity(1).unwrap().components().next().unwrap();
    assert_eq!((left.name(), left.arguments().count()), ("A", 1));
    assert_eq!((right.name(), right.arguments().count()), ("B", 2));
}

#[test]
fn duplicate_ids_and_id_overflow_fail_without_sparse_allocation() {
    for body in ["#1=A();#001=B();", "#1=A();ENDSEC;DATA;#1=B();"] {
        let source = file(body);
        assert!(matches!(
            parse(source.as_bytes(), &ReadLimits::default())
                .unwrap_err()
                .kind,
            ErrorKind::DuplicateEntity(1)
        ));
    }
    let source = file("#18446744073709551615=A(#18446744073709551615);");
    let exchange = parse(source.as_bytes(), &ReadLimits::default()).unwrap();
    assert_eq!(exchange.entities().len(), 1);
    assert_eq!(exchange.unresolved_references().count(), 0);
    assert!(exchange.entity(u64::MAX).is_some());
    for body in ["#0=A();", "#000=A();", "#1=A(#0);", "#1=A(#000);"] {
        let source = file(body);
        assert!(matches!(
            parse(source.as_bytes(), &ReadLimits::default())
                .unwrap_err()
                .kind,
            ErrorKind::Syntax(_)
        ));
    }
    assert!(
        parse(
            file("#18446744073709551616=A();").as_bytes(),
            &ReadLimits::default()
        )
        .is_err()
    );
}

#[test]
fn every_resource_accepts_its_boundary_and_rejects_a_smaller_allowance() {
    // Independently enumerated lexical stream; eight flat value nodes:
    // H's list, A's list, inner list + two integers, typed wrapper + integer, ref.
    let tokens = [
        "ISO-10303-21",
        ";",
        "HEADER",
        ";",
        "H",
        "(",
        ")",
        ";",
        "ENDSEC",
        ";",
        "DATA",
        ";",
        "#1",
        "=",
        "A",
        "(",
        "(",
        "1",
        ",",
        "2",
        ")",
        ",",
        "T",
        "(",
        "3",
        ")",
        ",",
        "#1",
        ")",
        ";",
        "ENDSEC",
        ";",
        "END-ISO-10303-21",
        ";",
    ];
    let source = tokens.join(" ");
    let exact = ReadLimits {
        input_bytes: source.len(),
        tokens: tokens.len(),
        token_bytes: 16,
        nesting_depth: 2,
        values: 8,
        records: 2,
        entities: 1,
        sections: 1,
    };
    let expected = parse(source.as_bytes(), &exact)
        .unwrap()
        .to_step(10000)
        .unwrap();
    for resource in [
        Resource::InputBytes,
        Resource::Tokens,
        Resource::TokenBytes,
        Resource::NestingDepth,
        Resource::Values,
        Resource::Records,
        Resource::Entities,
        Resource::Sections,
    ] {
        let mut limits = exact;
        let count = match resource {
            Resource::InputBytes => &mut limits.input_bytes,
            Resource::Tokens => &mut limits.tokens,
            Resource::TokenBytes => &mut limits.token_bytes,
            Resource::NestingDepth => &mut limits.nesting_depth,
            Resource::Values => &mut limits.values,
            Resource::Records => &mut limits.records,
            Resource::Entities => &mut limits.entities,
            Resource::Sections => &mut limits.sections,
            _ => unreachable!(),
        };
        *count -= 1;
        let error = parse(source.as_bytes(), &limits).unwrap_err();
        assert!(
            matches!(error.kind, ErrorKind::Limit { resource: r, .. } if r == resource),
            "{error:?}"
        );
        assert_eq!(
            parse(source.as_bytes(), &exact)
                .unwrap()
                .to_step(10000)
                .unwrap(),
            expected
        );
    }
}

#[test]
fn deep_parse_write_clone_and_drop_use_no_recursive_tree() {
    std::thread::Builder::new()
        .stack_size(64 * 1024)
        .spawn(|| {
            let source = file(&format!(
                "#1=A({}1{});",
                "(".repeat(20_000),
                ")".repeat(20_000)
            ));
            assert!(matches!(
                parse(source.as_bytes(), &ReadLimits::default())
                    .unwrap_err()
                    .kind,
                ErrorKind::Limit {
                    resource: Resource::NestingDepth,
                    ..
                }
            ));
            let limits = ReadLimits {
                nesting_depth: 20_001,
                ..ReadLimits::default()
            };
            let exchange = parse(source.as_bytes(), &limits).unwrap();
            let written = exchange.to_step(100000).unwrap();
            let replay = parse(written.as_bytes(), &limits).unwrap();
            assert_eq!(replay.to_step(100000).unwrap(), written);
            drop(exchange.clone());
            drop(exchange);
            drop(replay);
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn malformed_delimiters_numbers_and_typed_values_are_rejected() {
    for body in [
        "#1=A(,);",
        "#1=A(1,);",
        "#1=A(T());",
        "#1=A(T(1,2));",
        "#1=A(1 2);",
        "#1=(A(),B());",
        "#1=();",
        "#1=A(.5);",
        "#1=A(1E3);",
        "#1=A(1.E);",
        "#1=A(+);",
        "#1=A('unterminated);",
        "#1=A(\"4F\");",
        "#1=A(\"3\");",
        "#1=A(.);",
        "#1=ISO-10303-21();",
        "#1=END-ISO-10303-21();",
        "#1=A(ISO-10303-21(1));",
        "#1=A(END-ISO-10303-21(1));",
        "/* unterminated",
    ] {
        let source = file(body);
        let error = parse(source.as_bytes(), &ReadLimits::default()).unwrap_err();
        assert!(error.offset <= source.len());
    }
}

#[test]
fn truncation_trailing_data_and_invalid_utf8_never_publish_partial_syntax() {
    let source = file("#1=(A('π''quote')B((1.,T(#2)),()));#2=C();");
    for length in 0..source.len() {
        assert!(
            parse(&source.as_bytes()[..length], &ReadLimits::default()).is_err(),
            "prefix {length}"
        );
    }
    for suffix in [";", "junk", "#3=A();"] {
        assert!(parse((source.clone() + suffix).as_bytes(), &ReadLimits::default()).is_err());
    }
    assert!(matches!(
        parse(&[255], &ReadLimits::default()).unwrap_err().kind,
        ErrorKind::InvalidUtf8
    ));
    assert!(matches!(
        parse(
            &[255],
            &ReadLimits {
                input_bytes: 0,
                ..ReadLimits::default()
            }
        )
        .unwrap_err()
        .kind,
        ErrorKind::Limit {
            resource: Resource::InputBytes,
            ..
        }
    ));
}

#[test]
fn unsupported_sections_and_references_are_not_silently_discarded() {
    for section in ["ANCHOR", "REFERENCE", "SIGNATURE"] {
        let source = format!("ISO-10303-21;HEADER;H();ENDSEC;{section};");
        assert!(matches!(
            parse(source.as_bytes(), &ReadLimits::default())
                .unwrap_err()
                .kind,
            ErrorKind::Unsupported(_)
        ));
    }
    for body in ["#1=A(@1);", "#1=A(#NAME);", "#1=&SCOPE A();"] {
        assert!(matches!(
            parse(file(body).as_bytes(), &ReadLimits::default())
                .unwrap_err()
                .kind,
            ErrorKind::Unsupported(_)
        ));
    }
    let signed = file("#1=A();") + "SIGNATURE;'digest';ENDSEC;";
    assert!(matches!(
        parse(signed.as_bytes(), &ReadLimits::default())
            .unwrap_err()
            .kind,
        ErrorKind::Unsupported(_)
    ));
}
