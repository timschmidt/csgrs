/// One curated report row, identifying estimates without proving workload equivalence.
///
/// The table renderer combines group/id with fixed backend labels and legacy key
/// aliases. These static references can point to literals or intentionally leaked
/// generated strings; copying a row does not copy or reclaim those strings.
#[derive(Clone, Copy)]
struct BenchRow {
    /// Display label inserted verbatim in a Markdown code span, without escaping.
    title: &'static str,
    /// Criterion group prefix used for estimate lookup, not a validated suite name.
    group: &'static str,
    /// Backend-independent operation suffix interpreted by the lookup heuristics.
    id: &'static str,
}

/// Builds thirty-six trig/inverse report rows from the shared case names, in order.
///
/// Expands six trig cases into sin/cos, four unit cases into asin/acos/atanh,
/// four real cases into atan/asinh, and four acosh cases into acosh. Titles use
/// operation then case; lookup IDs use case_operation. Case values are not checked
/// or evaluated here. Every call leaks two allocated strings per row to satisfy
/// BenchRow's static references, so dropping the returned vector cannot reclaim them.
fn scalar_trig_rows() -> Vec<BenchRow> {
    let mut rows = Vec::new();
    for case in trig_cases() {
        for op in ["sin", "cos"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{op} {}", case.name).into_boxed_str()),
                group: "scalar_trig",
                id: Box::leak(format!("{}_{}", case.name, op).into_boxed_str()),
            });
        }
    }
    for case in inverse_unit_cases() {
        for op in ["asin", "acos", "atanh"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{op} {}", case.name).into_boxed_str()),
                group: "scalar_trig",
                id: Box::leak(format!("{}_{}", case.name, op).into_boxed_str()),
            });
        }
    }
    for case in inverse_real_cases() {
        for op in ["atan", "asinh"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{op} {}", case.name).into_boxed_str()),
                group: "scalar_trig",
                id: Box::leak(format!("{}_{}", case.name, op).into_boxed_str()),
            });
        }
    }
    for case in inverse_acosh_cases() {
        rows.push(BenchRow {
            title: Box::leak(format!("acosh {}", case.name).into_boxed_str()),
            group: "scalar_trig",
            id: Box::leak(format!("{}_acosh", case.name).into_boxed_str()),
        });
    }
    rows
}

/// Builds twelve operation/case rows for the supplied group without validating it.
///
/// Orders half, negative_tiny, positive_20 and negative_20, each with sinh/cosh/tanh.
/// The renderer uses this for both construction and explicit-f64 groups; identical
/// IDs do not imply identical output contracts. Two strings per row are leaked on
/// each call, independently of whether any corresponding estimate exists.
fn scalar_hyperbolic_case_rows(group: &'static str) -> Vec<BenchRow> {
    let mut rows = Vec::new();
    for case in ["half", "negative_tiny", "positive_20", "negative_20"] {
        for operation in ["sinh", "cosh", "tanh"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{operation} {case}").into_boxed_str()),
                group,
                id: Box::leak(format!("{operation}/{case}").into_boxed_str()),
            });
        }
    }
    rows
}

/// Builds the sixty curated borrowed-operation rows in benchmark registration order.
///
/// Includes twenty scalar mixes, fourteen vector and eighteen matrix operations,
/// two matrix-vector products and six complex rows. Names match the registration
/// schedule but do not certify equal native/external ownership or timing boundaries.
/// The first fifty-two rows each leak separate title/id strings; the final eight
/// reuse literals. Repeated rendering retains those allocations for process lifetime.
fn borrowed_op_rows() -> Vec<BenchRow> {
    let mut rows = Vec::new();
    for op in ["add", "sub", "mul", "div"] {
        for mode in [
            "owned_ref",
            "ref_owned",
            "refs",
            "owned_ref_with_clone",
            "ref_owned_with_clone",
        ] {
            rows.push(BenchRow {
                title: Box::leak(format!("scalar {op} {mode}").into_boxed_str()),
                group: "borrowed_ops",
                id: Box::leak(format!("scalar {op} {mode}").into_boxed_str()),
            });
        }
    }
    for dim in ["vec3", "vec4"] {
        for op in ["add refs", "sub refs", "neg ref"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{dim} {op}").into_boxed_str()),
                group: "borrowed_ops",
                id: Box::leak(format!("{dim} {op}").into_boxed_str()),
            });
        }
        for op in [
            "add_scalar_ref",
            "sub_scalar_ref",
            "mul_scalar_ref",
            "div_scalar_ref",
        ] {
            rows.push(BenchRow {
                title: Box::leak(format!("{dim} {op}").into_boxed_str()),
                group: "borrowed_ops",
                id: Box::leak(format!("{dim} {op}").into_boxed_str()),
            });
        }
    }
    for dim in ["mat3", "mat4"] {
        for op in ["add refs", "sub refs", "mul refs", "div refs", "neg ref"] {
            rows.push(BenchRow {
                title: Box::leak(format!("{dim} {op}").into_boxed_str()),
                group: "borrowed_ops",
                id: Box::leak(format!("{dim} {op}").into_boxed_str()),
            });
        }
        for op in [
            "add_scalar_ref",
            "sub_scalar_ref",
            "mul_scalar_ref",
            "div_scalar_ref",
        ] {
            rows.push(BenchRow {
                title: Box::leak(format!("{dim} {op}").into_boxed_str()),
                group: "borrowed_ops",
                id: Box::leak(format!("{dim} {op}").into_boxed_str()),
            });
        }
    }
    for op in [
        "mat3 transform_vec refs",
        "mat4 transform_vec refs",
        "complex add refs",
        "complex sub refs",
        "complex mul refs",
        "complex div refs",
        "complex neg ref",
        "complex div_real_ref",
    ] {
        rows.push(BenchRow {
            title: op,
            group: "borrowed_ops",
            id: op,
        });
    }
    rows
}

/// Forty-one selected scalar rows, including exp_128 from a separate group.
///
/// Includes external-only zero_status_abort but omits powi_negative_one and the
/// dedicated sqrt-case group. This is a presentation catalog, not complete native
/// API coverage or proof that every backend supplies every cell.
const SCALAR_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "zero",
        group: "scalar_ops",
        id: "zero",
    },
    BenchRow {
        title: "one",
        group: "scalar_ops",
        id: "one",
    },
    BenchRow {
        title: "e",
        group: "scalar_ops",
        id: "e",
    },
    BenchRow {
        title: "pi",
        group: "scalar_ops",
        id: "pi",
    },
    BenchRow {
        title: "tau",
        group: "scalar_ops",
        id: "tau",
    },
    BenchRow {
        title: "add",
        group: "scalar_ops",
        id: "add",
    },
    BenchRow {
        title: "sub",
        group: "scalar_ops",
        id: "sub",
    },
    BenchRow {
        title: "neg",
        group: "scalar_ops",
        id: "neg",
    },
    BenchRow {
        title: "mul",
        group: "scalar_ops",
        id: "mul",
    },
    BenchRow {
        title: "div",
        group: "scalar_ops",
        id: "div",
    },
    BenchRow {
        title: "reciprocal",
        group: "scalar_ops",
        id: "reciprocal",
    },
    BenchRow {
        title: "reciprocal checked",
        group: "scalar_ops",
        id: "reciprocal_checked",
    },
    BenchRow {
        title: "reciprocal checked abort",
        group: "scalar_ops",
        id: "reciprocal_checked_abort",
    },
    BenchRow {
        title: "pow",
        group: "scalar_ops",
        id: "pow",
    },
    BenchRow {
        title: "powi",
        group: "scalar_ops",
        id: "powi",
    },
    BenchRow {
        title: "exp",
        group: "scalar_ops",
        id: "exp",
    },
    BenchRow {
        title: "exp 128",
        group: "scalar_large_integer_exp",
        id: "exp_128",
    },
    BenchRow {
        title: "ln",
        group: "scalar_ops",
        id: "ln",
    },
    BenchRow {
        title: "log10",
        group: "scalar_ops",
        id: "log10",
    },
    BenchRow {
        title: "log10 abort",
        group: "scalar_ops",
        id: "log10_abort",
    },
    BenchRow {
        title: "sqrt",
        group: "scalar_ops",
        id: "sqrt",
    },
    BenchRow {
        title: "sin",
        group: "scalar_ops",
        id: "sin",
    },
    BenchRow {
        title: "cos",
        group: "scalar_ops",
        id: "cos",
    },
    BenchRow {
        title: "tan",
        group: "scalar_ops",
        id: "tan",
    },
    BenchRow {
        title: "sinh",
        group: "scalar_ops",
        id: "sinh",
    },
    BenchRow {
        title: "cosh",
        group: "scalar_ops",
        id: "cosh",
    },
    BenchRow {
        title: "tanh",
        group: "scalar_ops",
        id: "tanh",
    },
    BenchRow {
        title: "asin",
        group: "scalar_ops",
        id: "asin",
    },
    BenchRow {
        title: "asin abort",
        group: "scalar_ops",
        id: "asin_abort",
    },
    BenchRow {
        title: "acos",
        group: "scalar_ops",
        id: "acos",
    },
    BenchRow {
        title: "acos abort",
        group: "scalar_ops",
        id: "acos_abort",
    },
    BenchRow {
        title: "atan",
        group: "scalar_ops",
        id: "atan",
    },
    BenchRow {
        title: "atan abort",
        group: "scalar_ops",
        id: "atan_abort",
    },
    BenchRow {
        title: "asinh",
        group: "scalar_ops",
        id: "asinh",
    },
    BenchRow {
        title: "asinh abort",
        group: "scalar_ops",
        id: "asinh_abort",
    },
    BenchRow {
        title: "acosh",
        group: "scalar_ops",
        id: "acosh",
    },
    BenchRow {
        title: "acosh abort",
        group: "scalar_ops",
        id: "acosh_abort",
    },
    BenchRow {
        title: "atanh",
        group: "scalar_ops",
        id: "atanh",
    },
    BenchRow {
        title: "atanh abort",
        group: "scalar_ops",
        id: "atanh_abort",
    },
    BenchRow {
        title: "zero status",
        group: "scalar_ops",
        id: "zero_status",
    },
    BenchRow {
        title: "zero status abort",
        group: "scalar_ops",
        id: "zero_status_abort",
    },
];

/// Nineteen selected complex rows, omitting both negative-one power registrations.
///
/// Checked labels are retained as lookup names without asserting equal backend guards.
const COMPLEX_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "zero",
        group: "complex_ops",
        id: "zero",
    },
    BenchRow {
        title: "one",
        group: "complex_ops",
        id: "one",
    },
    BenchRow {
        title: "i",
        group: "complex_ops",
        id: "i",
    },
    BenchRow {
        title: "free i",
        group: "complex_ops",
        id: "free_i",
    },
    BenchRow {
        title: "conjugate",
        group: "complex_ops",
        id: "conjugate",
    },
    BenchRow {
        title: "norm squared",
        group: "complex_ops",
        id: "norm_squared",
    },
    BenchRow {
        title: "reciprocal",
        group: "complex_ops",
        id: "reciprocal",
    },
    BenchRow {
        title: "reciprocal checked",
        group: "complex_ops",
        id: "reciprocal_checked",
    },
    BenchRow {
        title: "powi",
        group: "complex_ops",
        id: "powi",
    },
    BenchRow {
        title: "powi checked",
        group: "complex_ops",
        id: "powi_checked",
    },
    BenchRow {
        title: "div checked",
        group: "complex_ops",
        id: "div_checked",
    },
    BenchRow {
        title: "div real checked",
        group: "complex_ops",
        id: "div_real_checked",
    },
    BenchRow {
        title: "from scalar",
        group: "complex_ops",
        id: "from_scalar",
    },
    BenchRow {
        title: "add",
        group: "complex_ops",
        id: "add",
    },
    BenchRow {
        title: "sub",
        group: "complex_ops",
        id: "sub",
    },
    BenchRow {
        title: "neg",
        group: "complex_ops",
        id: "neg",
    },
    BenchRow {
        title: "mul",
        group: "complex_ops",
        id: "mul",
    },
    BenchRow {
        title: "div",
        group: "complex_ops",
        id: "div",
    },
    BenchRow {
        title: "div real",
        group: "complex_ops",
        id: "div_real",
    },
];

/// Single varying-input complex multiplication row; the label is not a cold-cache proof.
const COMPLEX_MUL_COLD_ROWS: &[BenchRow] = &[BenchRow {
    title: "varying exact inputs",
    group: "complex_mul_cold",
    id: "varying",
}];

/// Three baseline Vec3 comparison rows from the separate vectors group.
const VECTOR_COMPARISON_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "vec3 dot",
        group: "vectors",
        id: "vec3 dot",
    },
    BenchRow {
        title: "vec3 magnitude",
        group: "vectors",
        id: "vec3 magnitude",
    },
    BenchRow {
        title: "vec3 normalize",
        group: "vectors",
        id: "vec3 normalize",
    },
];

/// Twenty-five vector API rows, omitting the six native sparse/active-dot rows.
///
/// Keeps fifteen Vec3 and ten Vec4 names in presentation order; checked/abort
/// names do not establish matching output types, guards or signals across engines.
const VECTOR_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "vec3 new",
        group: "vector_ops",
        id: "vec3 new",
    },
    BenchRow {
        title: "vec3 zero",
        group: "vector_ops",
        id: "vec3 zero",
    },
    BenchRow {
        title: "vec3 dot abort",
        group: "vector_ops",
        id: "vec3 dot_abort",
    },
    BenchRow {
        title: "vec3 magnitude abort",
        group: "vector_ops",
        id: "vec3 magnitude_abort",
    },
    BenchRow {
        title: "vec3 normalize checked",
        group: "vector_ops",
        id: "vec3 normalize_checked",
    },
    BenchRow {
        title: "vec3 normalize checked abort",
        group: "vector_ops",
        id: "vec3 normalize_checked_abort",
    },
    BenchRow {
        title: "vec3 div scalar checked",
        group: "vector_ops",
        id: "vec3 div_scalar_checked",
    },
    BenchRow {
        title: "vec3 div scalar checked abort",
        group: "vector_ops",
        id: "vec3 div_scalar_checked_abort",
    },
    BenchRow {
        title: "vec3 add",
        group: "vector_ops",
        id: "vec3 add",
    },
    BenchRow {
        title: "vec3 add scalar",
        group: "vector_ops",
        id: "vec3 add_scalar",
    },
    BenchRow {
        title: "vec3 sub",
        group: "vector_ops",
        id: "vec3 sub",
    },
    BenchRow {
        title: "vec3 sub scalar",
        group: "vector_ops",
        id: "vec3 sub_scalar",
    },
    BenchRow {
        title: "vec3 neg",
        group: "vector_ops",
        id: "vec3 neg",
    },
    BenchRow {
        title: "vec3 mul scalar",
        group: "vector_ops",
        id: "vec3 mul_scalar",
    },
    BenchRow {
        title: "vec3 div scalar",
        group: "vector_ops",
        id: "vec3 div_scalar",
    },
    BenchRow {
        title: "vec4 dot",
        group: "vector_ops",
        id: "vec4 dot",
    },
    BenchRow {
        title: "vec4 magnitude",
        group: "vector_ops",
        id: "vec4 magnitude",
    },
    BenchRow {
        title: "vec4 normalize",
        group: "vector_ops",
        id: "vec4 normalize",
    },
    BenchRow {
        title: "vec4 add",
        group: "vector_ops",
        id: "vec4 add",
    },
    BenchRow {
        title: "vec4 add scalar",
        group: "vector_ops",
        id: "vec4 add_scalar",
    },
    BenchRow {
        title: "vec4 sub",
        group: "vector_ops",
        id: "vec4 sub",
    },
    BenchRow {
        title: "vec4 sub scalar",
        group: "vector_ops",
        id: "vec4 sub_scalar",
    },
    BenchRow {
        title: "vec4 neg",
        group: "vector_ops",
        id: "vec4 neg",
    },
    BenchRow {
        title: "vec4 mul scalar",
        group: "vector_ops",
        id: "vec4 mul_scalar",
    },
    BenchRow {
        title: "vec4 div scalar",
        group: "vector_ops",
        id: "vec4 div_scalar",
    },
];

/// Eight baseline matrix comparisons: determinant, inverse, product and transform per size.
///
/// Looks in matrix3/matrix4; specialized transform/fact and targeted-form rows are
/// not inferred from this catalog or from matching backend labels.
const MATRIX_COMPARISON_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "mat3 determinant",
        group: "matrix3",
        id: "mat3 determinant",
    },
    BenchRow {
        title: "mat3 inverse",
        group: "matrix3",
        id: "mat3 inverse",
    },
    BenchRow {
        title: "mat3 mul mat3",
        group: "matrix3",
        id: "mat3 mul mat3",
    },
    BenchRow {
        title: "mat3 transform vec3",
        group: "matrix3",
        id: "mat3 transform vec3",
    },
    BenchRow {
        title: "mat4 determinant",
        group: "matrix4",
        id: "mat4 determinant",
    },
    BenchRow {
        title: "mat4 inverse",
        group: "matrix4",
        id: "mat4 inverse",
    },
    BenchRow {
        title: "mat4 mul mat4",
        group: "matrix4",
        id: "mat4 mul mat4",
    },
    BenchRow {
        title: "mat4 transform vec4",
        group: "matrix4",
        id: "mat4 transform vec4",
    },
];

/// Forty curated matrix API rows: twenty-four Mat3 and sixteen Mat4 entries.
///
/// Deliberately separate from baseline matrix comparisons; this static list is
/// not an inventory of all current registrations or a qualification of their results.
const MATRIX_OP_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "mat3 new",
        group: "matrix_ops",
        id: "mat3 new",
    },
    BenchRow {
        title: "mat3 zero",
        group: "matrix_ops",
        id: "mat3 zero",
    },
    BenchRow {
        title: "mat3 identity",
        group: "matrix_ops",
        id: "mat3 identity",
    },
    BenchRow {
        title: "mat3 transpose",
        group: "matrix_ops",
        id: "mat3 transpose",
    },
    BenchRow {
        title: "mat3 reciprocal",
        group: "matrix_ops",
        id: "mat3 reciprocal",
    },
    BenchRow {
        title: "mat3 reciprocal checked",
        group: "matrix_ops",
        id: "mat3 reciprocal_checked",
    },
    BenchRow {
        title: "mat3 inverse checked",
        group: "matrix_ops",
        id: "mat3 inverse_checked",
    },
    BenchRow {
        title: "mat3 inverse checked abort",
        group: "matrix_ops",
        id: "mat3 inverse_checked_abort",
    },
    BenchRow {
        title: "mat3 powi",
        group: "matrix_ops",
        id: "mat3 powi",
    },
    BenchRow {
        title: "mat3 powi checked",
        group: "matrix_ops",
        id: "mat3 powi_checked",
    },
    BenchRow {
        title: "mat3 powi checked abort",
        group: "matrix_ops",
        id: "mat3 powi_checked_abort",
    },
    BenchRow {
        title: "mat3 div scalar checked",
        group: "matrix_ops",
        id: "mat3 div_scalar_checked",
    },
    BenchRow {
        title: "mat3 div scalar checked abort",
        group: "matrix_ops",
        id: "mat3 div_scalar_checked_abort",
    },
    BenchRow {
        title: "mat3 div matrix checked",
        group: "matrix_ops",
        id: "mat3 div_matrix_checked",
    },
    BenchRow {
        title: "mat3 div matrix checked abort",
        group: "matrix_ops",
        id: "mat3 div_matrix_checked_abort",
    },
    BenchRow {
        title: "mat3 add",
        group: "matrix_ops",
        id: "mat3 add",
    },
    BenchRow {
        title: "mat3 add scalar",
        group: "matrix_ops",
        id: "mat3 add_scalar",
    },
    BenchRow {
        title: "mat3 sub",
        group: "matrix_ops",
        id: "mat3 sub",
    },
    BenchRow {
        title: "mat3 sub scalar",
        group: "matrix_ops",
        id: "mat3 sub_scalar",
    },
    BenchRow {
        title: "mat3 neg",
        group: "matrix_ops",
        id: "mat3 neg",
    },
    BenchRow {
        title: "mat3 mul scalar",
        group: "matrix_ops",
        id: "mat3 mul_scalar",
    },
    BenchRow {
        title: "mat3 div scalar",
        group: "matrix_ops",
        id: "mat3 div_scalar",
    },
    BenchRow {
        title: "mat3 div matrix",
        group: "matrix_ops",
        id: "mat3 div_matrix",
    },
    BenchRow {
        title: "mat3 bitxor",
        group: "matrix_ops",
        id: "mat3 bitxor",
    },
    BenchRow {
        title: "mat4 zero",
        group: "matrix_ops",
        id: "mat4 zero",
    },
    BenchRow {
        title: "mat4 identity",
        group: "matrix_ops",
        id: "mat4 identity",
    },
    BenchRow {
        title: "mat4 transpose",
        group: "matrix_ops",
        id: "mat4 transpose",
    },
    BenchRow {
        title: "mat4 reciprocal",
        group: "matrix_ops",
        id: "mat4 reciprocal",
    },
    BenchRow {
        title: "mat4 reciprocal checked",
        group: "matrix_ops",
        id: "mat4 reciprocal_checked",
    },
    BenchRow {
        title: "mat4 powi",
        group: "matrix_ops",
        id: "mat4 powi",
    },
    BenchRow {
        title: "mat4 powi checked",
        group: "matrix_ops",
        id: "mat4 powi_checked",
    },
    BenchRow {
        title: "mat4 add",
        group: "matrix_ops",
        id: "mat4 add",
    },
    BenchRow {
        title: "mat4 add scalar",
        group: "matrix_ops",
        id: "mat4 add_scalar",
    },
    BenchRow {
        title: "mat4 sub",
        group: "matrix_ops",
        id: "mat4 sub",
    },
    BenchRow {
        title: "mat4 sub scalar",
        group: "matrix_ops",
        id: "mat4 sub_scalar",
    },
    BenchRow {
        title: "mat4 neg",
        group: "matrix_ops",
        id: "mat4 neg",
    },
    BenchRow {
        title: "mat4 mul scalar",
        group: "matrix_ops",
        id: "mat4 mul_scalar",
    },
    BenchRow {
        title: "mat4 div scalar",
        group: "matrix_ops",
        id: "mat4 div_scalar",
    },
    BenchRow {
        title: "mat4 div matrix",
        group: "matrix_ops",
        id: "mat4 div_matrix",
    },
    BenchRow {
        title: "mat4 bitxor",
        group: "matrix_ops",
        id: "mat4 bitxor",
    },
];

/// Forms group/variant_id after replacing only forward slashes in id with underscores.
///
/// Group/variant text, spaces and existing underscores are unchanged. The mapping
/// is lossy (a/b and a_b collide) and is a lookup convention, not path sanitization.
fn estimate_key(group: &str, variant: &str, id: &str) -> String {
    format!("{group}/{}_{}", variant, id.replace('/', "_"))
}

/// Returns the first stored value found by four ordered historical key conventions.
///
/// Tries slash-flattened id, literal id, space-to-underscore id, then a nested
/// group/variant/id with every id underscore changed to a slash. Replacements are
/// independent, not cumulative; duplicate candidates are allowed. No ambiguity,
/// provenance, freshness, unit, sign or finiteness validation accompanies a match.
fn estimate_value(
    estimates: &BTreeMap<String, f64>,
    group: &str,
    variant: &str,
    id: &str,
) -> Option<f64> {
    [
        estimate_key(group, variant, id),
        format!("{group}/{variant}_{id}"),
        format!("{group}/{variant}_{}", id.replace(' ', "_")),
        format!("{group}/{variant}/{}", id.replace('_', "/")),
    ]
    .into_iter()
    .find_map(|key| estimates.get(&key).copied())
}

/// Chooses the stored-estimate root from environment, executable ancestry or cwd.
///
/// CRITERION_HOME wins verbatim, even if empty or nonexistent. Otherwise
/// CARGO_TARGET_DIR is joined with criterion without existence checks. With neither
/// set, uses the nearest executable ancestor containing a criterion directory;
/// inability to obtain the executable or find such an ancestor falls back to
/// relative target/criterion. Relative environment paths use the process cwd in
/// later I/O. No canonicalization, crate-root anchoring or directory creation occurs.
fn criterion_dir() -> std::path::PathBuf {
    if let Some(home) = std::env::var_os("CRITERION_HOME") {
        return std::path::PathBuf::from(home);
    }
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(path) => std::path::PathBuf::from(path).join("criterion"),
        None => std::env::current_exe()
            .ok()
            .and_then(|exe| {
                exe.ancestors()
                    .find(|dir| dir.join("criterion").is_dir())
                    .map(Path::to_path_buf)
            })
            .unwrap_or_else(|| std::path::PathBuf::from("target"))
            .join("criterion"),
    }
}

/// Collects stored median point estimates from recursive new/estimates.json files.
///
/// Uses criterion_dir and keys each value by its benchmark directory relative to
/// that root, with lossy Unicode components joined by forward slashes. Does not read
/// benchmark metadata, confidence intervals, sample counts or run timestamps.
/// Unreadable directories/files, bad JSON and missing/non-numeric median estimates
/// are silently skipped; an empty map cannot distinguish absent data from errors.
/// Numeric values are accepted without positivity or unit checks. Key collisions
/// overwrite in filesystem traversal order; the resulting map itself is sorted.
///
/// Traversal follows directory symlinks with no cycle detection, depth/work limit
/// or cancellation. Files are read whole and retained entries are unbounded here.
fn read_estimates() -> BTreeMap<String, f64> {
    /// Recurses from path, inserting readable numeric median estimates under root-relative keys.
    ///
    /// Directory/entry/read/parse failures are ignored. is_dir follows symlinks;
    /// there is no visited-set or recursion guard. Only files named estimates.json
    /// immediately inside a directory named new qualify, and the containing bench
    /// directory must strip root as a prefix. Lossy-key collisions keep the last
    /// visited value, with no stable traversal-order guarantee or finite/sign check.
    fn visit(root: &Path, path: &Path, out: &mut BTreeMap<String, f64>) {
        let Ok(entries) = fs::read_dir(path) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                visit(root, &path, out);
                continue;
            }
            if path.file_name().and_then(|name| name.to_str()) != Some("estimates.json")
                || path
                    .parent()
                    .and_then(|parent| parent.file_name())
                    .and_then(|name| name.to_str())
                    != Some("new")
            {
                continue;
            }
            let Some(bench_path) = path.parent().and_then(Path::parent) else {
                continue;
            };
            let Ok(relative) = bench_path.strip_prefix(root) else {
                continue;
            };
            let key = relative
                .components()
                .map(|component| component.as_os_str().to_string_lossy())
                .collect::<Vec<_>>()
                .join("/");
            let Ok(json) = fs::read_to_string(&path) else {
                continue;
            };
            let Ok(value) = serde_json::from_str::<serde_json::Value>(&json) else {
                continue;
            };
            let Some(ns) = value["median"]["point_estimate"].as_f64() else {
                continue;
            };
            out.insert(key, ns);
        }
    }

    let mut estimates = BTreeMap::new();
    let criterion = criterion_dir();
    visit(&criterion, &criterion, &mut estimates);
    estimates
}

/// Formats an optional assumed nanosecond estimate with two fractional digits.
///
/// None becomes a dash. Values below 1000 use ns, below one million use us, and
/// everything else uses ms; there is no seconds branch. Thresholds precede rounding,
/// and negative/nonfinite values are not rejected or clamped.
fn format_duration(ns: Option<f64>) -> String {
    let Some(ns) = ns else {
        return "-".to_string();
    };
    if ns < 1_000.0 {
        format!("{ns:.2} ns")
    } else if ns < 1_000_000.0 {
        format!("{:.2} us", ns / 1_000.0)
    } else {
        format!("{:.2} ms", ns / 1_000_000.0)
    }
}

/// Formats numerator/denominator to two fractional digits followed by x.
///
/// Missing operands or a denominator equal to either signed zero produce a dash.
/// Negative and nonfinite inputs otherwise use ordinary f64 arithmetic/formatting.
/// For report medians, ratios above one mean the numerator took longer; no confidence
/// interval or statistical significance accompanies this point-estimate ratio.
fn format_ratio(numerator: Option<f64>, denominator: Option<f64>) -> String {
    match (numerator, denominator) {
        (Some(numerator), Some(denominator)) if denominator != 0.0 => {
            format!("{:.2}x", numerator / denominator)
        }
        _ => "-".to_string(),
    }
}

/// Renders rows in supplied order into a five-backend Markdown timing table.
///
/// Native columns prefer hyperreal/hyperreal-rational keys, falling back to legacy
/// realistic/realistic-rational only when the preferred lookup finds no value.
/// External columns use numerica128, gmp_mpfr128 and symbolica. The last three
/// columns divide the native dyadic median by each external median; ratios above
/// one indicate a larger native estimate. Missing values display as dashes and values use
/// the duration/ratio formatters without statistical or workload validation.
/// Titles are inserted unescaped in code spans. This performs no I/O or benchmarks;
/// matching keys/headers alone do not certify input fidelity or equivalent work.
fn render_table(estimates: &BTreeMap<String, f64>, rows: &[BenchRow]) -> String {
    let mut out = String::new();
    out.push_str("| Benchmark | Hyperreal exact dyadic input | Hyperreal explicit exact rational | numerica128 | GMP/MPFR 128 | symbolica | Exact dyadic / numerica128 | Exact dyadic / GMP | Exact dyadic / symbolica |\n");
    out.push_str("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
    for row in rows {
        let hyperreal = estimate_value(estimates, row.group, "hyperreal", row.id)
            .or_else(|| estimate_value(estimates, row.group, "realistic", row.id));
        let rational = estimate_value(estimates, row.group, "hyperreal-rational", row.id)
            .or_else(|| estimate_value(estimates, row.group, "realistic-rational", row.id));
        let numerica = estimate_value(estimates, row.group, "numerica128", row.id);
        let gmp = estimate_value(estimates, row.group, "gmp_mpfr128", row.id);
        let symbolica = estimate_value(estimates, row.group, "symbolica", row.id);
        out.push_str(&format!(
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            row.title,
            format_duration(hyperreal),
            format_duration(rational),
            format_duration(numerica),
            format_duration(gmp),
            format_duration(symbolica),
            format_ratio(hyperreal, numerica),
            format_ratio(hyperreal, gmp),
            format_ratio(hyperreal, symbolica),
        ));
    }
    out
}

/// Builds the legacy curated Markdown document from supplied stored estimates.
///
/// Emits eleven ordered tables containing 257 row descriptors with current case
/// lists, including missing cells; unlisted estimates are ignored. It does not
/// read files, run benchmarks, verify data freshness or establish complete coverage.
/// The fixed prose includes a Ryzen/Fedora attribution and general workload/input
/// claims that are not derived from the supplied map or checked against the host.
/// Those strings must not be treated as per-run provenance or completion evidence.
///
/// Calls the dynamic row builders, which together leak 224 generated title/id
/// strings per render with the current lists. The returned document is ordinary
/// owned text; dropping it does not reclaim those static row strings. No output
/// size/work limit or Markdown escaping is added here.
fn render_benchmarks_md(estimates: &BTreeMap<String, f64>) -> String {
    let mut out = String::new();
    out.push_str(
        "# Benchmarks\n\nRun the Criterion benchmark suite:\n\n```sh\ncargo bench --bench mathbench\n```\n\nRun dispatch path tracing separately:\n\n```sh\ncargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md\n```\n\nRefresh this file from existing Criterion estimates without rerunning the full suite:\n\n```sh\ncargo bench --bench mathbench -- --update-benchmarks-md\n```\n\n",
    );
    out.push_str("The `mathbench` suite benchmarks the Real-primary crate path and writes this file from Criterion's median estimates after a real benchmark run. The exact-dyadic column imports each finite binary64 fixture as its exact dyadic rational value; it does not perform binary64 arithmetic. The explicit-rational column constructs the corresponding authored rational inputs directly. The `numerica128` comparison column runs at 128-bit precision, `gmp_mpfr128` uses Rug's GMP/MPFR stack with 128-bit MPFR scalars, and the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated.\n\n");
    out.push_str("Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.\n\n");
    out.push_str("## Operation Coverage\n\n");
    out.push_str("- Real construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.\n");
    out.push_str("- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.\n");
    out.push_str("- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.\n");
    out.push_str("- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.\n");
    out.push_str("- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.\n\n");
    out.push_str("## Benchmark Results\n\nThe following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.\n\n");
    out.push_str("### Real Operations\n\n#### Real Trigonometric And Inverse Comparisons\n\n");
    out.push_str(&render_table(estimates, &scalar_trig_rows()));
    out.push_str("\n#### Forward Hyperbolic Construction Cases\n\n");
    out.push_str(&render_table(
        estimates,
        &scalar_hyperbolic_case_rows("scalar_hyperbolic_cases"),
    ));
    out.push_str("\n#### Forward Hyperbolic Explicit f64 Output Cases\n\n");
    out.push_str(&render_table(
        estimates,
        &scalar_hyperbolic_case_rows("scalar_hyperbolic_f64_cases"),
    ));
    out.push_str("\n#### Real API Operations\n\n");
    out.push_str(&render_table(estimates, SCALAR_OP_ROWS));
    out.push_str("\n### Complex Operations\n\n");
    out.push_str(&render_table(estimates, COMPLEX_OP_ROWS));
    out.push_str("\n#### Cold Complex Multiplication\n\n");
    out.push_str(&render_table(estimates, COMPLEX_MUL_COLD_ROWS));
    out.push_str("\n### Vector Operations\n\n#### Vector Comparisons\n\n");
    out.push_str(&render_table(estimates, VECTOR_COMPARISON_ROWS));
    out.push_str("\n#### Vector API Operations\n\n");
    out.push_str(&render_table(estimates, VECTOR_OP_ROWS));
    out.push_str("\n### Matrix Operations\n\n#### Matrix Comparisons\n\n");
    out.push_str(&render_table(estimates, MATRIX_COMPARISON_ROWS));
    out.push_str("\n#### Matrix API Operations\n\n");
    out.push_str(&render_table(estimates, MATRIX_OP_ROWS));
    out.push_str("\n### Borrowed API Operations\n\n");
    out.push_str(&render_table(estimates, &borrowed_op_rows()));
    out
}

/// Optionally overwrites cwd/benchmarks.md from stored estimates, logging write failure.
///
/// Returns before reading data if either HYPERLATTICE_SKIP_BENCHMARK_REPORTS or
/// REALISTIC_BLAS_SKIP_BENCHMARK_DOC_UPDATE exists, even empty, or argv contains an
/// exact --test/--list/--help token. No --bench token is required. Argument parsing
/// is just string comparison; non-Unicode argv can panic through std::env::args.
/// If collection yields no estimates, leaves any existing document untouched.
/// Otherwise renders the curated document and directly truncates/writes the cwd
/// file, without reading/preserving its old contents, atomic replacement or writer
/// coordination. A failed write is printed to stderr and does not propagate as a
/// return value; partial output is possible. Traversal errors can silently produce
/// a partial nonempty map that still triggers a full replacement.
///
/// This is separate from the shared benchmark_report writer, which targets the
/// crate's generated section and uses a different argv gate. They coincide in
/// destination only when cwd is the crate directory. No benchmarks run here.
fn update_benchmarks_doc() {
    if std::env::var_os("HYPERLATTICE_SKIP_BENCHMARK_REPORTS").is_some()
        || std::env::var_os("REALISTIC_BLAS_SKIP_BENCHMARK_DOC_UPDATE").is_some()
        || std::env::args().any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
    {
        return;
    }

    let estimates = read_estimates();
    if estimates.is_empty() {
        return;
    }
    let markdown = render_benchmarks_md(&estimates);
    if let Err(error) = fs::write("benchmarks.md", markdown) {
        eprintln!("failed to update benchmarks.md: {error}");
    }
}
