#[derive(Clone, Copy)]
struct BenchRow {
    title: &'static str,
    group: &'static str,
    id: &'static str,
}

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

fn borrowed_op_rows() -> Vec<BenchRow> {
    let mut rows = Vec::new();
    for op in ["add", "sub", "mul", "div"] {
        for mode in ["owned_ref", "ref_owned", "refs"] {
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

const PRECISION_ROWS: &[BenchRow] = &[
    BenchRow {
        title: "astro sin 128",
        group: "scalar_trig_by_precision",
        id: "astro_sin/128",
    },
    BenchRow {
        title: "astro sin 160",
        group: "scalar_trig_by_precision",
        id: "astro_sin/160",
    },
    BenchRow {
        title: "astro sin 192",
        group: "scalar_trig_by_precision",
        id: "astro_sin/192",
    },
    BenchRow {
        title: "astro sin 256",
        group: "scalar_trig_by_precision",
        id: "astro_sin/256",
    },
];

fn estimate_key(group: &str, variant: &str, id: &str) -> String {
    format!("{group}/{}_{}", variant, id.replace('/', "_"))
}

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

fn read_estimates() -> BTreeMap<String, f64> {
    fn visit(path: &Path, out: &mut BTreeMap<String, f64>) {
        let Ok(entries) = fs::read_dir(path) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                visit(&path, out);
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
            let Ok(relative) = bench_path.strip_prefix("target/criterion") else {
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
    visit(Path::new("target/criterion"), &mut estimates);
    estimates
}

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

fn format_ratio(numerator: Option<f64>, denominator: Option<f64>) -> String {
    match (numerator, denominator) {
        (Some(numerator), Some(denominator)) if denominator != 0.0 => {
            format!("{:.2}x", numerator / denominator)
        }
        _ => "-".to_string(),
    }
}

fn render_table(estimates: &BTreeMap<String, f64>, rows: &[BenchRow]) -> String {
    let mut out = String::new();
    out.push_str("| Benchmark | Approx | Hyperreal from f64 | Hyperreal rational | astro-float 128 | numerica128 | symbolica | Hyperreal f64 / approx | Hyperreal f64 / astro | Hyperreal f64 / numerica128 | Hyperreal f64 / symbolica |\n");
    out.push_str("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
    for row in rows {
        let approx = estimate_value(estimates, row.group, "approx", row.id);
        let hyperreal = estimate_value(estimates, row.group, "hyperreal", row.id)
            .or_else(|| estimate_value(estimates, row.group, "realistic", row.id));
        let rational = estimate_value(estimates, row.group, "hyperreal-rational", row.id)
            .or_else(|| estimate_value(estimates, row.group, "realistic-rational", row.id));
        let astro = estimate_value(estimates, row.group, "astro128", row.id);
        let numerica = estimate_value(estimates, row.group, "numerica128", row.id);
        let symbolica = estimate_value(estimates, row.group, "symbolica", row.id);
        out.push_str(&format!(
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            row.title,
            format_duration(approx),
            format_duration(hyperreal),
            format_duration(rational),
            format_duration(astro),
            format_duration(numerica),
            format_duration(symbolica),
            format_ratio(hyperreal, approx),
            format_ratio(hyperreal, astro),
            format_ratio(hyperreal, numerica),
            format_ratio(hyperreal, symbolica),
        ));
    }
    out
}

fn render_precision_table(estimates: &BTreeMap<String, f64>) -> String {
    let mut out = String::new();
    out.push_str("| Benchmark | Median |\n");
    out.push_str("| --- | ---: |\n");
    for row in PRECISION_ROWS {
        let key = format!("{}/{}", row.group, row.id);
        out.push_str(&format!(
            "| `{}` | {} |\n",
            row.title,
            format_duration(estimates.get(&key).copied())
        ));
    }
    out
}

fn render_benchmarks_md(estimates: &BTreeMap<String, f64>) -> String {
    let mut out = String::new();
    out.push_str(
        "# Benchmarks\n\nRun the Criterion benchmark suite:\n\n```sh\ncargo bench --bench mathbench\n```\n\nRefresh this file from existing Criterion estimates without rerunning the full suite:\n\n```sh\ncargo bench --bench mathbench -- --update-benchmarks-md\n```\n\n",
    );
    out.push_str("The `mathbench` suite benchmarks both crate backends and writes this file from Criterion's median estimates after a real benchmark run. The `astro-float 128` and `numerica128` comparison columns run at 128-bit precision, while the `symbolica` column exercises Symbolica's symbolic expression engine. Missing cells mean that the corresponding estimate was not present in `target/criterion` when this file was generated, or that the external library does not expose a directly comparable operation in this suite.\n\n");
    out.push_str("Each benchmarked operation rotates through adversarial inputs for its valid domain: near-zero values, large and tiny magnitudes, cancellation-prone vectors, near-singular matrices, range-reduction-heavy trigonometric arguments, and boundary-adjacent inverse trigonometric and inverse hyperbolic values.\n\n");
    out.push_str("## Operation Coverage\n\n");
    out.push_str("- Scalar construction/constants, arithmetic, reciprocal, powers, exponentials, logarithms, square root, trigonometric and hyperbolic functions, inverse helpers, zero-status checks, and abort-aware variants.\n");
    out.push_str("- Complex construction/constants, conjugate, norm squared, reciprocal, powers, checked division, scalar conversion, arithmetic, and real scalar division.\n");
    out.push_str("- Vector construction, zero, dot product, magnitude, normalization, vector/vector arithmetic, vector/scalar arithmetic, scalar division, and checked/abort-aware variants for 3D and 4D vectors.\n");
    out.push_str("- Matrix construction, zero, identity, transpose, determinant, inverse, reciprocal, powers, matrix/matrix arithmetic, matrix/scalar arithmetic, matrix/vector transformation, scalar division, matrix division, and checked/abort-aware variants for 3x3 and 4x4 matrices.\n");
    out.push_str("- Borrowed API operator coverage for scalar, vector, matrix, matrix/vector, and complex reference combinations.\n\n");
    out.push_str("## Benchmark Results\n\nThe following Criterion median estimates were collected on an AMD Ryzen 7 5800X3D on Fedora. Values are formatted to two digits after the decimal.\n\n");
    out.push_str("### Scalar Operations\n\n#### Scalar Trigonometric And Inverse Comparisons\n\n");
    out.push_str(&render_table(estimates, &scalar_trig_rows()));
    out.push_str("\n#### Scalar API Operations\n\n");
    out.push_str(&render_table(estimates, SCALAR_OP_ROWS));
    out.push_str("\n### Complex Operations\n\n");
    out.push_str(&render_table(estimates, COMPLEX_OP_ROWS));
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
    out.push_str("\n### Precision Scaling\n\n");
    out.push_str(&render_precision_table(estimates));
    out
}

fn update_benchmarks_doc() {
    if std::env::var_os("REALISTIC_BLAS_SKIP_BENCHMARK_DOC_UPDATE").is_some()
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
