use hyperreal::{Computable, Rational};
use num::BigInt;

#[derive(Clone)]
struct Graphed {
    id: usize,
    label: String,
    value: Computable,
}

#[derive(Default)]
struct Graph {
    name: &'static str,
    nodes: Vec<(usize, String)>,
    edges: Vec<(usize, usize)>,
}

impl Graph {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            ..Self::default()
        }
    }

    fn leaf(&mut self, label: impl Into<String>, value: Computable) -> Graphed {
        let id = self.nodes.len();
        let label = label.into();
        self.nodes.push((id, label.clone()));
        Graphed { id, label, value }
    }

    fn unary(
        &mut self,
        label: impl Into<String>,
        input: &Graphed,
        op: impl FnOnce(Computable) -> Computable,
    ) -> Graphed {
        let id = self.nodes.len();
        let label = label.into();
        self.nodes.push((id, label.clone()));
        self.edges.push((input.id, id));
        Graphed {
            id,
            label,
            value: op(input.value.clone()),
        }
    }

    fn binary(
        &mut self,
        label: impl Into<String>,
        left: &Graphed,
        right: &Graphed,
        op: impl FnOnce(Computable, Computable) -> Computable,
    ) -> Graphed {
        let id = self.nodes.len();
        let label = label.into();
        self.nodes.push((id, label.clone()));
        self.edges.push((left.id, id));
        self.edges.push((right.id, id));
        Graphed {
            id,
            label,
            value: op(left.value.clone(), right.value.clone()),
        }
    }

    fn markdown(&self, root: &Graphed) -> String {
        let mut out = String::new();
        out.push_str("```mermaid\nflowchart TD\n");
        for (id, label) in &self.nodes {
            out.push_str(&format!("    n{id}[\"{}\"]\n", escape_mermaid(label)));
        }
        for (from, to) in &self.edges {
            out.push_str(&format!("    n{from} --> n{to}\n"));
        }
        out.push_str(&format!("    n{}:::root\n", root.id));
        out.push_str("    classDef root fill:#f7f3c6,stroke:#8a6d00,stroke-width:2px\n");
        out.push_str("```\n");
        out
    }
}

fn escape_mermaid(label: &str) -> String {
    label.replace('\\', "\\\\").replace('"', "\\\"")
}

fn rational(n: i64, d: u64) -> Computable {
    Computable::rational(Rational::fraction(n, d).expect("valid nonzero denominator"))
}

fn integer(n: i64) -> Computable {
    Computable::rational(Rational::new(n))
}

fn huge_integer_pow10(exp: u32) -> Computable {
    Computable::rational(Rational::from_bigint(BigInt::from(10_u8).pow(exp)))
}

fn build_argument_reduction_graph() -> (Graph, Graphed) {
    let mut graph = Graph::new("argument_reduction_tower");

    let pi = graph.leaf("pi shared constant", Computable::pi());
    let huge = graph.leaf("10^30 exact integer", huge_integer_pow10(30));
    let seven_fifths = graph.leaf("7/5 exact rational", rational(7, 5));
    let three_fifths = graph.leaf("3/5 exact rational", rational(3, 5));
    let seven_tenths = graph.leaf("7/10 exact rational", rational(7, 10));
    let tiny = graph.leaf("1/2^40 exact rational", rational(1, 1_u64 << 40));

    let huge_pi = graph.binary("multiply", &pi, &huge, Computable::multiply);
    let phase = graph.binary("add residual", &huge_pi, &seven_fifths, Computable::add);
    let phase_plus_tiny = graph.binary("add tiny perturbation", &phase, &tiny, Computable::add);

    let sin_phase = graph.unary("sin", &phase_plus_tiny, Computable::sin);
    let cos_phase = graph.unary("cos", &phase, Computable::cos);
    let tan_atan = {
        let atan = graph.unary("atan", &seven_tenths, Computable::atan);
        graph.unary("tan", &atan, Computable::tan)
    };
    let asin_sin = {
        let asin = graph.unary("asin", &three_fifths, Computable::asin);
        graph.unary("sin", &asin, Computable::sin)
    };

    let sin_sq = graph.unary("square", &sin_phase, Computable::square);
    let cos_sq = graph.unary("square", &cos_phase, Computable::square);
    let trig_norm = graph.binary("add", &sin_sq, &cos_sq, Computable::add);
    let inverse_norm = graph.unary("inverse", &trig_norm, Computable::inverse);
    let numerator = graph.binary("add", &tan_atan, &asin_sin, Computable::add);
    let product = graph.binary("multiply", &numerator, &inverse_norm, Computable::multiply);
    let root = graph.unary("sqrt", &product, Computable::sqrt);

    (graph, root)
}

fn build_cancellation_graph() -> (Graph, Graphed) {
    let mut graph = Graph::new("cancellation_and_nested_inverse_tower");

    let pi = graph.leaf("pi shared constant", Computable::pi());
    let e = graph.leaf("e shared constant", Computable::e());
    let two = graph.leaf("2 exact integer", integer(2));
    let twelve = graph.leaf("12 exact integer", integer(12));
    let forty_five_fourteen = graph.leaf("45/14 exact rational", rational(45, 14));
    let almost_one = graph.leaf(
        "999999/1000000 exact rational",
        rational(999_999, 1_000_000),
    );
    let half = graph.leaf("1/2 exact rational", rational(1, 2));
    let epsilon = graph.leaf("1/2^50 exact rational", rational(1, 1_u64 << 50));

    let sqrt2 = graph.unary("sqrt", &two, Computable::sqrt);
    let sqrt2_square = graph.unary("square", &sqrt2, Computable::square);

    let sqrt12 = graph.unary("sqrt", &twelve, Computable::sqrt);
    let radical_plus_e = graph.binary("add", &sqrt12, &e, Computable::add);
    let ln_radical = graph.unary("ln", &radical_plus_e, Computable::ln);
    let exp_ln_radical = graph.unary("exp", &ln_radical, Computable::exp);

    let smooth_ln = graph.unary("ln", &forty_five_fourteen, Computable::ln);
    let smooth_exp = graph.unary("exp", &smooth_ln, Computable::exp);

    let atanh = graph.unary("atanh", &almost_one, Computable::atanh);
    let asinh = graph.unary("asinh", &half, Computable::asinh);
    let acosh_input = graph.binary("add", &sqrt2_square, &half, Computable::add);
    let acosh = graph.unary("acosh", &acosh_input, Computable::acosh);
    let hyperbolic_sum = graph.binary("add", &atanh, &asinh, Computable::add);
    let hyperbolic_sum = graph.binary("add", &hyperbolic_sum, &acosh, Computable::add);

    let pi_plus_epsilon = graph.binary("add", &pi, &epsilon, Computable::add);
    let neg_pi = graph.unary("negate", &pi, Computable::negate);
    let near_cancel = graph.binary("add", &pi_plus_epsilon, &neg_pi, Computable::add);
    let inverse_cancel = graph.unary("inverse", &near_cancel, Computable::inverse);
    let inverse_pair = graph.unary("inverse", &inverse_cancel, Computable::inverse);

    let positive_sum = graph.binary("add", &exp_ln_radical, &smooth_exp, Computable::add);
    let positive_sum = graph.binary("add", &positive_sum, &hyperbolic_sum, Computable::add);
    let positive_sum = graph.binary("add", &positive_sum, &inverse_pair, Computable::add);
    let root = graph.unary("sqrt", &positive_sum, Computable::sqrt);

    (graph, root)
}

fn print_report(graph: &Graph, root: &Graphed) {
    let p = -80;
    let scaled = root.value.approx(p);

    println!("## {}", graph.name);
    println!();
    println!("{}", graph.markdown(root));
    println!();
    println!("Root node: `{}`", root.label);
    println!("Evaluation request: `root.approx({p})`");
    println!("Scaled integer result: `{scaled}`");
    println!("Decimal display: `{:.24}`", root.value);
    println!();
}

fn main() {
    let (argument_graph, argument_root) = build_argument_reduction_graph();
    print_report(&argument_graph, &argument_root);

    let (cancellation_graph, cancellation_root) = build_cancellation_graph();
    print_report(&cancellation_graph, &cancellation_root);
}
