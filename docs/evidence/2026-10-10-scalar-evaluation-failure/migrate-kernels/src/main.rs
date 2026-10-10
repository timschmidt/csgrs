//! One-shot, source-preserving mechanical migration of the archived v0 kernels.
//! The resulting prototype still requires manual caller and precision audits.
use std::{collections::BTreeSet, env, fs, path::PathBuf};
use proc_macro2::{LineColumn, Span};
use syn::{spanned::Spanned, visit::{self, Visit}, *};

fn bigint_output(output: &ReturnType) -> bool {
    match output {
        ReturnType::Type(_, ty) => match ty.as_ref() {
            Type::Path(p) => p.path.is_ident("BigInt"),
            Type::Tuple(t) => t.elems.iter().all(|x| matches!(x, Type::Path(p) if p.path.is_ident("BigInt"))),
            _ => false,
        },
        _ => false,
    }
}

fn selected(sig: &Signature) -> bool {
    if let Ok(names) = env::var("MIGRATE_NAMES") {
        return names.split(',').any(|name| sig.ident == name);
    }
    bigint_output(&sig.output) && sig.ident != "large_rational_quadrant"
}

#[derive(Default)]
struct Inventory { names: BTreeSet<String> }
impl<'a> Visit<'a> for Inventory {
    fn visit_item_fn(&mut self, f: &'a ItemFn) {
        if selected(&f.sig) { self.names.insert(f.sig.ident.to_string()); }
        visit::visit_item_fn(self, f);
    }
}

struct Edit { start: usize, end: usize, text: String }
struct Migration<'a> {
    source: &'a str,
    lines: Vec<usize>,
    names: &'a BTreeSet<String>,
    edits: Vec<Edit>,
    active: bool,
}
impl<'a> Migration<'a> {
    fn offset(&self, p: LineColumn) -> usize { self.lines[p.line - 1] + p.column }
    fn range(&self, span: Span) -> (usize, usize) { (self.offset(span.start()), self.offset(span.end())) }
    fn insert(&mut self, at: usize, text: &str) {
        self.edits.push(Edit { start: at, end: at, text: text.to_owned() });
    }
    fn wrap(&mut self, span: Span, before: &str, after: &str) {
        let (s, e) = self.range(span);
        self.insert(s, before);
        self.insert(e, after);
    }
    fn convert(&mut self, sig: &Signature, body: &Block) {
        let ReturnType::Type(_, ty) = &sig.output else { unreachable!() };
        self.wrap(ty.span(), "Result<", ", Problem>");
        let (s, e) = self.range(body.span());
        self.insert(s + 1, "\nOk({");
        self.insert(e - 1, "})\n");
        let old = self.active;
        self.active = true;
        self.visit_block(body);
        self.active = old;
    }
}
impl<'a> Visit<'a> for Migration<'_> {
    fn visit_item_fn(&mut self, f: &'a ItemFn) {
        if selected(&f.sig) {
            self.convert(&f.sig, &f.block);
        } else {
            let old = self.active;
            self.active = false;
            visit::visit_item_fn(self, f);
            self.active = old;
        }
    }
    fn visit_impl_item_fn(&mut self, f: &'a ImplItemFn) {
        if selected(&f.sig) { self.convert(&f.sig, &f.block); }
    }
    fn visit_expr_closure(&mut self, _c: &'a ExprClosure) {
        // Deferred component closures have differing contracts; migrate by hand.
    }
    fn visit_expr_if(&mut self, i: &'a ExprIf) {
        let (s, e) = self.range(i.cond.span());
        if self.active && self.source[s..e].trim() == "should_stop(signal)" {
            let (s, e) = self.range(i.then_branch.span());
            self.edits.push(Edit { start: s, end: e, text: "{ return Err(Problem::Exhausted); }".to_owned() });
            if let Some((_, other)) = &i.else_branch { self.visit_expr(other); }
        } else {
            visit::visit_expr_if(self, i);
        }
    }
    fn visit_expr_return(&mut self, r: &'a ExprReturn) {
        if self.active && let Some(e) = &r.expr { self.wrap(e.span(), "Ok(", ")"); }
        visit::visit_expr_return(self, r);
    }
    fn visit_expr_call(&mut self, c: &'a ExprCall) {
        if self.active && let Expr::Path(p) = c.func.as_ref()
            && p.path.segments.len() == 1 && self.names.contains(&p.path.segments[0].ident.to_string()) {
            self.insert(self.offset(c.span().end()), "?");
        }
        visit::visit_expr_call(self, c);
    }
    fn visit_expr_method_call(&mut self, c: &'a ExprMethodCall) {
        if self.active && matches!(c.method.to_string().as_str(), "approx" | "approx_signal" | "approximate") {
            self.insert(self.offset(c.span().end()), "?");
        }
        visit::visit_expr_method_call(self, c);
    }
}

fn main() {
    let root = PathBuf::from(env::args().nth(1).expect("isolated approximation directory"));
    let names = ["dispatch", "constants", "arithmetic_kernels", "exp_sqrt", "trig", "logarithms", "inverse_trig", "inverse_hyperbolic", "statistics"];
    let paths: Vec<_> = if env::var_os("MIGRATE_NAMES").is_some() {
        vec![root]
    } else {
        names.iter().map(|n| root.join(format!("{n}.rs"))).collect()
    };
    let files: Vec<_> = paths.into_iter().map(|p| {
        let s = fs::read_to_string(&p).unwrap();
        let ast = syn::parse_file(&s).unwrap();
        (p, s, ast)
    }).collect();
    let mut inventory = Inventory::default();
    for (_, _, ast) in &files { inventory.visit_file(ast); }
    for (path, source, ast) in &files {
        let lines = std::iter::once(0).chain(source.match_indices('\n').map(|(i, _)| i + 1)).collect();
        let mut m = Migration { source, lines, names: &inventory.names, edits: vec![], active: false };
        m.visit_file(ast);
        // At a shared endpoint, close the inner expression before its parent.
        // Traversal records the outer edit first, so applying that edit first
        // leaves later inner suffixes to its left in the final source.
        m.edits.sort_by_key(|e| std::cmp::Reverse(e.start));
        let mut result = source.clone();
        for e in &m.edits { result.replace_range(e.start..e.end, &e.text); }
        syn::parse_file(&result).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        fs::write(path, result).unwrap();
        println!("{}: {} edits", path.display(), m.edits.len());
    }
}
