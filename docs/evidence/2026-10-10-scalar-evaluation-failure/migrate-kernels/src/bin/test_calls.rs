//! Preserve existing successful-test assertions while migrating typed calls.
//! Cancellation and domain-error assertions must be audited separately.
use std::{env, fs, path::Path};
use proc_macro2::{LineColumn, TokenStream, TokenTree};
use syn::{spanned::Spanned, visit::{self, Visit}};

struct TestRanges { lines: Vec<usize>, ranges: Vec<(usize, usize)> }
impl TestRanges {
    fn offset(&self, p: LineColumn) -> usize { self.lines[p.line - 1] + p.column }
    fn add(&mut self, span: proc_macro2::Span) {
        self.ranges.push((self.offset(span.start()), self.offset(span.end())));
    }
}
fn test_attr(attrs: &[syn::Attribute]) -> bool {
    attrs.iter().any(|a| a.path().is_ident("test") ||
        (a.path().is_ident("cfg") && a.parse_args::<syn::Ident>().is_ok_and(|ident| ident == "test")))
}
impl<'a> Visit<'a> for TestRanges {
    fn visit_item_mod(&mut self, item: &'a syn::ItemMod) {
        if test_attr(&item.attrs) { self.add(item.span()); }
        visit::visit_item_mod(self, item);
    }
    fn visit_item_fn(&mut self, item: &'a syn::ItemFn) {
        if test_attr(&item.attrs) { self.add(item.span()); }
        visit::visit_item_fn(self, item);
    }
}
fn collect(stream: TokenStream, ranges: &TestRanges, edits: &mut Vec<usize>) {
    let tokens: Vec<_> = stream.into_iter().collect();
    for window in tokens.windows(3) {
        if let [TokenTree::Punct(dot), TokenTree::Ident(method), TokenTree::Group(args)] = window
            && dot.as_char() == '.' && args.delimiter() == proc_macro2::Delimiter::Parenthesis
            && matches!(method.to_string().as_str(), "approx" | "approx_signal" | "approximate" | "near_integer" | "compare_absolute" | "msd" | "iter_msd") {
            let at = ranges.offset(args.span().end());
            if ranges.ranges.iter().any(|&(s,e)| s <= at && at <= e) { edits.push(at); }
        }
    }
    for token in tokens {
        if let TokenTree::Group(group) = token { collect(group.stream(), ranges, edits); }
    }
}
fn main() {
    for argument in env::args().skip(1) {
        let path = Path::new(&argument);
        let source = fs::read_to_string(path).unwrap();
        let lines = std::iter::once(0).chain(source.match_indices('\n').map(|(i,_)| i+1)).collect();
        let mut ranges = TestRanges { lines, ranges: vec![] };
        if argument.contains("/tests/") { ranges.ranges.push((0, source.len())); }
        ranges.visit_file(&syn::parse_file(&source).unwrap());
        let mut edits = vec![];
        collect(source.parse().unwrap(), &ranges, &mut edits);
        edits.sort_unstable(); edits.dedup();
        let mut output = source;
        for &at in edits.iter().rev() { output.insert_str(at, ".expect(\"test evaluation should finish\")"); }
        if !edits.is_empty() {
            syn::parse_file(&output).unwrap();
            fs::write(path, output).unwrap();
            println!("{}: {} calls", path.display(), edits.len());
        }
    }
}
