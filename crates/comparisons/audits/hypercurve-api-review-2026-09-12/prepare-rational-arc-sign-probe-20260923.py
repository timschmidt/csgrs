from pathlib import Path
import hashlib, json, shutil

audit = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-contact-callers-v2-2026-09-23')
root = Path('/tmp/hypercurve-rational-arc-sign-2026-09-23')
prior = json.loads((audit/'contact-callers-20260923-focused2-terminal.json').read_text())
assert prior['all_processes_reaped'] and prior['all_sources_unchanged']
assert not root.exists()
root.mkdir()
shutil.copytree(source/'hypercurve', root/'hypercurve')
for path in source.iterdir():
    if path.name != 'hypercurve' and path.is_dir():
        (root/path.name).symlink_to(path.resolve(), target_is_directory=True)

path = root/'hypercurve/src/classify.rs'
text = path.read_text()
start = text.index('pub(crate) fn real_sign(')
end = text.index('\npub(crate) fn is_zero(', start)
body = text[start:end]
needle = '    policy\n        .consume_predicate(hypersolve::classify_real_sign_predicate('
assert body.count(needle) == 1
body = body.replace(needle, '    let result = policy\n        .consume_predicate(hypersolve::classify_real_sign_predicate(', 1)
assert body.endswith('        })\n}\n')
body = body[:-len('        })\n}\n')] + '''        });
    if result.is_none() {
        static SITES: std::sync::OnceLock<std::sync::Mutex<std::collections::HashSet<(&'static str, u32, u32)>>> = std::sync::OnceLock::new();
        let site = std::panic::Location::caller();
        let mut sites = SITES.get_or_init(Default::default).lock().unwrap();
        if sites.len() < 64 && sites.insert((site.file(), site.line(), site.column())) {
            eprintln!("SIGN-UNRESOLVED site={site} policy={policy:?} zero={:?} rational={}", value.zero_status(), value.exact_rational_ref().is_some());
        }
    }
    result
}
'''
path.write_text(text[:start]+body+text[end:])
bindings = json.loads((audit/'contact-callers-20260923-focused2-sources.json').read_text())
bindings['hypercurve/src/classify.rs'] = hashlib.sha256(path.read_bytes()).hexdigest()
for name, sha in bindings.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
(audit/'rational-arc-sign-20260923-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner = (audit/'run-contact-callers-v2-20260923.py').read_text()
runner = runner.replace(str(source), str(root)).replace("prefix = 'contact-callers-20260923-focused2'", "prefix = 'rational-arc-sign-20260923-probe1'").replace("audit/'contact-callers-20260923-v2-sources.json'", "audit/'rational-arc-sign-20260923-sources.json'")
start = runner.index('selected=[')
end = runner.index('\n]\n', start)+3
runner = runner[:start]+'''selected=[
    ('nonrepresented_chord_and_retained_rational_arc_share_the_fillet_kernel',90),
]
'''+runner[end:]
(audit/'run-rational-arc-sign-probe-20260923.py').write_text(runner)
print('Prepared diagnostic-only immutable sources:',root)
