"""Check the exact transformation at every migrated macro, beyond type checking."""
import difflib
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
candidate = Path(sys.argv[1]).resolve()
record = json.loads((candidate / 'sources.json').read_text())
source = Path(record['source_directory'])
parent = Path(json.loads(Path(record['parent_source_record']).read_text())['source_directory'])
lexer = (EVIDENCE / 'migrate_diagnostics.py').read_text()
namespace = {}
exec(lexer[lexer.index('def skip_literal'):lexer.index('\nfor path in sorted')], {'re': re}, namespace)
namespace['tokens'].__globals__['skip_literal'] = namespace['skip_literal']
tokens = namespace['tokens']
compact = lambda value: ''.join(value.split())
inventory = json.loads((EVIDENCE / 'production-migration.json').read_text())


def macros(text):
    found = []
    for match in re.finditer(r'(?:format!|log::trace!)\s*\(', text):
        start = match.end()
        depth, commas = 0, []
        for index, char in tokens(text, start, len(text)):
            if char in '([{':
                depth += 1
            elif char in ')]}':
                if char == ')' and depth == 0:
                    end = index
                    break
                depth -= 1
            elif char == ',' and depth == 0:
                commas.append(index)
        else:
            raise AssertionError('unclosed macro')
        args = [text[a + 1:b].strip() for a, b in zip([start - 1] + commas, commas + [end])]
        args = [arg for arg in args if arg]
        if args and '#.' in args[0]:
            found.append({'start': match.start(), 'end': end + 1, 'arguments': args})
    return found


rows = []
for name in sorted({row['path'] for row in inventory}):
    before, after = (parent / name).read_text(), (source / name).read_text()
    old, new = macros(before), macros(after)
    expected = [row for row in inventory if row['path'] == name]
    assert len(old) == len(new) == len(expected)
    for left, right, planned in zip(old, new, expected):
        left_args, right_args = left['arguments'], right['arguments']
        assert left_args[0] == right_args[0] == planned['format']
        replacements = {field['positional']: field['expression'] for field in planned['fields'] if 'positional' in field}
        names = list(dict.fromkeys(field['named'] for field in planned['fields'] if 'named' in field))
        assert len(right_args) == len(left_args) + len(names)
        for index, (old_arg, new_arg) in enumerate(zip(left_args[1:], right_args[1:])):
            if index in replacements:
                assert compact(old_arg) == compact(replacements[index])
                assert compact(new_arg) == compact('(' + old_arg + ').diagnostic_decimal(-256)')
            else:
                assert compact(new_arg) == compact(old_arg)
        for index, variable in enumerate(names):
            assert compact(right_args[len(left_args) + index]) == variable + '=' + variable + '.diagnostic_decimal(-256)'
        rows.append({'path': name, 'line': after.count('\n', 0, right['start']) + 1, 'format': left_args[0], 'bindings': planned['fields']})
    for data, found in [(before, old), (after, new)]:
        for item in reversed(found):
            data = data[:item['start']] + '<migrated-macro>' + data[item['end']:]
        if name.endswith('/excellon.rs'):
            # rustfmt wraps the expanded expression-only closure in a block.
            data = compact(data).replace('.map(|diameter|{<migrated-macro>})', '.map(|diameter|<migrated-macro>)')
        if 'clean_before' not in locals():
            clean_before = compact(data)
        else:
            assert compact(data) == clean_before, name
            del clean_before
patch = []
for name in record['changed_files']:
    before = (parent / name).read_text() if (parent / name).is_file() else ''
    after = (source / name).read_text()
    patch.extend(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True), fromfile='a/' + name, tofile='b/' + name))
(candidate / 'production.patch').write_text(''.join(patch))
result = {'source_record': str(candidate / 'sources.json'), 'macro_sites': len(rows), 'files': len({row['path'] for row in rows}), 'assertions': ['Format literals unchanged.', 'Only designated numeric arguments receive the diagnostic view; all other arguments retain their tokens.', 'Named arguments bind exactly the original captured scalar names.', 'Outside tokens are unchanged except the reviewed rustfmt block around the expression-only Excellon diameter formatting closure.'], 'scope': 'Token/argument transformation audit, not a claim of reading every source file or proving numeric algorithms.', 'sites': rows, 'patch_sha256': hashlib.sha256((candidate / 'production.patch').read_bytes()).hexdigest()}
(candidate / 'migration-audit.json').write_text(json.dumps(result, indent=2) + '\n')
print('Audited',len(rows),'macro transformations; outside tokens unchanged.')
